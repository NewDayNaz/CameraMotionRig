/**
 * @file tmc_driver.c
 * @brief TMC2209 UART datagrams (write + optional read-back)
 */

#include "tmc_driver.h"
#include "board.h"
#include "stepper_limits.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "soc/gpio_struct.h"

#include <stdio.h>
#include <string.h>

static const char *TAG = "tmc_driver";

#define TMC_UART           UART_NUM_1
#define TMC_UART_BUF_SIZE  256
#define TMC_SYNC           0x05
#define TMC_WRITE_BIT      0x80

#define TMC_REG_GCONF      0x00
#define TMC_REG_GSTAT      0x01
#define TMC_REG_IFCNT      0x02
#define TMC_REG_IOIN       0x06
#define TMC_REG_IHOLD_IRUN 0x10
#define TMC_REG_TPOWERDOWN 0x11
#define TMC_REG_TSTEP      0x12
#define TMC_REG_TPWMTHRS   0x13
#define TMC_REG_TCOOLTHRS  0x14
#define TMC_REG_SGTHRS     0x40
#define TMC_REG_SG_RESULT  0x41
#define TMC_REG_CHOPCONF   0x6C

#define TMC_GCONF_VALUE    0x000001C4u
#define TMC_CHOPCONF_MRES8 0x15000053u
#define TMC_TPWMTHRS       0u
#define TMC_TCOOLTHRS_ZOOM 0x000FFFFFu
#define TMC_SGTHRS_ZOOM    60u
#define TMC_TPOWERDOWN     20u  /* ~0.4 s until IHOLD after last step */

#define TMC_2209_VERSION   0x21u
#define TMC_REPLY_MASTER   0xFFu  /* driver → MCU address in read replies */
#define TMC_READ_RETRIES   3
#define TMC_READ_TIMEOUT_MS 20

static uint32_t pack_ihold_irun(uint8_t ihold, uint8_t irun)
{
    return ((uint32_t)(ihold & 0x1F)) |
           ((uint32_t)(irun & 0x1F) << 8) |
           ((uint32_t)(TMC_IHOLDDELAY & 0x0F) << 16);
}

static uint32_t axis_ihold_irun(uint8_t axis, bool standby)
{
    uint8_t irun = TMC_IRUN_PAN;
    uint8_t ihold = TMC_IHOLD_PAN;
    if (axis == AXIS_TILT) {
        irun = TMC_IRUN_TILT;
        ihold = standby ? TMC_IHOLD_STANDBY_TILT : TMC_IHOLD_TILT;
    } else if (axis == AXIS_ZOOM) {
        irun = TMC_IRUN_ZOOM;
        ihold = standby ? TMC_IHOLD_STANDBY_ZOOM : TMC_IHOLD_ZOOM;
    } else {
        ihold = standby ? TMC_IHOLD_STANDBY_PAN : TMC_IHOLD_PAN;
    }
    return pack_ihold_irun(ihold, irun);
}

static bool tmc_uart_up = false;
static bool tmc_ready = false;
static SemaphoreHandle_t tmc_mutex;
static int s_last_rx_len;
static uint8_t s_last_rx[24];

static uint8_t tmc_crc8(const uint8_t *data, size_t nbytes)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < nbytes; i++) {
        uint8_t current = data[i];
        for (int bit = 0; bit < 8; bit++) {
            if ((crc >> 7) ^ (current & 0x01)) {
                crc = (uint8_t)((crc << 1) ^ 0x07);
            } else {
                crc = (uint8_t)(crc << 1);
            }
            current >>= 1;
        }
    }
    return crc;
}

static void tmc_flush_rx(void)
{
    uart_flush_input(TMC_UART);
}

static void tmc_bytes_to_hex(char *dst, size_t dst_sz, const uint8_t *src, int n)
{
    if (dst == NULL || dst_sz == 0) {
        return;
    }
    dst[0] = '\0';
    if (src == NULL || n <= 0) {
        return;
    }
    size_t used = 0;
    for (int i = 0; i < n; i++) {
        int written = snprintf(dst + used, dst_sz - used, "%s%02X", (i == 0) ? "" : " ", src[i]);
        if (written < 0 || (size_t)written >= dst_sz - used) {
            dst[dst_sz - 1] = '\0';
            return;
        }
        used += (size_t)written;
    }
}

/* First byte waits up to first_wait; remaining bytes drain with a short idle gap.
 * Asking uart_read_bytes() for 24 bytes at once can miss short TMC frames sitting
 * in the HW FIFO until rx_full_threshold (default 120) or RX timeout fires. */
static int tmc_uart_collect(uint8_t *buf, int max_len, TickType_t first_wait)
{
    if (buf == NULL || max_len <= 0) {
        return 0;
    }
    int n = uart_read_bytes(TMC_UART, buf, 1, first_wait);
    if (n <= 0) {
        return 0;
    }
    int len = n;
    while (len < max_len) {
        n = uart_read_bytes(TMC_UART, buf + len, (uint32_t)(max_len - len), pdMS_TO_TICKS(5));
        if (n <= 0) {
            break;
        }
        len += n;
    }
    return len;
}

static bool tmc_write_reg_unlocked(uint8_t slave, uint8_t reg, uint32_t value)
{
    uint8_t datagram[8];
    datagram[0] = TMC_SYNC;
    datagram[1] = slave;
    datagram[2] = (uint8_t)(reg | TMC_WRITE_BIT);
    datagram[3] = (uint8_t)((value >> 24) & 0xFF);
    datagram[4] = (uint8_t)((value >> 16) & 0xFF);
    datagram[5] = (uint8_t)((value >> 8) & 0xFF);
    datagram[6] = (uint8_t)(value & 0xFF);
    datagram[7] = tmc_crc8(datagram, 7);

    tmc_flush_rx();
    int written = uart_write_bytes(TMC_UART, (const char *)datagram, sizeof(datagram));
    uart_wait_tx_done(TMC_UART, pdMS_TO_TICKS(20));
    esp_rom_delay_us(200);
    return written == (int)sizeof(datagram);
}

/* FYSETC E4 ties UART TX to PDN through a resistor and RX directly, so the
 * 4-byte read request is echoed on RX before the 8-byte 0x05 0xFF reply. */
static bool tmc_parse_read_reply(const uint8_t *buf, int len, uint8_t reg, uint32_t *value)
{
    uint8_t want_reg = (uint8_t)(reg & 0x7F);
    if (buf == NULL || len < 8) {
        return false;
    }
    for (int i = 0; i <= len - 8; i++) {
        if (buf[i] != TMC_SYNC || buf[i + 1] != TMC_REPLY_MASTER) {
            continue;
        }
        if ((buf[i + 2] & 0x7F) != want_reg) {
            continue;
        }
        if (tmc_crc8(&buf[i], 7) != buf[i + 7]) {
            continue;
        }
        if (value != NULL) {
            *value = ((uint32_t)buf[i + 3] << 24) | ((uint32_t)buf[i + 4] << 16) |
                     ((uint32_t)buf[i + 5] << 8) | (uint32_t)buf[i + 6];
        }
        return true;
    }
    return false;
}

static bool tmc_read_reg_unlocked(uint8_t slave, uint8_t reg, uint32_t *value)
{
    uint8_t request[4];
    request[0] = TMC_SYNC;
    request[1] = slave;
    request[2] = (uint8_t)(reg & 0x7F);
    request[3] = tmc_crc8(request, 3);

    for (int attempt = 0; attempt < TMC_READ_RETRIES; attempt++) {
        tmc_flush_rx();
        uart_write_bytes(TMC_UART, (const char *)request, sizeof(request));
        uart_wait_tx_done(TMC_UART, pdMS_TO_TICKS(20));
        /* Reply starts within a few bit times; echo is already in the FIFO. */
        esp_rom_delay_us(200);

        uint8_t buf[24];
        int len = tmc_uart_collect(buf, (int)sizeof(buf), pdMS_TO_TICKS(TMC_READ_TIMEOUT_MS));
        s_last_rx_len = len;
        memset(s_last_rx, 0, sizeof(s_last_rx));
        if (len > 0) {
            int copy = len;
            if (copy > (int)sizeof(s_last_rx)) {
                copy = (int)sizeof(s_last_rx);
            }
            memcpy(s_last_rx, buf, (size_t)copy);
        }
        if (tmc_parse_read_reply(buf, len, request[2], value)) {
            return true;
        }
        if (len > 0) {
            ESP_LOGD(TAG, "addr %u reg 0x%02X: %d RX bytes, no 05 FF frame",
                     (unsigned)slave, (unsigned)request[2], len);
        }
    }
    return false;
}

static void tmc_lock(void)
{
    if (tmc_mutex != NULL) {
        xSemaphoreTake(tmc_mutex, portMAX_DELAY);
    }
}

static void tmc_unlock(void)
{
    if (tmc_mutex != NULL) {
        xSemaphoreGive(tmc_mutex);
    }
}

static bool tmc_configure_axis_unlocked(uint8_t axis)
{
    uint8_t slave = board_get_tmc2209_address(axis);
    const char *name = axis_names[axis];

    if (!tmc_write_reg_unlocked(slave, TMC_REG_GCONF, TMC_GCONF_VALUE)) {
        ESP_LOGW(TAG, "%s: UART write GCONF failed (addr %u)", name, slave);
        return false;
    }
    tmc_write_reg_unlocked(slave, TMC_REG_IHOLD_IRUN, axis_ihold_irun(axis, false));
    tmc_write_reg_unlocked(slave, TMC_REG_TPOWERDOWN, TMC_TPOWERDOWN);
    tmc_write_reg_unlocked(slave, TMC_REG_TPWMTHRS, TMC_TPWMTHRS);
    tmc_write_reg_unlocked(slave, TMC_REG_CHOPCONF, TMC_CHOPCONF_MRES8);

    if (axis == AXIS_ZOOM) {
        tmc_write_reg_unlocked(slave, TMC_REG_TCOOLTHRS, TMC_TCOOLTHRS_ZOOM);
        tmc_write_reg_unlocked(slave, TMC_REG_SGTHRS, TMC_SGTHRS_ZOOM);
        ESP_LOGI(TAG, "ZOOM: stallGuard enabled (TCOOLTHRS=max SGTHRS=%u)",
                 (unsigned)TMC_SGTHRS_ZOOM);
    } else {
        tmc_write_reg_unlocked(slave, TMC_REG_TCOOLTHRS, 0);
        tmc_write_reg_unlocked(slave, TMC_REG_SGTHRS, 0);
    }

    uint32_t gconf = 0;
    if (tmc_read_reg_unlocked(slave, TMC_REG_GCONF, &gconf)) {
        ESP_LOGI(TAG, "%s: TMC2209 addr %u GCONF=0x%08lX (8 ustep, spreadCycle)",
                 name, slave, (unsigned long)gconf);
        return true;
    }

    ESP_LOGW(TAG, "%s: TMC2209 addr %u wrote config but read-back failed", name, slave);
    return false;
}

bool tmc_driver_init(void)
{
    tmc_mutex = xSemaphoreCreateMutex();

    uart_config_t uart_config = {
        .baud_rate = TMC2209_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(TMC_UART, TMC_UART_BUF_SIZE, TMC_UART_BUF_SIZE, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART1 install failed: %s", esp_err_to_name(err));
        return false;
    }
    ESP_ERROR_CHECK(uart_param_config(TMC_UART, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(TMC_UART, PIN_UART1_TX, PIN_UART1_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    /* Short TMC replies never fill the default 120-byte RX FIFO threshold. */
    (void)uart_set_rx_full_threshold(TMC_UART, 1);
    (void)uart_set_rx_timeout(TMC_UART, 3);
    /* Open-drain TX so a TMC can pull PDN low against idle-high. Do not
     * gpio_set_direction() here — that would detach the UART matrix. */
    GPIO.pin[PIN_UART1_TX].pad_driver = 1;
    gpio_pullup_en(PIN_UART1_TX);
    gpio_pullup_en(PIN_UART1_RX);
    tmc_uart_up = true;

    vTaskDelay(pdMS_TO_TICKS(20));

    bool all_ok = true;
    tmc_lock();
    for (uint8_t axis = 0; axis < NUM_AXES; axis++) {
        if (!tmc_configure_axis_unlocked(axis)) {
            all_ok = false;
        }
    }
    tmc_unlock();

    tmc_ready = all_ok;
    if (all_ok) {
        ESP_LOGI(TAG, "TMC2209 UART configuration complete");
    } else {
        ESP_LOGW(TAG, "TMC2209 UART incomplete — motors still run from hardware pin config");
        tmc_lock();
        for (uint8_t addr = 0; addr < 4; addr++) {
            uint32_t ioin = 0;
            if (tmc_read_reg_unlocked(addr, TMC_REG_IOIN, &ioin)) {
                ESP_LOGI(TAG, "UART scan: addr %u answered IOIN=0x%08lX version=0x%02X",
                         (unsigned)addr, (unsigned long)ioin,
                         (unsigned)((ioin >> 24) & 0xFF));
            }
        }
        tmc_unlock();
    }
    return all_ok;
}

bool tmc_driver_is_ready(void)
{
    return tmc_ready;
}

bool tmc_driver_uart_installed(void)
{
    return tmc_uart_up;
}

bool tmc_driver_read_sg_result(uint8_t axis, uint16_t *sg_result)
{
    if (!tmc_uart_up || axis >= NUM_AXES || sg_result == NULL) {
        return false;
    }
    uint32_t value = 0;
    tmc_lock();
    bool ok = tmc_read_reg_unlocked(board_get_tmc2209_address(axis), TMC_REG_SG_RESULT, &value);
    tmc_unlock();
    if (!ok) {
        return false;
    }
    *sg_result = (uint16_t)(value & 0x3FF);
    return true;
}

bool tmc_driver_diagnose_axis(uint8_t axis, tmc_diag_t *out)
{
    if (out == NULL || axis >= NUM_AXES) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    out->axis = axis;
    out->address = board_get_tmc2209_address(axis);

    if (!tmc_uart_up) {
        return false;
    }

    uint8_t slave = out->address;
    tmc_lock();

    out->uart_ok = tmc_read_reg_unlocked(slave, TMC_REG_GCONF, &out->gconf);
    out->rx_len = s_last_rx_len;
    tmc_bytes_to_hex(out->rx_hex, sizeof(out->rx_hex), s_last_rx, s_last_rx_len);
    tmc_read_reg_unlocked(slave, TMC_REG_GSTAT, &out->gstat);
    tmc_read_reg_unlocked(slave, TMC_REG_IOIN, &out->ioin);
    out->ic_version = (uint8_t)((out->ioin >> 24) & 0xFF);

    tmc_read_reg_unlocked(slave, TMC_REG_IFCNT, &out->ifcnt_before);
    tmc_write_reg_unlocked(slave, TMC_REG_TPOWERDOWN, TMC_TPOWERDOWN);
    tmc_read_reg_unlocked(slave, TMC_REG_IFCNT, &out->ifcnt_after);
    uint8_t before8 = (uint8_t)(out->ifcnt_before & 0xFF);
    uint8_t after8 = (uint8_t)(out->ifcnt_after & 0xFF);
    out->write_ack = (uint8_t)(after8 - before8) != 0;

    tmc_read_reg_unlocked(slave, TMC_REG_TSTEP, &out->tstep);
    tmc_read_reg_unlocked(slave, TMC_REG_TCOOLTHRS, &out->tcoolthrs);
    tmc_read_reg_unlocked(slave, TMC_REG_SGTHRS, &out->sgthrs);
    tmc_read_reg_unlocked(slave, TMC_REG_SG_RESULT, &out->sg_result);
    out->sg_result &= 0x3FF;
    tmc_read_reg_unlocked(slave, TMC_REG_CHOPCONF, &out->chopconf);
    tmc_read_reg_unlocked(slave, TMC_REG_IHOLD_IRUN, &out->ihold_irun);
    out->ihold = (uint8_t)(out->ihold_irun & 0x1F);
    out->irun = (uint8_t)((out->ihold_irun >> 8) & 0x1F);

    if (!out->uart_ok && out->ic_version == TMC_2209_VERSION) {
        out->uart_ok = true;
    }

    tmc_unlock();
    return out->uart_ok;
}

bool tmc_driver_bus_echo(tmc_loopback_t *out)
{
    if (out == NULL) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    if (!tmc_uart_up) {
        return false;
    }

    const uint8_t probe[4] = { 0xA5, 0x5A, 0xC3, 0x3C };
    uint8_t buf[16];
    tmc_lock();
    tmc_flush_rx();
    int written = uart_write_bytes(TMC_UART, (const char *)probe, sizeof(probe));
    uart_wait_tx_done(TMC_UART, pdMS_TO_TICKS(20));
    int len = tmc_uart_collect(buf, (int)sizeof(buf), pdMS_TO_TICKS(20));
    tmc_unlock();

    out->tx_len = written;
    out->rx_len = len;
    tmc_bytes_to_hex(out->rx_hex, sizeof(out->rx_hex), buf, len);
    return len > 0;
}

bool tmc_driver_reconfigure(void)
{
    if (!tmc_uart_up) {
        return false;
    }
    bool all_ok = true;
    tmc_lock();
    for (uint8_t axis = 0; axis < NUM_AXES; axis++) {
        if (!tmc_configure_axis_unlocked(axis)) {
            all_ok = false;
        }
    }
    tmc_unlock();
    tmc_ready = all_ok;
    return all_ok;
}

bool tmc_driver_read_sg_tstep(uint8_t axis, uint16_t *sg_result, uint32_t *tstep)
{
    if (!tmc_uart_up || axis >= NUM_AXES) {
        return false;
    }
    uint8_t slave = board_get_tmc2209_address(axis);
    uint32_t sg = 0;
    uint32_t ts = 0;
    tmc_lock();
    bool ok_sg = tmc_read_reg_unlocked(slave, TMC_REG_SG_RESULT, &sg);
    bool ok_ts = tmc_read_reg_unlocked(slave, TMC_REG_TSTEP, &ts);
    tmc_unlock();
    if (sg_result != NULL) {
        *sg_result = (uint16_t)(sg & 0x3FF);
    }
    if (tstep != NULL) {
        *tstep = ts;
    }
    return ok_sg || ok_ts;
}

void tmc_driver_set_standby(bool standby)
{
    if (!tmc_uart_up) {
        return;
    }
    tmc_lock();
    for (uint8_t axis = 0; axis < NUM_AXES; axis++) {
        tmc_write_reg_unlocked(board_get_tmc2209_address(axis),
                               TMC_REG_IHOLD_IRUN,
                               axis_ihold_irun(axis, standby));
    }
    tmc_unlock();
    ESP_LOGI(TAG, "Motor current profile: %s (zoom IHOLD=%d)",
             standby ? "standby" : "run",
             standby ? TMC_IHOLD_STANDBY_ZOOM : TMC_IHOLD_ZOOM);
}
