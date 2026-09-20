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
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/gpio_struct.h"
#include "soc/soc.h"
#include "soc/uart_reg.h"

#ifndef GPIO_FUNC0_OUT_SEL_CFG_REG
#define GPIO_FUNC0_OUT_SEL_CFG_REG (DR_REG_GPIO_BASE + 0x0530)
#endif
#ifndef GPIO_FUNC0_IN_SEL_CFG_REG
#define GPIO_FUNC0_IN_SEL_CFG_REG (DR_REG_GPIO_BASE + 0x0130)
#endif

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
#define TMC_REG_PWMCONF    0x70
#define TMC_REG_PWM_SCALE  0x71

/* GCONF: pdn_disable | mstep_reg_select | multistep_filt. Bit 2 en_spreadCycle
 * is set for pan/tilt. Zoom clears it: StallGuard4 only works in stealthChop. */
#define TMC_GCONF_SPREADCYCLE  0x000001C4u
#define TMC_GCONF_STEALTHCHOP  0x000001C0u
#define TMC_CHOPCONF_MRES8 0x15000053u
#define TMC_PWMCONF_STEALTH 0xC10D0024u  /* autoscale stealthChop (TMC2209 reset-ish) */
#define TMC_TPWMTHRS       0u
#define TMC_TCOOLTHRS_ZOOM 0x000FFFFFu
#define TMC_SGTHRS_ZOOM    60u
#define TMC_TPOWERDOWN     20u  /* ~0.4 s until IHOLD after last step */

#define TMC_2209_VERSION   0x21u
#define TMC_REPLY_MASTER   0xFFu  /* driver → MCU address in read replies */
#define TMC_READ_RETRIES   3
#define TMC_READ_TIMEOUT_MS 20

static const uint8_t k_irun_default[NUM_AXES] = {
    TMC_IRUN_PAN, TMC_IRUN_TILT, TMC_IRUN_ZOOM
};
static const char *k_irun_nvs_key[NUM_AXES] = {
    "pirun", "tirun", "zirun"
};
static uint8_t s_irun[NUM_AXES] = {
    TMC_IRUN_PAN, TMC_IRUN_TILT, TMC_IRUN_ZOOM
};

static uint8_t clamp_irun_cs(uint8_t cs)
{
    if (cs < 3u) {
        return 3u;
    }
    if (cs > 16u) {
        return 16u;
    }
    return cs;
}

static void irun_load(void)
{
    nvs_handle_t h;
    if (nvs_open("tmc", NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    for (uint8_t axis = 0; axis < NUM_AXES; axis++) {
        uint8_t v = k_irun_default[axis];
        if (nvs_get_u8(h, k_irun_nvs_key[axis], &v) == ESP_OK && v >= 3u && v <= 31u) {
            s_irun[axis] = clamp_irun_cs(v);
        }
    }
    nvs_close(h);
}

static void irun_save_axis(uint8_t axis)
{
    if (axis >= NUM_AXES) {
        return;
    }
    nvs_handle_t h;
    if (nvs_open("tmc", NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_u8(h, k_irun_nvs_key[axis], s_irun[axis]);
    nvs_commit(h);
    nvs_close(h);
}

static uint32_t pack_ihold_irun(uint8_t ihold, uint8_t irun)
{
    return ((uint32_t)(ihold & 0x1F)) |
           ((uint32_t)(irun & 0x1F) << 8) |
           ((uint32_t)(TMC_IHOLDDELAY & 0x0F) << 16);
}

static uint32_t axis_ihold_irun(uint8_t axis, bool standby)
{
    uint8_t irun = (axis < NUM_AXES) ? s_irun[axis] : TMC_IRUN_PAN;
    uint8_t ihold;
    if (axis == AXIS_TILT) {
        ihold = standby ? TMC_IHOLD_STANDBY_TILT : TMC_IHOLD_TILT;
    } else if (axis == AXIS_ZOOM) {
        ihold = standby ? TMC_IHOLD_STANDBY_ZOOM : TMC_IHOLD_ZOOM;
    } else {
        ihold = standby ? TMC_IHOLD_STANDBY_PAN : TMC_IHOLD_PAN;
    }
    return pack_ihold_irun(ihold, irun);
}

static bool tmc_uart_up = false;
static bool tmc_ready = false;
static bool tmc_standby = false;
static SemaphoreHandle_t tmc_mutex;
static int s_last_rx_len;
static uint8_t s_last_rx[24];
static volatile uint16_t s_zoom_sg_cache;
static volatile uint32_t s_zoom_tstep_cache;
static volatile uint8_t s_zoom_pwm_cache;
static volatile uint8_t s_zoom_sg_cache_ok;

static int tmc_uart_collect(uint8_t *buf, int max_len, TickType_t first_wait);
static void tmc_bus_probe_unlocked(tmc_loopback_t *out);

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

static int tmc_hw_rxfifo_cnt(void)
{
    return (int)(REG_READ(UART_STATUS_REG(TMC_UART)) & 0xFFu);
}

static void tmc_uart_bind_pins(void)
{
    gpio_reset_pin(PIN_UART1_TX);
    gpio_reset_pin(PIN_UART1_RX);
    ESP_ERROR_CHECK(uart_set_pin(TMC_UART, PIN_UART1_TX, PIN_UART1_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    /* E4 TX goes through 1k onto PDN. Push-pull matches FluidNC/Marlin. */
    GPIO.pin[PIN_UART1_TX].pad_driver = 0;
    gpio_pullup_en(PIN_UART1_TX);
    gpio_pullup_en(PIN_UART1_RX);
    esp_rom_gpio_connect_out_signal(PIN_UART1_TX, U1TXD_OUT_IDX, false, false);
    esp_rom_gpio_connect_in_signal(PIN_UART1_RX, U1RXD_IN_IDX, false);
}

static int tmc_gpio_out_sel(gpio_num_t pin)
{
    return (int)(REG_READ(GPIO_FUNC0_OUT_SEL_CFG_REG + ((uint32_t)pin * 4u)) & 0x1FFu);
}

static int tmc_uart_rx_in_sel(void)
{
    return (int)(REG_READ(GPIO_FUNC0_IN_SEL_CFG_REG + ((uint32_t)U1RXD_IN_IDX * 4u)) & 0x3Fu);
}

static int tmc_uart_write_collect(const uint8_t *bytes, size_t nbytes, uint8_t *buf, int max_len,
                                  TickType_t wait, int *written, int *hw_rxfifo, bool *tx_done)
{
    tmc_flush_rx();
    int w = uart_write_bytes(TMC_UART, (const char *)bytes, nbytes);
    if (written != NULL) {
        *written = w;
    }
    esp_err_t done = uart_wait_tx_done(TMC_UART, pdMS_TO_TICKS(20));
    if (tx_done != NULL) {
        *tx_done = (done == ESP_OK);
    }
    esp_rom_delay_us(1000);
    if (hw_rxfifo != NULL) {
        *hw_rxfifo = tmc_hw_rxfifo_cnt();
    }
    return tmc_uart_collect(buf, max_len, wait);
}

static void tmc_pin_tie_test(int *when_low, int *when_high)
{
    gpio_reset_pin(PIN_UART1_TX);
    gpio_reset_pin(PIN_UART1_RX);

    gpio_config_t tx_conf = {
        .pin_bit_mask = (1ULL << PIN_UART1_TX),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config_t rx_conf = {
        .pin_bit_mask = (1ULL << PIN_UART1_RX),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&tx_conf);
    gpio_config(&rx_conf);

    gpio_set_level(PIN_UART1_TX, 0);
    esp_rom_delay_us(50);
    *when_low = gpio_get_level(PIN_UART1_RX);
    gpio_set_level(PIN_UART1_TX, 1);
    esp_rom_delay_us(50);
    *when_high = gpio_get_level(PIN_UART1_RX);
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

/* First byte waits up to first_wait. The rest is echo (4) + reply (8) already in
 * the FIFO or arriving within a couple of character times. Do not block on a
 * FreeRTOS tick here: pdMS_TO_TICKS(5) is a full 10 ms at 100 Hz, and that
 * stall in the 1 ms step task cuts the pulse rate in half (TSTEP 3400→8600). */
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
    int64_t deadline = esp_timer_get_time() + 2000;
    while (len < max_len && esp_timer_get_time() < deadline) {
        n = uart_read_bytes(TMC_UART, buf + len, (uint32_t)(max_len - len), 0);
        if (n > 0) {
            len += n;
            if (len >= 12) {
                break;
            }
            continue;
        }
        if (len >= 12) {
            break;
        }
        esp_rom_delay_us(50);
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
    uint32_t gconf_w = (axis == AXIS_ZOOM) ? TMC_GCONF_STEALTHCHOP : TMC_GCONF_SPREADCYCLE;

    if (!tmc_write_reg_unlocked(slave, TMC_REG_GCONF, gconf_w)) {
        ESP_LOGW(TAG, "%s: UART write GCONF failed (addr %u)", name, slave);
        return false;
    }
    tmc_write_reg_unlocked(slave, TMC_REG_IHOLD_IRUN, axis_ihold_irun(axis, false));
    tmc_write_reg_unlocked(slave, TMC_REG_TPOWERDOWN, TMC_TPOWERDOWN);
    tmc_write_reg_unlocked(slave, TMC_REG_TPWMTHRS, TMC_TPWMTHRS);
    tmc_write_reg_unlocked(slave, TMC_REG_CHOPCONF, TMC_CHOPCONF_MRES8);

    if (axis == AXIS_ZOOM) {
        tmc_write_reg_unlocked(slave, TMC_REG_PWMCONF, TMC_PWMCONF_STEALTH);
        tmc_write_reg_unlocked(slave, TMC_REG_TCOOLTHRS, TMC_TCOOLTHRS_ZOOM);
        tmc_write_reg_unlocked(slave, TMC_REG_SGTHRS, TMC_SGTHRS_ZOOM);
        ESP_LOGI(TAG, "ZOOM: stealthChop + stallGuard (TCOOLTHRS=max SGTHRS=%u)",
                 (unsigned)TMC_SGTHRS_ZOOM);
    } else {
        tmc_write_reg_unlocked(slave, TMC_REG_TCOOLTHRS, 0);
        tmc_write_reg_unlocked(slave, TMC_REG_SGTHRS, 0);
    }

    uint32_t gconf = 0;
    if (tmc_read_reg_unlocked(slave, TMC_REG_GCONF, &gconf)) {
        ESP_LOGI(TAG, "%s: TMC2209 addr %u GCONF=0x%08lX (%s)",
                 name, slave, (unsigned long)gconf,
                 (axis == AXIS_ZOOM) ? "8 ustep, stealthChop" : "8 ustep, spreadCycle");
        return true;
    }

    ESP_LOGW(TAG, "%s: TMC2209 addr %u wrote config but read-back failed", name, slave);
    return false;
}

static void tmc_zoom_sg_poll_task(void *arg)
{
    (void)arg;
    while (1) {
        uint16_t sg = 0;
        uint32_t ts = 0;
        uint8_t pwm = 0;
        if (tmc_uart_up && tmc_driver_read_sg_tstep_pwm(AXIS_ZOOM, &sg, &ts, &pwm)) {
            s_zoom_sg_cache = sg;
            s_zoom_tstep_cache = ts;
            s_zoom_pwm_cache = pwm;
            s_zoom_sg_cache_ok = 1;
        }
        vTaskDelay(pdMS_TO_TICKS(HOME_ZOOM_SG_POLL_MS));
    }
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
    tmc_uart_bind_pins();
    /* Short TMC replies never fill the default 120-byte RX FIFO threshold. */
    (void)uart_set_rx_full_threshold(TMC_UART, 1);
    (void)uart_set_rx_timeout(TMC_UART, 3);
    tmc_uart_up = true;

    irun_load();
    vTaskDelay(pdMS_TO_TICKS(20));

    bool all_ok = true;
    tmc_lock();
    {
        tmc_loopback_t probe;
        tmc_bus_probe_unlocked(&probe);
        ESP_LOGW(TAG,
                 "UART probe loopback=%d echo=%d hw_rxfifo=%d tx_done=%d tied=%d "
                 "(21@tx0=%d 21@tx1=%d) idle21=%d idle22=%d out_sel=%d in_sel=%d",
                 probe.loopback_rx, probe.rx_len, probe.hw_rxfifo, (int)probe.tx_done,
                 (int)probe.pins_tied, probe.gpio21_tx_low, probe.gpio21_tx_high,
                 probe.gpio21_idle, probe.gpio22_idle, probe.out_sel_tx, probe.in_sel_rx);
    }
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

    xTaskCreate(tmc_zoom_sg_poll_task, "tmc_sg", 2048, NULL, 3, NULL);
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
    tmc_read_reg_unlocked(slave, TMC_REG_SG_RESULT, &out->sg_result);
    out->sg_result &= 0x3FF;
    tmc_read_reg_unlocked(slave, TMC_REG_CHOPCONF, &out->chopconf);
    {
        uint32_t pwm_scale = 0;
        tmc_read_reg_unlocked(slave, TMC_REG_PWM_SCALE, &pwm_scale);
        out->pwm_scale_sum = (uint8_t)(pwm_scale & 0xFF);
    }
    /* IHOLD_IRUN, TCOOLTHRS, SGTHRS are write-only on TMC2209. Reads return 0. */
    out->ihold_irun = axis_ihold_irun(axis, tmc_standby);
    out->ihold = (uint8_t)(out->ihold_irun & 0x1F);
    out->irun = (uint8_t)((out->ihold_irun >> 8) & 0x1F);
    out->tcoolthrs = (axis == AXIS_ZOOM) ? TMC_TCOOLTHRS_ZOOM : 0;
    out->sgthrs = (axis == AXIS_ZOOM) ? TMC_SGTHRS_ZOOM : 0;

    if (!out->uart_ok && out->ic_version == TMC_2209_VERSION) {
        out->uart_ok = true;
    }

    tmc_unlock();
    return out->uart_ok;
}

static void tmc_bus_probe_unlocked(tmc_loopback_t *out)
{
    const uint8_t probe[4] = { 0xA5, 0x5A, 0xC3, 0x3C };
    uint8_t buf[16];
    int written = 0;

    memset(out, 0, sizeof(*out));
    out->gpio21_idle = gpio_get_level(PIN_UART1_RX);
    out->gpio22_idle = gpio_get_level(PIN_UART1_TX);
    out->out_sel_tx = tmc_gpio_out_sel(PIN_UART1_TX);
    out->in_sel_rx = tmc_uart_rx_in_sel();

    out->rx_len = tmc_uart_write_collect(probe, sizeof(probe), buf, (int)sizeof(buf),
                                         pdMS_TO_TICKS(20), &written, &out->hw_rxfifo, &out->tx_done);
    out->tx_len = written;
    tmc_bytes_to_hex(out->rx_hex, sizeof(out->rx_hex), buf, out->rx_len);

    tmc_flush_rx();
    uart_set_loop_back(TMC_UART, true);
    out->loopback_rx = tmc_uart_write_collect(probe, sizeof(probe), buf, (int)sizeof(buf),
                                              pdMS_TO_TICKS(20), NULL, NULL, NULL);
    uart_set_loop_back(TMC_UART, false);
    tmc_flush_rx();

    tmc_pin_tie_test(&out->gpio21_tx_low, &out->gpio21_tx_high);
    out->pins_tied = (out->gpio21_tx_low == 0 && out->gpio21_tx_high == 1);
    tmc_uart_bind_pins();
    (void)uart_set_rx_full_threshold(TMC_UART, 1);
    (void)uart_set_rx_timeout(TMC_UART, 3);

    if (out->rx_len == 0) {
        int retry_fifo = 0;
        bool retry_done = false;
        int retry_rx = tmc_uart_write_collect(probe, sizeof(probe), buf, (int)sizeof(buf),
                                              pdMS_TO_TICKS(20), &written, &retry_fifo, &retry_done);
        if (retry_rx > 0) {
            out->rx_len = retry_rx;
            out->tx_len = written;
            out->hw_rxfifo = retry_fifo;
            out->tx_done = retry_done;
            tmc_bytes_to_hex(out->rx_hex, sizeof(out->rx_hex), buf, retry_rx);
        }
    }
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

    tmc_lock();
    tmc_bus_probe_unlocked(out);
    tmc_unlock();
    return out->rx_len > 0;
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

bool tmc_driver_read_sg_tstep_pwm(uint8_t axis, uint16_t *sg_result, uint32_t *tstep, uint8_t *pwm_sum)
{
    if (!tmc_uart_up || axis >= NUM_AXES) {
        return false;
    }
    uint8_t slave = board_get_tmc2209_address(axis);
    uint32_t sg = 0;
    uint32_t ts = 0;
    uint32_t pwm = 0;
    tmc_lock();
    bool ok_sg = tmc_read_reg_unlocked(slave, TMC_REG_SG_RESULT, &sg);
    bool ok_ts = tmc_read_reg_unlocked(slave, TMC_REG_TSTEP, &ts);
    bool ok_pwm = true;
    if (pwm_sum != NULL) {
        ok_pwm = tmc_read_reg_unlocked(slave, TMC_REG_PWM_SCALE, &pwm);
    }
    tmc_unlock();
    if (sg_result != NULL) {
        *sg_result = (uint16_t)(sg & 0x3FF);
    }
    if (tstep != NULL) {
        *tstep = ts;
    }
    if (pwm_sum != NULL) {
        *pwm_sum = (uint8_t)(pwm & 0xFF);
    }
    return ok_sg || ok_ts || ok_pwm;
}

bool tmc_driver_read_sg_tstep(uint8_t axis, uint16_t *sg_result, uint32_t *tstep)
{
    return tmc_driver_read_sg_tstep_pwm(axis, sg_result, tstep, NULL);
}

bool tmc_driver_get_cached_sg_tstep(uint16_t *sg_result, uint32_t *tstep)
{
    return tmc_driver_get_cached_sg_tstep_pwm(sg_result, tstep, NULL);
}

bool tmc_driver_get_cached_sg_tstep_pwm(uint16_t *sg_result, uint32_t *tstep, uint8_t *pwm_sum)
{
    if (!s_zoom_sg_cache_ok) {
        return false;
    }
    if (sg_result != NULL) {
        *sg_result = s_zoom_sg_cache;
    }
    if (tstep != NULL) {
        *tstep = s_zoom_tstep_cache;
    }
    if (pwm_sum != NULL) {
        *pwm_sum = s_zoom_pwm_cache;
    }
    return true;
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
    tmc_standby = standby;
    tmc_unlock();
    ESP_LOGI(TAG, "Motor current profile: %s (zoom IHOLD=%d)",
             standby ? "standby" : "run",
             standby ? TMC_IHOLD_STANDBY_ZOOM : TMC_IHOLD_ZOOM);
}

uint8_t tmc_driver_get_irun(uint8_t axis)
{
    if (axis >= NUM_AXES) {
        return TMC_IRUN_PAN;
    }
    return s_irun[axis];
}

bool tmc_driver_set_irun(uint8_t axis, uint8_t cs)
{
    if (axis >= NUM_AXES) {
        return false;
    }
    cs = clamp_irun_cs(cs);
    s_irun[axis] = cs;
    irun_save_axis(axis);
    if (!tmc_uart_up) {
        return false;
    }
    tmc_lock();
    bool ok = tmc_write_reg_unlocked(board_get_tmc2209_address(axis),
                                     TMC_REG_IHOLD_IRUN,
                                     axis_ihold_irun(axis, tmc_standby));
    tmc_unlock();
    ESP_LOGI(TAG, "%s IRUN CS=%u (~%.2f A RMS)",
             axis_names[axis], (unsigned)cs,
             (double)(cs + 1) * 0.325 / (32.0 * 0.11 * 1.41421356));
    return ok;
}
