/**
 * @file tmc2209.c
 * @brief TMC2209 stepper driver UART interface implementation
 * 
 * Implements UART communication with TMC2209 drivers for sensorless homing.
 * Based on TMC2209 UART protocol: 8 bytes per transaction with CRC.
 */

#include "tmc2209.h"
#include "board.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>
#include <stdint.h>

// Forward declaration
extern uint8_t board_get_tmc2209_address(uint8_t axis);

static const char* TAG = "tmc2209";

static uart_port_t tmc2209_uart = TMC2209_UART_NUM;
static bool uart_initialized = false;

// TMC2209 UART Protocol constants
#define TMC2209_SYNC_WRITE    0x80
#define TMC2209_SYNC_READ     0x05
#define TMC2209_ADDRESS_MASK  0x7F

// CRC calculation polynomial for TMC2209
static uint8_t tmc2209_crc(uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t byte = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (byte & 0x01)) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = (crc << 1);
            }
            byte >>= 1;
        }
    }
    return crc;
}

bool tmc2209_uart_init(uart_port_t uart_num) {
    if (uart_initialized) {
        return true;
    }

    uart_config_t uart_config = {
        .baud_rate = TMC2209_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,  // TMC2209 requires even parity
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_driver_install(uart_num, TMC2209_UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return false;
    }

    ret = uart_param_config(uart_num, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART: %s", esp_err_to_name(ret));
        uart_driver_delete(uart_num);
        return false;
    }

    ret = uart_set_pin(uart_num, PIN_UART1_TX, PIN_UART1_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        uart_driver_delete(uart_num);
        return false;
    }

    tmc2209_uart = uart_num;
    uart_initialized = true;
    ESP_LOGI(TAG, "TMC2209 UART initialized on UART%d", uart_num);
    return true;
}

bool tmc2209_write_register(uint8_t axis, uint8_t address, uint32_t data) {
    if (!uart_initialized) {
        ESP_LOGE(TAG, "UART not initialized");
        return false;
    }

    // Get TMC2209 driver address for this axis
    extern uint8_t board_get_tmc2209_address(uint8_t axis);
    uint8_t driver_addr = board_get_tmc2209_address(axis);
    
    // TMC2209 UART write packet: 8 bytes
    // Byte 0: Sync + Register Address (0x80 | (register_addr & 0x7F))
    // Note: Driver address is set via hardware (PDN_UART pin), not in packet
    // Byte 1: Register address
    // Bytes 2-5: 32-bit data (MSB first)
    // Bytes 6-7: CRC (calculated over bytes 0-5)
    uint8_t packet[8];
    
    packet[0] = TMC2209_SYNC_WRITE | (address & TMC2209_ADDRESS_MASK);
    packet[1] = address;
    packet[2] = (data >> 24) & 0xFF;  // MSB
    packet[3] = (data >> 16) & 0xFF;
    packet[4] = (data >> 8) & 0xFF;
    packet[5] = data & 0xFF;          // LSB
    
    // Calculate CRC
    uint8_t crc = tmc2209_crc(packet, 6);
    packet[6] = crc;
    packet[7] = 0;  // Reserved byte

    // Flush RX buffer before sending
    uart_flush_input(tmc2209_uart);

    // Log write packet for debugging (first few writes only)
    static int write_count = 0;
    if (write_count++ < 10) {
        ESP_LOGI(TAG, "TMC2209 write (axis=%d, driver_addr=%d, reg=0x%02X): %02X %02X %02X %02X %02X %02X %02X %02X",
                axis, driver_addr, address, packet[0], packet[1], packet[2], packet[3],
                packet[4], packet[5], packet[6], packet[7]);
    }

    // Send packet
    int bytes_written = uart_write_bytes(tmc2209_uart, packet, 8);
    if (bytes_written != 8) {
        ESP_LOGE(TAG, "Failed to write all bytes: %d/8", bytes_written);
        return false;
    }

    // Wait for transmission to complete
    uart_wait_tx_done(tmc2209_uart, pdMS_TO_TICKS(10));
    
    // Small delay after write to allow TMC2209 to process
    vTaskDelay(pdMS_TO_TICKS(2));

    return true;
}

bool tmc2209_read_register(uint8_t axis, uint8_t address, uint32_t* data) {
    if (!uart_initialized || data == NULL) {
        ESP_LOGE(TAG, "Invalid parameters");
        return false;
    }

    // Get TMC2209 driver address for this axis
    // Note: TMC2209 driver addresses are set via hardware (PDN_UART pin configuration)
    // The address is not encoded in the UART packet - it's used for reference/logging only
    uint8_t driver_addr = board_get_tmc2209_address(axis);

    // First, send a write command to request the register
    // TMC2209 read: send write command with register address, then read response
    uint8_t write_packet[8];
    write_packet[0] = TMC2209_SYNC_READ | (address & TMC2209_ADDRESS_MASK);
    write_packet[1] = address;
    write_packet[2] = 0;
    write_packet[3] = 0;
    write_packet[4] = 0;
    write_packet[5] = 0;
    
    uint8_t crc = tmc2209_crc(write_packet, 6);
    write_packet[6] = crc;
    write_packet[7] = 0;

    // Flush RX buffer
    uart_flush_input(tmc2209_uart);

    // Send read request
    int bytes_written = uart_write_bytes(tmc2209_uart, write_packet, 8);
    if (bytes_written != 8) {
        ESP_LOGE(TAG, "Failed to send read request");
        return false;
    }

    uart_wait_tx_done(tmc2209_uart, pdMS_TO_TICKS(10));

    // Small delay for TMC2209 to process
    vTaskDelay(pdMS_TO_TICKS(1));

    // Read response (8 bytes)
    uint8_t response[8];
    int bytes_read = uart_read_bytes(tmc2209_uart, response, 8, pdMS_TO_TICKS(50));
    if (bytes_read != 8) {
        ESP_LOGW(TAG, "Failed to read response: %d/8 bytes", bytes_read);
        // Log what we did get
        if (bytes_read > 0) {
            ESP_LOGD(TAG, "Partial response: %02X %02X %02X %02X %02X %02X %02X %02X",
                    bytes_read > 0 ? response[0] : 0, bytes_read > 1 ? response[1] : 0,
                    bytes_read > 2 ? response[2] : 0, bytes_read > 3 ? response[3] : 0,
                    bytes_read > 4 ? response[4] : 0, bytes_read > 5 ? response[5] : 0,
                    bytes_read > 6 ? response[6] : 0, bytes_read > 7 ? response[7] : 0);
        }
        return false;
    }

    // Log raw response for debugging (first few reads only)
    static int read_count = 0;
    if (read_count++ < 5) {
        ESP_LOGI(TAG, "TMC2209 read response (axis=%d, driver_addr=%d, reg=0x%02X): %02X %02X %02X %02X %02X %02X %02X %02X",
                axis, driver_addr, address, response[0], response[1], response[2], response[3],
                response[4], response[5], response[6], response[7]);
    }

    // Verify CRC
    uint8_t calc_crc = tmc2209_crc(response, 6);
    if (calc_crc != response[6]) {
        ESP_LOGW(TAG, "CRC mismatch: calc=0x%02X, recv=0x%02X (addr=0x%02X)", 
                calc_crc, response[6], address);
        // Still try to extract data for debugging
    }

    // Extract 32-bit data (MSB first)
    *data = ((uint32_t)response[2] << 24) |
            ((uint32_t)response[3] << 16) |
            ((uint32_t)response[4] << 8) |
            (uint32_t)response[5];

    return true;
}

bool tmc2209_init(uint8_t axis) {
    if (!uart_initialized) {
        if (!tmc2209_uart_init(TMC2209_UART_NUM)) {
            return false;
        }
    }

    // Small delay for driver to be ready
    vTaskDelay(pdMS_TO_TICKS(200));  // Longer delay to ensure driver is ready

    // Read GSTAT to verify communication (should be readable even if not configured)
    uint32_t gstat = 0;
    ESP_LOGI(TAG, "Attempting to read GSTAT from TMC2209 axis %d...", axis);
    if (!tmc2209_read_register(axis, TMC2209_GSTAT, &gstat)) {
        ESP_LOGW(TAG, "Failed to read GSTAT for axis %d - driver may not be in UART mode or not responding", axis);
        ESP_LOGW(TAG, "Check: 1) UART wiring (TX/RX), 2) Power to driver, 3) Driver in UART mode");
        // Continue anyway, might work after configuration
    } else {
        ESP_LOGI(TAG, "TMC2209 axis %d GSTAT: 0x%08lX (communication OK)", axis, gstat);
    }
    
    // Try reading GCONF before writing to see current state
    uint32_t gconf_initial = 0;
    if (tmc2209_read_register(axis, TMC2209_GCONF, &gconf_initial)) {
        ESP_LOGI(TAG, "TMC2209 axis %d initial GCONF: 0x%08lX", axis, gconf_initial);
    } else {
        ESP_LOGW(TAG, "TMC2209 axis %d could not read initial GCONF", axis);
    }

    // Configure GCONF: Enable UART, enable stallGuard
    // Bit 2: Enable stallGuard (SG_ENABLE)
    // Bit 6: Enable UART mode (already set, but ensure it)
    // Bit 10: Enable stealthChop (smooth operation, lower noise)
    // NOTE: TMC2209 may require UART to be enabled in hardware (PDN_UART pin) before software can enable it
    uint32_t gconf = (1 << 2) | (1 << 6) | (1 << 10);  // SG_ENABLE, UART_EN, EN_PWM_MODE(stealthChop)
    
    ESP_LOGI(TAG, "TMC2209 axis %d writing GCONF: 0x%08lX (SG_ENABLE|UART_EN|PWM_MODE)", axis, gconf);
    bool write_success = tmc2209_write_register(axis, TMC2209_GCONF, gconf);
    if (!write_success) {
        ESP_LOGE(TAG, "TMC2209 axis %d GCONF write failed!", axis);
    }
    
    // Give time for write to complete and verify
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Read back immediately after write to verify
    uint32_t gconf_verify = 0;
    if (tmc2209_read_register(axis, TMC2209_GCONF, &gconf_verify)) {
        ESP_LOGI(TAG, "TMC2209 axis %d GCONF immediately after write: 0x%08lX", axis, gconf_verify);
        if (gconf_verify != gconf) {
            ESP_LOGW(TAG, "TMC2209 axis %d GCONF write did not take effect (expected 0x%08lX, got 0x%08lX)", 
                    axis, gconf, gconf_verify);
            ESP_LOGW(TAG, "This may indicate: 1) Driver not in UART mode (check PDN_UART pin), 2) Write protection, or 3) Hardware issue");
        }
    } else {
        ESP_LOGW(TAG, "TMC2209 axis %d could not verify GCONF write", axis);
    }

    // Set motor current (IHOLD_IRUN)
    // For zoom axis, use very low hold current to prevent overheating when idle
    // IHOLD=3 (~10% of max) - minimal current for holding position, prevents overheating when stationary
    // IRUN=12 (~39% of max) - conservative run current for motion
    uint32_t ihold_irun = (3 << 0) | (12 << 8) | (5 << 16);  // IHOLD=3, IRUN=12, IHOLDDELAY=5
    ESP_LOGI(TAG, "TMC2209 axis %d writing IHOLD_IRUN: 0x%08lX", axis, ihold_irun);
    bool write_ok = tmc2209_write_register(axis, TMC2209_IHOLD_IRUN, ihold_irun);
    if (!write_ok) {
        ESP_LOGE(TAG, "TMC2209 axis %d IHOLD_IRUN write failed!", axis);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_LOGI(TAG, "TMC2209 axis %d current set: IHOLD=3, IRUN=12 (low hold to prevent overheating)", axis);
    
    // Verify IHOLD_IRUN was written correctly
    uint32_t verify_ihold_irun = 0;
    if (tmc2209_read_register(axis, TMC2209_IHOLD_IRUN, &verify_ihold_irun)) {
        uint8_t irun_verify = (verify_ihold_irun >> 8) & 0x1F;
        uint8_t ihold_verify = (verify_ihold_irun >> 0) & 0x1F;
        ESP_LOGI(TAG, "TMC2209 axis %d IHOLD_IRUN verify: IRUN=%d, IHOLD=%d", 
                 axis, irun_verify, ihold_verify);
        if (irun_verify != 12 || ihold_verify != 3) {
            ESP_LOGW(TAG, "TMC2209 axis %d IHOLD_IRUN mismatch! Expected IRUN=12 IHOLD=3, got IRUN=%d IHOLD=%d",
                     axis, irun_verify, ihold_verify);
        }
    } else {
        ESP_LOGW(TAG, "TMC2209 axis %d could not read back IHOLD_IRUN for verification", axis);
    }

    // Set TCOOLTHRS (minimum velocity for stallGuard)
    ESP_LOGI(TAG, "TMC2209 axis %d writing TCOOLTHRS: %d", axis, TMC2209_DEFAULT_TCOOLTHRS);
    write_ok = tmc2209_write_register(axis, TMC2209_TCOOLTHRS, TMC2209_DEFAULT_TCOOLTHRS);
    if (!write_ok) {
        ESP_LOGE(TAG, "TMC2209 axis %d TCOOLTHRS write failed!", axis);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_LOGI(TAG, "TMC2209 axis %d TCOOLTHRS: %d", axis, TMC2209_DEFAULT_TCOOLTHRS);

    // Set stallGuard threshold
    ESP_LOGI(TAG, "TMC2209 axis %d writing SGTHRS: %d", axis, TMC2209_DEFAULT_SGTHRS);
    tmc2209_set_stallguard_threshold(axis, TMC2209_DEFAULT_SGTHRS);
    vTaskDelay(pdMS_TO_TICKS(20));

    // Verify configuration
    vTaskDelay(pdMS_TO_TICKS(50));  // Longer delay to ensure registers are written
    uint32_t verify_config = 0;
    uint32_t verify_sgthrs = 0;
    if (tmc2209_read_register(axis, TMC2209_GCONF, &verify_config)) {
        bool sg_enabled = (verify_config & (1 << 2)) != 0;
        ESP_LOGI(TAG, "TMC2209 axis %d GCONF verify: 0x%08lX (stallGuard %s)", 
                axis, verify_config, sg_enabled ? "enabled" : "DISABLED!");
        
        if (tmc2209_read_register(axis, TMC2209_SGTHRS, &verify_sgthrs)) {
            ESP_LOGI(TAG, "TMC2209 axis %d SGTHRS verify: %lu", axis, verify_sgthrs & 0xFF);
        }
        
        // Read initial SG_RESULT to check if it's working
        uint8_t initial_sg = tmc2209_get_stallguard_result(axis);
        ESP_LOGI(TAG, "TMC2209 axis %d initial SG_RESULT: %d", axis, initial_sg);
        
        if (sg_enabled) {
            ESP_LOGI(TAG, "TMC2209 axis %d configured successfully", axis);
            return true;
        } else {
            ESP_LOGW(TAG, "TMC2209 axis %d stallGuard not enabled - may need reconfiguration", axis);
        }
    }

    ESP_LOGW(TAG, "TMC2209 axis %d configuration verification failed", axis);
    return true;  // Return true anyway, might work
}

void tmc2209_set_stallguard_threshold(uint8_t axis, uint8_t threshold) {
    if (!uart_initialized) {
        return;
    }

    tmc2209_write_register(axis, TMC2209_SGTHRS, (uint32_t)threshold);
    ESP_LOGI(TAG, "TMC2209 axis %d SGTHRS set to %d", axis, threshold);
}

uint8_t tmc2209_get_stallguard_result(uint8_t axis) {
    if (!uart_initialized) {
        return 255;  // Return max value if not initialized
    }

    uint32_t sg_result = 0;
    if (tmc2209_read_register(axis, TMC2209_SG_RESULT, &sg_result)) {
        uint8_t result = (uint8_t)(sg_result & 0xFF);
        // Log occasional reads for debugging (only every 100th call to avoid spam)
        static int call_count = 0;
        if ((call_count++ % 100) == 0) {
            ESP_LOGD(TAG, "TMC2209 axis %d SG_RESULT read: 0x%02X (%d)", axis, result, result);
        }
        return result;
    }
    
    ESP_LOGW(TAG, "TMC2209 axis %d failed to read SG_RESULT", axis);
    return 255;  // Return max value on error
}

bool tmc2209_is_stalled(uint8_t axis, uint8_t threshold) {
    uint8_t sg_result = tmc2209_get_stallguard_result(axis);
    
    // Lower SG_RESULT means more load/stall
    // If SG_RESULT < threshold, consider it stalled
    // Note: SG_RESULT of 255 typically means no valid reading (motor not moving fast enough)
    bool stalled = (sg_result < threshold && sg_result != 255);
    
    return stalled;
}

void tmc2209_enable_stallguard(uint8_t axis, bool enable) {
    if (!uart_initialized) {
        return;
    }

    uint32_t gconf = 0;
    if (tmc2209_read_register(axis, TMC2209_GCONF, &gconf)) {
        if (enable) {
            gconf |= (1 << 2);  // Set SG_ENABLE bit
        } else {
            gconf &= ~(1 << 2);  // Clear SG_ENABLE bit
        }
        tmc2209_write_register(axis, TMC2209_GCONF, gconf);
        ESP_LOGI(TAG, "TMC2209 axis %d stallGuard %s", axis, enable ? "enabled" : "disabled");
    }
}

void tmc2209_set_current(uint8_t axis, uint8_t hold_current, uint8_t run_current) {
    if (!uart_initialized) {
        return;
    }

    // Clamp values to valid range (0-31)
    if (hold_current > 31) hold_current = 31;
    if (run_current > 31) run_current = 31;

    uint32_t ihold_irun = ((uint32_t)hold_current << 0) | 
                          ((uint32_t)run_current << 8) | 
                          (5 << 16);  // IHOLDDELAY=5
    tmc2209_write_register(axis, TMC2209_IHOLD_IRUN, ihold_irun);
    ESP_LOGI(TAG, "TMC2209 axis %d current: hold=%d, run=%d", axis, hold_current, run_current);
}

void tmc2209_set_coolthrs(uint8_t axis, uint32_t threshold) {
    if (!uart_initialized) {
        return;
    }

    tmc2209_write_register(axis, TMC2209_TCOOLTHRS, threshold);
    ESP_LOGI(TAG, "TMC2209 axis %d TCOOLTHRS set to %lu", axis, threshold);
}

