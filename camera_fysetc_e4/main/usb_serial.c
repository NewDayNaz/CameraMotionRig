/**
 * @file usb_serial.c
 * @brief USB serial command parser implementation
 */

#include "usb_serial.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

static const char* TAG = "usb_serial";

#define BUF_SIZE 1024
static char rx_buf[BUF_SIZE];
static size_t rx_buf_len = 0;

// UART configuration - USB Serial uses UART0 typically
#define UART_NUM UART_NUM_0
#define UART_BAUD_RATE 115200

void usb_serial_init(void) {
    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // Install UART driver
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    
    // USB Serial uses UART0 - pins are typically GPIO1 (TX) and GPIO3 (RX)
    // But these may vary - check your board configuration
    // For USB Serial, we typically don't set GPIO pins manually
    // ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    ESP_LOGI(TAG, "USB serial initialized at %d baud", UART_BAUD_RATE);
}

bool usb_serial_parse_command(parsed_cmd_t* cmd) {
    if (cmd == NULL) {
        return false;
    }
    
    memset(cmd, 0, sizeof(parsed_cmd_t));
    
    // Read available data
    int len = uart_read_bytes(UART_NUM, rx_buf + rx_buf_len, BUF_SIZE - rx_buf_len - 1, 0);
    if (len > 0) {
        rx_buf_len += len;
        rx_buf[rx_buf_len] = '\0';  // Null terminate
    }
    
    if (rx_buf_len == 0) {
        return false;
    }
    
    // Look for newline or carriage return (end of command)
    char* line_end = strchr(rx_buf, '\n');
    if (line_end == NULL) {
        line_end = strchr(rx_buf, '\r');
    }
    
    if (line_end == NULL) {
        // No complete line yet
        if (rx_buf_len >= BUF_SIZE - 1) {
            // Buffer full, reset
            rx_buf_len = 0;
        }
        return false;
    }
    
    // Null terminate the line
    *line_end = '\0';
    
    // Parse the command
    char* line = rx_buf;
    char* token;
    char* saveptr;
    
    token = strtok_r(line, " \t", &saveptr);
    if (token == NULL) {
        // Empty line
        rx_buf_len = 0;
        return false;
    }
    
    // Parse command type
    if (strcmp(token, "VEL") == 0) {
        cmd->type = CMD_VEL;
        // Parse velocities
        for (int i = 0; i < 3; i++) {
            token = strtok_r(NULL, " \t", &saveptr);
            if (token != NULL) {
                cmd->velocities[i] = strtof(token, NULL);
            }
        }
    } else if (strcmp(token, "j") == 0 || strncmp(line, "j,", 2) == 0) {
        // Joystick command: j,yaw,pitch,zoom (values -32768 to 32768)
        cmd->type = CMD_JOYSTICK;
        // Parse comma-separated values (line format: j,yaw,pitch,zoom)
        char* comma = strchr(line, ',');
        if (comma != NULL) {
            // Parse yaw (pan)
            comma++;
            char* next_comma = strchr(comma, ',');
            if (next_comma != NULL) {
                *next_comma = '\0';
                int32_t yaw_int = (int32_t)atoi(comma);
                cmd->velocities[0] = (float)yaw_int;  // Store raw value, will scale later
                comma = next_comma + 1;
                
                // Parse pitch (tilt)
                next_comma = strchr(comma, ',');
                if (next_comma != NULL) {
                    *next_comma = '\0';
                    int32_t pitch_int = (int32_t)atoi(comma);
                    cmd->velocities[1] = (float)pitch_int;
                    comma = next_comma + 1;
                    
                    // Parse zoom (last value, may have trailing space)
                    int32_t zoom_int = (int32_t)atoi(comma);
                    cmd->velocities[2] = (float)zoom_int;
                } else {
                    // Only two values provided, set zoom to 0
                    int32_t pitch_int = (int32_t)atoi(comma);
                    cmd->velocities[1] = (float)pitch_int;
                    cmd->velocities[2] = 0.0f;
                }
            } else {
                // Only one value provided, set others to 0
                int32_t yaw_int = (int32_t)atoi(comma);
                cmd->velocities[0] = (float)yaw_int;
                cmd->velocities[1] = 0.0f;
                cmd->velocities[2] = 0.0f;
            }
        } else {
            // No comma found, invalid format
            cmd->type = CMD_UNKNOWN;
        }
    } else if (strcmp(token, "GOTO") == 0) {
        cmd->type = CMD_GOTO;
        token = strtok_r(NULL, " \t", &saveptr);
        if (token != NULL) {
            cmd->preset_index = (uint8_t)atoi(token);
        }
    } else if (strcmp(token, "SAVE") == 0) {
        cmd->type = CMD_SAVE;
        token = strtok_r(NULL, " \t", &saveptr);
        if (token != NULL) {
            cmd->preset_index = (uint8_t)atoi(token);
        }
    } else if (strcmp(token, "HOME") == 0) {
        cmd->type = CMD_HOME;
    } else if (strcmp(token, "POS") == 0) {
        cmd->type = CMD_POS;
    } else if (strcmp(token, "STATUS") == 0) {
        cmd->type = CMD_STATUS;
    } else if (strcmp(token, "STOP") == 0) {
        cmd->type = CMD_STOP;
    } else if (strcmp(token, "LIMITS") == 0) {
        cmd->type = CMD_LIMITS;
        token = strtok_r(NULL, " \t", &saveptr);
        if (token != NULL) {
            if (strcmp(token, "PAN") == 0) {
                cmd->limits_axis = 0;
            } else if (strcmp(token, "TILT") == 0) {
                cmd->limits_axis = 1;
            } else if (strcmp(token, "ZOOM") == 0) {
                cmd->limits_axis = 2;
            }
            token = strtok_r(NULL, " \t", &saveptr);
            if (token != NULL) {
                cmd->limits_min = strtof(token, NULL);
            }
            token = strtok_r(NULL, " \t", &saveptr);
            if (token != NULL) {
                cmd->limits_max = strtof(token, NULL);
            }
        }
    } else if (strcmp(token, "s") == 0) {
        // Legacy: Save preset 0
        cmd->type = CMD_SAVE;
        cmd->preset_index = 0;
    } else if (strncmp(token, "s", 1) == 0 && strlen(token) > 1) {
        // Legacy: Save preset n (s2, s3, s4, etc.)
        cmd->type = CMD_SAVE;
        cmd->preset_index = (uint8_t)atoi(token + 1);
    } else if (strcmp(token, "t") == 0) {
        // Legacy: Goto preset 0
        cmd->type = CMD_GOTO;
        cmd->preset_index = 0;
    } else if (strncmp(token, "t", 1) == 0 && strlen(token) > 1) {
        // Legacy: Goto preset n (t2, t3, t4, etc.)
        cmd->type = CMD_GOTO;
        cmd->preset_index = (uint8_t)atoi(token + 1);
    } else if (strcmp(token, "ea") == 0) {
        // Legacy: Home all axes
        cmd->type = CMD_HOME;
    } else if (strcmp(token, "eb") == 0) {
        // Legacy: Stop
        cmd->type = CMD_STOP;
    } else {
        cmd->type = CMD_UNKNOWN;
    }
    
    // Remove processed line from buffer
    size_t remaining = rx_buf_len - (line_end - rx_buf) - 1;
    if (remaining > 0) {
        memmove(rx_buf, line_end + 1, remaining);
        rx_buf_len = remaining;
    } else {
        rx_buf_len = 0;
    }
    
    return true;
}

void usb_serial_send(const char* format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    if (len > 0 && len < sizeof(buffer)) {
        uart_write_bytes(UART_NUM, buffer, len);
    }
}

void usb_serial_send_position(float pan, float tilt, float zoom) {
    usb_serial_send("POS:%.2f,%.2f,%.2f\n", pan, tilt, zoom);
}

void usb_serial_send_status(const char* status) {
    usb_serial_send("STATUS:%s\n", status);
}

