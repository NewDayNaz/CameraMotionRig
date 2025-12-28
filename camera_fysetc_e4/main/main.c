/**
 * @file main.c
 * @brief Main application for FYSETC E4 PTZ Camera Rig
 * 
 * Coordinates all subsystems:
 * - USB Serial command parsing
 * - Motion controller
 * - Stepper executor (ISR-based)
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"

#include "board.h"
#include "motion_controller.h"
#include "usb_serial.h"
#include "preset_storage.h"
#include "stepper_executor.h"
#include "wifi_manager.h"
#include "http_server.h"
#include "wifi_config.h"

static const char* TAG = "main";

#define UPDATE_TASK_PERIOD_MS 10  // 10ms update period = 100Hz
#define UPDATE_TASK_STACK_SIZE 4096
#define UPDATE_TASK_PRIORITY 5

#define SERIAL_TASK_STACK_SIZE 4096
#define SERIAL_TASK_PRIORITY 3

// Update task - runs motion controller and planner updates
static void update_task(void* pvParameters) {
    const TickType_t xDelay = pdMS_TO_TICKS(UPDATE_TASK_PERIOD_MS);
    float dt = UPDATE_TASK_PERIOD_MS / 1000.0f;
    
    ESP_LOGI(TAG, "Update task started");
    
    while (1) {
        motion_controller_update(dt);
        vTaskDelay(xDelay);
    }
}

// Serial command task - handles incoming commands
static void serial_task(void* pvParameters) {
    parsed_cmd_t cmd;
    float positions[3];
    
    ESP_LOGI(TAG, "Serial task started");
    
    while (1) {
        if (usb_serial_parse_command(&cmd)) {
            switch (cmd.type) {
                case CMD_VEL:
                    // Set velocities for manual mode
                    motion_controller_set_velocities(cmd.velocities);
                    ESP_LOGI(TAG, "VEL: %.2f, %.2f, %.2f", 
                            cmd.velocities[0], cmd.velocities[1], cmd.velocities[2]);
                    break;
                    
                case CMD_JOYSTICK:
                    // Convert joystick values (-32768 to 32768) to velocities
                    // Scale to MAX_VELOCITY range
                    float scaled_velocities[3];
                    const float JOYSTICK_MAX = 32768.0f;
                    const float MAX_VEL_PAN = 2000.0f;
                    const float MAX_VEL_TILT = 2000.0f;
                    const float MAX_VEL_ZOOM = 200.0f;   // Reduced significantly for slower zoom control
                    
                    // Scale yaw (pan) from -32768..32768 to -MAX_VEL_PAN..MAX_VEL_PAN
                    scaled_velocities[0] = (cmd.velocities[0] / JOYSTICK_MAX) * MAX_VEL_PAN;
                    // Scale pitch (tilt) from -32768..32768 to -MAX_VEL_TILT..MAX_VEL_TILT
                    scaled_velocities[1] = (cmd.velocities[1] / JOYSTICK_MAX) * MAX_VEL_TILT;
                    // Scale zoom from -32768..32768 to -MAX_VEL_ZOOM..MAX_VEL_ZOOM
                    scaled_velocities[2] = (cmd.velocities[2] / JOYSTICK_MAX) * MAX_VEL_ZOOM;
                    
                    motion_controller_set_velocities(scaled_velocities);
                    break;
                    
                case CMD_GOTO:
                    // Move to preset
                    if (motion_controller_goto_preset(cmd.preset_index)) {
                        usb_serial_send_status("OK");
                    } else {
                        usb_serial_send_status("ERROR: Preset not found");
                    }
                    break;
                    
                case CMD_SAVE:
                    // Save current position as preset
                    if (motion_controller_save_preset(cmd.preset_index)) {
                        usb_serial_send_status("OK");
                    } else {
                        usb_serial_send_status("ERROR: Save failed");
                    }
                    break;
                    
                case CMD_HOME:
                    // Start homing sequence
                    if (motion_controller_home(255)) {  // Home all axes
                        usb_serial_send_status("HOMING");
                    } else {
                        usb_serial_send_status("ERROR: Homing failed");
                    }
                    break;
                    
                case CMD_POS:
                    // Query current positions
                    motion_controller_get_positions(positions);
                    usb_serial_send_position(positions[0], positions[1], positions[2]);
                    break;
                    
                case CMD_STATUS:
                    // Query system status
                    motion_controller_get_positions(positions);
                    usb_serial_send("STATUS:PAN:%.2f TILT:%.2f ZOOM:%.2f\n",
                                   positions[0], positions[1], positions[2]);
                    break;
                    
                case CMD_STOP:
                    // Stop all motion
                    motion_controller_stop();
                    usb_serial_send_status("STOPPED");
                    break;
                    
                case CMD_PRECISION:
                    // Set precision mode
                    motion_controller_set_precision_mode(cmd.precision_enable);
                    usb_serial_send_status(cmd.precision_enable ? "PRECISION_ON" : "PRECISION_OFF");
                    break;
                    
                case CMD_LIMITS:
                    // Set soft limits
                    motion_controller_set_limits(cmd.limits_axis, cmd.limits_min, cmd.limits_max);
                    usb_serial_send_status("OK");
                    break;
                    
                case CMD_BOOTLOADER:
                    // Enter download mode on next reboot
                    // NOTE: This may not work if GPIO0 is connected to USB-to-serial chip
                    // If this doesn't work, use manual method: connect GPIO0 to GND, then reset
                    usb_serial_send_status("BOOTLOADER: Attempting to enter download mode...");
                    vTaskDelay(pdMS_TO_TICKS(200));  // Give time for message to be sent
                    
                    // Try to set GPIO0 low via software (may not work if connected to USB chip)
                    // GPIO0 must be LOW during reset to enter bootloader mode
                    gpio_config_t io_conf = {
                        .pin_bit_mask = (1ULL << GPIO_NUM_0),
                        .mode = GPIO_MODE_OUTPUT,
                        .pull_up_en = GPIO_PULLUP_DISABLE,
                        .pull_down_en = GPIO_PULLDOWN_ENABLE,  // Enable pulldown to help keep it low
                        .intr_type = GPIO_INTR_DISABLE
                    };
                    gpio_config(&io_conf);
                    gpio_set_level(GPIO_NUM_0, 0);  // Set GPIO0 LOW
                    vTaskDelay(pdMS_TO_TICKS(100));  // Delay to ensure GPIO0 is set low
                    
                    // Reboot - ESP32 will enter bootloader mode if GPIO0 stays LOW
                    // If this doesn't work, manually connect GPIO0 to GND and reset
                    esp_restart();
                    break;
                    
                case CMD_UNKNOWN:
                    usb_serial_send_status("ERROR: Unknown command");
                    break;
                    
                default:
                    break;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));  // Small delay to prevent tight loop
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "FYSETC E4 PTZ Camera Rig Firmware Starting");
    
    // Initialize NVS (required for preset storage)
    preset_storage_init();
    
    // Initialize board GPIO
    board_init();
    
    // Initialize USB serial
    usb_serial_init();
    
    // Initialize motion controller (includes executor and planner)
    motion_controller_init();
    
    // Enable stepper drivers
    board_set_enable(true);
    
    // Start stepper executor
    stepper_executor_start();
    
    ESP_LOGI(TAG, "System initialized, starting tasks");
    
    // Initialize WiFi
    ESP_LOGI(TAG, "Initializing WiFi...");
    if (wifi_manager_init(WIFI_SSID, WIFI_PASSWORD)) {
        // Wait for WiFi connection (with timeout)
        int timeout = 0;
        while (!wifi_manager_is_connected() && timeout < 100) {
            vTaskDelay(pdMS_TO_TICKS(100));
            timeout++;
        }
        
        if (wifi_manager_is_connected()) {
            ESP_LOGI(TAG, "WiFi connected! IP: %s", wifi_manager_get_ip());
            
            // Start HTTP server
            if (http_server_start()) {
                ESP_LOGI(TAG, "HTTP server started at http://%s/", wifi_manager_get_ip());
            } else {
                ESP_LOGE(TAG, "Failed to start HTTP server");
            }
        } else {
            ESP_LOGW(TAG, "WiFi connection timeout - continuing anyway");
        }
    } else {
        ESP_LOGE(TAG, "Failed to initialize WiFi");
    }
    
    // Create update task
    xTaskCreate(update_task, "update_task", UPDATE_TASK_STACK_SIZE, NULL, 
                UPDATE_TASK_PRIORITY, NULL);
    
    // Create serial command task
    xTaskCreate(serial_task, "serial_task", SERIAL_TASK_STACK_SIZE, NULL,
                SERIAL_TASK_PRIORITY, NULL);
    
    ESP_LOGI(TAG, "Tasks started, system ready");
    
    // Main task - reduced logging to avoid serial interference
    // Position can be queried via POS command or web interface
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));  // Reduced logging frequency: every 10 seconds
        // Periodic status logging (much less frequent)
        // Commented out entirely - use POS command or web interface for position info
        // float pos[3];
        // motion_controller_get_positions(pos);
        // ESP_LOGI(TAG, "Pos: PAN=%.1f TILT=%.1f ZOOM=%.1f", pos[0], pos[1], pos[2]);
    }
}

