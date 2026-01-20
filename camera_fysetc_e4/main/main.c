/**
 * @file main.c
 * @brief Main application for FYSETC E4 PTZ Camera Rig
 * 
 * Simplified implementation based on camera_async but with:
 * - Web server with OTA firmware updates
 * - Board config
 * - Preset loading/saving commands and web UI
 * - Joystick/velocity control web UI
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "board.h"
#include "stepper_simple.h"
#include "preset_storage.h"
#include "usb_serial.h"
#include "wifi_manager.h"
#include "http_server.h"
#include "wifi_config.h"

static const char* TAG = "main";

#define UPDATE_TASK_PERIOD_MS 1  // 1ms update period for stepper control
#define UPDATE_TASK_STACK_SIZE 4096
#define UPDATE_TASK_PRIORITY 5

#define SERIAL_TASK_STACK_SIZE 4096
#define SERIAL_TASK_PRIORITY 3

// Update task - runs stepper control
static void update_task(void* pvParameters) {
    const TickType_t xDelay = pdMS_TO_TICKS(UPDATE_TASK_PERIOD_MS);
    
    ESP_LOGI(TAG, "Update task started");
    
    while (1) {
        stepper_simple_update();
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
                    stepper_simple_set_velocities(cmd.velocities[0], 
                                                   cmd.velocities[1], 
                                                   cmd.velocities[2]);
                    ESP_LOGI(TAG, "VEL: %.2f, %.2f, %.2f", 
                            cmd.velocities[0], cmd.velocities[1], cmd.velocities[2]);
                    break;
                    
                case CMD_JOYSTICK:
                    // Convert joystick values (-32768 to 32768) to velocities
                    // Note: stepper_simple_set_velocities will apply max velocity limits
                    const float JOYSTICK_MAX = 32768.0f;
                    const float MAX_VEL_PAN = 500.0f;   
                    const float MAX_VEL_TILT = 500.0f;  
                    const float MAX_VEL_ZOOM = 100.0f;   
                    
                    float pan_vel = (cmd.velocities[0] / JOYSTICK_MAX) * MAX_VEL_PAN;
                    float tilt_vel = (cmd.velocities[1] / JOYSTICK_MAX) * MAX_VEL_TILT;
                    float zoom_vel = (cmd.velocities[2] / JOYSTICK_MAX) * MAX_VEL_ZOOM;
                    
                    stepper_simple_set_velocities(pan_vel, tilt_vel, zoom_vel);
                    break;
                    
                case CMD_GOTO:
                    // Move to preset
                    if (stepper_simple_goto_preset(cmd.preset_index)) {
                        usb_serial_send_status("OK");
                    } else {
                        usb_serial_send_status("ERROR: Preset not found");
                    }
                    break;
                    
                case CMD_SAVE:
                    // Save current position as preset
                    if (stepper_simple_save_preset(cmd.preset_index)) {
                        usb_serial_send_status("OK");
                    } else {
                        usb_serial_send_status("ERROR: Save failed");
                    }
                    break;
                    
                case CMD_HOME:
                    // Start homing sequence
                    stepper_simple_home();
                    usb_serial_send_status("HOMING");
                    break;
                    
                case CMD_POS:
                    // Query current positions
                    stepper_simple_get_positions(&positions[0], &positions[1], &positions[2]);
                    usb_serial_send_position(positions[0], positions[1], positions[2]);
                    break;
                    
                case CMD_STATUS:
                    // Query system status
                    stepper_simple_get_positions(&positions[0], &positions[1], &positions[2]);
                    usb_serial_send("STATUS:PAN:%.2f TILT:%.2f ZOOM:%.2f\n",
                                   positions[0], positions[1], positions[2]);
                    break;
                    
                case CMD_STOP:
                    // Stop all motion
                    stepper_simple_stop();
                    usb_serial_send_status("STOPPED");
                    break;
                    
                case CMD_PRECISION:
                    // Set precision mode
                    stepper_simple_set_precision_mode(cmd.precision_enable);
                    usb_serial_send_status(cmd.precision_enable ? "PRECISION_ON" : "PRECISION_OFF");
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
    
    // Initialize simple stepper control
    stepper_simple_init();
    
    // Enable stepper drivers
    board_set_enable(true);
    
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
    
    // Main task
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
