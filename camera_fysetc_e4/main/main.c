/**
 * @file main.c
 * @brief FYSETC E4 PTZ camera rig — open-loop motion with trusted homing
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "board.h"
#include "tmc_driver.h"
#include "stepper_simple.h"
#include "stepper_limits.h"
#include "preset_storage.h"
#include "zoom_cal.h"
#include "usb_serial.h"
#include "wifi_manager.h"
#include "http_server.h"
#include "wifi_config.h"

static const char* TAG = "main";

#define UPDATE_TASK_PERIOD_MS 1
#define UPDATE_TASK_STACK_SIZE 4096
#define UPDATE_TASK_PRIORITY 5

#define SERIAL_TASK_STACK_SIZE 4096
#define SERIAL_TASK_PRIORITY 3

static void update_task(void* pvParameters) {
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(UPDATE_TASK_PERIOD_MS);

    ESP_LOGI(TAG, "Update task started");
    if (esp_task_wdt_add(NULL) != ESP_OK) {
        ESP_LOGW(TAG, "Could not add update task to watchdog");
    }

    while (1) {
        stepper_simple_update();
        (void)esp_task_wdt_reset();
        vTaskDelayUntil(&last_wake, period);
    }
}

static void send_full_status(void)
{
    motion_status_t st;
    stepper_simple_get_status(&st);
    usb_serial_send("STATUS:PAN:%.2f TILT:%.2f ZOOM:%.2f HOMED:%d MOVING:%d HOMING:%d FAULT:%d%d%d AUTO:%d\n",
                    (float)st.position[AXIS_PAN],
                    (float)st.position[AXIS_TILT],
                    (float)st.position[AXIS_ZOOM],
                    st.homed ? 1 : 0,
                    st.moving ? 1 : 0,
                    st.homing ? 1 : 0,
                    st.axis_fault[AXIS_PAN] ? 1 : 0,
                    st.axis_fault[AXIS_TILT] ? 1 : 0,
                    st.axis_fault[AXIS_ZOOM] ? 1 : 0,
                    st.preset_recall ? 1 : 0);
}

static void serial_task(void* pvParameters) {
    parsed_cmd_t cmd;
    float positions[3];

    ESP_LOGI(TAG, "Serial task started");

    while (1) {
        if (usb_serial_parse_command(&cmd)) {
            switch (cmd.type) {
                case CMD_VEL:
                    stepper_simple_set_velocities(cmd.velocities[0],
                                                   cmd.velocities[1],
                                                   cmd.velocities[2]);
                    ESP_LOGI(TAG, "VEL: %.2f, %.2f, %.2f",
                            cmd.velocities[0], cmd.velocities[1], cmd.velocities[2]);
                    break;

                case CMD_JOYSTICK: {
                    const float JOYSTICK_MAX = 32768.0f;
                    float pan_vel = (cmd.velocities[0] / JOYSTICK_MAX) * MAX_PAN_VELOCITY;
                    float tilt_vel = (cmd.velocities[1] / JOYSTICK_MAX) * MAX_TILT_VELOCITY;
                    float zoom_vel = (cmd.velocities[2] / JOYSTICK_MAX) * MAX_ZOOM_VELOCITY;
                    stepper_simple_set_velocities(pan_vel, tilt_vel, zoom_vel);
                    break;
                }

                case CMD_GOTO:
                    if (stepper_simple_goto_preset(cmd.preset_index)) {
                        usb_serial_send_status("OK");
                    } else {
                        usb_serial_send("STATUS:ERROR: %s\n", stepper_simple_last_error());
                    }
                    break;

                case CMD_SAVE:
                    if (stepper_simple_save_preset(cmd.preset_index)) {
                        usb_serial_send_status("OK");
                    } else {
                        usb_serial_send("STATUS:ERROR: %s\n", stepper_simple_last_error());
                    }
                    break;

                case CMD_HOME:
                    stepper_simple_home();
                    usb_serial_send_status("HOMING");
                    break;

                case CMD_POS:
                    stepper_simple_get_positions(&positions[0], &positions[1], &positions[2]);
                    usb_serial_send_position(positions[0], positions[1], positions[2]);
                    break;

                case CMD_STATUS:
                    send_full_status();
                    break;

                case CMD_STOP:
                    stepper_simple_stop();
                    usb_serial_send_status("STOPPED");
                    break;

                case CMD_AUTO: {
                    bool on;
                    if (cmd.preset_index == 255) {
                        on = !stepper_simple_preset_recall_enabled();
                    } else {
                        on = cmd.preset_index != 0;
                    }
                    stepper_simple_set_preset_recall(on);
                    usb_serial_send("STATUS:AUTO:%d\n", on ? 1 : 0);
                    break;
                }

                case CMD_UNKNOWN:
                    usb_serial_send_status("ERROR: Unknown command");
                    break;

                default:
                    break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "FYSETC E4 PTZ Camera Rig Firmware Starting");

    preset_storage_init();
    zoom_cal_init();
    board_init();
    usb_serial_init();

    board_set_enable(true);
    tmc_driver_init();
    stepper_simple_init();

    ESP_LOGI(TAG, "System initialized, starting tasks");

    ESP_LOGI(TAG, "Initializing WiFi...");
    if (wifi_manager_init(WIFI_SSID, WIFI_PASSWORD)) {
        int timeout = 0;
        while (!wifi_manager_is_connected() && timeout < 100) {
            vTaskDelay(pdMS_TO_TICKS(100));
            timeout++;
        }

        if (wifi_manager_is_connected()) {
            ESP_LOGI(TAG, "WiFi connected! IP: %s", wifi_manager_get_ip());
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

    xTaskCreate(update_task, "update_task", UPDATE_TASK_STACK_SIZE, NULL,
                UPDATE_TASK_PRIORITY, NULL);
    xTaskCreate(serial_task, "serial_task", SERIAL_TASK_STACK_SIZE, NULL,
                SERIAL_TASK_PRIORITY, NULL);

    ESP_LOGI(TAG, "Tasks started — beginning startup homing");
    stepper_simple_home();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
