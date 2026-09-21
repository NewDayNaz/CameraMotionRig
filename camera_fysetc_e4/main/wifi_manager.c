/**
 * @file wifi_manager.c
 * @brief WiFi connection manager implementation
 */

#include "wifi_manager.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "http_server.h"
#include "stepper_limits.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>

static const char* TAG = "wifi_manager";
static volatile bool wifi_connected = false;
static char ip_address[16] = "0.0.0.0";
static SemaphoreHandle_t ntp_mux;
static bool sntp_active;
static volatile bool ntp_synced_this_attempt;
static int ntp_fail_logs;

static void ntp_stop(void)
{
    if (ntp_mux == NULL) {
        return;
    }
    xSemaphoreTake(ntp_mux, portMAX_DELAY);
    if (sntp_active) {
        esp_sntp_stop();
        sntp_active = false;
    }
    xSemaphoreGive(ntp_mux);
}

static void ntp_on_sync(struct timeval *tv)
{
    (void)tv;
    ntp_synced_this_attempt = true;
}

static void ntp_start(void)
{
    xSemaphoreTake(ntp_mux, portMAX_DELAY);
    if (!sntp_active) {
        ntp_synced_this_attempt = false;
        esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
        esp_sntp_setservername(0, "pool.ntp.org");
        /* While the client is running, lwIP retries on its own. Keep that
         * interval long; the supervisor stops SNTP after SNTP_ATTEMPT_MS so a
         * dead WAN cannot spin DNS/UDP every few seconds. */
        esp_sntp_set_sync_interval((uint32_t)SNTP_SYNC_INTERVAL_MS);
        esp_sntp_set_time_sync_notification_cb(ntp_on_sync);
        esp_sntp_init();
        sntp_active = true;
    }
    xSemaphoreGive(ntp_mux);
}

static void ntp_task(void *arg)
{
    (void)arg;
    setenv("TZ", IDLE_REHOME_TZ, 1);
    tzset();

    while (1) {
        if (!wifi_connected) {
            ntp_stop();
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        ntp_start();
        for (int i = 0; i < (SNTP_ATTEMPT_MS / 1000); i++) {
            if (ntp_synced_this_attempt || !wifi_connected) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        ntp_stop();

        if (!wifi_connected) {
            continue;
        }
        if (ntp_synced_this_attempt) {
            ntp_fail_logs = 0;
            ESP_LOGI(TAG, "NTP synced, next refresh in %d min (daily re-home 7:45 AM)",
                     SNTP_SYNC_INTERVAL_MS / 60000);
            vTaskDelay(pdMS_TO_TICKS(SNTP_SYNC_INTERVAL_MS));
        } else {
            if (ntp_fail_logs < 2 || (ntp_fail_logs % 4) == 0) {
                ESP_LOGW(TAG, "NTP unreachable, retry in %d min (daily re-home skipped until clock is set)",
                         SNTP_RETRY_UNSYNCED_MS / 60000);
            }
            ntp_fail_logs++;
            vTaskDelay(pdMS_TO_TICKS(SNTP_RETRY_UNSYNCED_MS));
        }
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_connected = false;
        ntp_stop();
        ESP_LOGW(TAG, "WiFi disconnected, attempting to reconnect...");
        http_server_stop();
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        snprintf(ip_address, sizeof(ip_address), IPSTR, IP2STR(&event->ip_info.ip));
        wifi_connected = true;
        ESP_LOGI(TAG, "WiFi connected! IP address: %s", ip_address);

        if (http_server_start()) {
            ESP_LOGI(TAG, "HTTP server started at http://%s/", ip_address);
        } else {
            ESP_LOGE(TAG, "Failed to start HTTP server");
        }
    }
}

bool wifi_manager_init(const char* ssid, const char* password) {
    ESP_LOGI(TAG, "Initializing WiFi...");

    ntp_mux = xSemaphoreCreateMutex();
    if (ntp_mux == NULL) {
        ESP_LOGE(TAG, "Failed to create NTP mutex");
        return false;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {0};
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_ERROR_CHECK(esp_wifi_start());

    xTaskCreate(ntp_task, "ntp", 3072, NULL, 1, NULL);

    ESP_LOGI(TAG, "WiFi initialization finished, connecting to SSID: %s", ssid);

    return true;
}

const char* wifi_manager_get_ip(void) {
    return ip_address;
}

bool wifi_manager_is_connected(void) {
    return wifi_connected;
}

void wifi_manager_deinit(void) {
    wifi_connected = false;
    ntp_stop();
    esp_wifi_stop();
    esp_wifi_deinit();
}
