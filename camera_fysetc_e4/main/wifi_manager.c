/**
 * @file wifi_manager.c
 * @brief WiFi connection manager implementation
 */

#include "wifi_manager.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "http_server.h"
#include <string.h>

static const char* TAG = "wifi_manager";
static bool wifi_connected = false;
static char ip_address[16] = "0.0.0.0";

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_connected = false;
        ESP_LOGW(TAG, "WiFi disconnected, attempting to reconnect...");
        // Stop HTTP server when WiFi disconnects
        http_server_stop();
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        snprintf(ip_address, sizeof(ip_address), IPSTR, IP2STR(&event->ip_info.ip));
        wifi_connected = true;
        ESP_LOGI(TAG, "WiFi connected! IP address: %s", ip_address);
        
        // Start HTTP server when WiFi connects
        if (http_server_start()) {
            ESP_LOGI(TAG, "HTTP server started at http://%s/", ip_address);
        } else {
            ESP_LOGE(TAG, "Failed to start HTTP server");
        }
    }
}

bool wifi_manager_init(const char* ssid, const char* password) {
    ESP_LOGI(TAG, "Initializing WiFi...");
    
    // Initialize network interface
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // Initialize WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register event handlers
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

    // Configure WiFi station
    wifi_config_t wifi_config = {0};
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    
    // Disable WiFi power saving to prevent disconnections
    // WIFI_PS_NONE = no power saving (best for servers/always-on applications)
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    
    ESP_ERROR_CHECK(esp_wifi_start());

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
    esp_wifi_stop();
    esp_wifi_deinit();
}


