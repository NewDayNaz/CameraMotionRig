/**
 * @file wifi_manager.h
 * @brief WiFi connection manager
 */

#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stdbool.h>

/**
 * @brief Initialize WiFi and connect to network
 * @param ssid WiFi SSID
 * @param password WiFi password
 * @return true on success
 */
bool wifi_manager_init(const char* ssid, const char* password);

/**
 * @brief Get current IP address
 * @return IP address string (static buffer)
 */
const char* wifi_manager_get_ip(void);

/**
 * @brief Check if WiFi is connected
 * @return true if connected
 */
bool wifi_manager_is_connected(void);

/**
 * @brief Deinitialize WiFi
 */
void wifi_manager_deinit(void);

#endif // WIFI_MANAGER_H


