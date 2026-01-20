/**
 * @file http_server.c
 * @brief HTTP server for web-based motor control
 */

#include "http_server.h"
#include "stepper_simple.h"
#include "preset_storage.h"
#include "wifi_manager.h"

// Maximum velocities for web UI (steps/sec)
#define MAX_VELOCITY_PAN  500.0f  // Increased for better responsiveness
#define MAX_VELOCITY_TILT 500.0f  // Increased for better responsiveness
#define MAX_VELOCITY_ZOOM 100.0f
#include "esp_http_server.h"
#include "esp_log.h"
#include "cJSON.h"
#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include "esp_partition.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static const char* TAG = "http_server";
static httpd_handle_t server_handle = NULL;

// Web UI HTML (embedded) - auto-generated from web_ui.html
#include "web_ui_html.h"

// Handler for root path - serve HTML page
static esp_err_t root_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    
    // Format slider inputs using macro values
    char slider_pan[128], slider_tilt[128], slider_zoom[128];
    int pan_step = (MAX_VELOCITY_PAN >= 100.0f) ? 5 : 1;
    int tilt_step = (MAX_VELOCITY_TILT >= 100.0f) ? 5 : 1;
    
    snprintf(slider_pan, sizeof(slider_pan), 
             "<input type=\"range\" id=\"pan_vel\" min=\"-%.0f\" max=\"%.0f\" value=\"0\" step=\"%d\">",
             MAX_VELOCITY_PAN, MAX_VELOCITY_PAN, pan_step);
    snprintf(slider_tilt, sizeof(slider_tilt),
             "<input type=\"range\" id=\"tilt_vel\" min=\"-%.0f\" max=\"%.0f\" value=\"0\" step=\"%d\">",
             MAX_VELOCITY_TILT, MAX_VELOCITY_TILT, tilt_step);
    snprintf(slider_zoom, sizeof(slider_zoom),
             "<input type=\"range\" id=\"zoom_vel\" min=\"-%.0f\" max=\"%.0f\" value=\"0\" step=\"1\">",
             MAX_VELOCITY_ZOOM, MAX_VELOCITY_ZOOM);
    
    // Replace placeholders in HTML
    // Calculate buffer size needed (original size + slider strings - placeholder strings)
    size_t html_len = strlen(html_page);
    size_t slider_pan_len = strlen(slider_pan);
    size_t slider_tilt_len = strlen(slider_tilt);
    size_t slider_zoom_len = strlen(slider_zoom);
    size_t placeholder_len = strlen("%SLIDER_PAN%") + strlen("%SLIDER_TILT%") + strlen("%SLIDER_ZOOM%");
    size_t buffer_size = html_len - placeholder_len + slider_pan_len + slider_tilt_len + slider_zoom_len + 1;
    
    char *html_buffer = malloc(buffer_size);
    if (html_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for HTML");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    // Replace placeholders using simple string replacement
    char *html_work = (char *)html_page;
    char *output = html_buffer;
    const char *placeholder_pan = "%SLIDER_PAN%";
    const char *placeholder_tilt = "%SLIDER_TILT%";
    const char *placeholder_zoom = "%SLIDER_ZOOM%";
    
    // Replace %SLIDER_PAN%
    char *pos = strstr(html_work, placeholder_pan);
    if (pos) {
        size_t len = pos - html_work;
        memcpy(output, html_work, len);
        output += len;
        memcpy(output, slider_pan, slider_pan_len);
        output += slider_pan_len;
        html_work = pos + strlen(placeholder_pan);
    } else {
        free(html_buffer);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    // Replace %SLIDER_TILT%
    pos = strstr(html_work, placeholder_tilt);
    if (pos) {
        size_t len = pos - html_work;
        memcpy(output, html_work, len);
        output += len;
        memcpy(output, slider_tilt, slider_tilt_len);
        output += slider_tilt_len;
        html_work = pos + strlen(placeholder_tilt);
    } else {
        free(html_buffer);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    // Replace %SLIDER_ZOOM%
    pos = strstr(html_work, placeholder_zoom);
    if (pos) {
        size_t len = pos - html_work;
        memcpy(output, html_work, len);
        output += len;
        memcpy(output, slider_zoom, slider_zoom_len);
        output += slider_zoom_len;
        html_work = pos + strlen(placeholder_zoom);
    } else {
        free(html_buffer);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    // Copy remaining suffix
    size_t suffix_len = strlen(html_work);
    memcpy(output, html_work, suffix_len);
    output += suffix_len;
    *output = '\0';
    
    httpd_resp_send(req, html_buffer, HTTPD_RESP_USE_STRLEN);
    free(html_buffer);
    return ESP_OK;
}

// Handler for /api/positions - GET current positions
static esp_err_t api_positions_handler(httpd_req_t *req) {
    float pan, tilt, zoom;
    stepper_simple_get_positions(&pan, &tilt, &zoom);
    
    cJSON *json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "pan", pan);
    cJSON_AddNumberToObject(json, "tilt", tilt);
    cJSON_AddNumberToObject(json, "zoom", zoom);
    
    char *json_string = cJSON_Print(json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));
    
    free(json_string);
    cJSON_Delete(json);
    return ESP_OK;
}

// Handler for /api/velocity - POST set velocities
static esp_err_t api_velocity_handler(httpd_req_t *req) {
    char content[256];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    content[ret] = '\0';
    
    cJSON *json = cJSON_Parse(content);
    if (json == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    float velocities[3] = {0, 0, 0};
    cJSON *pan = cJSON_GetObjectItem(json, "pan");
    cJSON *tilt = cJSON_GetObjectItem(json, "tilt");
    cJSON *zoom = cJSON_GetObjectItem(json, "zoom");
    
    if (pan) velocities[0] = (float)pan->valuedouble;
    if (tilt) velocities[1] = (float)tilt->valuedouble;
    if (zoom) velocities[2] = (float)zoom->valuedouble;
    
    stepper_simple_set_velocities(velocities[0], velocities[1], velocities[2]);
    
    cJSON_Delete(json);
    
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "status", "ok");
    char *response_str = cJSON_Print(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_str, strlen(response_str));
    free(response_str);
    cJSON_Delete(response);
    
    return ESP_OK;
}

// Handler for /api/command - POST command
static esp_err_t api_command_handler(httpd_req_t *req) {
    char content[256];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    content[ret] = '\0';
    
    cJSON *json = cJSON_Parse(content);
    if (json == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *cmd = cJSON_GetObjectItem(json, "command");
    if (cmd == NULL || !cJSON_IsString(cmd)) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing or invalid command field");
        return ESP_FAIL;
    }
    
    const char *command = cmd->valuestring;
    bool success = false;
    
    if (strcmp(command, "home") == 0) {
        stepper_simple_home();
        success = true;
    } else if (strcmp(command, "stop") == 0) {
        stepper_simple_stop();
        success = true;
    } else if (strcmp(command, "precision") == 0) {
        // Toggle precision mode - we'd need to track state or query it
        stepper_simple_set_precision_mode(true);  // For now, just enable
        success = true;
    }
    
    cJSON_Delete(json);
    
    cJSON *response = cJSON_CreateObject();
    if (success) {
        cJSON_AddStringToObject(response, "status", "ok");
    } else {
        cJSON_AddStringToObject(response, "status", "error");
        cJSON_AddStringToObject(response, "error", "Command failed");
    }
    
    char *response_str = cJSON_Print(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_str, strlen(response_str));
    free(response_str);
    cJSON_Delete(response);
    
    return ESP_OK;
}

// Handler for /api/preset/goto - POST goto preset
static esp_err_t api_preset_goto_handler(httpd_req_t *req) {
    char content[256];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    content[ret] = '\0';
    
    cJSON *json = cJSON_Parse(content);
    if (json == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *idx = cJSON_GetObjectItem(json, "index");
    if (idx == NULL || !cJSON_IsNumber(idx)) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing or invalid index field");
        return ESP_FAIL;
    }
    
    uint8_t preset_idx = (uint8_t)idx->valueint;
    bool success = stepper_simple_goto_preset(preset_idx);
    
    cJSON_Delete(json);
    
    cJSON *response = cJSON_CreateObject();
    if (success) {
        cJSON_AddStringToObject(response, "status", "ok");
    } else {
        cJSON_AddStringToObject(response, "status", "error");
        cJSON_AddStringToObject(response, "error", "Failed to move to preset");
    }
    
    char *response_str = cJSON_Print(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_str, strlen(response_str));
    free(response_str);
    cJSON_Delete(response);
    
    return ESP_OK;
}

// Handler for /api/preset/save - POST save preset
static esp_err_t api_preset_save_handler(httpd_req_t *req) {
    char content[256];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    content[ret] = '\0';
    
    cJSON *json = cJSON_Parse(content);
    if (json == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *idx = cJSON_GetObjectItem(json, "index");
    if (idx == NULL || !cJSON_IsNumber(idx)) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing or invalid index field");
        return ESP_FAIL;
    }
    
    uint8_t preset_idx = (uint8_t)idx->valueint;
    bool success = stepper_simple_save_preset(preset_idx);
    
    cJSON_Delete(json);
    
    cJSON *response = cJSON_CreateObject();
    if (success) {
        cJSON_AddStringToObject(response, "status", "ok");
    } else {
        cJSON_AddStringToObject(response, "status", "error");
        cJSON_AddStringToObject(response, "error", "Failed to save preset");
    }
    
    char *response_str = cJSON_Print(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_str, strlen(response_str));
    free(response_str);
    cJSON_Delete(response);
    
    return ESP_OK;
}

// Handler for /api/preset/get - GET preset data
static esp_err_t api_preset_get_handler(httpd_req_t *req) {
    char query[64];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing query parameter");
        return ESP_FAIL;
    }
    
    char index_str[8];
    if (httpd_query_key_value(query, "index", index_str, sizeof(index_str)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing index parameter");
        return ESP_FAIL;
    }
    
    uint8_t preset_idx = (uint8_t)atoi(index_str);
    
    // Preset 0 is hidden but can still be queried (returns 0,0,0)
    preset_t preset;
    bool success = preset_load(preset_idx, &preset);
    
    cJSON *response = cJSON_CreateObject();
    if (success && preset.valid) {
        cJSON_AddStringToObject(response, "status", "ok");
        cJSON *preset_json = cJSON_CreateObject();
        
        // Add position array
        cJSON *pos_array = cJSON_CreateArray();
        for (int i = 0; i < 3; i++) {
            cJSON_AddItemToArray(pos_array, cJSON_CreateNumber(preset.pos[i]));
        }
        cJSON_AddItemToObject(preset_json, "pos", pos_array);
        
        cJSON_AddNumberToObject(preset_json, "max_speed", preset.max_speed);
        cJSON_AddNumberToObject(preset_json, "accel_factor", preset.accel_factor);
        cJSON_AddNumberToObject(preset_json, "decel_factor", preset.decel_factor);
        cJSON_AddBoolToObject(preset_json, "valid", preset.valid);
        
        cJSON_AddItemToObject(response, "preset", preset_json);
    } else {
        cJSON_AddStringToObject(response, "status", "not_found");
    }
    
    char *response_str = cJSON_Print(response);
    if (response_str == NULL) {
        cJSON_Delete(response);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    httpd_resp_set_type(req, "application/json");
    esp_err_t ret = httpd_resp_send(req, response_str, strlen(response_str));
    free(response_str);
    cJSON_Delete(response);
    
    // Ignore connection reset errors (client disconnected)
    if (ret == ESP_ERR_HTTPD_RESP_SEND) {
        return ESP_OK;
    }
    
    return ret;
}

// Handler for /api/preset/update - POST update preset
static esp_err_t api_preset_update_handler(httpd_req_t *req) {
    char content[512];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    content[ret] = '\0';
    
    cJSON *json = cJSON_Parse(content);
    if (json == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *idx = cJSON_GetObjectItem(json, "index");
    if (idx == NULL || !cJSON_IsNumber(idx)) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing or invalid index field");
        return ESP_FAIL;
    }
    
    uint8_t preset_idx = (uint8_t)idx->valueint;
    
    // Preset 0 is read-only and cannot be updated
    if (preset_idx == 0) {
        cJSON *response = cJSON_CreateObject();
        cJSON_AddStringToObject(response, "status", "error");
        cJSON_AddStringToObject(response, "error", "Preset 0 is read-only and cannot be updated");
        char *response_str = cJSON_Print(response);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, response_str, strlen(response_str));
        free(response_str);
        cJSON_Delete(response);
        return ESP_OK;
    }
    
    preset_t preset;
    preset_init_default(&preset);
    
    // Parse position array
    cJSON *pos_array = cJSON_GetObjectItem(json, "pos");
    if (pos_array != NULL && cJSON_IsArray(pos_array)) {
        int array_size = cJSON_GetArraySize(pos_array);
        for (int i = 0; i < array_size && i < 3; i++) {
            cJSON *item = cJSON_GetArrayItem(pos_array, i);
            if (cJSON_IsNumber(item)) {
                preset.pos[i] = (float)item->valuedouble;
            }
        }
    }
    
    // Parse other fields
    cJSON *item;
    if ((item = cJSON_GetObjectItem(json, "max_speed")) != NULL && cJSON_IsNumber(item)) {
        preset.max_speed = (float)item->valuedouble;
    }
    if ((item = cJSON_GetObjectItem(json, "accel_factor")) != NULL && cJSON_IsNumber(item)) {
        preset.accel_factor = (float)item->valuedouble;
    }
    if ((item = cJSON_GetObjectItem(json, "decel_factor")) != NULL && cJSON_IsNumber(item)) {
        preset.decel_factor = (float)item->valuedouble;
    }
    if ((item = cJSON_GetObjectItem(json, "valid")) != NULL && cJSON_IsBool(item)) {
        preset.valid = cJSON_IsTrue(item);
    }
    
    cJSON_Delete(json);
    
    bool success = preset_save(preset_idx, &preset);
    
    cJSON *response = cJSON_CreateObject();
    if (success) {
        cJSON_AddStringToObject(response, "status", "ok");
    } else {
        cJSON_AddStringToObject(response, "status", "error");
        cJSON_AddStringToObject(response, "error", "Failed to update preset");
    }
    
    char *response_str = cJSON_Print(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_str, strlen(response_str));
    free(response_str);
    cJSON_Delete(response);
    
    return ESP_OK;
}

// Handler for /api/update - POST firmware update (OTA)
static esp_err_t api_update_handler(httpd_req_t *req) {
    esp_ota_handle_t ota_handle = 0;
    const esp_partition_t *update_partition = NULL;
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
    
    // Check if OTA is already in progress
    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            ESP_LOGW(TAG, "OTA update pending verification");
            cJSON *response = cJSON_CreateObject();
            cJSON_AddStringToObject(response, "status", "error");
            cJSON_AddStringToObject(response, "error", "OTA update pending verification. Please reboot.");
            char *response_str = cJSON_Print(response);
            httpd_resp_set_type(req, "application/json");
            httpd_resp_send(req, response_str, strlen(response_str));
            free(response_str);
            cJSON_Delete(response);
            return ESP_OK;
        }
    }
    
    // Diagnostic: Log running partition info
    if (running != NULL) {
        ESP_LOGI(TAG, "Running partition: %s, type=%d, subtype=%d, address=0x%x, size=%d", 
                 running->label, running->type, running->subtype, running->address, running->size);
    } else {
        ESP_LOGW(TAG, "Running partition is NULL");
    }
    
    // Diagnostic: List all OTA partitions
    const esp_partition_t *ota_0 = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
    const esp_partition_t *ota_1 = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1, NULL);
    
    if (ota_0 != NULL) {
        ESP_LOGI(TAG, "Found ota_0: %s, address=0x%x, size=%d", ota_0->label, ota_0->address, ota_0->size);
    } else {
        ESP_LOGW(TAG, "ota_0 partition not found");
    }
    
    if (ota_1 != NULL) {
        ESP_LOGI(TAG, "Found ota_1: %s, address=0x%x, size=%d", ota_1->label, ota_1->address, ota_1->size);
    } else {
        ESP_LOGW(TAG, "ota_1 partition not found");
    }
    
    // Find next OTA partition
    update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL) {
        ESP_LOGE(TAG, "No OTA partition found by esp_ota_get_next_update_partition");
        ESP_LOGE(TAG, "This usually means: 1) No OTA partitions in partition table, or 2) Running from factory partition");
        cJSON *response = cJSON_CreateObject();
        cJSON_AddStringToObject(response, "status", "error");
        cJSON_AddStringToObject(response, "error", "No OTA partition available. Device may need reflashing with OTA partition table.");
        char *response_str = cJSON_Print(response);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, response_str, strlen(response_str));
        free(response_str);
        cJSON_Delete(response);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Starting OTA update on partition: %s", update_partition->label);
    
    // Begin OTA update
    esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        cJSON *response = cJSON_CreateObject();
        cJSON_AddStringToObject(response, "status", "error");
        cJSON_AddStringToObject(response, "error", "Failed to begin OTA update");
        char *response_str = cJSON_Print(response);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, response_str, strlen(response_str));
        free(response_str);
        cJSON_Delete(response);
        return ESP_FAIL;
    }
    
    // Receive firmware data
    char *buf = malloc(1024);
    if (buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate buffer");
        esp_ota_abort(ota_handle);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    int total_len = req->content_len;
    int received = 0;
    int content_received = 0;
    bool content_length_known = (total_len > 0);
    
    ESP_LOGI(TAG, "Receiving firmware update%s", content_length_known ? 
             " (size unknown, streaming)" : "");
    if (content_length_known) {
        ESP_LOGI(TAG, "Expected size: %d bytes", total_len);
    }
    
    // Receive firmware data
    // If content_len is 0, we'll receive until connection closes (chunked transfer)
    while (!content_length_known || content_received < total_len) {
        int recv_len = httpd_req_recv(req, buf, 1024);
        if (recv_len < 0) {
            if (recv_len == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            // If we don't know the content length and recv_len is 0, we're done
            if (!content_length_known && recv_len == 0) {
                break;
            }
            ESP_LOGE(TAG, "OTA receive error: %d", recv_len);
            free(buf);
            esp_ota_abort(ota_handle);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        
        if (recv_len == 0) {
            // Connection closed, we're done
            if (!content_length_known) {
                break;
            }
            // If we expected more data, this is an error
            if (content_received < total_len) {
                ESP_LOGE(TAG, "OTA receive incomplete: %d / %d bytes", content_received, total_len);
                free(buf);
                esp_ota_abort(ota_handle);
                httpd_resp_send_500(req);
                return ESP_FAIL;
            }
            break;
        }
        
        err = esp_ota_write(ota_handle, buf, recv_len);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
            free(buf);
            esp_ota_abort(ota_handle);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        content_received += recv_len;
        received += recv_len;
        
        // Log progress every 64KB
        if (received % 65536 < 1024) {
            if (content_length_known) {
                ESP_LOGI(TAG, "OTA progress: %d / %d bytes (%.1f%%)", 
                        received, total_len, (received * 100.0f) / total_len);
            } else {
                ESP_LOGI(TAG, "OTA progress: %d bytes received", received);
            }
        }
    }
    
    ESP_LOGI(TAG, "OTA data reception complete: %d bytes", received);
    
    free(buf);
    
    // Finish OTA update
    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
            ESP_LOGE(TAG, "OTA validation failed");
            cJSON *response = cJSON_CreateObject();
            cJSON_AddStringToObject(response, "status", "error");
            cJSON_AddStringToObject(response, "error", "Firmware validation failed");
            char *response_str = cJSON_Print(response);
            httpd_resp_set_type(req, "application/json");
            httpd_resp_send(req, response_str, strlen(response_str));
            free(response_str);
            cJSON_Delete(response);
            return ESP_FAIL;
        } else {
            ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
    }
    
    // Set boot partition
    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "OTA update completed successfully. Firmware will be active after reboot.");
    
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "status", "ok");
    cJSON_AddStringToObject(response, "message", "OTA update completed. Device will reboot.");
    char *response_str = cJSON_Print(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_str, strlen(response_str));
    free(response_str);
    cJSON_Delete(response);
    
    // Reboot after a short delay to allow response to be sent
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    
    return ESP_OK;
}

bool http_server_start(void) {
    // If server is already running, return success
    if (server_handle != NULL) {
        ESP_LOGI(TAG, "HTTP server already running");
        return true;
    }
    
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 10;
    
    ESP_LOGI(TAG, "Starting HTTP server on port %d", config.server_port);
    
    if (httpd_start(&server_handle, &config) == ESP_OK) {
        // Register URI handlers
        httpd_uri_t root_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = root_handler,
        };
        httpd_register_uri_handler(server_handle, &root_uri);
        
        httpd_uri_t positions_uri = {
            .uri = "/api/positions",
            .method = HTTP_GET,
            .handler = api_positions_handler,
        };
        httpd_register_uri_handler(server_handle, &positions_uri);
        
        httpd_uri_t velocity_uri = {
            .uri = "/api/velocity",
            .method = HTTP_POST,
            .handler = api_velocity_handler,
        };
        httpd_register_uri_handler(server_handle, &velocity_uri);
        
        httpd_uri_t command_uri = {
            .uri = "/api/command",
            .method = HTTP_POST,
            .handler = api_command_handler,
        };
        httpd_register_uri_handler(server_handle, &command_uri);
        
        httpd_uri_t preset_goto_uri = {
            .uri = "/api/preset/goto",
            .method = HTTP_POST,
            .handler = api_preset_goto_handler,
        };
        httpd_register_uri_handler(server_handle, &preset_goto_uri);
        
        httpd_uri_t preset_save_uri = {
            .uri = "/api/preset/save",
            .method = HTTP_POST,
            .handler = api_preset_save_handler,
        };
        httpd_register_uri_handler(server_handle, &preset_save_uri);
        
        httpd_uri_t preset_get_uri = {
            .uri = "/api/preset/get",
            .method = HTTP_GET,
            .handler = api_preset_get_handler,
        };
        httpd_register_uri_handler(server_handle, &preset_get_uri);
        
        httpd_uri_t preset_update_uri = {
            .uri = "/api/preset/update",
            .method = HTTP_POST,
            .handler = api_preset_update_handler,
        };
        httpd_register_uri_handler(server_handle, &preset_update_uri);
        
        httpd_uri_t update_uri = {
            .uri = "/api/update",
            .method = HTTP_POST,
            .handler = api_update_handler,
        };
        httpd_register_uri_handler(server_handle, &update_uri);
        
        ESP_LOGI(TAG, "HTTP server started");
        return true;
    }
    
    ESP_LOGE(TAG, "Failed to start HTTP server");
    return false;
}

void http_server_stop(void) {
    if (server_handle) {
        httpd_stop(server_handle);
        server_handle = NULL;
        ESP_LOGI(TAG, "HTTP server stopped");
    }
}


