/**
 * @file http_server.c
 * @brief HTTP server for web-based motor control
 */

#include "http_server.h"
#include "motion_controller.h"
#include "wifi_manager.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "cJSON.h"
#include <string.h>
#include <stdio.h>

static const char* TAG = "http_server";
static httpd_handle_t server_handle = NULL;

// Web UI HTML (embedded)
static const char html_page[] = 
"<!DOCTYPE html>"
"<html><head>"
"<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"
"<title>PTZ Camera Control</title>"
"<style>"
"body { font-family: Arial, sans-serif; margin: 20px; background: #f0f0f0; }"
".container { max-width: 800px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }"
"h1 { color: #333; text-align: center; }"
".section { margin: 20px 0; padding: 15px; background: #f9f9f9; border-radius: 5px; }"
".section h2 { margin-top: 0; color: #555; }"
".control-group { margin: 15px 0; }"
"label { display: block; margin-bottom: 5px; font-weight: bold; color: #666; }"
"input[type=\"range\"] { width: 100%; margin: 10px 0; }"
"input[type=\"text\"] { width: 100px; padding: 5px; margin: 0 10px; }"
"button { padding: 10px 20px; margin: 5px; font-size: 16px; cursor: pointer; border: none; border-radius: 5px; }"
".btn-primary { background: #4CAF50; color: white; }"
".btn-primary:hover { background: #45a049; }"
".btn-secondary { background: #2196F3; color: white; }"
".btn-secondary:hover { background: #0b7dda; }"
".btn-danger { background: #f44336; color: white; }"
".btn-danger:hover { background: #da190b; }"
".btn-warning { background: #ff9800; color: white; }"
".btn-warning:hover { background: #e68900; }"
".status { padding: 10px; margin: 10px 0; border-radius: 5px; }"
".status-info { background: #e3f2fd; color: #1976d2; }"
".status-success { background: #e8f5e9; color: #388e3c; }"
"#positions { font-family: monospace; font-size: 18px; }"
".preset-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(150px, 1fr)); gap: 10px; margin: 10px 0; }"
".preset-btn { padding: 15px; font-size: 14px; }"
"</style>"
"</head><body>"
"<div class=\"container\">"
"<h1>🎥 PTZ Camera Control</h1>"
"<div class=\"section\">"
"<h2>Position Status</h2>"
"<div id=\"positions\" class=\"status status-info\">Loading...</div>"
"</div>"
"<div class=\"section\">"
"<h2>Velocity Control</h2>"
"<div class=\"control-group\">"
"<label>PAN: <span id=\"pan_val\">0.0</span> steps/s</label>"
"<input type=\"range\" id=\"pan_vel\" min=\"-500\" max=\"500\" value=\"0\" step=\"10\">"
"</div>"
"<div class=\"control-group\">"
"<label>TILT: <span id=\"tilt_val\">0.0</span> steps/s</label>"
"<input type=\"range\" id=\"tilt_vel\" min=\"-500\" max=\"500\" value=\"0\" step=\"10\">"
"</div>"
"<div class=\"control-group\">"
"<label>ZOOM: <span id=\"zoom_val\">0.0</span> steps/s</label>"
"<input type=\"range\" id=\"zoom_vel\" min=\"-500\" max=\"500\" value=\"0\" step=\"10\">"
"</div>"
"</div>"
"<div class=\"section\">"
"<h2>Commands</h2>"
"<button class=\"btn-primary\" onclick=\"sendCommand('home')\">🏠 Home All Axes</button>"
"<button class=\"btn-danger\" onclick=\"sendCommand('stop')\">⏹ Stop</button>"
"<button class=\"btn-secondary\" onclick=\"sendCommand('precision')\">🎯 Toggle Precision</button>"
"</div>"
"<div class=\"section\">"
"<h2>Presets</h2>"
"<div class=\"preset-grid\" id=\"preset_grid\"></div>"
"<div style=\"margin-top: 10px;\">"
"<button class=\"btn-warning\" onclick=\"savePreset()\">💾 Save Current Position</button>"
"<input type=\"number\" id=\"preset_save_idx\" min=\"0\" max=\"9\" value=\"0\" style=\"width: 60px; padding: 5px; margin-left: 10px;\">"
"</div>"
"</div>"
"</div>"
"<script>"
"let precisionMode = false;"
"let updateInterval;"
"function updatePositions() {"
"  fetch('/api/positions').then(r=>r.json()).then(data => {"
"    document.getElementById('positions').textContent = `PAN: ${data.pan.toFixed(1)} | TILT: ${data.tilt.toFixed(1)} | ZOOM: ${data.zoom.toFixed(1)}`;"
"  }).catch(e => console.error('Failed to fetch positions:', e));"
"}"
"function updateVelocities() {"
"  const pan = parseFloat(document.getElementById('pan_vel').value);"
"  const tilt = parseFloat(document.getElementById('tilt_vel').value);"
"  const zoom = parseFloat(document.getElementById('zoom_vel').value);"
"  document.getElementById('pan_val').textContent = pan.toFixed(1);"
"  document.getElementById('tilt_val').textContent = tilt.toFixed(1);"
"  document.getElementById('zoom_val').textContent = zoom.toFixed(1);"
"  fetch('/api/velocity', {"
"    method: 'POST',"
"    headers: { 'Content-Type': 'application/json' },"
"    body: JSON.stringify({pan, tilt, zoom})"
"  }).catch(e => console.error('Failed to set velocity:', e));"
"}"
"document.getElementById('pan_vel').addEventListener('input', updateVelocities);"
"document.getElementById('tilt_vel').addEventListener('input', updateVelocities);"
"document.getElementById('zoom_vel').addEventListener('input', updateVelocities);"
"function sendCommand(cmd) {"
"  fetch('/api/command', {"
"    method: 'POST',"
"    headers: { 'Content-Type': 'application/json' },"
"    body: JSON.stringify({command: cmd})"
"  }).then(r => r.json()).then(data => {"
"    if (data.status === 'ok') {"
"      alert('Command executed: ' + cmd);"
"      if (cmd === 'precision') precisionMode = !precisionMode;"
"    } else {"
"      alert('Error: ' + (data.error || 'Unknown error'));"
"    }"
"  }).catch(e => {"
"    console.error('Command failed:', e);"
"    alert('Failed to send command');"
"  });"
"}"
"function gotoPreset(idx) {"
"  fetch('/api/preset/goto', {"
"    method: 'POST',"
"    headers: { 'Content-Type': 'application/json' },"
"    body: JSON.stringify({index: idx})"
"  }).then(r => r.json()).then(data => {"
"    if (data.status === 'ok') {"
"      alert('Moving to preset ' + idx);"
"    } else {"
"      alert('Error: ' + (data.error || 'Failed to move to preset'));"
"    }"
"  }).catch(e => {"
"    console.error('Goto preset failed:', e);"
"    alert('Failed to move to preset');"
"  });"
"}"
"function savePreset() {"
"  const idx = parseInt(document.getElementById('preset_save_idx').value);"
"  fetch('/api/preset/save', {"
"    method: 'POST',"
"    headers: { 'Content-Type': 'application/json' },"
"    body: JSON.stringify({index: idx})"
"  }).then(r => r.json()).then(data => {"
"    if (data.status === 'ok') {"
"      alert('Preset ' + idx + ' saved!');"
"      createPresetButtons();"
"    } else {"
"      alert('Error: ' + (data.error || 'Failed to save preset'));"
"    }"
"  }).catch(e => {"
"    console.error('Save preset failed:', e);"
"    alert('Failed to save preset');"
"  });"
"}"
"function createPresetButtons() {"
"  const grid = document.getElementById('preset_grid');"
"  grid.innerHTML = '';"
"  for (let i = 0; i < 10; i++) {"
"    const btn = document.createElement('button');"
"    btn.className = 'btn-secondary preset-btn';"
"    btn.textContent = 'Preset ' + i;"
"    btn.onclick = () => gotoPreset(i);"
"    grid.appendChild(btn);"
"  }"
"}"
"updatePositions();"
"updateInterval = setInterval(updatePositions, 500);"
"createPresetButtons();"
"</script>"
"</body></html>";

// Handler for root path - serve HTML page
static esp_err_t root_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_page, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Handler for /api/positions - GET current positions
static esp_err_t api_positions_handler(httpd_req_t *req) {
    float positions[3];
    motion_controller_get_positions(positions);
    
    cJSON *json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "pan", positions[0]);
    cJSON_AddNumberToObject(json, "tilt", positions[1]);
    cJSON_AddNumberToObject(json, "zoom", positions[2]);
    
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
    
    motion_controller_set_velocities(velocities);
    
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
        success = motion_controller_home(255);  // Home all axes
    } else if (strcmp(command, "stop") == 0) {
        motion_controller_stop();
        success = true;
    } else if (strcmp(command, "precision") == 0) {
        // Toggle precision mode - we'd need to track state or query it
        motion_controller_set_precision_mode(true);  // For now, just enable
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
    bool success = motion_controller_goto_preset(preset_idx);
    
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
    bool success = motion_controller_save_preset(preset_idx);
    
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

bool http_server_start(void) {
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

