/**
 * @file preset_storage.c
 * @brief Preset storage implementation using NVS
 */

#include "preset_storage.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>

static const char* TAG = "preset_storage";
static const char* NVS_NAMESPACE = "ptz_presets";

void preset_storage_init(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "Preset storage initialized");
}

bool preset_load(uint8_t index, preset_t* preset) {
    if (index >= MAX_PRESETS) {
        ESP_LOGE(TAG, "Invalid preset index: %d", index);
        return false;
    }
    
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(err));
        return false;
    }
    
    char key[16];
    snprintf(key, sizeof(key), "preset_%02d", index);
    
    size_t required_size = sizeof(preset_t);
    err = nvs_get_blob(nvs_handle, key, preset, &required_size);
    
    nvs_close(nvs_handle);
    
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        preset->valid = false;
        return false;
    }
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error reading preset %d: %s", index, esp_err_to_name(err));
        preset->valid = false;
        return false;
    }
    
    if (required_size != sizeof(preset_t)) {
        ESP_LOGE(TAG, "Preset %d size mismatch", index);
        preset->valid = false;
        return false;
    }
    
    preset->valid = true;
    ESP_LOGI(TAG, "Loaded preset %d", index);
    return true;
}

bool preset_save(uint8_t index, const preset_t* preset) {
    if (index >= MAX_PRESETS) {
        ESP_LOGE(TAG, "Invalid preset index: %d", index);
        return false;
    }
    
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(err));
        return false;
    }
    
    char key[16];
    snprintf(key, sizeof(key), "preset_%02d", index);
    
    err = nvs_set_blob(nvs_handle, key, preset, sizeof(preset_t));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error writing preset %d: %s", index, esp_err_to_name(err));
        nvs_close(nvs_handle);
        return false;
    }
    
    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error committing preset %d: %s", index, esp_err_to_name(err));
        return false;
    }
    
    ESP_LOGI(TAG, "Saved preset %d", index);
    return true;
}

bool preset_delete(uint8_t index) {
    if (index >= MAX_PRESETS) {
        ESP_LOGE(TAG, "Invalid preset index: %d", index);
        return false;
    }
    
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(err));
        return false;
    }
    
    char key[16];
    snprintf(key, sizeof(key), "preset_%02d", index);
    
    err = nvs_erase_key(nvs_handle, key);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        // Preset doesn't exist - not an error
        nvs_close(nvs_handle);
        return true;
    }
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error deleting preset %d: %s", index, esp_err_to_name(err));
        nvs_close(nvs_handle);
        return false;
    }
    
    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error committing preset deletion %d: %s", index, esp_err_to_name(err));
        return false;
    }
    
    ESP_LOGI(TAG, "Deleted preset %d", index);
    return true;
}

void preset_init_default(preset_t* preset) {
    memset(preset, 0, sizeof(preset_t));
    for (int i = 0; i < NUM_AXES; i++) {
        preset->pos[i] = 0.0f;
    }
    preset->easing_type = EASING_SMOOTHERSTEP;
    preset->duration_s = 2.0f;  // 2 second default
    preset->max_speed_scale = 0.0f;  // Use duration
    preset->arrival_overshoot = 0.0f;
    preset->approach_mode = APPROACH_DIRECT;
    preset->speed_multiplier = 1.0f;
    preset->accel_multiplier = 1.0f;
    preset->precision_preferred = false;
    preset->valid = true;
}

bool preset_is_valid(uint8_t index) {
    preset_t preset;
    if (!preset_load(index, &preset)) {
        return false;
    }
    return preset.valid;
}

