/**
 * @file preset_storage.c
 * @brief Preset storage implementation using NVS
 */

#include "preset_storage.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <ctype.h>
#include <stdio.h>
#include <string.h>

static const char* TAG = "preset_store";
static const char* NVS_NAMESPACE = "preset_data";

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
    
    if (index == 0) {
        preset_init_default(preset);
        for (int i = 0; i < NUM_AXES; i++) {
            preset->pos[i] = 0.0f;
        }
        strncpy(preset->name, "Home", PRESET_NAME_LEN - 1);
        preset->valid = true;
        return true;
    }
    
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(err));
        return false;
    }

    char key[16];
    snprintf(key, sizeof(key), "preset_%02d", index);

    size_t sz = 0;
    err = nvs_get_blob(nvs_handle, key, NULL, &sz);
    if (err == ESP_ERR_NVS_INVALID_LENGTH && sz > 0) {
        err = ESP_OK;
    }
    if (err == ESP_ERR_NVS_NOT_FOUND || sz == 0) {
        nvs_close(nvs_handle);
        preset->valid = false;
        return false;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error sizing preset %d: %s", index, esp_err_to_name(err));
        nvs_close(nvs_handle);
        preset->valid = false;
        return false;
    }

    uint8_t buf[sizeof(preset_t)];
    memset(buf, 0, sizeof(buf));
    if (sz > sizeof(buf)) {
        sz = sizeof(buf);
    }
    err = nvs_get_blob(nvs_handle, key, buf, &sz);
    nvs_close(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error reading preset %d: %s", index, esp_err_to_name(err));
        preset->valid = false;
        return false;
    }

    preset_init_default(preset);
    memcpy(preset, buf, sz);
    preset->name[PRESET_NAME_LEN - 1] = '\0';
    preset_sanitize_name(preset->name);
    if (!(preset->duration_s >= 0.0f) || preset->duration_s > 120.0f) {
        preset->duration_s = 0.0f;
    }
    preset->valid = true;
    ESP_LOGI(TAG, "Loaded preset %d name='%s' duration=%.2f",
             index, preset->name, (double)preset->duration_s);
    return true;
}

bool preset_save(uint8_t index, const preset_t* preset) {
    if (index >= MAX_PRESETS) {
        ESP_LOGE(TAG, "Invalid preset index: %d", index);
        return false;
    }
    
    // Preset 0 is read-only and cannot be saved
    if (index == 0) {
        ESP_LOGW(TAG, "Preset 0 is read-only and cannot be saved");
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
    
    // Preset 0 is read-only and cannot be deleted
    if (index == 0) {
        ESP_LOGW(TAG, "Preset 0 is read-only and cannot be deleted");
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
    preset->max_speed = 0.0f;
    preset->accel_factor = 1.0f;
    preset->decel_factor = 1.0f;
    preset->duration_s = 0.0f;
    preset->valid = true;
}

void preset_sanitize_name(char *name)
{
    if (name == NULL) {
        return;
    }
    char tmp[PRESET_NAME_LEN];
    size_t o = 0;
    bool saw_space = true;
    for (size_t i = 0; name[i] != '\0' && o < PRESET_NAME_LEN - 1; i++) {
        unsigned char c = (unsigned char)name[i];
        if (isalnum(c) || c == '-' || c == '_' || c == '.') {
            tmp[o++] = (char)c;
            saw_space = false;
        } else if ((c == ' ' || c == '\t') && !saw_space) {
            tmp[o++] = ' ';
            saw_space = true;
        }
    }
    while (o > 0 && tmp[o - 1] == ' ') {
        o--;
    }
    tmp[o] = '\0';
    memcpy(name, tmp, o + 1);
}

bool preset_recall_load(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        return true;
    }
    uint8_t v = 1;
    err = nvs_get_u8(nvs_handle, "recall_en", &v);
    nvs_close(nvs_handle);
    if (err != ESP_OK) {
        return true;
    }
    return v != 0;
}

void preset_recall_save(bool enabled)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS to save recall: %s", esp_err_to_name(err));
        return;
    }
    err = nvs_set_u8(nvs_handle, "recall_en", enabled ? 1 : 0);
    if (err == ESP_OK) {
        err = nvs_commit(nvs_handle);
    }
    nvs_close(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving recall: %s", esp_err_to_name(err));
    }
}

bool preset_is_valid(uint8_t index) {
    // Preset 0 is always valid (hidden default preset)
    if (index == 0) {
        return true;
    }
    
    preset_t preset;
    if (!preset_load(index, &preset)) {
        return false;
    }
    return preset.valid;
}

