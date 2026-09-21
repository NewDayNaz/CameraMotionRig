/**
 * @file preset_storage.h
 * @brief Preset storage using ESP32 NVS (Non-Volatile Storage)
 * 
 * Simplified preset storage for basic PTZ camera control.
 * Stores positions and basic motion parameters.
 */

#ifndef PRESET_STORAGE_H
#define PRESET_STORAGE_H

#include <stdint.h>
#include <stdbool.h>
#include "board.h"

#define MAX_PRESETS 16
#define PRESET_NAME_LEN 24

/**
 * Positions plus optional name and shared move duration.
 * Older NVS blobs (no name/duration) still load; new fields default to empty/0.
 */
typedef struct {
    float pos[NUM_AXES];
    float max_speed;          /* legacy pan/tilt speed if duration_s == 0 */
    float accel_factor;       /* unused */
    float decel_factor;       /* unused */
    bool valid;
    uint8_t _pad[3];
    char name[PRESET_NAME_LEN];
    float duration_s;         /* 0 = auto (axes finish together at default speeds) */
} preset_t;

/**
 * @brief Initialize preset storage (initialize NVS)
 */
void preset_storage_init(void);

/**
 * @brief Load a preset from NVS
 * @param index Preset index (0 to MAX_PRESETS-1)
 * @param preset Output preset structure
 * @return true if preset was loaded successfully
 */
bool preset_load(uint8_t index, preset_t* preset);

/**
 * @brief Save a preset to NVS
 * @param index Preset index (0 to MAX_PRESETS-1)
 * @param preset Preset structure to save
 * @return true if saved successfully
 */
bool preset_save(uint8_t index, const preset_t* preset);

/**
 * @brief Delete a preset
 * @param index Preset index
 * @return true if deleted successfully
 */
bool preset_delete(uint8_t index);

/**
 * @brief Initialize a preset with default values
 */
void preset_init_default(preset_t* preset);

bool preset_is_valid(uint8_t index);

/** Keep letters, digits, space, - _ . Trim and NUL-terminate. */
void preset_sanitize_name(char *name);

/** MIDI/Companion/web GOTO. Missing NVS key means enabled. */
bool preset_recall_load(void);
void preset_recall_save(bool enabled);

#endif // PRESET_STORAGE_H

