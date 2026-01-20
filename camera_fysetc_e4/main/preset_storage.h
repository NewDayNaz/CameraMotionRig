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

// Maximum number of presets
#define MAX_PRESETS 16

/**
 * @brief Preset structure (simplified)
 */
typedef struct {
    float pos[NUM_AXES];      // Target positions (pan, tilt, zoom)
    float max_speed;          // Maximum speed for this move (steps/sec, 0 = use default)
    float accel_factor;       // Acceleration factor (1.0 = normal, >1.0 = faster accel, <1.0 = slower accel)
    float decel_factor;       // Deceleration factor (1.0 = normal, >1.0 = faster decel, <1.0 = slower decel) - most important for accuracy
    bool valid;               // Is this preset valid/initialized?
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

/**
 * @brief Check if a preset is valid
 */
bool preset_is_valid(uint8_t index);

#endif // PRESET_STORAGE_H

