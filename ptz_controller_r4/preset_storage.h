/**
 * @file preset_storage.h
 * @brief Preset storage using Arduino EEPROM
 * 
 * Stores presets with cinematic parameters including:
 * - Position (pan, tilt, zoom)
 * - Easing type
 * - Duration or speed scale
 * - Arrival overshoot
 * - Approach mode
 */

#ifndef PRESET_STORAGE_H
#define PRESET_STORAGE_H

#include <stdint.h>
#include <stdbool.h>
#include "quintic.h"
#include "board.h"
#include <EEPROM.h>

// Maximum number of presets
#define MAX_PRESETS 16

// EEPROM magic number and version
#define EEPROM_MAGIC 0xCAFE
#define EEPROM_VERSION 1

/**
 * @brief Approach mode for preset moves
 */
typedef enum {
    APPROACH_DIRECT,      // Direct move to target
    APPROACH_HOME_FIRST,  // Move to home, then to target
    APPROACH_SAFE_ROUTE   // Safe route (e.g. lift tilt, pan, then tilt)
} approach_mode_t;

/**
 * @brief Preset structure
 */
typedef struct {
    float pos[NUM_AXES];           // Target positions
    easing_type_t easing_type;     // Easing for move
    float duration_s;              // Move duration in seconds (0 = auto)
    float max_speed_scale;         // Speed multiplier (0 = use duration)
    float arrival_overshoot;       // Overshoot amount (0.0-0.01 typical)
    approach_mode_t approach_mode; // How to approach this preset
    float speed_multiplier;        // Per-preset speed multiplier
    float accel_multiplier;        // Per-preset acceleration multiplier
    bool precision_preferred;      // Prefer precision mode for this preset
    bool valid;                    // Is this preset valid/initialized?
} preset_t;

/**
 * @brief Initialize preset storage (initialize EEPROM)
 */
void preset_storage_init(void);

/**
 * @brief Load a preset from EEPROM
 * @param index Preset index (0 to MAX_PRESETS-1)
 * @param preset Output preset structure
 * @return true if preset was loaded successfully
 */
bool preset_load(uint8_t index, preset_t* preset);

/**
 * @brief Save a preset to EEPROM
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

