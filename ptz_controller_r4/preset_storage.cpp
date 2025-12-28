/**
 * @file preset_storage.cpp
 * @brief Preset storage implementation using EEPROM
 */

#include "preset_storage.h"
#include <string.h>

// EEPROM layout:
// Address 0-1: Magic number (0xCAFE)
// Address 2: Version
// Address 3-2+sizeof(preset_t)*MAX_PRESETS: Preset data

#define EEPROM_MAGIC_ADDR 0
#define EEPROM_VERSION_ADDR 2
#define EEPROM_PRESET_START 3

// Calculate size of preset structure (packed)
// Note: This is approximate - actual size depends on compiler packing
// For Arduino, we'll use a fixed size structure
#define PRESET_STRUCT_SIZE 64  // Fixed size to ensure consistency

void preset_storage_init(void) {
    // Check if EEPROM is initialized
    uint16_t magic = EEPROM.read(EEPROM_MAGIC_ADDR) | (EEPROM.read(EEPROM_MAGIC_ADDR + 1) << 8);
    uint8_t version = EEPROM.read(EEPROM_VERSION_ADDR);
    
    if (magic != EEPROM_MAGIC || version != EEPROM_VERSION) {
        // Initialize EEPROM
        EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC & 0xFF);
        EEPROM.write(EEPROM_MAGIC_ADDR + 1, (EEPROM_MAGIC >> 8) & 0xFF);
        EEPROM.write(EEPROM_VERSION_ADDR, EEPROM_VERSION);
        
        // Clear all presets
        for (uint8_t i = 0; i < MAX_PRESETS; i++) {
            preset_t empty;
            memset(&empty, 0, sizeof(preset_t));
            empty.valid = false;
            preset_save(i, &empty);
        }
    }
}

static void preset_to_eeprom(uint8_t index, const preset_t* preset) {
    uint16_t addr = EEPROM_PRESET_START + index * PRESET_STRUCT_SIZE;
    
    // Write positions (3 floats = 12 bytes)
    uint8_t* pos_bytes = (uint8_t*)preset->pos;
    for (int i = 0; i < 12; i++) {
        EEPROM.write(addr + i, pos_bytes[i]);
    }
    
    // Write easing type (1 byte)
    EEPROM.write(addr + 12, (uint8_t)preset->easing_type);
    
    // Write duration (4 bytes float)
    uint8_t* dur_bytes = (uint8_t*)&preset->duration_s;
    for (int i = 0; i < 4; i++) {
        EEPROM.write(addr + 13 + i, dur_bytes[i]);
    }
    
    // Write max_speed_scale (4 bytes float)
    uint8_t* speed_bytes = (uint8_t*)&preset->max_speed_scale;
    for (int i = 0; i < 4; i++) {
        EEPROM.write(addr + 17 + i, speed_bytes[i]);
    }
    
    // Write arrival_overshoot (4 bytes float)
    uint8_t* overshoot_bytes = (uint8_t*)&preset->arrival_overshoot;
    for (int i = 0; i < 4; i++) {
        EEPROM.write(addr + 21 + i, overshoot_bytes[i]);
    }
    
    // Write approach_mode (1 byte)
    EEPROM.write(addr + 25, (uint8_t)preset->approach_mode);
    
    // Write speed_multiplier (4 bytes float)
    uint8_t* sm_bytes = (uint8_t*)&preset->speed_multiplier;
    for (int i = 0; i < 4; i++) {
        EEPROM.write(addr + 26 + i, sm_bytes[i]);
    }
    
    // Write accel_multiplier (4 bytes float)
    uint8_t* am_bytes = (uint8_t*)&preset->accel_multiplier;
    for (int i = 0; i < 4; i++) {
        EEPROM.write(addr + 30 + i, am_bytes[i]);
    }
    
    // Write precision_preferred (1 byte)
    EEPROM.write(addr + 34, preset->precision_preferred ? 1 : 0);
    
    // Write valid flag (1 byte)
    EEPROM.write(addr + 35, preset->valid ? 1 : 0);
    
    // Remaining bytes unused (for future expansion)
}

static void preset_from_eeprom(uint8_t index, preset_t* preset) {
    uint16_t addr = EEPROM_PRESET_START + index * PRESET_STRUCT_SIZE;
    
    // Read positions (3 floats = 12 bytes)
    uint8_t* pos_bytes = (uint8_t*)preset->pos;
    for (int i = 0; i < 12; i++) {
        pos_bytes[i] = EEPROM.read(addr + i);
    }
    
    // Read easing type (1 byte)
    preset->easing_type = (easing_type_t)EEPROM.read(addr + 12);
    
    // Read duration (4 bytes float)
    uint8_t* dur_bytes = (uint8_t*)&preset->duration_s;
    for (int i = 0; i < 4; i++) {
        dur_bytes[i] = EEPROM.read(addr + 13 + i);
    }
    
    // Read max_speed_scale (4 bytes float)
    uint8_t* speed_bytes = (uint8_t*)&preset->max_speed_scale;
    for (int i = 0; i < 4; i++) {
        speed_bytes[i] = EEPROM.read(addr + 17 + i);
    }
    
    // Read arrival_overshoot (4 bytes float)
    uint8_t* overshoot_bytes = (uint8_t*)&preset->arrival_overshoot;
    for (int i = 0; i < 4; i++) {
        overshoot_bytes[i] = EEPROM.read(addr + 21 + i);
    }
    
    // Read approach_mode (1 byte)
    preset->approach_mode = (approach_mode_t)EEPROM.read(addr + 25);
    
    // Read speed_multiplier (4 bytes float)
    uint8_t* sm_bytes = (uint8_t*)&preset->speed_multiplier;
    for (int i = 0; i < 4; i++) {
        sm_bytes[i] = EEPROM.read(addr + 26 + i);
    }
    
    // Read accel_multiplier (4 bytes float)
    uint8_t* am_bytes = (uint8_t*)&preset->accel_multiplier;
    for (int i = 0; i < 4; i++) {
        am_bytes[i] = EEPROM.read(addr + 30 + i);
    }
    
    // Read precision_preferred (1 byte)
    preset->precision_preferred = (EEPROM.read(addr + 34) != 0);
    
    // Read valid flag (1 byte)
    preset->valid = (EEPROM.read(addr + 35) != 0);
}

bool preset_load(uint8_t index, preset_t* preset) {
    if (index >= MAX_PRESETS) {
        return false;
    }
    
    preset_from_eeprom(index, preset);
    return preset->valid;
}

bool preset_save(uint8_t index, const preset_t* preset) {
    if (index >= MAX_PRESETS) {
        return false;
    }
    
    preset_to_eeprom(index, preset);
    return true;
}

bool preset_delete(uint8_t index) {
    if (index >= MAX_PRESETS) {
        return false;
    }
    
    preset_t empty;
    memset(&empty, 0, sizeof(preset_t));
    empty.valid = false;
    return preset_save(index, &empty);
}

void preset_init_default(preset_t* preset) {
    memset(preset, 0, sizeof(preset_t));
    preset->easing_type = EASING_SMOOTHERSTEP;
    preset->duration_s = 0.0f;  // Auto-calculate
    preset->max_speed_scale = 0.0f;  // Use duration
    preset->arrival_overshoot = 0.0f;
    preset->approach_mode = APPROACH_DIRECT;
    preset->speed_multiplier = 1.0f;
    preset->accel_multiplier = 1.0f;
    preset->precision_preferred = false;
    preset->valid = true;
}

bool preset_is_valid(uint8_t index) {
    if (index >= MAX_PRESETS) {
        return false;
    }
    
    preset_t preset;
    preset_from_eeprom(index, &preset);
    return preset.valid;
}

