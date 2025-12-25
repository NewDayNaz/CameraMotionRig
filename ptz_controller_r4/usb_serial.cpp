/**
 * @file usb_serial.cpp
 * @brief USB serial command parser implementation
 */

#include "usb_serial.h"
#include "motion_planner.h"
#include "homing.h"
#include "preset_storage.h"
#include "stepper_executor.h"
#include <Arduino.h>
#include <string.h>
#include <ctype.h>

// Forward declarations - these will be set by main
extern motion_planner_t* g_planner;
extern bool g_precision_mode;

// Serial buffer
#define SERIAL_BUFFER_SIZE 128
static char s_serial_buffer[SERIAL_BUFFER_SIZE];
static int s_buffer_index = 0;

// Forward declaration
static bool parse_command(const char* cmd);

void usb_serial_init(void) {
    Serial.begin(115200);
    s_buffer_index = 0;
}

bool usb_serial_process(void) {
    while (Serial.available() > 0) {
        char c = Serial.read();
        
        if (c == '\n' || c == '\r') {
            if (s_buffer_index > 0) {
                s_serial_buffer[s_buffer_index] = '\0';
                
                // Parse command
                if (parse_command(s_serial_buffer)) {
                    s_buffer_index = 0;
                    return true;
                }
                
                s_buffer_index = 0;
            }
        } else if (s_buffer_index < SERIAL_BUFFER_SIZE - 1) {
            s_serial_buffer[s_buffer_index++] = c;
        } else {
            // Buffer overflow - clear
            s_buffer_index = 0;
        }
    }
    
    return false;
}

static bool parse_command(const char* cmd) {
    // Tokenize command
    char cmd_copy[SERIAL_BUFFER_SIZE];
    strncpy(cmd_copy, cmd, SERIAL_BUFFER_SIZE);
    cmd_copy[SERIAL_BUFFER_SIZE - 1] = '\0';
    
    char* token = strtok(cmd_copy, " ");
    if (token == nullptr) {
        return false;
    }
    
    // Convert to uppercase for case-insensitive matching
    for (int i = 0; token[i]; i++) {
        token[i] = toupper(token[i]);
    }
    
    if (strcmp(token, "VEL") == 0) {
        // VEL <pan> <tilt> <zoom>
        float velocities[NUM_AXES] = {0.0f, 0.0f, 0.0f};
        
        for (int i = 0; i < NUM_AXES; i++) {
            token = strtok(nullptr, " ");
            if (token == nullptr) {
                usb_serial_send_error("VEL: Missing velocity values");
                return true;
            }
            velocities[i] = atof(token);
        }
        
        motion_planner_set_velocities(g_planner, velocities);
        motion_planner_set_manual_mode(g_planner, true);
        Serial.println("OK");
        return true;
        
    } else if (strcmp(token, "GOTO") == 0) {
        // GOTO <n>
        token = strtok(nullptr, " ");
        if (token == nullptr) {
            usb_serial_send_error("GOTO: Missing preset number");
            return true;
        }
        
        uint8_t preset_index = atoi(token);
        if (preset_index >= MAX_PRESETS) {
            usb_serial_send_error("GOTO: Invalid preset number");
            return true;
        }
        
        preset_t preset;
        if (!preset_load(preset_index, &preset)) {
            usb_serial_send_error("GOTO: Preset not found");
            return true;
        }
        
        // Plan move to preset position
        motion_planner_set_manual_mode(g_planner, false);
        if (motion_planner_plan_move(g_planner, preset.pos, preset.duration_s, preset.easing_type)) {
            Serial.println("OK");
        } else {
            usb_serial_send_error("GOTO: Move planning failed");
        }
        return true;
        
    } else if (strcmp(token, "SAVE") == 0) {
        // SAVE <n>
        token = strtok(nullptr, " ");
        if (token == nullptr) {
            usb_serial_send_error("SAVE: Missing preset number");
            return true;
        }
        
        uint8_t preset_index = atoi(token);
        if (preset_index >= MAX_PRESETS) {
            usb_serial_send_error("SAVE: Invalid preset number");
            return true;
        }
        
        preset_t preset;
        preset_init_default(&preset);
        
        // Get current positions
        for (int i = 0; i < NUM_AXES; i++) {
            preset.pos[i] = motion_planner_get_position(g_planner, i);
        }
        
        if (preset_save(preset_index, &preset)) {
            Serial.println("OK");
        } else {
            usb_serial_send_error("SAVE: Failed to save preset");
        }
        return true;
        
    } else if (strcmp(token, "HOME") == 0) {
        // HOME [axis]
        token = strtok(nullptr, " ");
        uint8_t axis = AXIS_PAN;  // Default to PAN
        
        if (token != nullptr) {
            // Parse axis name
            for (int i = 0; token[i]; i++) {
                token[i] = toupper(token[i]);
            }
            if (strcmp(token, "TILT") == 0) {
                axis = AXIS_TILT;
            } else if (strcmp(token, "PAN") == 0) {
                axis = AXIS_PAN;
            }
        }
        
        if (homing_start(axis, nullptr)) {
            Serial.println("OK");
        } else {
            usb_serial_send_error("HOME: Homing failed to start");
        }
        return true;
        
    } else if (strcmp(token, "STOP") == 0) {
        motion_planner_stop(g_planner);
        homing_stop();
        Serial.println("OK");
        return true;
        
    } else if (strcmp(token, "PRECISION") == 0) {
        // PRECISION <0|1>
        token = strtok(nullptr, " ");
        if (token == nullptr) {
            usb_serial_send_error("PRECISION: Missing value");
            return true;
        }
        
        g_precision_mode = (atoi(token) != 0);
        motion_planner_set_precision_mode(g_planner, g_precision_mode);
        Serial.println("OK");
        return true;
        
    } else if (strcmp(token, "POS") == 0) {
        // Return current positions
        Serial.print("POS ");
        for (int i = 0; i < NUM_AXES; i++) {
            Serial.print(motion_planner_get_position(g_planner, i));
            if (i < NUM_AXES - 1) Serial.print(" ");
        }
        Serial.println();
        return true;
        
    } else if (strcmp(token, "STATUS") == 0) {
        // Return system status
        Serial.print("STATUS ");
        Serial.print(motion_planner_is_busy(g_planner) ? "BUSY" : "IDLE");
        Serial.print(" ");
        Serial.print(homing_is_active() ? "HOMING" : "READY");
        Serial.print(" ");
        Serial.print(g_precision_mode ? "1" : "0");
        Serial.println();
        return true;
    }
    
    return false;
}

void usb_serial_send_status(const char* message) {
    Serial.print("STATUS: ");
    Serial.println(message);
}

void usb_serial_send_error(const char* message) {
    Serial.print("ERROR: ");
    Serial.println(message);
}

