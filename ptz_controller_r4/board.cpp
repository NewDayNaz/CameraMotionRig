/**
 * @file board.cpp
 * @brief Board initialization and pin control implementation
 */

#include "board.h"

// Pin mapping arrays
const uint8_t step_pins[NUM_AXES] = {
    PIN_X_STEP,  // PAN
    PIN_Y_STEP,  // TILT
    PIN_Z_STEP   // ZOOM
};

const uint8_t dir_pins[NUM_AXES] = {
    PIN_X_DIR,   // PAN
    PIN_Y_DIR,   // TILT
    PIN_Z_DIR    // ZOOM
};

const int8_t endstop_pins[NUM_AXES] = {
    PIN_X_MIN,   // PAN
    PIN_Y_MIN,   // TILT
    PIN_Z_MIN    // ZOOM (-1 if not used)
};

const char* axis_names[NUM_AXES] = {
    "PAN",
    "TILT",
    "ZOOM"
};

void board_init(void) {
    // Initialize step pins as outputs
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        pinMode(step_pins[i], OUTPUT);
        digitalWrite(step_pins[i], LOW);
    }
    
    // Initialize direction pins as outputs
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        pinMode(dir_pins[i], OUTPUT);
        digitalWrite(dir_pins[i], LOW);
    }
    
    // Initialize enable pin
    pinMode(PIN_X_EN, OUTPUT);
    board_set_enable(false);  // Start with drivers disabled
    
    // Initialize endstop pins as inputs with pullup
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        if (endstop_pins[i] >= 0) {
            pinMode(endstop_pins[i], INPUT_PULLUP);
        }
    }
}

void board_set_enable(bool enable) {
    // Enable pin is active LOW on most CNC shields
    digitalWrite(PIN_X_EN, enable ? LOW : HIGH);
}

bool board_read_endstop(uint8_t axis) {
    if (axis >= NUM_AXES) {
        return false;
    }
    
    if (endstop_pins[axis] < 0) {
        return false;  // No endstop for this axis
    }
    
    // Endstop is active LOW (pressed = LOW)
    return (digitalRead(endstop_pins[axis]) == LOW);
}

