/**
 * @file board.h
 * @brief CNC Shield pin definitions and axis assignments for Arduino Uno R4 Minima
 * 
 * Pin mapping for standard Arduino CNC Shield with TMC2209 drivers
 * Hardware: https://blog.protoneer.co.nz/arduino-cnc-shield/
 */

#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>
#include <Arduino.h>

// Number of axes (PAN, TILT, ZOOM)
#define NUM_AXES 3

// Axis indices
#define AXIS_PAN  0
#define AXIS_TILT 1
#define AXIS_ZOOM 2

// CNC Shield Pin Definitions
// X-axis (PAN)
#define PIN_X_STEP  2
#define PIN_X_DIR   5
#define PIN_X_EN    8   // Enable pin (shared, or individual if using jumpers)

// Y-axis (TILT)
#define PIN_Y_STEP  3
#define PIN_Y_DIR   6
#define PIN_Y_EN    8   // Enable pin (shared)

// Z-axis (ZOOM)
#define PIN_Z_STEP  4
#define PIN_Z_DIR   7
#define PIN_Z_EN    8   // Enable pin (shared)

// Endstops (active LOW with INPUT_PULLUP)
#define PIN_X_MIN   9   // PAN min endstop
#define PIN_Y_MIN   10  // TILT min endstop
#define PIN_Z_MIN   -1  // ZOOM endstop (optional, -1 if not used)

// Axis to Pin Mapping Arrays
extern const uint8_t step_pins[NUM_AXES];
extern const uint8_t dir_pins[NUM_AXES];
extern const int8_t endstop_pins[NUM_AXES];  // -1 for axes without endstops

// Axis Names
extern const char* axis_names[NUM_AXES];

/**
 * @brief Initialize GPIO pins for stepper control
 */
void board_init(void);

/**
 * @brief Set stepper enable pin state (shared for all axes)
 * @param enable true to enable drivers, false to disable
 */
void board_set_enable(bool enable);

/**
 * @brief Read endstop state for an axis
 * @param axis Axis index (0=PAN, 1=TILT, 2=ZOOM)
 * @return true if endstop is triggered (active LOW), false otherwise
 */
bool board_read_endstop(uint8_t axis);

#endif // BOARD_H

