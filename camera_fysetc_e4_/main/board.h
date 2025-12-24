/**
 * @file board.h
 * @brief FYSETC E4 board pin definitions and axis assignments
 * 
 * Pin mapping for FYSETC E4 board with ESP32 and TMC2209 drivers
 * Hardware: https://fysetc.github.io/E4/
 */

#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>
#include "driver/gpio.h"

// Number of axes (PAN, TILT, ZOOM)
#define NUM_AXES 3

// Axis indices
#define AXIS_PAN  0
#define AXIS_TILT 1
#define AXIS_ZOOM 2

// Stepper Motor Socket Pins (Step/Dir/Enable)
// X-MOTOR (used for PAN)
#define PIN_X_STEP   GPIO_NUM_27
#define PIN_X_DIR    GPIO_NUM_26
#define PIN_X_EN     GPIO_NUM_25  // Shared enable

// Y-MOTOR (used for TILT)
#define PIN_Y_STEP   GPIO_NUM_33
#define PIN_Y_DIR    GPIO_NUM_32
#define PIN_Y_EN     GPIO_NUM_25  // Shared enable

// Z-MOTOR (unused, reserved)
#define PIN_Z_STEP   GPIO_NUM_14
#define PIN_Z_DIR    GPIO_NUM_12
#define PIN_Z_EN     GPIO_NUM_25  // Shared enable

// E0-MOTOR (used for ZOOM)
#define PIN_E0_STEP  GPIO_NUM_16
#define PIN_E0_DIR   GPIO_NUM_17
#define PIN_E0_EN    GPIO_NUM_25  // Shared enable

// Endstops (MIN) - Active LOW (require external pullups on GPIO34/35)
#define PIN_X_MIN    GPIO_NUM_34  // PAN min endstop (input-only, no pullup)
#define PIN_Y_MIN    GPIO_NUM_35  // TILT min endstop (input-only, no pullup)
#define PIN_Z_MIN    GPIO_NUM_15  // ZOOM min endstop (can be used with external pullup, or use sensorless)

// Motor UART (Optional TMC configuration)
#define PIN_UART1_TX GPIO_NUM_22
#define PIN_UART1_RX GPIO_NUM_21

// Axis to Pin Mapping Arrays
extern const gpio_num_t step_pins[NUM_AXES];
extern const gpio_num_t dir_pins[NUM_AXES];

// Endstop pin mapping
extern const gpio_num_t endstop_pins[NUM_AXES];  // -1 for axes without endstops

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

#endif // BOARD_H

