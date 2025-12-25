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

// Z-MOTOR (used for ZOOM)
#define PIN_Z_STEP   GPIO_NUM_14
#define PIN_Z_DIR    GPIO_NUM_12
#define PIN_Z_EN     GPIO_NUM_25  // Shared enable

// E0-MOTOR (unused, reserved)
#define PIN_E0_STEP  GPIO_NUM_16
#define PIN_E0_DIR   GPIO_NUM_17
#define PIN_E0_EN    GPIO_NUM_25  // Shared enable

// Endstops (MIN) - Active LOW (require external pullups on GPIO34/35)
// Each axis uses its own endstop pin
#define PIN_X_MIN    GPIO_NUM_15  // PAN min endstop (X axis limit_neg)
#define PIN_Y_MIN    GPIO_NUM_35  // TILT min endstop (Y axis limit_neg)
#define PIN_Z_MIN    GPIO_NUM_34  // ZOOM min endstop (Z axis limit_neg)

// Motor UART (TMC2209 configuration)
#define PIN_UART1_TX GPIO_NUM_22
#define PIN_UART1_RX GPIO_NUM_21
#define TMC2209_UART_BAUD 115200

// TMC2209 Driver UART Addresses (based on FluidNC config)
// X axis (PAN): addr=1
// Y axis (TILT): addr=3
// Z axis (ZOOM): addr=0
#define TMC2209_ADDR_PAN   1  // X axis driver address
#define TMC2209_ADDR_TILT  3  // Y axis driver address
#define TMC2209_ADDR_ZOOM  0  // Z axis driver address

// Axis to Pin Mapping Arrays
extern const gpio_num_t step_pins[NUM_AXES];
extern const gpio_num_t dir_pins[NUM_AXES];

// Endstop pin mapping
extern const gpio_num_t endstop_pins[NUM_AXES];  // -1 for axes without endstops

// Axis Names
extern const char* axis_names[NUM_AXES];

// TMC2209 driver address mapping (indexed by axis)
extern const uint8_t tmc2209_addresses[NUM_AXES];

/**
 * @brief Get TMC2209 driver UART address for an axis
 * @param axis Axis index (AXIS_PAN, AXIS_TILT, AXIS_ZOOM)
 * @return TMC2209 driver address (0-3)
 */
uint8_t board_get_tmc2209_address(uint8_t axis);

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

