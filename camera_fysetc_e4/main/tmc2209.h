/**
 * @file tmc2209.h
 * @brief TMC2209 stepper driver UART interface for sensorless homing
 * 
 * Provides UART communication with TMC2209 drivers for:
 * - Configuring stallGuard parameters
 * - Reading stallGuard values
 * - Detecting motor stalls
 */

#ifndef TMC2209_H
#define TMC2209_H

#include <stdint.h>
#include <stdbool.h>
#include "board.h"
#include "driver/uart.h"

// TMC2209 UART configuration
#define TMC2209_UART_NUM        UART_NUM_1
#define TMC2209_BAUD_RATE       115200
#define TMC2209_UART_BUF_SIZE   128

// TMC2209 Register Addresses
#define TMC2209_GCONF           0x00
#define TMC2209_GSTAT           0x01
#define TMC2209_IFCNT           0x02
#define TMC2209_SLAVECONF       0x03
#define TMC2209_IHOLD_IRUN      0x10
#define TMC2209_TSTEP           0x12
#define TMC2209_TPWMTHRS        0x13
#define TMC2209_TCOOLTHRS       0x14
#define TMC2209_XACTUAL         0x21
#define TMC2209_SGTHRS          0x40  // stallGuard threshold
#define TMC2209_SG_RESULT       0x41  // stallGuard result (0-255, lower = more load)
#define TMC2209_COOLCONF        0x42

// Default stallGuard threshold (0-255, lower = more sensitive)
// This needs to be calibrated for your specific motor/load
#define TMC2209_DEFAULT_SGTHRS  150

// Default TCOOLTHRS (minimum velocity for stallGuard, steps/sec)
// Must be lower than slow homing speed (50 steps/sec) for stallGuard to work during slow approach
#define TMC2209_DEFAULT_TCOOLTHRS 30

/**
 * @brief Initialize TMC2209 UART communication
 * @param uart_num UART peripheral number (e.g., UART_NUM_1)
 * @return true on success
 */
bool tmc2209_uart_init(uart_port_t uart_num);

/**
 * @brief Initialize TMC2209 driver for an axis
 * @param axis Axis index (AXIS_PAN, AXIS_TILT, AXIS_ZOOM)
 * @return true on success
 */
bool tmc2209_init(uint8_t axis);

/**
 * @brief Write to a TMC2209 register
 * @param axis Axis index
 * @param address Register address
 * @param data 32-bit data to write
 * @return true on success
 */
bool tmc2209_write_register(uint8_t axis, uint8_t address, uint32_t data);

/**
 * @brief Read from a TMC2209 register
 * @param axis Axis index
 * @param address Register address
 * @param data Pointer to store 32-bit read data
 * @return true on success
 */
bool tmc2209_read_register(uint8_t axis, uint8_t address, uint32_t* data);

/**
 * @brief Set stallGuard threshold
 * @param axis Axis index
 * @param threshold Threshold value (0-255, lower = more sensitive)
 */
void tmc2209_set_stallguard_threshold(uint8_t axis, uint8_t threshold);

/**
 * @brief Get stallGuard result (current load reading)
 * @param axis Axis index
 * @return stallGuard result (0-255, lower = more load/stall)
 */
uint8_t tmc2209_get_stallguard_result(uint8_t axis);

/**
 * @brief Check if motor is stalled based on stallGuard reading
 * @param axis Axis index
 * @param threshold Stall threshold (if SG_RESULT < threshold, consider stalled)
 * @return true if stalled
 */
bool tmc2209_is_stalled(uint8_t axis, uint8_t threshold);

/**
 * @brief Enable stallGuard for an axis
 * @param axis Axis index
 * @param enable true to enable, false to disable
 */
void tmc2209_enable_stallguard(uint8_t axis, bool enable);

/**
 * @brief Set motor current
 * @param axis Axis index
 * @param hold_current Current during standstill (0-31)
 * @param run_current Current during operation (0-31)
 */
void tmc2209_set_current(uint8_t axis, uint8_t hold_current, uint8_t run_current);

/**
 * @brief Set TCOOLTHRS (minimum velocity for stallGuard)
 * @param axis Axis index
 * @param threshold Velocity threshold in steps/sec
 */
void tmc2209_set_coolthrs(uint8_t axis, uint32_t threshold);

#endif // TMC2209_H

