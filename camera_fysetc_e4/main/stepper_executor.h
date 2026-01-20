/**
 * @file stepper_executor.h
 * @brief Simplified stepper executor with direct velocity control
 * 
 * Uses ESP-IDF GPTimer ISR for responsive joystick control.
 * Supports both direct velocity control (manual mode) and segment queue (preset moves).
 */

#ifndef STEPPER_EXECUTOR_H
#define STEPPER_EXECUTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "segment.h"

/**
 * @brief Initialize the stepper executor
 * @param queue Pointer to the segment queue (for preset moves, can be NULL if only using manual mode)
 * @return true on success, false on failure
 */
bool stepper_executor_init(segment_queue_t* queue);

/**
 * @brief Start the stepper executor timer
 */
void stepper_executor_start(void);

/**
 * @brief Stop the stepper executor timer
 */
void stepper_executor_stop(void);

/**
 * @brief Set velocity for manual/joystick mode (direct control, bypasses queue)
 * @param axis Axis index (0=PAN, 1=TILT, 2=ZOOM)
 * @param velocity Velocity in steps/second (can be negative for reverse)
 */
void stepper_executor_set_velocity(uint8_t axis, float velocity);

/**
 * @brief Enable/disable manual velocity mode
 * @param enabled true to enable manual mode, false to use segment queue
 */
void stepper_executor_set_manual_mode(bool enabled);

/**
 * @brief Get current position of an axis
 * @param axis Axis index (0=PAN, 1=TILT, 2=ZOOM)
 * @return Current position in steps
 */
int32_t stepper_executor_get_position(uint8_t axis);

/**
 * @brief Set position of an axis (for homing/calibration)
 * @param axis Axis index
 * @param position New position value
 */
void stepper_executor_set_position(uint8_t axis, int32_t position);

/**
 * @brief Check if executor is currently executing a segment
 */
bool stepper_executor_is_busy(void);

#endif // STEPPER_EXECUTOR_H

