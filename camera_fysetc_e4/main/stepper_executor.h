/**
 * @file stepper_executor.h
 * @brief Deterministic step generation using GPTimer ISR
 * 
 * Uses ESP-IDF GPTimer to generate step pulses at high frequency (20-40kHz).
 * Executes segments from the queue using DDA/Bresenham style distribution.
 */

#ifndef STEPPER_EXECUTOR_H
#define STEPPER_EXECUTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "segment.h"

/**
 * @brief Initialize the stepper executor
 * @param queue Pointer to the segment queue to consume from
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

