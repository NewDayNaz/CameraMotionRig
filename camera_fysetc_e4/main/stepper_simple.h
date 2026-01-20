/**
 * @file stepper_simple.h
 * @brief Simple direct stepper control (based on camera_async approach)
 * 
 * Simplified stepper control without complex motion planner.
 * Direct step/dir control with velocity-based movement.
 */

#ifndef STEPPER_SIMPLE_H
#define STEPPER_SIMPLE_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize simple stepper control
 */
void stepper_simple_init(void);

/**
 * @brief Update stepper control (call periodically, e.g. every 1ms)
 */
void stepper_simple_update(void);

/**
 * @brief Set velocities for manual control
 * @param pan_vel Pan velocity (steps/sec)
 * @param tilt_vel Tilt velocity (steps/sec)
 * @param zoom_vel Zoom velocity (steps/sec)
 */
void stepper_simple_set_velocities(float pan_vel, float tilt_vel, float zoom_vel);

/**
 * @brief Get current positions
 */
void stepper_simple_get_positions(float* pan, float* tilt, float* zoom);

/**
 * @brief Stop all motion
 */
void stepper_simple_stop(void);

/**
 * @brief Move to preset position
 * @param preset_index Preset index (1-15, 0 is reserved)
 * @return true if move started successfully
 */
bool stepper_simple_goto_preset(uint8_t preset_index);

/**
 * @brief Save current position as preset
 * @param preset_index Preset index (1-15, 0 is reserved)
 * @return true if saved successfully
 */
bool stepper_simple_save_preset(uint8_t preset_index);

/**
 * @brief Start homing sequence
 */
void stepper_simple_home(void);

/**
 * @brief Set precision mode (reduces speeds)
 * @param enabled true to enable precision mode
 */
void stepper_simple_set_precision_mode(bool enabled);

/**
 * @brief Check if homing is currently active
 * @return true if homing sequence is in progress
 */
bool stepper_simple_is_homing(void);

#endif // STEPPER_SIMPLE_H
