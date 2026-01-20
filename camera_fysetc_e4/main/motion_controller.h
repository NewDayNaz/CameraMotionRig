/**
 * @file motion_controller.h
 * @brief Simplified motion controller with direct velocity control
 */

#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "homing.h"
#include "preset_storage.h"

/**
 * @brief Initialize motion controller
 */
void motion_controller_init(void);

/**
 * @brief Update motion controller (call periodically)
 * @param dt Time delta since last update (seconds)
 */
void motion_controller_update(float dt);

/**
 * @brief Handle command to move to preset
 * @param preset_index Preset index
 * @return true if move started successfully
 */
bool motion_controller_goto_preset(uint8_t preset_index);

/**
 * @brief Handle command to save current position as preset
 * @param preset_index Preset index
 * @return true if saved successfully
 */
bool motion_controller_save_preset(uint8_t preset_index);

/**
 * @brief Get preset data
 * @param preset_index Preset index
 * @param preset Output preset structure
 * @return true if preset was loaded successfully
 */
bool motion_controller_get_preset(uint8_t preset_index, preset_t* preset);

/**
 * @brief Update preset with new data
 * @param preset_index Preset index
 * @param preset Preset structure with new data
 * @return true if updated successfully
 */
bool motion_controller_update_preset(uint8_t preset_index, const preset_t* preset);

/**
 * @brief Handle command to set velocities (manual mode)
 * @param velocities Velocities for each axis
 */
void motion_controller_set_velocities(const float velocities[3]);

/**
 * @brief Handle homing command
 * @param axis Axis to home (or 255 for all)
 * @return true if homing started
 */
bool motion_controller_home(uint8_t axis);

/**
 * @brief Get current positions
 */
void motion_controller_get_positions(float positions[3]);

/**
 * @brief Stop all motion
 */
void motion_controller_stop(void);

/**
 * @brief Set precision mode
 */
void motion_controller_set_precision_mode(bool enabled);

/**
 * @brief Set soft limits for an axis
 */
void motion_controller_set_limits(uint8_t axis, float min, float max);

#endif // MOTION_CONTROLLER_H

