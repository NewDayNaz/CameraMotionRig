/**
 * @file homing.h
 * @brief Endstop-based homing for PAN and TILT axes
 * 
 * Implements simple endstop homing:
 * 1) Fast approach toward min endstop until triggered
 * 2) Back off a fixed distance
 * 3) Slow re-approach until triggered
 * 4) Set axis position to known home value
 */

#ifndef HOMING_H
#define HOMING_H

#include <stdint.h>
#include <stdbool.h>
#include "board.h"
#include "motion_planner.h"

/**
 * @brief Homing state
 */
typedef enum {
    HOMING_IDLE,
    HOMING_FAST_APPROACH,
    HOMING_BACKOFF,
    HOMING_SLOW_APPROACH,
    HOMING_COMPLETE,
    HOMING_ERROR
} homing_state_t;

/**
 * @brief Homing configuration
 */
typedef struct {
    float fast_velocity;      // Fast approach velocity (steps/sec)
    float slow_velocity;      // Slow approach velocity (steps/sec)
    float backoff_distance;   // Distance to back off (steps)
    int32_t home_position;    // Position to set when homed (typically 0)
    uint32_t timeout_ms;       // Maximum time for homing (0 = no timeout)
} homing_config_t;

/**
 * @brief Homing context
 */
typedef struct {
    homing_state_t state;
    homing_config_t config;
    uint8_t axis;
    float current_velocity;
    uint32_t start_time_ms;
    bool triggered;
} homing_context_t;

/**
 * @brief Initialize homing system
 * @param planner Motion planner to use for homing moves
 */
void homing_init(motion_planner_t* planner);

/**
 * @brief Start homing sequence for an axis
 * @param axis Axis to home (AXIS_PAN or AXIS_TILT)
 * @param config Homing configuration (NULL for defaults)
 * @return true if homing started successfully
 */
bool homing_start(uint8_t axis, const homing_config_t* config);

/**
 * @brief Update homing state (call periodically)
 * @param dt Time delta since last update (seconds)
 * @return true if homing is complete or idle
 */
bool homing_update(float dt);

/**
 * @brief Check if homing is in progress
 */
bool homing_is_active(void);

/**
 * @brief Get homing state
 */
homing_state_t homing_get_state(void);

/**
 * @brief Stop homing (emergency stop)
 */
void homing_stop(void);

#endif // HOMING_H

