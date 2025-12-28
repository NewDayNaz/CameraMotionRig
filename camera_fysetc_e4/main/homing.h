/**
 * @file homing.h
 * @brief Homing routines for PAN and TILT axes using endstops
 * 
 * Implements simple endstop homing:
 * 1. Fast approach toward min endstop until triggered
 * 2. Back off a fixed distance
 * 3. Slow re-approach until triggered
 * 4. Set axis position to known home value
 */

#ifndef HOMING_H
#define HOMING_H

#include <stdint.h>
#include <stdbool.h>
#include "board.h"

// Homing speeds (steps per second)
#define HOMING_FAST_SPEED 500.0f
#define HOMING_SLOW_SPEED  50.0f

// Configuration: Enable sensorless homing for zoom axis
// Set to 1 to use TMC2209 stallGuard (requires UART mode)
// Set to 0 to use physical endstop (PIN_Z_MIN / GPIO15)
#define ZOOM_USE_SENSORLESS_HOMING 0

// Backoff distance (steps)
#define HOMING_BACKOFF_STEPS 200

// Timeout for homing (seconds) - prevent infinite loops
#define HOMING_TIMEOUT_S 30.0f

/**
 * @brief Homing state machine
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
 * @brief Homing status structure
 */
typedef struct {
    homing_state_t state;
    uint8_t axis;  // Axis being homed
    float start_position;
    float backoff_target;
    bool endstop_triggered;
    float timeout_start_time;
    
    // Sequential homing support
    uint8_t axes_to_home[3];  // Queue of axes to home (PAN, TILT, ZOOM)
    uint8_t num_axes_to_home;  // Number of axes in queue
    uint8_t current_axis_index;  // Index in queue of current axis being homed
    
    // Sensorless homing support
    bool is_sensorless;  // true if using sensorless (stallGuard), false if physical endstop
    uint8_t stall_threshold;  // stallGuard threshold for sensorless homing
    uint8_t stall_readings;  // Consecutive stall readings for debouncing
    float stall_check_start_pos;  // Position when we started checking for stall (ignore stall if too close to start)
} homing_status_t;

/**
 * @brief Initialize homing system
 */
void homing_init(void);

/**
 * @brief Start homing sequence for an axis
 * @param axis Axis to home (AXIS_PAN or AXIS_TILT), or 255 for all axes sequentially
 * @return true if homing started, false if invalid axis or already homing
 */
bool homing_start(uint8_t axis);

/**
 * @brief Start sequential homing for multiple axes
 * @param axes Array of axis indices to home
 * @param num_axes Number of axes in the array
 * @return true if homing started
 */
bool homing_start_sequential(const uint8_t* axes, uint8_t num_axes);

/**
 * @brief Update homing state machine (call periodically)
 * @param dt Time delta since last update (seconds)
 * @param current_position Current position of the axis being homed
 * @param endstop_state Current endstop state (true = triggered) - only used for physical endstops
 * @return true if homing is in progress
 */
bool homing_update(float dt, float current_position, bool endstop_state);

/**
 * @brief Check for stall (sensorless homing)
 * @param axis Axis to check
 * @return true if stalled
 */
bool homing_check_stall(uint8_t axis);

/**
 * @brief Get current homing status
 */
homing_status_t homing_get_status(void);

/**
 * @brief Check if homing is in progress
 */
bool homing_is_active(void);

/**
 * @brief Abort homing sequence
 */
void homing_abort(void);

/**
 * @brief Get target velocity for homing move
 * @return Velocity in steps/sec (positive = toward endstop)
 */
float homing_get_target_velocity(void);

#endif // HOMING_H

