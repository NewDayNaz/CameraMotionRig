/**
 * @file motion_planner.h
 * @brief Motion planner that generates segments from quintic trajectories
 * 
 * Converts high-level motion commands (waypoints, velocities) into
 * motion segments that are fed to the stepper executor.
 */

#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

#include <stdint.h>
#include <stdbool.h>
#include "segment.h"
#include "quintic.h"

// Maximum velocity (steps per second) - for manual/joystick control
#define MAX_VELOCITY_PAN  500.0f   // Reduced for testing
#define MAX_VELOCITY_TILT 500.0f   // Reduced for testing
#define MAX_VELOCITY_ZOOM 50.0f    // Very slow for zoom control (small travel range)

// Maximum acceleration (steps per second squared) - for manual/joystick control
#define MAX_ACCEL_PAN  250.0f      // Reduced proportionally for testing
#define MAX_ACCEL_TILT 250.0f      // Reduced proportionally for testing
#define MAX_ACCEL_ZOOM 25.0f       // Reduced proportionally for zoom

// Preset move limits (very conservative for cinematic moves)
// Quintic curves can have high peak velocities (1.5-2x average), so use very low limits
// These are 10% of manual max to ensure smooth, cinematic motion without step skipping
// With 16x microstepping: 200 steps/sec = 3,200 microsteps/sec (very safe for cinematic moves)
#define PRESET_MAX_VELOCITY_PAN   200.0f   // 10% of manual max
#define PRESET_MAX_VELOCITY_TILT  200.0f   // 10% of manual max
#define PRESET_MAX_VELOCITY_ZOOM  5.0f     // 10% of manual max (very slow for zoom)

#define PRESET_MAX_ACCEL_PAN    100.0f     // 10% of manual max
#define PRESET_MAX_ACCEL_TILT   100.0f     // 10% of manual max
#define PRESET_MAX_ACCEL_ZOOM    2.5f      // 10% of manual max (very slow for zoom)

// Soft limit zones (percentage of travel)
#define SOFT_LIMIT_ZONE 0.05f  // Last 5% of travel

// Pan axis limits (degrees)
// Maximum pan range: 240 degrees total (±120 degrees from center/home position)
#define PAN_MAX_DEGREES 240.0f
// Steps per degree for pan axis (adjust based on your gear ratio)
// Typical values: 200 steps/rev motor = 200/360 = 0.556 steps/degree
// With gear reduction, multiply by gear ratio (e.g., 10:1 = 5.56 steps/degree)
#define PAN_STEPS_PER_DEGREE 100.0f  // Adjust this to match your hardware

// Tilt axis limits (degrees)
// Maximum tilt range: separate limits for down (negative) and up (positive)
// Down angle is typically less than up angle
#define TILT_MAX_DEGREES_DOWN 20210.0f  // Maximum tilt down (negative direction)
#define TILT_MAX_DEGREES_UP 27296.0f    // Maximum tilt up (positive direction)
// Steps per degree for tilt axis (adjust based on your gear ratio)
// Typical values: 200 steps/rev motor = 200/360 = 0.556 steps/degree
// With gear reduction, multiply by gear ratio (e.g., 10:1 = 5.56 steps/degree)
#define TILT_STEPS_PER_DEGREE 0.556f  // Adjust this to match your hardware

/**
 * @brief Motion planner state
 */
typedef struct {
    segment_queue_t* queue;
    
    // Current positions (in steps, float for sub-step precision)
    float positions[NUM_AXES];
    
    // Target positions for waypoint moves
    float targets[NUM_AXES];
    
    // Current velocities (for manual/joystick mode)
    float velocities[NUM_AXES];
    
    // Soft limits
    float limits_min[NUM_AXES];
    float limits_max[NUM_AXES];
    
    // Max speeds and accelerations
    float max_velocity[NUM_AXES];
    float max_accel[NUM_AXES];
    
    // Precision mode multiplier (0.25 = 25% speed)
    float precision_multiplier;
    bool precision_mode;
    
    // Active move state
    bool move_in_progress;
    quintic_coeffs_t move_coeffs[NUM_AXES];
    float move_start_time;
    float move_duration;
    easing_type_t move_easing;
    
    // Manual velocity mode state
    bool manual_mode;
    float manual_slew_limit[NUM_AXES];  // Current velocity after slew limiting
    float fractional_step_accum[NUM_AXES];  // Fractional step accumulator for smooth low-speed motion
} motion_planner_t;

/**
 * @brief Initialize motion planner
 * @param planner Planner structure
 * @param queue Segment queue to feed
 */
void motion_planner_init(motion_planner_t* planner, segment_queue_t* queue);

/**
 * @brief Set current position of an axis
 */
void motion_planner_set_position(motion_planner_t* planner, uint8_t axis, float position);

/**
 * @brief Get current position of an axis
 */
float motion_planner_get_position(const motion_planner_t* planner, uint8_t axis);

/**
 * @brief Set soft limits for an axis
 */
void motion_planner_set_limits(motion_planner_t* planner, uint8_t axis, float min, float max);

/**
 * @brief Plan a quintic move to target positions
 * @param planner Planner structure
 * @param targets Target positions for each axis
 * @param duration Move duration in seconds (0 = auto-calculate)
 * @param easing Easing type to apply
 * @return true if planning started successfully
 */
bool motion_planner_plan_move(motion_planner_t* planner, const float targets[NUM_AXES], 
                              float duration, easing_type_t easing);

/**
 * @brief Set velocity for manual/joystick mode
 * @param planner Planner structure
 * @param velocities Target velocities for each axis (steps/sec)
 */
void motion_planner_set_velocities(motion_planner_t* planner, const float velocities[NUM_AXES]);

/**
 * @brief Enable/disable manual velocity mode
 */
void motion_planner_set_manual_mode(motion_planner_t* planner, bool enabled);

/**
 * @brief Enable/disable precision mode (slower speeds)
 */
void motion_planner_set_precision_mode(motion_planner_t* planner, bool enabled);

/**
 * @brief Update planner (call periodically to generate segments)
 * @param planner Planner structure
 * @param dt Time delta since last update (seconds)
 */
void motion_planner_update(motion_planner_t* planner, float dt);

/**
 * @brief Check if planner is currently executing a move
 */
bool motion_planner_is_busy(const motion_planner_t* planner);

/**
 * @brief Stop current motion (emergency stop)
 */
void motion_planner_stop(motion_planner_t* planner);

#endif // MOTION_PLANNER_H

