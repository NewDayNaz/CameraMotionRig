/**
 * @file motion_planner.cpp
 * @brief Motion planner implementation
 */

#include "motion_planner.h"
#include "board.h"
#include "segment.h"
#include <string.h>
#include <math.h>

// Slew rate limit (steps/sec^2) - 1/3 to 1/2 of max acceleration
#define SLEW_RATE_LIMIT_PAN  (MAX_ACCEL_PAN / 2.0f)
#define SLEW_RATE_LIMIT_TILT (MAX_ACCEL_TILT / 2.0f)
#define SLEW_RATE_LIMIT_ZOOM (MAX_ACCEL_ZOOM / 2.0f)

static const float slew_rate_limits[NUM_AXES] = {
    SLEW_RATE_LIMIT_PAN,
    SLEW_RATE_LIMIT_TILT,
    SLEW_RATE_LIMIT_ZOOM
};

/**
 * @brief Apply deadband and expo curve to joystick input
 */
static float apply_joystick_filter(float input) {
    // Deadband
    float abs_input = fabsf(input);
    if (abs_input < JOYSTICK_DEADBAND) {
        return 0.0f;
    }
    
    // Normalize after deadband
    float sign = (input >= 0.0f) ? 1.0f : -1.0f;
    float normalized = (abs_input - JOYSTICK_DEADBAND) / (1.0f - JOYSTICK_DEADBAND);
    
    // Expo curve: output = input^expo
    normalized = powf(normalized, JOYSTICK_EXPO);
    
    return sign * normalized;
}

/**
 * @brief Calculate soft limit velocity scaling factor
 */
static float calculate_soft_limit_scale(float pos, float min, float max, float zone) {
    float range = max - min;
    if (range <= 0.0f) return 1.0f;
    
    float zone_size = range * zone;
    float distance_from_min = pos - min;
    float distance_from_max = max - pos;
    
    // Scale down if near limits
    if (distance_from_min < zone_size) {
        // Near minimum - use smootherstep
        float u = distance_from_min / zone_size;
        u = u * u * u * (u * (u * 6.0f - 15.0f) + 10.0f);  // Smootherstep
        return 0.1f + 0.9f * u;  // Scale from 10% to 100%
    } else if (distance_from_max < zone_size) {
        // Near maximum - use smootherstep
        float u = distance_from_max / zone_size;
        u = u * u * u * (u * (u * 6.0f - 15.0f) + 10.0f);  // Smootherstep
        return 0.1f + 0.9f * u;  // Scale from 10% to 100%
    }
    
    return 1.0f;  // Not near limits
}

void motion_planner_init(motion_planner_t* planner, segment_queue_t* queue) {
    memset(planner, 0, sizeof(motion_planner_t));
    planner->queue = queue;
    
    // Initialize max speeds
    planner->max_velocity[AXIS_PAN] = MAX_VELOCITY_PAN;
    planner->max_velocity[AXIS_TILT] = MAX_VELOCITY_TILT;
    planner->max_velocity[AXIS_ZOOM] = MAX_VELOCITY_ZOOM;
    
    // Initialize max accelerations
    planner->max_accel[AXIS_PAN] = MAX_ACCEL_PAN;
    planner->max_accel[AXIS_TILT] = MAX_ACCEL_TILT;
    planner->max_accel[AXIS_ZOOM] = MAX_ACCEL_ZOOM;
    
    // Initialize limits (default to very large range)
    for (int i = 0; i < NUM_AXES; i++) {
        planner->limits_min[i] = -100000.0f;
        planner->limits_max[i] = 100000.0f;
    }
    
    planner->precision_multiplier = 0.25f;
    planner->precision_mode = false;
    planner->move_easing = EASING_SMOOTHERSTEP;
}

void motion_planner_set_position(motion_planner_t* planner, uint8_t axis, float position) {
    if (axis < NUM_AXES) {
        planner->positions[axis] = position;
    }
}

float motion_planner_get_position(const motion_planner_t* planner, uint8_t axis) {
    if (axis >= NUM_AXES) {
        return 0.0f;
    }
    return planner->positions[axis];
}

void motion_planner_set_limits(motion_planner_t* planner, uint8_t axis, float min, float max) {
    if (axis < NUM_AXES) {
        planner->limits_min[axis] = min;
        planner->limits_max[axis] = max;
    }
}

bool motion_planner_plan_move(motion_planner_t* planner, const float targets[NUM_AXES], 
                              float duration, easing_type_t easing) {
    if (planner->move_in_progress) {
        return false;  // Already planning a move
    }
    
    // Check soft limits
    for (int i = 0; i < NUM_AXES; i++) {
        if (targets[i] < planner->limits_min[i] || targets[i] > planner->limits_max[i]) {
            return false;  // Target out of limits
        }
    }
    
    // Calculate required duration if not specified
    if (duration <= 0.0f) {
        float max_duration = 0.0f;
        for (int i = 0; i < NUM_AXES; i++) {
            float distance = fabsf(targets[i] - planner->positions[i]);
            if (distance > 0.0f) {
                float axis_duration = distance / planner->max_velocity[i];
                if (axis_duration > max_duration) {
                    max_duration = axis_duration;
                }
            }
        }
        duration = max_duration;
        if (duration < 0.1f) duration = 0.1f;  // Minimum 100ms
    }
    
    // Initialize quintic coefficients for each axis
    for (int i = 0; i < NUM_AXES; i++) {
        quintic_init(&planner->move_coeffs[i], 
                    planner->positions[i], 
                    targets[i], 
                    duration);
        planner->targets[i] = targets[i];
    }
    
    planner->move_start_time = -1.0f;  // Mark as not started (will be set to 0 on first update)
    planner->move_duration = duration;
    planner->move_easing = easing;
    planner->move_in_progress = true;
    planner->manual_mode = false;
    
    return true;
}

void motion_planner_set_velocities(motion_planner_t* planner, const float velocities[NUM_AXES]) {
    for (int i = 0; i < NUM_AXES; i++) {
        // Apply joystick filtering (deadband + expo)
        planner->manual_target_vel[i] = apply_joystick_filter(velocities[i]);
    }
}

void motion_planner_set_manual_mode(motion_planner_t* planner, bool enabled) {
    planner->manual_mode = enabled;
    if (!enabled) {
        // Reset velocities when leaving manual mode
        for (int i = 0; i < NUM_AXES; i++) {
            planner->velocities[i] = 0.0f;
            planner->manual_slew_limit[i] = 0.0f;
            planner->manual_target_vel[i] = 0.0f;
        }
    }
}

void motion_planner_set_precision_mode(motion_planner_t* planner, bool enabled) {
    planner->precision_mode = enabled;
}

void motion_planner_update(motion_planner_t* planner, float dt) {
    if (planner->manual_mode) {
        // Manual velocity mode - apply slew limiting and generate constant velocity segments
        for (int i = 0; i < NUM_AXES; i++) {
            float target_vel = planner->manual_target_vel[i];
            
            // Convert normalized velocity (-1 to 1) to steps/sec
            float max_vel = planner->max_velocity[i];
            target_vel *= max_vel;
            
            // Apply precision mode multiplier
            if (planner->precision_mode) {
                target_vel *= planner->precision_multiplier;
            }
            
            // Apply soft limit scaling
            float limit_scale = calculate_soft_limit_scale(
                planner->positions[i],
                planner->limits_min[i],
                planner->limits_max[i],
                SOFT_LIMIT_ZONE
            );
            target_vel *= limit_scale;
            
            // Clamp to max velocity
            if (target_vel > planner->max_velocity[i]) {
                target_vel = planner->max_velocity[i];
            } else if (target_vel < -planner->max_velocity[i]) {
                target_vel = -planner->max_velocity[i];
            }
            
            // Apply slew rate limiting
            float max_change = slew_rate_limits[i] * dt;
            float vel_diff = target_vel - planner->manual_slew_limit[i];
            if (vel_diff > max_change) {
                planner->manual_slew_limit[i] += max_change;
            } else if (vel_diff < -max_change) {
                planner->manual_slew_limit[i] -= max_change;
            } else {
                planner->manual_slew_limit[i] = target_vel;
            }
            
            // Update position based on velocity
            planner->positions[i] += planner->manual_slew_limit[i] * dt;
            
            // Clamp position to limits
            if (planner->positions[i] < planner->limits_min[i]) {
                planner->positions[i] = planner->limits_min[i];
                planner->manual_slew_limit[i] = 0.0f;
            } else if (planner->positions[i] > planner->limits_max[i]) {
                planner->positions[i] = planner->limits_max[i];
                planner->manual_slew_limit[i] = 0.0f;
            }
        }
        
        // Generate constant velocity segment
        motion_segment_t segment;
        segment.duration_us = SEGMENT_DURATION_US;
        for (int i = 0; i < NUM_AXES; i++) {
            float steps_per_sec = planner->manual_slew_limit[i];
            float steps = steps_per_sec * (SEGMENT_DURATION_US / 1000000.0f);
            segment.steps[i] = (int32_t)roundf(steps);
        }
        
        // Push segment if queue has space
        if (!segment_queue_is_full(planner->queue)) {
            segment_queue_push(planner->queue, &segment);
        }
        
    } else if (planner->move_in_progress) {
        // Waypoint move mode - generate segments from quintic trajectory
        if (planner->move_start_time < 0.0f) {
            planner->move_start_time = 0.0f;  // Start timing
        }
        
        float elapsed = planner->move_start_time;
        planner->move_start_time += dt;
        
        if (elapsed >= planner->move_duration) {
            // Move complete - snap to final position
            for (int i = 0; i < NUM_AXES; i++) {
                planner->positions[i] = planner->targets[i];
            }
            planner->move_in_progress = false;
            return;
        }
        
        // Generate segment from quintic evaluation
        motion_segment_t segment;
        segment.duration_us = SEGMENT_DURATION_US;
        
        float t_prev = elapsed;
        float t_next = elapsed + (SEGMENT_DURATION_US / 1000000.0f);
        if (t_next > planner->move_duration) {
            t_next = planner->move_duration;
        }
        
        for (int i = 0; i < NUM_AXES; i++) {
            float pos_prev = quintic_evaluate_eased(&planner->move_coeffs[i], t_prev, planner->move_easing);
            float pos_next = quintic_evaluate_eased(&planner->move_coeffs[i], t_next, planner->move_easing);
            float steps = pos_next - pos_prev;
            segment.steps[i] = (int32_t)roundf(steps);
        }
        
        // Push segment if queue has space
        if (!segment_queue_is_full(planner->queue)) {
            segment_queue_push(planner->queue, &segment);
        }
    }
}

bool motion_planner_is_busy(const motion_planner_t* planner) {
    return planner->move_in_progress || planner->manual_mode;
}

void motion_planner_stop(motion_planner_t* planner) {
    planner->move_in_progress = false;
    planner->manual_mode = false;
    for (int i = 0; i < NUM_AXES; i++) {
        planner->velocities[i] = 0.0f;
        planner->manual_slew_limit[i] = 0.0f;
        planner->manual_target_vel[i] = 0.0f;
    }
    segment_queue_clear(planner->queue);
}

