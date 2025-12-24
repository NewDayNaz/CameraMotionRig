/**
 * @file motion_planner.c
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
        planner->velocities[i] = velocities[i];
    }
}

void motion_planner_set_manual_mode(motion_planner_t* planner, bool enabled) {
    planner->manual_mode = enabled;
    if (!enabled) {
        // Reset velocities when leaving manual mode
        for (int i = 0; i < NUM_AXES; i++) {
            planner->velocities[i] = 0.0f;
            planner->manual_slew_limit[i] = 0.0f;
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
            float target_vel = planner->velocities[i];
            
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
            
        }
        
        // Generate segments (one segment per update cycle)
        // Calculate how many segments we can fit in dt
        uint32_t segments_per_update = (uint32_t)(dt * 1e6f / SEGMENT_DURATION_US) + 1;
        uint32_t free_slots = segment_queue_free_slots(planner->queue);
        if (segments_per_update > free_slots) {
            segments_per_update = free_slots;
        }
        
        for (uint32_t seg_idx = 0; seg_idx < segments_per_update; seg_idx++) {
            motion_segment_t seg;
            seg.duration_us = SEGMENT_DURATION_US;
            
            float segment_dt = SEGMENT_DURATION_US / 1e6f;
            
            for (int i = 0; i < NUM_AXES; i++) {
                // Check soft limits before moving
                float steps_float = planner->manual_slew_limit[i] * segment_dt;
                float new_pos = planner->positions[i] + steps_float;
                
                if (new_pos < planner->limits_min[i] || new_pos > planner->limits_max[i]) {
                    // Clamp to limit
                    steps_float = 0.0f;
                    planner->manual_slew_limit[i] = 0.0f;
                }
                
                seg.steps[i] = (int32_t)roundf(steps_float);
                planner->positions[i] += steps_float;
            }
            
            if (!segment_queue_push(planner->queue, &seg)) {
                break;  // Queue full, stop generating segments
            }
        }
    } else if (planner->move_in_progress) {
        // Waypoint move in progress - generate segments from quintic trajectory
        static float current_time = 0.0f;
        
        if (planner->move_start_time < 0.0f) {
            // New move starting - reset time
            current_time = 0.0f;
            planner->move_start_time = 0.0f;  // Mark as started
        }
        
        // Generate segments until move is complete
        while (current_time < planner->move_duration && 
               segment_queue_free_slots(planner->queue) > 4) {
            // Calculate positions at start and end of this segment
            float t_start = current_time;
            float t_end = current_time + (SEGMENT_DURATION_US / 1e6f);
            if (t_end > planner->move_duration) {
                t_end = planner->move_duration;
            }
            
            motion_segment_t seg;
            seg.duration_us = SEGMENT_DURATION_US;
            
            for (int i = 0; i < NUM_AXES; i++) {
                float pos_start = quintic_evaluate_eased(&planner->move_coeffs[i], t_start, planner->move_easing);
                float pos_end = quintic_evaluate_eased(&planner->move_coeffs[i], t_end, planner->move_easing);
                
                // Calculate steps for this segment (round to nearest)
                float steps_float = pos_end - pos_start;
                seg.steps[i] = (int32_t)roundf(steps_float);
                
                // Accumulate rounding error correction (simple approach)
                // In production, use proper error accumulation
            }
            
            segment_queue_push(planner->queue, &seg);
            current_time = t_end;
        }
        
        // Check if move is complete
        if (current_time >= planner->move_duration) {
            // Snap to final positions
            for (int i = 0; i < NUM_AXES; i++) {
                planner->positions[i] = planner->targets[i];
            }
            planner->move_in_progress = false;
            planner->move_start_time = -1.0f;  // Reset for next move
            current_time = 0.0f;
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
    }
    segment_queue_clear(planner->queue);
}

