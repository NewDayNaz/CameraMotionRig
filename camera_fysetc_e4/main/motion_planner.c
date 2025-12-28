/**
 * @file motion_planner.c
 * @brief Motion planner implementation
 */

#include "motion_planner.h"
#include "board.h"
#include "segment.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char* TAG = "motion_planner";

// Import microstepping scale from board.h
#ifndef MICROSTEP_SCALE
#define MICROSTEP_SCALE 1.0f  // Default to no scaling if not defined
#endif

// Slew rate limit multiplier for manual/joystick mode
// Higher values = faster acceleration response
// 0.5 = 50% of max acceleration (conservative, smooth)
// 1.0 = 100% of max acceleration (aggressive, immediate response)
#define MANUAL_MODE_ACCEL_MULTIPLIER 1.0f  // Use full acceleration for joystick/velocity commands

// Slew rate limits for manual mode (will be scaled by microstepping and multiplier)
// These are computed at runtime to account for microstepping scaling
static float slew_rate_limits[NUM_AXES];

void motion_planner_init(motion_planner_t* planner, segment_queue_t* queue) {
    memset(planner, 0, sizeof(motion_planner_t));
    planner->queue = queue;
    
    // Initialize max speeds (scale by microstepping factor)
    // Velocities are specified in "full steps/sec" but need to be converted to "microsteps/sec"
    planner->max_velocity[AXIS_PAN] = MAX_VELOCITY_PAN * MICROSTEP_SCALE;
    planner->max_velocity[AXIS_TILT] = MAX_VELOCITY_TILT * MICROSTEP_SCALE;
    planner->max_velocity[AXIS_ZOOM] = MAX_VELOCITY_ZOOM * MICROSTEP_SCALE;
    
    // Initialize max accelerations (scale by microstepping factor)
    planner->max_accel[AXIS_PAN] = MAX_ACCEL_PAN * MICROSTEP_SCALE;
    planner->max_accel[AXIS_TILT] = MAX_ACCEL_TILT * MICROSTEP_SCALE;
    planner->max_accel[AXIS_ZOOM] = MAX_ACCEL_ZOOM * MICROSTEP_SCALE;
    
    // Initialize slew rate limits (scaled by microstepping and acceleration multiplier)
    // These control how fast velocities can change in manual/joystick mode
    // Using full acceleration (1.0 multiplier) for responsive joystick control
    slew_rate_limits[AXIS_PAN] = MAX_ACCEL_PAN * MICROSTEP_SCALE * MANUAL_MODE_ACCEL_MULTIPLIER;
    slew_rate_limits[AXIS_TILT] = MAX_ACCEL_TILT * MICROSTEP_SCALE * MANUAL_MODE_ACCEL_MULTIPLIER;
    slew_rate_limits[AXIS_ZOOM] = MAX_ACCEL_ZOOM * MICROSTEP_SCALE * MANUAL_MODE_ACCEL_MULTIPLIER;
    
    // Initialize limits (default to very large range, scaled by microstepping)
    // Limits are in microsteps to match position tracking
    for (int i = 0; i < NUM_AXES; i++) {
        planner->limits_min[i] = -100000.0f * MICROSTEP_SCALE;
        planner->limits_max[i] = 100000.0f * MICROSTEP_SCALE;
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
        // Assume limits are provided in full steps, convert to microsteps
        planner->limits_min[axis] = min * MICROSTEP_SCALE;
        planner->limits_max[axis] = max * MICROSTEP_SCALE;
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
    // Use preset-specific limits (more conservative) to prevent step skipping
    // Quintic curves can have high peak velocities/accelerations
    if (duration <= 0.0f) {
        // Use preset-specific limits (scaled by microstepping)
        // These are 50% of manual control limits to prevent step skipping
        const float preset_max_vel[NUM_AXES] = {
            PRESET_MAX_VELOCITY_PAN * MICROSTEP_SCALE,
            PRESET_MAX_VELOCITY_TILT * MICROSTEP_SCALE,
            PRESET_MAX_VELOCITY_ZOOM * MICROSTEP_SCALE
        };
        const float preset_max_accel[NUM_AXES] = {
            PRESET_MAX_ACCEL_PAN * MICROSTEP_SCALE,
            PRESET_MAX_ACCEL_TILT * MICROSTEP_SCALE,
            PRESET_MAX_ACCEL_ZOOM * MICROSTEP_SCALE
        };
        
        float max_duration = 0.0f;
        for (int i = 0; i < NUM_AXES; i++) {
            float distance = fabsf(targets[i] - planner->positions[i]);
            if (distance > 0.0f) {
                // PRIORITIZE ACCELERATION CONSTRAINTS FIRST
                // Quintic curves have peak accelerations ~2-3x average acceleration
                // Use only 15% of preset max acceleration to account for peaks (very conservative)
                // This ensures we don't exceed acceleration limits and skip steps
                float accel_limit_used = preset_max_accel[i] * 0.15f;
                
                // Calculate minimum duration based on acceleration constraint
                // For a quintic curve with zero start/end velocity, the peak acceleration occurs
                // during the acceleration phase. We need to ensure the acceleration phase
                // doesn't exceed our limits.
                // Using: t = sqrt(2 * distance / a) for constant acceleration
                // But quintic curves are more complex, so we add safety factors
                float accel_duration = sqrtf(2.0f * distance / accel_limit_used) * 2.0f;
                
                // Now check if this duration allows us to stay within velocity limits
                // Average velocity = distance / duration
                // Peak velocity for quintic is ~1.5-2x average
                float avg_velocity = distance / accel_duration;
                float peak_velocity_estimate = avg_velocity * 2.0f;  // Conservative estimate
                
                // Use only 15% of preset max velocity as our limit (very conservative)
                float vel_limit_used = preset_max_vel[i] * 0.15f;
                
                // If peak velocity exceeds limit, increase duration
                if (peak_velocity_estimate > vel_limit_used) {
                    // Recalculate duration to satisfy velocity constraint
                    // peak_velocity = 2 * distance / duration
                    // duration = 2 * distance / peak_velocity
                    float vel_duration = (2.0f * distance) / vel_limit_used;
                    // Use the longer duration (acceleration or velocity constrained)
                    accel_duration = (vel_duration > accel_duration) ? vel_duration : accel_duration;
                }
                
                // Add safety factor for quintic curve peaks
                // Base safety factor
                float safety_factor = 3.0f;
                
                // Additional multiplier for non-linear easing functions
                // Smootherstep and sigmoid can create higher peak velocities due to their
                // non-linear time warping, so we need longer durations
                // Linear easing: 1.0x (no additional scaling needed)
                // Smootherstep: ~1.5x higher peak velocities
                // Sigmoid: ~2.0x higher peak velocities (steeper curve)
                float easing_multiplier = 1.0f;
                if (easing == EASING_SMOOTHERSTEP) {
                    easing_multiplier = 1.8f;  // Smootherstep needs more time
                } else if (easing == EASING_SIGMOID) {
                    easing_multiplier = 2.2f;  // Sigmoid needs even more time
                }
                
                float axis_duration = accel_duration * safety_factor * easing_multiplier;
                
                // Add minimum duration based on distance to ensure smooth motion
                // At least 1.5 seconds per 1000 full steps (16000 microsteps with 16x microstepping) for cinematic moves
                float min_duration = (distance / (1000.0f * MICROSTEP_SCALE)) * 1.5f;
                if (axis_duration < min_duration) {
                    axis_duration = min_duration;
                }
                
                if (axis_duration > max_duration) {
                    max_duration = axis_duration;
                }
            }
        }
        duration = max_duration;
        if (duration < 0.5f) duration = 0.5f;  // Minimum 500ms for smooth cinematic motion
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
    // Scale velocities by microstepping factor
    // Input velocities are in "full steps/sec", convert to "microsteps/sec"
    for (int i = 0; i < NUM_AXES; i++) {
        planner->velocities[i] = velocities[i] * MICROSTEP_SCALE;
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
        // Manual velocity mode - immediate response (no slew rate limiting for joystick/velocity commands)
        // Generate constant velocity segments
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
            
            // No slew rate limiting for manual mode - immediate response for joystick/velocity commands
            // Acceleration limiting is only used for automated GOTO moves (quintic trajectories)
            planner->manual_slew_limit[i] = target_vel;
            
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
                
                // Calculate steps for this segment
                // Use the quintic curve directly - the duration calculation ensures we don't exceed limits
                float steps_float = pos_end - pos_start;
                
                seg.steps[i] = (int32_t)roundf(steps_float);
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

