/**
 * @file homing.cpp
 * @brief Homing implementation
 */

#include "homing.h"
#include "motion_planner.h"
#include "stepper_executor.h"
#include <Arduino.h>

static homing_context_t s_homing;
static motion_planner_t* s_planner = nullptr;

// Default homing configuration
static const homing_config_t default_config = {
    .fast_velocity = 1000.0f,      // 1000 steps/sec
    .slow_velocity = 200.0f,        // 200 steps/sec
    .backoff_distance = 500.0f,    // 500 steps
    .home_position = 0,             // Home at position 0
    .timeout_ms = 30000             // 30 second timeout
};

void homing_init(motion_planner_t* planner) {
    s_planner = planner;
    // Initialize homing context
    s_homing.state = HOMING_IDLE;
    s_homing.config.fast_velocity = 0.0f;
    s_homing.config.slow_velocity = 0.0f;
    s_homing.config.backoff_distance = 0.0f;
    s_homing.config.home_position = 0;
    s_homing.config.timeout_ms = 0;
    s_homing.axis = 0;
    s_homing.current_velocity = 0.0f;
    s_homing.start_time_ms = 0;
    s_homing.triggered = false;
}

bool homing_start(uint8_t axis, const homing_config_t* config) {
    if (axis >= NUM_AXES || axis == AXIS_ZOOM) {
        return false;  // Only PAN and TILT have endstops
    }
    
    if (s_homing.state != HOMING_IDLE) {
        return false;  // Already homing
    }
    
    if (endstop_pins[axis] < 0) {
        return false;  // No endstop for this axis
    }
    
    // Use provided config or defaults
    if (config) {
        s_homing.config = *config;
    } else {
        s_homing.config = default_config;
    }
    
    s_homing.axis = axis;
    s_homing.state = HOMING_FAST_APPROACH;
    s_homing.current_velocity = -s_homing.config.fast_velocity;  // Negative = toward min endstop
    s_homing.start_time_ms = millis();
    s_homing.triggered = false;
    
    // Enable manual mode for homing
    motion_planner_set_manual_mode(s_planner, true);
    
    return true;
}

bool homing_update(float dt) {
    if (s_homing.state == HOMING_IDLE || s_homing.state == HOMING_COMPLETE) {
        return true;
    }
    
    if (s_homing.state == HOMING_ERROR) {
        return false;
    }
    
    // Check timeout
    if (s_homing.config.timeout_ms > 0) {
        uint32_t elapsed = millis() - s_homing.start_time_ms;
        if (elapsed > s_homing.config.timeout_ms) {
            s_homing.state = HOMING_ERROR;
            return false;
        }
    }
    
    // Read endstop
    bool endstop_triggered = board_read_endstop(s_homing.axis);
    float current_pos = motion_planner_get_position(s_planner, s_homing.axis);
    
    switch (s_homing.state) {
        case HOMING_FAST_APPROACH: {
            if (endstop_triggered) {
                // Endstop hit - switch to backoff
                s_homing.state = HOMING_BACKOFF;
                s_homing.current_velocity = s_homing.config.fast_velocity;  // Positive = away from endstop
                s_homing.triggered = true;
            } else {
                // Continue fast approach
                float velocities[NUM_AXES] = {0.0f, 0.0f, 0.0f};
                velocities[s_homing.axis] = s_homing.current_velocity / s_planner->max_velocity[s_homing.axis];
                motion_planner_set_velocities(s_planner, velocities);
            }
            break;
        }
        
        case HOMING_BACKOFF: {
            // Move away from endstop
            float target_pos = current_pos + s_homing.config.backoff_distance;
            if (current_pos >= target_pos) {
                // Backoff complete - switch to slow approach
                s_homing.state = HOMING_SLOW_APPROACH;
                s_homing.current_velocity = -s_homing.config.slow_velocity;  // Negative = toward endstop
            } else {
                float velocities[NUM_AXES] = {0.0f, 0.0f, 0.0f};
                velocities[s_homing.axis] = s_homing.current_velocity / s_planner->max_velocity[s_homing.axis];
                motion_planner_set_velocities(s_planner, velocities);
            }
            break;
        }
        
        case HOMING_SLOW_APPROACH: {
            if (endstop_triggered) {
                // Endstop hit again - set home position
                motion_planner_set_position(s_planner, s_homing.axis, (float)s_homing.config.home_position);
                stepper_executor_set_position(s_homing.axis, s_homing.config.home_position);
                
                // Stop motion and disable manual mode
                float velocities[NUM_AXES] = {0.0f, 0.0f, 0.0f};
                motion_planner_set_velocities(s_planner, velocities);
                motion_planner_set_manual_mode(s_planner, false);
                
                s_homing.state = HOMING_COMPLETE;
                return true;
            } else {
                // Continue slow approach
                float velocities[NUM_AXES] = {0.0f, 0.0f, 0.0f};
                velocities[s_homing.axis] = s_homing.current_velocity / s_planner->max_velocity[s_homing.axis];
                motion_planner_set_velocities(s_planner, velocities);
            }
            break;
        }
        
        default:
            break;
    }
    
    return false;  // Still homing
}

bool homing_is_active(void) {
    return (s_homing.state != HOMING_IDLE && s_homing.state != HOMING_COMPLETE && s_homing.state != HOMING_ERROR);
}

homing_state_t homing_get_state(void) {
    return s_homing.state;
}

void homing_stop(void) {
    if (s_homing.state != HOMING_IDLE && s_homing.state != HOMING_COMPLETE) {
        float velocities[NUM_AXES] = {0.0f, 0.0f, 0.0f};
        motion_planner_set_velocities(s_planner, velocities);
        motion_planner_set_manual_mode(s_planner, false);
        s_homing.state = HOMING_IDLE;
    }
}

