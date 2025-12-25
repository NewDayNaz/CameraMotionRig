/**
 * @file stepper_executor.cpp
 * @brief Stepper executor implementation using micros() timing
 */

#include "stepper_executor.h"
#include "board.h"
#include <Arduino.h>

// Segment queue reference
static segment_queue_t* s_queue = nullptr;

// Current segment being executed
static motion_segment_t s_current_segment;
static bool s_segment_active = false;

// Step tracking for DDA distribution
static int32_t s_steps_emitted[NUM_AXES];  // Steps already emitted in current segment
static int32_t s_total_steps[NUM_AXES];     // Total steps for current segment
static uint32_t s_segment_start_us = 0;
static uint32_t s_last_step_us[NUM_AXES];   // Per-axis last step time

// Current positions (in steps)
static volatile int32_t s_positions[NUM_AXES] = {0, 0, 0};

// Step timing (minimum microseconds between steps per axis)
#define MIN_STEP_INTERVAL_US 50  // 20kHz max step rate per axis
// Step pulse width (TMC2209 minimum is ~100ns, but we use 1us for safety)
#define STEP_PULSE_WIDTH_US 1

bool stepper_executor_init(segment_queue_t* queue) {
    s_queue = queue;
    s_segment_active = false;
    
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        s_steps_emitted[i] = 0;
        s_total_steps[i] = 0;
        s_positions[i] = 0;
        s_last_step_us[i] = 0;
    }
    
    s_segment_start_us = 0;
    
    return true;
}

void stepper_executor_start(void) {
    // Executor runs continuously via update() calls
    // No separate timer needed for Arduino
}

void stepper_executor_stop(void) {
    s_segment_active = false;
    segment_queue_clear(s_queue);
}

int32_t stepper_executor_get_position(uint8_t axis) {
    if (axis >= NUM_AXES) {
        return 0;
    }
    return s_positions[axis];
}

void stepper_executor_set_position(uint8_t axis, int32_t position) {
    if (axis >= NUM_AXES) {
        return;
    }
    s_positions[axis] = position;
}

bool stepper_executor_is_busy(void) {
    return s_segment_active || !segment_queue_is_empty(s_queue);
}

void stepper_executor_update(void) {
    uint32_t now_us = micros();
    
    // Load new segment if needed
    if (!s_segment_active) {
        if (segment_queue_pop(s_queue, &s_current_segment)) {
            s_segment_active = true;
            s_segment_start_us = now_us;
            
            // Initialize step tracking
            for (uint8_t axis = 0; axis < NUM_AXES; axis++) {
                s_total_steps[axis] = abs(s_current_segment.steps[axis]);
                s_steps_emitted[axis] = 0;
                
                // Set direction pin
                if (s_current_segment.steps[axis] < 0) {
                    digitalWrite(dir_pins[axis], HIGH);
                } else {
                    digitalWrite(dir_pins[axis], LOW);
                }
            }
        } else {
            // No segment available - hold position (do nothing)
            return;
        }
    }
    
    // Check if segment duration has elapsed
    uint32_t elapsed_us = now_us - s_segment_start_us;
    if (elapsed_us >= s_current_segment.duration_us) {
        // Segment complete - mark as inactive
        s_segment_active = false;
        return;
    }
    
    // Distribute steps evenly across segment duration using DDA
    // Calculate how many steps should have been emitted by now
    float progress = (float)elapsed_us / (float)s_current_segment.duration_us;
    if (progress > 1.0f) progress = 1.0f;
    
    // First pass: Determine which axes need to step (per-axis timing check)
    // Emit at most one step per axis per update to maintain even distribution
    bool axes_to_step[NUM_AXES] = {false};
    
    for (uint8_t axis = 0; axis < NUM_AXES; axis++) {
        if (s_total_steps[axis] == 0) {
            continue;  // No steps for this axis
        }
        
        // Calculate how many steps should have been emitted by now
        int32_t steps_due = (int32_t)(progress * s_total_steps[axis]);
        int32_t steps_needed = steps_due - s_steps_emitted[axis];
        
        if (steps_needed > 0) {
            // Check minimum step interval for THIS axis (independent per axis)
            uint32_t time_since_last_step = now_us - s_last_step_us[axis];
            
            if (time_since_last_step >= MIN_STEP_INTERVAL_US) {
                // Emit one step per update cycle to maintain even distribution
                axes_to_step[axis] = true;
            }
        }
    }
    
    // Second pass: Generate step pulses for all axes that need stepping
    // This allows multiple axes to step in parallel with a single delay
    // All step pins are set HIGH simultaneously, then LOW simultaneously
    // This minimizes jitter compared to sequential per-axis stepping
    bool any_axis_stepping = false;
    for (uint8_t axis = 0; axis < NUM_AXES; axis++) {
        if (axes_to_step[axis]) {
            // Set step pin HIGH (rising edge will trigger step in TMC2209)
            digitalWrite(step_pins[axis], HIGH);
            any_axis_stepping = true;
        }
    }
    
    // Single minimal delay for all axes (instead of per-axis delays)
    // TMC2209 minimum pulse width is ~100ns, 1us is well above that
    // This is the only blocking delay, and it's shared across all axes
    // For truly zero-jitter, a timer interrupt would be needed, but 1us
    // is acceptable for most applications and much better than sequential delays
    if (any_axis_stepping) {
        delayMicroseconds(1);  // Minimal pulse width for all axes simultaneously
    }
    
    // Third pass: Bring all step pins LOW and update positions
    for (uint8_t axis = 0; axis < NUM_AXES; axis++) {
        if (axes_to_step[axis]) {
            // Bring step pin LOW (completing the pulse)
            digitalWrite(step_pins[axis], LOW);
            
            // Update position (one step per axis)
            if (s_current_segment.steps[axis] < 0) {
                s_positions[axis]--;
            } else {
                s_positions[axis]++;
            }
            
            s_steps_emitted[axis]++;
            s_last_step_us[axis] = now_us;
        }
    }
}

