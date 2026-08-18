/**
 * @file stepper_simple.c
 * @brief Simple direct stepper control implementation
 * 
 * Based on camera_async simple approach but adapted for ESP-IDF and FYSETC E4 board.
 */

#include "stepper_simple.h"
#include "stepper_limits.h"
#include "board.h"
#include "preset_storage.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <string.h>

static const char* TAG = "stepper_simple";

// Stepper state
typedef struct {
    int32_t position;  // Current position in steps
    float velocity;    // Current velocity (steps/sec)
    float target_velocity;  // Target velocity (steps/sec)
    int move_direction;  // 0=stop, 1=forward, 2=reverse
    uint32_t step_delay_us;  // Step delay in microseconds
    int64_t last_step_time;  // Last step time (microseconds)
} axis_state_t;

static axis_state_t axes[NUM_AXES];
static bool initialized = false;
static bool homing_active = false;
static bool startup_homing = false;
static uint8_t homing_axis = 0;
static int32_t homing_start_position[NUM_AXES];  // Starting position when homing began
static int32_t homing_steps_taken[NUM_AXES];     // Steps taken during homing (absolute value)

// Preset move state
static bool preset_move_active = false;
static uint8_t preset_move_index = 0;
static float preset_target[3];
static float preset_start[3];
static float preset_max_speed[3];      // Max speed per axis (steps/sec)
static float preset_accel_factor;      // Acceleration factor
static float preset_decel_factor;      // Deceleration factor (most important for accuracy)
static float preset_total_distance[3]; // Total distance to travel per axis

// Constants
#define MIN_STEP_DELAY_US 250  // Minimum step delay (max speed ~2000 steps/sec)
#define PRESET_FINAL_APPROACH_STEPS 10.0f  // Disable min-velocity clamp within this many steps of target
// Note: MIN/MAX velocity constants are defined in stepper_limits.h

// Helper function to get homing velocity for an axis
static float get_homing_velocity(uint8_t axis) {
    if (axis == AXIS_PAN) {
        return HOMING_PAN_VELOCITY;
    } else if (axis == AXIS_TILT) {
        return HOMING_TILT_VELOCITY;
    } else if (axis == AXIS_ZOOM) {
        return HOMING_ZOOM_VELOCITY;
    }
    return 200.0f;  // Default fallback
}

// Helper function to get homing direction for an axis
static float get_homing_direction(uint8_t axis) {
    if (axis == AXIS_PAN) {
        return (float)HOMING_PAN_DIRECTION;
    } else if (axis == AXIS_TILT) {
        return (float)HOMING_TILT_DIRECTION;
    } else if (axis == AXIS_ZOOM) {
        return (float)HOMING_ZOOM_DIRECTION;
    }
    return -1.0f;  // Default fallback (negative direction)
}

// Helper function to convert velocity to step delay
static uint32_t velocity_to_step_delay(float velocity) {
    if (fabsf(velocity) < 0.1f) {
        return 0;  // Stopped
    }
    
    // Calculate step delay: delay_us = 1000000 / velocity
    uint32_t delay_us = (uint32_t)(1000000.0f / fabsf(velocity));
    
    // Limit minimum delay
    if (delay_us < MIN_STEP_DELAY_US) {
        delay_us = MIN_STEP_DELAY_US;
    }
    
    return delay_us;
}

// Snap axis to preset target and stop motion (preset moves only)
static void preset_axis_reach_target(uint8_t axis) {
    axes[axis].position = (int32_t)lroundf(preset_target[axis]);
    axes[axis].target_velocity = 0.0f;
    axes[axis].velocity = 0.0f;
    axes[axis].move_direction = 0;
}

void stepper_simple_init(void) {
    if (initialized) {
        return;
    }
    
    // Initialize all axes
    for (int i = 0; i < NUM_AXES; i++) {
        axes[i].position = 0;
        axes[i].velocity = 0.0f;
        axes[i].target_velocity = 0.0f;
        axes[i].move_direction = 0;
        axes[i].step_delay_us = 0;
        axes[i].last_step_time = esp_timer_get_time();
    }
    
    initialized = true;
    homing_active = false;
    preset_move_active = false;
    
    // Initialize preset move state
    for (int i = 0; i < NUM_AXES; i++) {
        preset_total_distance[i] = 0.0f;
        preset_max_speed[i] = 0.0f;
        homing_start_position[i] = 0;
        homing_steps_taken[i] = 0;
    }
    
    ESP_LOGI(TAG, "Simple stepper control initialized");

    startup_homing = true;
    stepper_simple_home(); // Start homing sequence on startup
}

static void start_startup_preset(void)
{
    // This should only happen for the homing sequence started at startup.
    if (!startup_homing) {
        return;
    }

    // Clear the flag so preset 1 can only be recalled once.
    startup_homing = false;

    // Check whether preset 1 actually exists.
    preset_t preset;

    if (!preset_load(1, &preset) || !preset.valid) {
        ESP_LOGI(TAG,
                 "Startup homing complete - preset 1 is not stored, staying at home");
        return;
    }

    ESP_LOGI(TAG, "Startup homing complete - recalling preset 1");

    if (!stepper_simple_goto_preset(1)) {
        ESP_LOGW(TAG, "Failed to recall preset 1 after startup homing");
    }
}

void stepper_simple_update(void) {
    if (!initialized) {
        return;
    }
    
    int64_t now_us = esp_timer_get_time();
    
    // Handle preset moves (distance-based with acceleration/deceleration)
    if (preset_move_active) {
        bool all_at_target = true;
        
        for (int i = 0; i < NUM_AXES; i++) {
            float current_pos = (float)axes[i].position;
            float remaining = preset_target[i] - current_pos;
            float remaining_abs = fabsf(remaining);
            float distance_traveled = fabsf(current_pos - preset_start[i]);
            
            if (remaining_abs < 0.5f) {
                // At target — stop immediately regardless of current velocity
                preset_axis_reach_target(i);
            } else {
                all_at_target = false;
                
                float max_vel = preset_max_speed[i];
                
                float decel_zone_percent = 0.3f * preset_decel_factor;
                if (decel_zone_percent > 0.8f) decel_zone_percent = 0.8f;
                float decel_zone_size = preset_total_distance[i] * decel_zone_percent;
                
                float accel_zone_percent = 0.2f / preset_accel_factor;
                if (accel_zone_percent > 0.5f) accel_zone_percent = 0.5f;
                float accel_zone_size = preset_total_distance[i] * accel_zone_percent;
                
                bool final_approach = (remaining_abs <= decel_zone_size + PRESET_FINAL_APPROACH_STEPS);
                
                float target_vel;
                if (remaining_abs <= decel_zone_size) {
                    float speed_factor = (decel_zone_size > 0.1f)
                        ? (remaining_abs / decel_zone_size) : 0.0f;
                    if (!final_approach && speed_factor < 0.02f) {
                        speed_factor = 0.02f;
                    } else if (speed_factor < 0.01f) {
                        speed_factor = 0.01f;
                    }
                    target_vel = max_vel * speed_factor;
                } else if (distance_traveled < accel_zone_size) {
                    float speed_factor = (accel_zone_size > 0.1f)
                        ? (distance_traveled / accel_zone_size) : 1.0f;
                    if (speed_factor < 0.2f) speed_factor = 0.2f;
                    target_vel = max_vel * speed_factor;
                } else {
                    target_vel = max_vel;
                }
                
                // Min-velocity clamp is for manual jog only — skip during preset final approach
                if (!final_approach && remaining_abs > PRESET_FINAL_APPROACH_STEPS) {
                    float min_vel = 0.0f;
                    if (i == AXIS_PAN || i == AXIS_TILT) {
                        min_vel = MIN_PAN_TILT_VELOCITY;
                    } else if (i == AXIS_ZOOM) {
                        min_vel = MIN_ZOOM_VELOCITY;
                    }
                    
                    if (min_vel > 0.0f && fabsf(target_vel) < min_vel) {
                        target_vel = (target_vel >= 0) ? min_vel : -min_vel;
                    }
                }
                
                axes[i].target_velocity = (remaining > 0) ? target_vel : -target_vel;
            }
        }
        
        // Check if all axes are at target
        if (all_at_target) {
            preset_move_active = false;
            ESP_LOGI(TAG, "Preset move complete");
        }
    }
    
    // Handle homing
    if (homing_active) {
        if (homing_axis < NUM_AXES) {
            // Calculate steps taken from starting position (absolute value)
            int32_t current_pos = axes[homing_axis].position;
            int32_t steps_from_start = current_pos - homing_start_position[homing_axis];
            homing_steps_taken[homing_axis] = (steps_from_start < 0) ? -steps_from_start : steps_from_start;
            
            // Get maximum range for this axis
            float max_range = 0.0f;
            if (homing_axis == AXIS_PAN) {
                max_range = MAX_PAN_RANGE_STEPS;
            } else if (homing_axis == AXIS_TILT) {
                max_range = MAX_TILT_RANGE_STEPS;
            } else if (homing_axis == AXIS_ZOOM) {
                max_range = MAX_ZOOM_RANGE_STEPS;
            }
            
            // Check if we've exceeded maximum range
            if (homing_steps_taken[homing_axis] >= (int32_t)max_range) {
                // Bail out - assume current position as home
                ESP_LOGW(TAG, "Homing axis %d: Max range reached (%d steps), assuming current position as home", 
                         homing_axis, homing_steps_taken[homing_axis]);
                axes[homing_axis].position = 0;
                axes[homing_axis].velocity = 0.0f;
                axes[homing_axis].target_velocity = 0.0f;
                axes[homing_axis].move_direction = 0;
                gpio_set_level(step_pins[homing_axis], 0);
                
                // Move to next axis
                homing_axis++;
                if (homing_axis >= NUM_AXES) {
                    homing_active = false;
                    ESP_LOGI(TAG, "Homing complete (some axes may have bailed out)");

                    // Recall startup preset after startup homing
                    start_startup_preset();
                } else {
                    // Start homing next axis
                    homing_start_position[homing_axis] = axes[homing_axis].position;
                    homing_steps_taken[homing_axis] = 0;
                    float homing_vel = get_homing_velocity(homing_axis);
                    float homing_dir = get_homing_direction(homing_axis);
                    axes[homing_axis].target_velocity = homing_vel * homing_dir;
                    ESP_LOGI(TAG, "Homing axis %d (%s) at %.1f steps/sec, direction %.0f", 
                             homing_axis, axis_names[homing_axis], homing_vel, homing_dir);
                }
            } else {
                // Read endstop
                bool endstop_triggered = false;
                if (endstop_pins[homing_axis] != GPIO_NUM_NC) {
                    endstop_triggered = (gpio_get_level(endstop_pins[homing_axis]) == 0);  // Active LOW
                }
                
                if (endstop_triggered) {
                    // Endstop hit - stop and set position to 0
                    ESP_LOGI(TAG, "Homing axis %d (%s): Endstop hit after %d steps", 
                             homing_axis, axis_names[homing_axis], homing_steps_taken[homing_axis]);
                    axes[homing_axis].position = 0;
                    axes[homing_axis].velocity = 0.0f;
                    axes[homing_axis].target_velocity = 0.0f;
                    axes[homing_axis].move_direction = 0;
                    gpio_set_level(step_pins[homing_axis], 0);
                    
                    // Move to next axis
                    homing_axis++;
                    if (homing_axis >= NUM_AXES) {
                        homing_active = false;
                        ESP_LOGI(TAG, "Homing complete");

                        // Recall startup preset after startup homing
                        start_startup_preset();
                    } else {
                        // Start homing next axis
                        homing_start_position[homing_axis] = axes[homing_axis].position;
                        homing_steps_taken[homing_axis] = 0;
                        float homing_vel = get_homing_velocity(homing_axis);
                        float homing_dir = get_homing_direction(homing_axis);
                        axes[homing_axis].target_velocity = homing_vel * homing_dir;
                        ESP_LOGI(TAG, "Homing axis %d (%s) at %.1f steps/sec, direction %.0f", 
                                 homing_axis, axis_names[homing_axis], homing_vel, homing_dir);
                    }
                } else {
                    // Move towards endstop
                    float homing_vel = get_homing_velocity(homing_axis);
                    float homing_dir = get_homing_direction(homing_axis);
                    axes[homing_axis].target_velocity = homing_vel * homing_dir;
                }
            }
        }
    }
    
    // Update each axis
    for (int i = 0; i < NUM_AXES; i++) {
        // Smooth velocity changes (simple slew rate limiting)
        // For stopping, use immediate response. For acceleration, use moderate slew rate
        float vel_diff = axes[i].target_velocity - axes[i].velocity;
        
        // If stopping (target is zero), stop immediately for responsive feel
        if (fabsf(axes[i].target_velocity) < 0.1f) {
            axes[i].velocity = axes[i].target_velocity;
        } else {
            // For acceleration/deceleration, use moderate slew rate
            // Much faster than before: 2000 steps/sec² = 2 steps/sec per ms
            float max_vel_change = 2000.0f;  // Max velocity change per update (steps/sec^2)
            float dt = 0.001f;  // 1ms update period
            float max_change = max_vel_change * dt;  // 2 steps/sec per update
            
            if (fabsf(vel_diff) > max_change) {
                if (vel_diff > 0) {
                    axes[i].velocity += max_change;
                } else {
                    axes[i].velocity -= max_change;
                }
            } else {
                axes[i].velocity = axes[i].target_velocity;
            }
        }
        
        // Update step delay
        axes[i].step_delay_us = velocity_to_step_delay(axes[i].velocity);
        
        // Determine move direction
        // Note: PAN and TILT directions are reversed from expected
        bool reverse_direction = (i == AXIS_PAN || i == AXIS_TILT);
        
        if (axes[i].velocity > 0.1f) {
            axes[i].move_direction = 1;  // Forward
            gpio_set_level(dir_pins[i], reverse_direction ? 0 : 1);
        } else if (axes[i].velocity < -0.1f) {
            axes[i].move_direction = 2;  // Reverse
            gpio_set_level(dir_pins[i], reverse_direction ? 1 : 0);
        } else {
            axes[i].move_direction = 0;  // Stopped
            gpio_set_level(step_pins[i], 0);
            continue;
        }
        
        // Generate step pulse
        if (axes[i].step_delay_us > 0) {
            int64_t time_since_last_step = now_us - axes[i].last_step_time;
            
            if (time_since_last_step >= axes[i].step_delay_us) {
                // Preset move: don't step past target — snap and stop instead
                if (preset_move_active) {
                    float remaining = preset_target[i] - (float)axes[i].position;
                    if ((axes[i].move_direction == 1 && remaining < 1.0f) ||
                        (axes[i].move_direction == 2 && remaining > -1.0f)) {
                        preset_axis_reach_target(i);
                        gpio_set_level(step_pins[i], 0);
                        continue;
                    }
                }
                
                gpio_set_level(step_pins[i], 1);
                esp_rom_delay_us(1);
                gpio_set_level(step_pins[i], 0);
                
                if (axes[i].move_direction == 1) {
                    axes[i].position++;
                } else {
                    axes[i].position--;
                }
                
                axes[i].last_step_time = now_us;
            }
        }
    }
}

void stepper_simple_set_velocities(float pan_vel, float tilt_vel, float zoom_vel) {
    if (!initialized) {
        return;
    }
    
    // Block velocity commands during homing
    if (homing_active) {
        ESP_LOGW(TAG, "Velocity command blocked - homing in progress");
        return;
    }
    
    // Cancel any active preset moves
    preset_move_active = false;
    
    // Apply minimum and maximum velocity limits to pan axis
    if (fabsf(pan_vel) > 0.1f) {
        if (fabsf(pan_vel) < MIN_PAN_TILT_VELOCITY) {
            // Clamp to minimum velocity with correct sign
            pan_vel = (pan_vel > 0) ? MIN_PAN_TILT_VELOCITY : -MIN_PAN_TILT_VELOCITY;
        } else if (fabsf(pan_vel) > MAX_PAN_VELOCITY) {
            // Clamp to maximum velocity with correct sign
            pan_vel = (pan_vel > 0) ? MAX_PAN_VELOCITY : -MAX_PAN_VELOCITY;
        }
    }
    axes[AXIS_PAN].target_velocity = pan_vel;
    
    // Apply minimum and maximum velocity limits to tilt axis
    if (fabsf(tilt_vel) > 0.1f) {
        if (fabsf(tilt_vel) < MIN_PAN_TILT_VELOCITY) {
            // Clamp to minimum velocity with correct sign
            tilt_vel = (tilt_vel > 0) ? MIN_PAN_TILT_VELOCITY : -MIN_PAN_TILT_VELOCITY;
        } else if (fabsf(tilt_vel) > MAX_TILT_VELOCITY) {
            // Clamp to maximum velocity with correct sign
            tilt_vel = (tilt_vel > 0) ? MAX_TILT_VELOCITY : -MAX_TILT_VELOCITY;
        }
    }
    axes[AXIS_TILT].target_velocity = tilt_vel;
    
    // Apply minimum and maximum velocity limits to zoom axis
    if (fabsf(zoom_vel) > 0.1f) {
        if (fabsf(zoom_vel) < MIN_ZOOM_VELOCITY) {
            // Clamp to minimum velocity with correct sign
            zoom_vel = (zoom_vel > 0) ? MIN_ZOOM_VELOCITY : -MIN_ZOOM_VELOCITY;
        } else if (fabsf(zoom_vel) > MAX_ZOOM_VELOCITY) {
            // Clamp to maximum velocity with correct sign to prevent step skipping
            zoom_vel = (zoom_vel > 0) ? MAX_ZOOM_VELOCITY : -MAX_ZOOM_VELOCITY;
        }
    }
    axes[AXIS_ZOOM].target_velocity = zoom_vel;
}

void stepper_simple_get_positions(float* pan, float* tilt, float* zoom) {
    if (!initialized || pan == NULL || tilt == NULL || zoom == NULL) {
        if (pan) *pan = 0.0f;
        if (tilt) *tilt = 0.0f;
        if (zoom) *zoom = 0.0f;
        return;
    }
    
    *pan = (float)axes[AXIS_PAN].position;
    *tilt = (float)axes[AXIS_TILT].position;
    *zoom = (float)axes[AXIS_ZOOM].position;
}

void stepper_simple_stop(void) {
    if (!initialized) {
        return;
    }
    
    for (int i = 0; i < NUM_AXES; i++) {
        axes[i].target_velocity = 0.0f;
    }
    
    preset_move_active = false;
    homing_active = false;
}

bool stepper_simple_goto_preset(uint8_t preset_index) {
    if (!initialized) {
        return false;
    }
    
    if (homing_active) {
        ESP_LOGW(TAG, "Goto preset blocked - homing in progress");
        return false;
    }
    
    preset_t preset;
    if (!preset_load(preset_index, &preset) || !preset.valid) {
        ESP_LOGE(TAG, "Preset %d not found or invalid", preset_index);
        return false;
    }
    
    // Get current positions
    stepper_simple_get_positions(&preset_start[0], &preset_start[1], &preset_start[2]);
    
    // Set target positions
    preset_target[0] = preset.pos[0];
    preset_target[1] = preset.pos[1];
    preset_target[2] = preset.pos[2];
    
    // Calculate distances and max speeds for each axis
    float max_distance = 0.0f;
    for (int i = 0; i < NUM_AXES; i++) {
        float distance = fabsf(preset_target[i] - preset_start[i]);
        preset_total_distance[i] = distance;
        if (distance > max_distance) {
            max_distance = distance;
        }
    }
    
    // Calculate max speed for each axis (distance-based)
    // If preset specifies max_speed, use it. Otherwise, calculate from distance.
    float default_max_speed = 300.0f;  // Default max speed (steps/sec)
    if (preset.max_speed > 0.0f) {
        default_max_speed = preset.max_speed;
    } else {
        // Auto-calculate speed based on longest distance
        // Longer moves get higher max speed, but cap at reasonable limit
        if (max_distance > 1000.0f) {
            default_max_speed = 400.0f;  // Faster for long moves
        } else if (max_distance > 100.0f) {
            default_max_speed = 300.0f;  // Medium speed
        } else {
            default_max_speed = 200.0f;  // Slower for short moves (more precision)
        }
    }
    
    // Set max speed for each axis (proportional to distance for coordinated motion)
    for (int i = 0; i < NUM_AXES; i++) {
        if (preset_total_distance[i] > 0.1f) {
            // Scale speed by distance so all axes finish together
            preset_max_speed[i] = default_max_speed * (preset_total_distance[i] / max_distance);
            
            // Apply maximum velocity limits per axis to prevent step skipping
            float max_vel_limit = 0.0f;
            if (i == AXIS_PAN) {
                max_vel_limit = MAX_PAN_VELOCITY;
            } else if (i == AXIS_TILT) {
                max_vel_limit = MAX_TILT_VELOCITY;
            } else if (i == AXIS_ZOOM) {
                max_vel_limit = MAX_ZOOM_VELOCITY;
            }
            
            if (max_vel_limit > 0.0f && preset_max_speed[i] > max_vel_limit) {
                preset_max_speed[i] = max_vel_limit;
            }
        } else {
            preset_max_speed[i] = 0.0f;
        }
    }
    
    // Store acceleration/deceleration factors
    preset_accel_factor = preset.accel_factor;
    preset_decel_factor = (preset.decel_factor > 0.1f) ? preset.decel_factor : 1.0f;
    
    // Cancel any in-progress preset move before starting new one
    preset_move_active = false;
    for (int i = 0; i < NUM_AXES; i++) {
        axes[i].target_velocity = 0.0f;
        axes[i].velocity = 0.0f;
    }
    
    preset_move_active = true;
    preset_move_index = preset_index;
    
    ESP_LOGI(TAG, "Moving to preset %d: (%.1f, %.1f, %.1f) from (%.1f, %.1f, %.1f), max_speed=%.1f, decel_factor=%.2f", 
             preset_index, preset_target[0], preset_target[1], preset_target[2],
             preset_start[0], preset_start[1], preset_start[2],
             default_max_speed, preset_decel_factor);
    
    return true;
}

bool stepper_simple_save_preset(uint8_t preset_index) {
    if (!initialized) {
        return false;
    }
    
    preset_t preset;
    preset_init_default(&preset);
    
    // Get current positions
    stepper_simple_get_positions(&preset.pos[0], &preset.pos[1], &preset.pos[2]);
    
    if (!preset_save(preset_index, &preset)) {
        ESP_LOGE(TAG, "Failed to save preset %d", preset_index);
        return false;
    }
    
    ESP_LOGI(TAG, "Saved preset %d: (%.1f, %.1f, %.1f)", 
             preset_index, preset.pos[0], preset.pos[1], preset.pos[2]);
    
    return true;
}

void stepper_simple_home(void) {
    if (!initialized) {
        return;
    }
    
    // Stop all motion
    stepper_simple_stop();
    
    // Start homing sequence
    homing_active = true;
    homing_axis = 0;
    
    // Record starting positions for all axes
    for (int i = 0; i < NUM_AXES; i++) {
        homing_start_position[i] = axes[i].position;
        homing_steps_taken[i] = 0;
    }
    
    // Start first axis moving towards endstop
    float homing_vel = get_homing_velocity(0);
    float homing_dir = get_homing_direction(0);
    axes[0].target_velocity = homing_vel * homing_dir;
    
    ESP_LOGI(TAG, "Homing started - axis 0 (%s) at %.1f steps/sec, direction %.0f", 
             axis_names[0], homing_vel, homing_dir);
}

bool stepper_simple_is_homing(void) {
    return homing_active;
}

bool stepper_simple_is_preset_moving(void) {
    return preset_move_active;
}

