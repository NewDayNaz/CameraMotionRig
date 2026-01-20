/**
 * @file stepper_simple.c
 * @brief Simple direct stepper control implementation
 * 
 * Based on camera_async simple approach but adapted for ESP-IDF and FYSETC E4 board.
 */

#include "stepper_simple.h"
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
static bool precision_mode = false;
static bool homing_active = false;
static uint8_t homing_axis = 0;

// Preset move state
static bool preset_move_active = false;
static uint8_t preset_move_index = 0;
static float preset_target[3];
static float preset_start[3];
static float preset_max_speed[3];      // Max speed per axis (steps/sec)
static float preset_accel_factor;      // Acceleration factor
static float preset_decel_factor;      // Deceleration factor (most important for accuracy)
static float preset_total_distance[3]; // Total distance to travel per axis
static float preset_decel_start_distance[3]; // Distance at which to start decelerating

// Constants
#define MIN_STEP_DELAY_US 250  // Minimum step delay (max speed ~2000 steps/sec)
#define PRECISION_MULTIPLIER 0.25f  // Precision mode speed multiplier
#define MIN_ZOOM_VELOCITY 10.0f  // Minimum zoom velocity (steps/sec) to prevent stalling

// Helper function to convert velocity to step delay
static uint32_t velocity_to_step_delay(float velocity) {
    if (fabsf(velocity) < 0.1f) {
        return 0;  // Stopped
    }
    
    // Apply precision mode
    if (precision_mode) {
        velocity *= PRECISION_MULTIPLIER;
    }
    
    // Calculate step delay: delay_us = 1000000 / velocity
    uint32_t delay_us = (uint32_t)(1000000.0f / fabsf(velocity));
    
    // Limit minimum delay
    if (delay_us < MIN_STEP_DELAY_US) {
        delay_us = MIN_STEP_DELAY_US;
    }
    
    return delay_us;
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
    precision_mode = false;
    homing_active = false;
    preset_move_active = false;
    
    // Initialize preset move state
    for (int i = 0; i < NUM_AXES; i++) {
        preset_total_distance[i] = 0.0f;
        preset_decel_start_distance[i] = 0.0f;
        preset_max_speed[i] = 0.0f;
    }
    
    ESP_LOGI(TAG, "Simple stepper control initialized");
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
            // Calculate remaining distance
            float current_pos = (float)axes[i].position;
            float remaining = preset_target[i] - current_pos;
            float distance_traveled = preset_total_distance[i] - fabsf(remaining);
            
            // Check if at target
            if (fabsf(remaining) < 0.5f) {
                // At target - stop immediately
                axes[i].target_velocity = 0.0f;
                axes[i].position = (int32_t)preset_target[i];  // Snap to exact target
            } else {
                all_at_target = false;
                
                // Distance-based velocity calculation with acceleration/deceleration zones
                float remaining_abs = fabsf(remaining);
                float max_vel = preset_max_speed[i];
                
                // Distance-based velocity with acceleration/deceleration zones
                // Deceleration zone: percentage of total distance where we slow down
                // Higher decel_factor = larger decel zone = gentler slowdown = more accurate positioning
                // decel_factor of 1.0 = 30% of distance for decel, 2.0 = 50%, 3.0 = 60%, etc.
                float decel_zone_percent = 0.3f * preset_decel_factor;  // Base 30% * factor
                if (decel_zone_percent > 0.8f) decel_zone_percent = 0.8f;  // Cap at 80% max
                float decel_zone_size = preset_total_distance[i] * decel_zone_percent;
                
                // Acceleration zone: start of movement where we ramp up
                // Higher accel_factor = smaller accel zone = faster to max speed
                // accel_factor of 1.0 = 20% of distance for accel, 2.0 = 10%, etc.
                float accel_zone_percent = 0.2f / preset_accel_factor;  // Base 20% / factor
                if (accel_zone_percent > 0.5f) accel_zone_percent = 0.5f;  // Cap at 50% max
                float accel_zone_size = preset_total_distance[i] * accel_zone_percent;
                float cruise_zone_start = accel_zone_size;
                float decel_zone_start = preset_total_distance[i] - decel_zone_size;
                
                float target_vel;
                if (remaining_abs <= decel_zone_size) {
                    // Deceleration zone - reduce speed linearly as we approach target
                    // This is most important for accurate positioning
                    float speed_factor = remaining_abs / decel_zone_size;
                    if (speed_factor < 0.05f) speed_factor = 0.05f;  // Minimum 5% speed for final approach
                    target_vel = max_vel * speed_factor;
                } else if (distance_traveled < accel_zone_size) {
                    // Acceleration zone - ramp up speed (handled by slew rate limiting)
                    // Could use linear ramp here if desired
                    float speed_factor = distance_traveled / accel_zone_size;
                    if (speed_factor < 0.2f) speed_factor = 0.2f;  // Start at 20% speed
                    target_vel = max_vel * speed_factor;
                } else {
                    // Cruise zone - maintain max speed
                    target_vel = max_vel;
                }
                
                // Apply minimum velocity to zoom axis for preset moves
                // (but allow stopping when very close to target)
                if (i == AXIS_ZOOM && remaining_abs > 1.0f) {
                    if (fabsf(target_vel) < MIN_ZOOM_VELOCITY) {
                        target_vel = (target_vel >= 0) ? MIN_ZOOM_VELOCITY : -MIN_ZOOM_VELOCITY;
                    }
                }
                
                // Set direction and target velocity
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
        // Read endstop
        bool endstop_triggered = false;
        if (homing_axis < NUM_AXES && endstop_pins[homing_axis] != GPIO_NUM_NC) {
            endstop_triggered = (gpio_get_level(endstop_pins[homing_axis]) == 0);  // Active LOW
        }
        
        if (endstop_triggered) {
            // Endstop hit - stop and set position to 0
            axes[homing_axis].position = 0;
            axes[homing_axis].velocity = 0.0f;
            axes[homing_axis].target_velocity = 0.0f;
            axes[homing_axis].move_direction = 0;
            gpio_set_level(step_pins[homing_axis], 0);
            
            homing_axis++;
            if (homing_axis >= NUM_AXES) {
                homing_active = false;
                ESP_LOGI(TAG, "Homing complete");
            } else {
                // Start homing next axis
                axes[homing_axis].target_velocity = -100.0f;  // Move towards endstop
                ESP_LOGI(TAG, "Homing axis %d", homing_axis);
            }
        } else {
            // Move towards endstop
            axes[homing_axis].target_velocity = -100.0f;  // Move towards endstop
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
                // Generate step pulse
                gpio_set_level(step_pins[i], 1);
                esp_rom_delay_us(1);  // Very short pulse
                gpio_set_level(step_pins[i], 0);
                
                // Update position
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
    
    // Cancel any active preset moves
    preset_move_active = false;
    
    axes[AXIS_PAN].target_velocity = pan_vel;
    axes[AXIS_TILT].target_velocity = tilt_vel;
    
    // Apply minimum velocity to zoom axis (but allow zero for stopping)
    if (fabsf(zoom_vel) > 0.1f && fabsf(zoom_vel) < MIN_ZOOM_VELOCITY) {
        // Clamp to minimum velocity with correct sign
        axes[AXIS_ZOOM].target_velocity = (zoom_vel > 0) ? MIN_ZOOM_VELOCITY : -MIN_ZOOM_VELOCITY;
    } else {
        axes[AXIS_ZOOM].target_velocity = zoom_vel;
    }
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
        } else {
            preset_max_speed[i] = 0.0f;
        }
    }
    
    // Store acceleration/deceleration factors
    preset_accel_factor = preset.accel_factor;
    preset_decel_factor = (preset.decel_factor > 0.1f) ? preset.decel_factor : 1.0f;  // Default to 1.0 if invalid
    
    preset_move_active = true;
    preset_move_index = preset_index;
    
    // Stop manual velocity control
    for (int i = 0; i < NUM_AXES; i++) {
        axes[i].target_velocity = 0.0f;
    }
    
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
    
    // Start first axis moving towards endstop
    axes[0].target_velocity = -100.0f;  // Move towards endstop
    
    ESP_LOGI(TAG, "Homing started");
}

void stepper_simple_set_precision_mode(bool enabled) {
    precision_mode = enabled;
    ESP_LOGI(TAG, "Precision mode: %s", enabled ? "ON" : "OFF");
}

