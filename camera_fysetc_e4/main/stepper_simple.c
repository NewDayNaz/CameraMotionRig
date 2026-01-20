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
static int64_t preset_start_time;
static float preset_duration = 6.0f;  // Default 6 seconds

// Constants
#define MIN_STEP_DELAY_US 250  // Minimum step delay (max speed ~2000 steps/sec)
#define PRECISION_MULTIPLIER 0.25f  // Precision mode speed multiplier

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
    
    ESP_LOGI(TAG, "Simple stepper control initialized");
}

void stepper_simple_update(void) {
    if (!initialized) {
        return;
    }
    
    int64_t now_us = esp_timer_get_time();
    
    // Handle preset moves
    if (preset_move_active) {
        int64_t elapsed_us = now_us - preset_start_time;
        float elapsed_s = elapsed_us / 1000000.0f;
        
        if (elapsed_s >= preset_duration) {
            // Move complete - set positions to target
            for (int i = 0; i < NUM_AXES; i++) {
                axes[i].position = (int32_t)preset_target[i];
                axes[i].velocity = 0.0f;
                axes[i].target_velocity = 0.0f;
                axes[i].move_direction = 0;
                gpio_set_level(step_pins[i], 0);
            }
            preset_move_active = false;
            ESP_LOGI(TAG, "Preset move complete");
        } else {
            // Interpolate positions using linear interpolation
            float t = elapsed_s / preset_duration;
            
            for (int i = 0; i < NUM_AXES; i++) {
                // Calculate velocity needed to reach target
                float remaining = preset_target[i] - (float)axes[i].position;
                float remaining_time = preset_duration - elapsed_s;
                
                if (fabsf(remaining) > 0.5f && remaining_time > 0.01f) {
                    float required_vel = remaining / remaining_time;
                    axes[i].target_velocity = required_vel;
                } else {
                    axes[i].target_velocity = 0.0f;
                }
            }
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
    
    // Use preset duration or default
    preset_duration = (preset.duration_s > 0.0f) ? preset.duration_s : 6.0f;
    
    // Apply speed multiplier
    if (preset.speed_multiplier > 0.0f && preset.speed_multiplier != 1.0f) {
        preset_duration = preset_duration / preset.speed_multiplier;
    }
    
    preset_start_time = esp_timer_get_time();
    preset_move_active = true;
    preset_move_index = preset_index;
    
    // Stop manual velocity control
    for (int i = 0; i < NUM_AXES; i++) {
        axes[i].target_velocity = 0.0f;
    }
    
    ESP_LOGI(TAG, "Moving to preset %d: (%.1f, %.1f, %.1f) in %.2fs", 
             preset_index, preset_target[0], preset_target[1], preset_target[2], preset_duration);
    
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

