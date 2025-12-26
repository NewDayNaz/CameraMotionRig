/**
 * @file motion_controller.c
 * @brief Motion controller implementation
 */

#include "motion_controller.h"
#include "stepper_executor.h"
#include "board.h"
#include "homing.h"
#include "tmc2209.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char* TAG = "motion_controller";

static segment_queue_t segment_queue;
static motion_planner_t planner;
static bool controller_initialized = false;

void motion_controller_init(void) {
    if (controller_initialized) {
        return;
    }
    
    // Initialize segment queue
    segment_queue_init(&segment_queue);
    
    // Initialize motion planner
    motion_planner_init(&planner, &segment_queue);
    
    // Initialize stepper executor
    if (!stepper_executor_init(&segment_queue)) {
        ESP_LOGE(TAG, "Failed to initialize stepper executor");
        return;
    }
    
    // Sync planner positions with executor positions
    for (int i = 0; i < NUM_AXES; i++) {
        int32_t pos = stepper_executor_get_position(i);
        motion_planner_set_position(&planner, i, (float)pos);
    }
    
    // Initialize TMC2209 UART for sensorless homing (zoom axis) - only if enabled
    #if ZOOM_USE_SENSORLESS_HOMING
    if (!tmc2209_uart_init(TMC2209_UART_NUM)) {
        ESP_LOGW(TAG, "TMC2209 UART init failed - sensorless homing unavailable");
    } else {
        // Initialize TMC2209 driver for zoom axis
        if (tmc2209_init(AXIS_ZOOM)) {
            ESP_LOGI(TAG, "TMC2209 initialized for zoom axis (sensorless homing enabled)");
        } else {
            ESP_LOGW(TAG, "TMC2209 init failed for zoom axis");
        }
    }
    #else
    ESP_LOGI(TAG, "Zoom axis using physical endstop (sensorless homing disabled)");
    #endif
    
    // Initialize homing
    homing_init();
    
    controller_initialized = true;
    ESP_LOGI(TAG, "Motion controller initialized");
}

void motion_controller_update(float dt) {
    if (!controller_initialized) {
        return;
    }
    
    // Update motion planner
    motion_planner_update(&planner, dt);
    
    // Check if homing is active
    if (homing_is_active()) {
        homing_status_t homing = homing_get_status();
        float current_pos = motion_planner_get_position(&planner, homing.axis);
        
        // Read endstop state
        bool endstop_triggered = false;
        if (homing.axis < NUM_AXES && endstop_pins[homing.axis] != GPIO_NUM_NC) {
            endstop_triggered = (gpio_get_level(endstop_pins[homing.axis]) == 0);  // Active LOW
        }
        
        // Store current axis before update (to detect axis change in sequential homing)
        uint8_t previous_axis = homing.axis;
        
        // Update homing state machine
        bool homing_active = homing_update(dt, current_pos, endstop_triggered);
        
        // Check if axis changed (sequential homing - previous axis completed)
        homing_status_t homing_after = homing_get_status();
        if (homing_after.state != HOMING_IDLE && 
            homing_after.state != HOMING_COMPLETE && 
            homing_after.state != HOMING_ERROR &&
            homing_after.axis != previous_axis) {
            // Previous axis completed, set its position to 0
            stepper_executor_set_position(previous_axis, 0);
            motion_planner_set_position(&planner, previous_axis, 0.0f);
            ESP_LOGI(TAG, "Axis %d homed, continuing with axis %d", previous_axis, homing_after.axis);
            
            // For sensorless homing, reset stall readings counter
            if (homing_after.is_sensorless) {
                // Small delay to allow motor to start moving before checking for stall
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        }
        
        if (homing_active || homing_is_active()) {
            // Set velocity for homing move (use updated axis if it changed)
            float vel = homing_get_target_velocity();
            float velocities[3] = {0, 0, 0};
            velocities[homing_after.axis] = vel;
            motion_planner_set_velocities(&planner, velocities);
            motion_planner_set_manual_mode(&planner, true);
        } else {
            // Homing complete or error
            motion_planner_set_manual_mode(&planner, false);
            
            if (homing_after.state == HOMING_COMPLETE) {
                // All axes homed - set final axis position
                stepper_executor_set_position(homing_after.axis, 0);
                motion_planner_set_position(&planner, homing_after.axis, 0.0f);
                ESP_LOGI(TAG, "Sequential homing complete - all %d axes homed", homing_after.num_axes_to_home);
            } else if (homing_after.state == HOMING_ERROR) {
                ESP_LOGE(TAG, "Homing error occurred");
            }
        }
    }
}

bool motion_controller_goto_preset(uint8_t preset_index) {
    if (!controller_initialized) {
        return false;
    }
    
    preset_t preset;
    if (!preset_load(preset_index, &preset)) {
        ESP_LOGE(TAG, "Failed to load preset %d", preset_index);
        return false;
    }
    
    if (!preset.valid) {
        ESP_LOGE(TAG, "Preset %d is not valid", preset_index);
        return false;
    }
    
    // Apply approach mode if needed
    // For now, just do direct move
    if (preset.approach_mode == APPROACH_DIRECT) {
        // Get current positions
        float current_pos[NUM_AXES];
        for (int i = 0; i < NUM_AXES; i++) {
            current_pos[i] = motion_planner_get_position(&planner, i);
        }
        
        // Calculate distance for duration calculation
        float max_distance = 0.0f;
        for (int i = 0; i < NUM_AXES; i++) {
            float distance = fabsf(preset.pos[i] - current_pos[i]);
            if (distance > max_distance) {
                max_distance = distance;
            }
        }
        
        // Calculate duration based on preset parameters
        float duration = preset.duration_s;
        
        if (duration <= 0.0f) {
            // Auto-calculate duration
            if (preset.max_speed_scale > 0.0f) {
                // Use speed scale: calculate duration based on distance and speed
                // max_speed_scale is a multiplier (1.0 = normal speed, 2.0 = half speed, etc.)
                // Apply speed_multiplier as well
                float effective_speed_scale = preset.max_speed_scale * preset.speed_multiplier;
                if (effective_speed_scale <= 0.0f) {
                    effective_speed_scale = 1.0f;  // Default to normal speed
                }
                
                // Calculate duration: use average velocity based on preset limits
                // With speed scale > 1.0, we want slower motion (longer duration)
                // Base duration on distance and preset max velocity (scaled by microstepping)
                float base_velocity = PRESET_MAX_VELOCITY_ZOOM * MICROSTEP_SCALE;  // Use zoom as conservative base
                float effective_velocity = base_velocity / effective_speed_scale;
                duration = max_distance / effective_velocity;
                
                // Ensure minimum duration for smooth motion
                if (duration < 0.5f) {
                    duration = 0.5f;
                }
            } else {
                // No speed scale specified - let motion_planner calculate duration automatically
                duration = 0.0f;  // 0.0 means auto-calculate in motion_planner
            }
        } else {
            // Duration specified - apply speed_multiplier to scale it
            // speed_multiplier > 1.0 makes it slower (longer duration)
            // speed_multiplier < 1.0 makes it faster (shorter duration)
            if (preset.speed_multiplier > 0.0f && preset.speed_multiplier != 1.0f) {
                duration = duration / preset.speed_multiplier;
            }
        }
        
        // Apply per-preset multipliers by temporarily scaling preset limits
        // Note: motion_planner uses preset limits internally, so we need to scale them
        // For now, we'll pass multipliers as a note - in the future, we could add
        // a function to motion_planner to apply multipliers per-move
        // speed_multiplier: affects velocity (inverse relationship with duration)
        // accel_multiplier: affects acceleration (could be applied in future)
        
        // Enable precision mode if preferred
        motion_planner_set_precision_mode(&planner, preset.precision_preferred);
        
        ESP_LOGI(TAG, "Moving to preset %d: target=(%.1f, %.1f, %.1f) from current=(%.1f, %.1f, %.1f), duration=%.2fs, speed_mult=%.2f, accel_mult=%.2f", 
                preset_index, 
                preset.pos[0], preset.pos[1], preset.pos[2],
                current_pos[0], current_pos[1], current_pos[2],
                duration, preset.speed_multiplier, preset.accel_multiplier);
        
        bool success = motion_planner_plan_move(&planner, preset.pos, duration, preset.easing_type);
        if (!success) {
            ESP_LOGE(TAG, "Failed to plan move to preset %d", preset_index);
            return false;
        }
        
        ESP_LOGI(TAG, "Moving to preset %d", preset_index);
        return true;
    }
    
    // Other approach modes not yet implemented
    ESP_LOGW(TAG, "Approach mode %d not yet implemented", preset.approach_mode);
    return false;
}

bool motion_controller_save_preset(uint8_t preset_index) {
    if (!controller_initialized) {
        return false;
    }
    
    preset_t preset;
    preset_init_default(&preset);
    
    // Get current positions
    for (int i = 0; i < NUM_AXES; i++) {
        preset.pos[i] = motion_planner_get_position(&planner, i);
    }
    
    if (!preset_save(preset_index, &preset)) {
        ESP_LOGE(TAG, "Failed to save preset %d", preset_index);
        return false;
    }
    
    ESP_LOGI(TAG, "Saved current position as preset %d", preset_index);
    return true;
}

bool motion_controller_get_preset(uint8_t preset_index, preset_t* preset) {
    if (!controller_initialized || preset == NULL) {
        return false;
    }
    
    return preset_load(preset_index, preset);
}

bool motion_controller_update_preset(uint8_t preset_index, const preset_t* preset) {
    if (!controller_initialized || preset == NULL) {
        return false;
    }
    
    if (!preset_save(preset_index, preset)) {
        ESP_LOGE(TAG, "Failed to update preset %d", preset_index);
        return false;
    }
    
    ESP_LOGI(TAG, "Updated preset %d", preset_index);
    return true;
}

void motion_controller_set_velocities(const float velocities[3]) {
    if (!controller_initialized) {
        return;
    }
    
    motion_planner_set_velocities(&planner, velocities);
    motion_planner_set_manual_mode(&planner, true);
}

bool motion_controller_home(uint8_t axis) {
    if (!controller_initialized) {
        return false;
    }
    
    // Home all axes sequentially if axis == 255
    if (axis == 255 || axis >= NUM_AXES) {
        // Home PAN, TILT, and ZOOM sequentially
        // PAN and TILT use physical endstops, ZOOM uses sensorless
        uint8_t axes[] = {AXIS_PAN, AXIS_TILT, AXIS_ZOOM};
        return homing_start_sequential(axes, 3);
    }
    
    // Home single axis (supports all axes now - zoom uses sensorless)
    return homing_start(axis);
}

void motion_controller_get_positions(float positions[3]) {
    if (!controller_initialized) {
        for (int i = 0; i < 3; i++) {
            positions[i] = 0.0f;
        }
        return;
    }
    
    for (int i = 0; i < 3; i++) {
        positions[i] = motion_planner_get_position(&planner, i);
    }
}

void motion_controller_stop(void) {
    if (!controller_initialized) {
        return;
    }
    
    motion_planner_stop(&planner);
    homing_abort();
    ESP_LOGI(TAG, "Motion stopped");
}

void motion_controller_set_precision_mode(bool enabled) {
    if (!controller_initialized) {
        return;
    }
    
    motion_planner_set_precision_mode(&planner, enabled);
}

void motion_controller_set_limits(uint8_t axis, float min, float max) {
    if (!controller_initialized || axis >= NUM_AXES) {
        return;
    }
    
    motion_planner_set_limits(&planner, axis, min, max);
}

