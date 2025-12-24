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
        // Direct move
        float duration = preset.duration_s;
        if (preset.max_speed_scale > 0.0f && duration <= 0.0f) {
            // Calculate duration from speed scale
            // This is simplified - in production, calculate based on distance and speed
            duration = 2.0f / preset.max_speed_scale;
        }
        
        // Apply per-preset multipliers
        // TODO: Apply speed_multiplier and accel_multiplier
        
        // Enable precision mode if preferred
        motion_planner_set_precision_mode(&planner, preset.precision_preferred);
        
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

