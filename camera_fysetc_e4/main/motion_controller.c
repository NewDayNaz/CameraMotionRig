/**
 * @file motion_controller.c
 * @brief Simplified motion controller with direct velocity control
 */

#include "motion_controller.h"
#include "stepper_executor.h"
#include "board.h"
#include "homing.h"
#include "tmc2209.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <math.h>

static const char* TAG = "motion_controller";

// Simple preset move state (linear interpolation)
static struct {
    bool active;
    float targets[NUM_AXES];
    float start_pos[NUM_AXES];
    float start_time;
    float duration;
    float current_time;
} preset_move;

// Soft limits (in steps, scaled by microstepping)
static float limits_min[NUM_AXES];
static float limits_max[NUM_AXES];

// Precision mode multiplier
static float precision_multiplier = 0.25f;
static bool precision_mode = false;

// Max velocities for manual/joystick control (full steps/sec, will be scaled by microstepping)
#define MAX_VELOCITY_PAN  500.0f
#define MAX_VELOCITY_TILT 500.0f
#define MAX_VELOCITY_ZOOM 50.0f

// Soft limit zones (percentage of travel)
#define SOFT_LIMIT_ZONE 0.05f  // Last 5% of travel

// Pan axis limits (degrees)
#define PAN_MAX_DEGREES 240.0f
#define PAN_STEPS_PER_DEGREE 100.0f

// Tilt axis limits (degrees)
#define TILT_MAX_DEGREES_DOWN 20210.0f
#define TILT_MAX_DEGREES_UP 27296.0f
#define TILT_STEPS_PER_DEGREE 0.556f

static bool controller_initialized = false;

// Idle detection: disable steppers after 5 minutes of inactivity
#define IDLE_TIMEOUT_US (5 * 60 * 1000 * 1000)  // 5 minutes in microseconds
static int64_t last_command_time[NUM_AXES];
static bool steppers_enabled = true;

// Preset goto debouncing
#define PRESET_GOTO_DEBOUNCE_MS 500
static int64_t last_preset_goto_time = 0;

// Preset save debouncing
#define PRESET_SAVE_DEBOUNCE_MS 500
static int64_t last_preset_save_time = 0;

/**
 * @brief Calculate soft limit velocity scaling factor
 */
static float calculate_soft_limit_scale(float pos, float min, float max) {
    float range = max - min;
    if (range <= 0.0f) return 1.0f;
    
    float zone_size = range * SOFT_LIMIT_ZONE;
    float distance_from_min = pos - min;
    float distance_from_max = max - pos;
    
    if (distance_from_min < zone_size) {
        // Near minimum
        float u = distance_from_min / zone_size;
        u = u * u * u * (u * (u * 6.0f - 15.0f) + 10.0f);  // Smootherstep
        return 0.1f + 0.9f * u;
    } else if (distance_from_max < zone_size) {
        // Near maximum
        float u = distance_from_max / zone_size;
        u = u * u * u * (u * (u * 6.0f - 15.0f) + 10.0f);  // Smootherstep
        return 0.1f + 0.9f * u;
    }
    
    return 1.0f;
}

void motion_controller_init(void) {
    if (controller_initialized) {
        return;
    }
    
    // Initialize segment queue (for preset moves, but we'll use simple linear interpolation)
    segment_queue_t segment_queue;
    segment_queue_init(&segment_queue);
    
    // Initialize stepper executor (with queue for preset moves, but manual mode bypasses it)
    if (!stepper_executor_init(&segment_queue)) {
        ESP_LOGE(TAG, "Failed to initialize stepper executor");
        return;
    }
    
    // Initialize TMC2209 UART for sensorless homing (zoom axis) - only if enabled
    #if ZOOM_USE_SENSORLESS_HOMING
    if (!tmc2209_uart_init(TMC2209_UART_NUM)) {
        ESP_LOGW(TAG, "TMC2209 UART init failed - sensorless homing unavailable");
    } else {
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
    
    // Initialize limits (scaled by microstepping)
    for (int i = 0; i < NUM_AXES; i++) {
        limits_min[i] = -100000.0f * MICROSTEP_SCALE;
        limits_max[i] = 100000.0f * MICROSTEP_SCALE;
    }
    
    // Set pan axis limits
    float pan_max_steps = (PAN_MAX_DEGREES / 2.0f) * PAN_STEPS_PER_DEGREE;
    limits_min[AXIS_PAN] = -pan_max_steps * MICROSTEP_SCALE;
    limits_max[AXIS_PAN] = pan_max_steps * MICROSTEP_SCALE;
    
    // Set tilt axis limits
    float tilt_down_steps = TILT_MAX_DEGREES_DOWN * TILT_STEPS_PER_DEGREE;
    float tilt_up_steps = TILT_MAX_DEGREES_UP * TILT_STEPS_PER_DEGREE;
    limits_min[AXIS_TILT] = -tilt_down_steps * MICROSTEP_SCALE;
    limits_max[AXIS_TILT] = tilt_up_steps * MICROSTEP_SCALE;
    
    // Initialize preset move state
    memset(&preset_move, 0, sizeof(preset_move));
    
    // Initialize idle detection
    int64_t now = esp_timer_get_time();
    for (int i = 0; i < NUM_AXES; i++) {
        last_command_time[i] = now;
    }
    steppers_enabled = true;
    
    controller_initialized = true;
    ESP_LOGI(TAG, "Motion controller initialized (simplified)");
}

void motion_controller_update(float dt) {
    if (!controller_initialized) {
        return;
    }
    
    // Update preset move (velocity-based linear interpolation)
    if (preset_move.active) {
        preset_move.current_time += dt;
        
        // Enable manual mode for preset moves (uses velocity control)
        stepper_executor_set_manual_mode(true);
        
        if (preset_move.current_time >= preset_move.duration) {
            // Move complete - stop motion and snap to target
            float velocities[3] = {0, 0, 0};
            motion_controller_set_velocities(velocities);
            for (int i = 0; i < NUM_AXES; i++) {
                int32_t target_steps = (int32_t)preset_move.targets[i];
                stepper_executor_set_position(i, target_steps);
            }
            preset_move.active = false;
            stepper_executor_set_manual_mode(false);
            ESP_LOGI(TAG, "Preset move complete");
        } else {
            // Calculate required velocity to reach target in remaining time
            float remaining_time = preset_move.duration - preset_move.current_time;
            if (remaining_time > 0.001f) {  // Avoid division by zero
                float velocities[3] = {0, 0, 0};
                
                for (int i = 0; i < NUM_AXES; i++) {
                    int32_t current_pos = stepper_executor_get_position(i);
                    float distance = preset_move.targets[i] - (float)current_pos;
                    
                    // Calculate velocity to cover remaining distance in remaining time
                    velocities[i] = distance / remaining_time;
                }
                
                // Apply velocities (with limits already enforced in set_velocities)
                motion_controller_set_velocities(velocities);
            }
        }
    }
    
    // Check for idle steppers
    int64_t now = esp_timer_get_time();
    bool any_axis_active = false;
    
    for (int i = 0; i < NUM_AXES; i++) {
        if ((now - last_command_time[i]) < IDLE_TIMEOUT_US) {
            any_axis_active = true;
            break;
        }
    }
    
    if (homing_is_active() || preset_move.active || stepper_executor_is_busy()) {
        any_axis_active = true;
        for (int i = 0; i < NUM_AXES; i++) {
            last_command_time[i] = now;
        }
    }
    
    if (any_axis_active) {
        if (!steppers_enabled) {
            board_set_enable(true);
            steppers_enabled = true;
            ESP_LOGI(TAG, "Steppers enabled");
        }
    } else {
        if (steppers_enabled) {
            board_set_enable(false);
            steppers_enabled = false;
            ESP_LOGI(TAG, "Steppers disabled (idle)");
        }
    }
    
    // Update homing
    if (homing_is_active()) {
        homing_status_t homing = homing_get_status();
        int32_t current_pos = stepper_executor_get_position(homing.axis);
        
        bool endstop_triggered = false;
        if (homing.axis < NUM_AXES && endstop_pins[homing.axis] != GPIO_NUM_NC) {
            endstop_triggered = (gpio_get_level(endstop_pins[homing.axis]) == 0);
        }
        
        uint8_t previous_axis = homing.axis;
        bool homing_active = homing_update(dt, (float)current_pos, endstop_triggered);
        homing_status_t homing_after = homing_get_status();
        
        if (homing_after.state != HOMING_IDLE && 
            homing_after.state != HOMING_COMPLETE && 
            homing_after.state != HOMING_ERROR &&
            homing_after.axis != previous_axis) {
            stepper_executor_set_position(previous_axis, 0);
            ESP_LOGI(TAG, "Axis %d homed, continuing with axis %d", previous_axis, homing_after.axis);
        }
        
        if (homing_active || homing_is_active()) {
            float vel = homing_get_target_velocity();
            float velocities[3] = {0, 0, 0};
            velocities[homing_after.axis] = vel;
            
            last_command_time[homing_after.axis] = now;
            motion_controller_set_velocities(velocities);
        } else {
            stepper_executor_set_manual_mode(false);
            if (homing_after.state == HOMING_COMPLETE) {
                stepper_executor_set_position(homing_after.axis, 0);
                ESP_LOGI(TAG, "Homing complete");
            }
        }
    }
}

bool motion_controller_goto_preset(uint8_t preset_index) {
    if (!controller_initialized) {
        return false;
    }
    
    // Debounce
    int64_t now = esp_timer_get_time();
    if ((now - last_preset_goto_time) < (PRESET_GOTO_DEBOUNCE_MS * 1000)) {
        return false;
    }
    last_preset_goto_time = now;
    
    preset_t preset;
    if (!preset_load(preset_index, &preset) || !preset.valid) {
        ESP_LOGE(TAG, "Preset %d not found or invalid", preset_index);
        return false;
    }
    
    // Stop any current motion
    float zero_vel[3] = {0, 0, 0};
    motion_controller_set_velocities(zero_vel);
    
    // Get current positions
    for (int i = 0; i < NUM_AXES; i++) {
        preset_move.start_pos[i] = (float)stepper_executor_get_position(i);
        preset_move.targets[i] = preset.pos[i];
    }
    
    // Calculate duration (simple: based on max distance and fixed speed)
    float max_distance = 0.0f;
    for (int i = 0; i < NUM_AXES; i++) {
        float distance = fabsf(preset_move.targets[i] - preset_move.start_pos[i]);
        if (distance > max_distance) {
            max_distance = distance;
        }
    }
    
    // Use a conservative speed for preset moves (scaled by microstepping)
    float preset_speed = 100.0f * MICROSTEP_SCALE;  // steps/sec
    float duration = max_distance / preset_speed;
    
    // Minimum duration
    if (duration < 0.5f) {
        duration = 0.5f;
    }
    
    // Apply preset speed multiplier if specified
    if (preset.speed_multiplier > 0.0f && preset.speed_multiplier != 1.0f) {
        duration = duration / preset.speed_multiplier;
    }
    
    preset_move.active = true;
    preset_move.start_time = (float)esp_timer_get_time() / 1e6f;
    preset_move.duration = duration;
    preset_move.current_time = 0.0f;
    
    ESP_LOGI(TAG, "Moving to preset %d: duration=%.2fs", preset_index, duration);
    return true;
}

bool motion_controller_save_preset(uint8_t preset_index) {
    if (!controller_initialized) {
        return false;
    }
    
    // Debounce
    int64_t now = esp_timer_get_time();
    if ((now - last_preset_save_time) < (PRESET_SAVE_DEBOUNCE_MS * 1000)) {
        return false;
    }
    last_preset_save_time = now;
    
    preset_t preset;
    preset_init_default(&preset);
    
    for (int i = 0; i < NUM_AXES; i++) {
        preset.pos[i] = (float)stepper_executor_get_position(i);
    }
    
    if (!preset_save(preset_index, &preset)) {
        ESP_LOGE(TAG, "Failed to save preset %d", preset_index);
        return false;
    }
    
    ESP_LOGI(TAG, "Saved preset %d", preset_index);
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
    return preset_save(preset_index, preset);
}

void motion_controller_set_velocities(const float velocities[3]) {
    if (!controller_initialized) {
        return;
    }
    
    // Enable manual mode
    stepper_executor_set_manual_mode(true);
    
    // Convert from full steps/sec to microsteps/sec and apply limits
    float max_velocities[3] = {
        MAX_VELOCITY_PAN * MICROSTEP_SCALE,
        MAX_VELOCITY_TILT * MICROSTEP_SCALE,
        MAX_VELOCITY_ZOOM * MICROSTEP_SCALE
    };
    
    // Update command timestamps
    int64_t now = esp_timer_get_time();
    for (int i = 0; i < NUM_AXES; i++) {
        float vel = velocities[i] * MICROSTEP_SCALE;
        
        // Apply precision mode
        if (precision_mode) {
            vel *= precision_multiplier;
        }
        
        // Check soft limits
        int32_t current_pos = stepper_executor_get_position(i);
        float limit_scale = calculate_soft_limit_scale((float)current_pos, limits_min[i], limits_max[i]);
        
        // If moving toward limit, reduce velocity
        if (vel > 0.0f && current_pos >= limits_max[i]) {
            vel = 0.0f;  // At max limit, stop
        } else if (vel < 0.0f && current_pos <= limits_min[i]) {
            vel = 0.0f;  // At min limit, stop
        } else {
            vel *= limit_scale;
        }
        
        // Clamp to max velocity
        if (vel > max_velocities[i]) {
            vel = max_velocities[i];
        } else if (vel < -max_velocities[i]) {
            vel = -max_velocities[i];
        }
        
        stepper_executor_set_velocity(i, vel);
        
        if (fabsf(vel) > 0.1f) {
            last_command_time[i] = now;
        }
    }
}

bool motion_controller_home(uint8_t axis) {
    if (!controller_initialized) {
        return false;
    }
    
    int64_t now = esp_timer_get_time();
    if (axis == 255 || axis >= NUM_AXES) {
        for (int i = 0; i < NUM_AXES; i++) {
            last_command_time[i] = now;
        }
        uint8_t axes[] = {AXIS_PAN, AXIS_TILT, AXIS_ZOOM};
        return homing_start_sequential(axes, 3);
    }
    
    last_command_time[axis] = now;
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
        positions[i] = (float)stepper_executor_get_position(i);
    }
}

void motion_controller_stop(void) {
    if (!controller_initialized) {
        return;
    }
    
    stepper_executor_set_manual_mode(false);
    preset_move.active = false;
    homing_abort();
    
    // Stop all velocities
    for (int i = 0; i < NUM_AXES; i++) {
        stepper_executor_set_velocity(i, 0.0f);
    }
    
    ESP_LOGI(TAG, "Motion stopped");
}

void motion_controller_set_precision_mode(bool enabled) {
    precision_mode = enabled;
}

void motion_controller_set_limits(uint8_t axis, float min, float max) {
    if (axis < NUM_AXES) {
        limits_min[axis] = min * MICROSTEP_SCALE;
        limits_max[axis] = max * MICROSTEP_SCALE;
    }
}