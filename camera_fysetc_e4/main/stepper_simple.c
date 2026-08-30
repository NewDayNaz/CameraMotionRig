/**
 * @file stepper_simple.c
 * @brief Simple direct stepper control implementation
 *
 * Preset recall uses constant-velocity positioning with uni-directional
 * approach (industry-standard backlash compensation). Manual jog keeps
 * slew-limited velocity for a smooth feel.
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

typedef enum {
    PRESET_AXIS_IDLE = 0,
    PRESET_AXIS_OVERSHOOT,
    PRESET_AXIS_APPROACH,
    PRESET_AXIS_DONE,
} preset_axis_phase_t;

typedef struct {
    int32_t position;
    float velocity;
    float target_velocity;
    int move_direction;
    uint32_t step_delay_us;
    int64_t last_step_time;
} axis_state_t;

static axis_state_t axes[NUM_AXES];
static bool initialized = false;
static bool homing_active = false;
static bool startup_homing = false;
static uint8_t homing_axis = 0;
static int32_t homing_start_position[NUM_AXES];
static int32_t homing_steps_taken[NUM_AXES];

static bool preset_move_active = false;
static uint8_t preset_move_index = 0;
static int32_t preset_final_target[NUM_AXES];
static int32_t preset_immediate_goal[NUM_AXES];
static float preset_speed[NUM_AXES];
static preset_axis_phase_t preset_phase[NUM_AXES];

#define MIN_STEP_DELAY_US 250

static float get_homing_velocity(uint8_t axis)
{
    if (axis == AXIS_PAN) {
        return HOMING_PAN_VELOCITY;
    }
    if (axis == AXIS_TILT) {
        return HOMING_TILT_VELOCITY;
    }
    if (axis == AXIS_ZOOM) {
        return HOMING_ZOOM_VELOCITY;
    }
    return 200.0f;
}

static float get_homing_direction(uint8_t axis)
{
    if (axis == AXIS_PAN) {
        return (float)HOMING_PAN_DIRECTION;
    }
    if (axis == AXIS_TILT) {
        return (float)HOMING_TILT_DIRECTION;
    }
    if (axis == AXIS_ZOOM) {
        return (float)HOMING_ZOOM_DIRECTION;
    }
    return -1.0f;
}

static uint32_t velocity_to_step_delay(float velocity)
{
    if (fabsf(velocity) < 0.1f) {
        return 0;
    }

    uint32_t delay_us = (uint32_t)(1000000.0f / fabsf(velocity));
    if (delay_us < MIN_STEP_DELAY_US) {
        delay_us = MIN_STEP_DELAY_US;
    }
    return delay_us;
}

static float get_axis_max_velocity(uint8_t axis)
{
    if (axis == AXIS_PAN) {
        return MAX_PAN_VELOCITY;
    }
    if (axis == AXIS_TILT) {
        return MAX_TILT_VELOCITY;
    }
    if (axis == AXIS_ZOOM) {
        return MAX_ZOOM_VELOCITY;
    }
    return MAX_PAN_VELOCITY;
}

static float get_default_preset_speed(uint8_t axis)
{
    if (axis == AXIS_ZOOM) {
        return PRESET_ZOOM_VELOCITY;
    }
    return PRESET_PAN_TILT_VELOCITY;
}

/**
 * Uni-directional approach: always finish the move with positive step count.
 * If currently above target, overshoot below first, then approach upward.
 */
static void setup_axis_preset_move(uint8_t axis, int32_t current, int32_t target, float speed)
{
    preset_final_target[axis] = target;
    preset_speed[axis] = speed;

    if (current == target) {
        preset_phase[axis] = PRESET_AXIS_DONE;
        preset_immediate_goal[axis] = target;
        return;
    }

    if (current <= target) {
        preset_phase[axis] = PRESET_AXIS_APPROACH;
        preset_immediate_goal[axis] = target;
        return;
    }

    preset_phase[axis] = PRESET_AXIS_OVERSHOOT;
    preset_immediate_goal[axis] = target - PRESET_BACKLASH_STEPS;
}

static void advance_preset_axis(uint8_t axis)
{
    int32_t pos = axes[axis].position;
    int32_t goal = preset_immediate_goal[axis];

    if (pos != goal) {
        return;
    }

    if (preset_phase[axis] == PRESET_AXIS_OVERSHOOT) {
        preset_phase[axis] = PRESET_AXIS_APPROACH;
        preset_immediate_goal[axis] = preset_final_target[axis];
        return;
    }

    if (preset_phase[axis] == PRESET_AXIS_APPROACH) {
        axes[axis].position = preset_final_target[axis];
        axes[axis].target_velocity = 0.0f;
        axes[axis].velocity = 0.0f;
        preset_phase[axis] = PRESET_AXIS_DONE;
    }
}

static void update_preset_axis_velocity(uint8_t axis)
{
    if (preset_phase[axis] == PRESET_AXIS_IDLE || preset_phase[axis] == PRESET_AXIS_DONE) {
        axes[axis].target_velocity = 0.0f;
        return;
    }

    advance_preset_axis(axis);

    if (preset_phase[axis] == PRESET_AXIS_DONE) {
        axes[axis].target_velocity = 0.0f;
        return;
    }

    int32_t pos = axes[axis].position;
    int32_t goal = preset_immediate_goal[axis];
    float speed = preset_speed[axis];

    if (pos < goal) {
        axes[axis].target_velocity = speed;
    } else if (pos > goal) {
        axes[axis].target_velocity = -speed;
    } else {
        axes[axis].target_velocity = 0.0f;
        advance_preset_axis(axis);
    }
}

void stepper_simple_init(void)
{
    if (initialized) {
        return;
    }

    for (int i = 0; i < NUM_AXES; i++) {
        axes[i].position = 0;
        axes[i].velocity = 0.0f;
        axes[i].target_velocity = 0.0f;
        axes[i].move_direction = 0;
        axes[i].step_delay_us = 0;
        axes[i].last_step_time = esp_timer_get_time();
        homing_start_position[i] = 0;
        homing_steps_taken[i] = 0;
        preset_phase[i] = PRESET_AXIS_IDLE;
    }

    initialized = true;
    homing_active = false;
    preset_move_active = false;

    ESP_LOGI(TAG, "Simple stepper control initialized");

    startup_homing = true;
    stepper_simple_home();
}

static void start_startup_preset(void)
{
    if (!startup_homing) {
        return;
    }

    startup_homing = false;

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

void stepper_simple_update(void)
{
    if (!initialized) {
        return;
    }

    int64_t now_us = esp_timer_get_time();

    if (preset_move_active) {
        bool all_done = true;

        for (int i = 0; i < NUM_AXES; i++) {
            update_preset_axis_velocity(i);
            if (preset_phase[i] != PRESET_AXIS_DONE) {
                all_done = false;
            }
        }

        if (all_done) {
            preset_move_active = false;
            ESP_LOGI(TAG, "Preset move complete (pos: %ld, %ld, %ld)",
                     (long)axes[AXIS_PAN].position,
                     (long)axes[AXIS_TILT].position,
                     (long)axes[AXIS_ZOOM].position);
        }
    }

    if (homing_active) {
        if (homing_axis < NUM_AXES) {
            int32_t current_pos = axes[homing_axis].position;
            int32_t steps_from_start = current_pos - homing_start_position[homing_axis];
            homing_steps_taken[homing_axis] =
                (steps_from_start < 0) ? -steps_from_start : steps_from_start;

            float max_range = 0.0f;
            if (homing_axis == AXIS_PAN) {
                max_range = MAX_PAN_RANGE_STEPS;
            } else if (homing_axis == AXIS_TILT) {
                max_range = MAX_TILT_RANGE_STEPS;
            } else if (homing_axis == AXIS_ZOOM) {
                max_range = MAX_ZOOM_RANGE_STEPS;
            }

            if (homing_steps_taken[homing_axis] >= (int32_t)max_range) {
                ESP_LOGW(TAG,
                         "Homing axis %d: Max range reached (%d steps), assuming current position as home",
                         homing_axis, homing_steps_taken[homing_axis]);
                axes[homing_axis].position = 0;
                axes[homing_axis].velocity = 0.0f;
                axes[homing_axis].target_velocity = 0.0f;
                axes[homing_axis].move_direction = 0;
                gpio_set_level(step_pins[homing_axis], 0);

                homing_axis++;
                if (homing_axis >= NUM_AXES) {
                    homing_active = false;
                    ESP_LOGI(TAG, "Homing complete (some axes may have bailed out)");
                    start_startup_preset();
                } else {
                    homing_start_position[homing_axis] = axes[homing_axis].position;
                    homing_steps_taken[homing_axis] = 0;
                    float homing_vel = get_homing_velocity(homing_axis);
                    float homing_dir = get_homing_direction(homing_axis);
                    axes[homing_axis].target_velocity = homing_vel * homing_dir;
                    ESP_LOGI(TAG, "Homing axis %d (%s) at %.1f steps/sec, direction %.0f",
                             homing_axis, axis_names[homing_axis], homing_vel, homing_dir);
                }
            } else {
                bool endstop_triggered = false;
                if (endstop_pins[homing_axis] != GPIO_NUM_NC) {
                    endstop_triggered = (gpio_get_level(endstop_pins[homing_axis]) == 0);
                }

                if (endstop_triggered) {
                    ESP_LOGI(TAG, "Homing axis %d (%s): Endstop hit after %d steps",
                             homing_axis, axis_names[homing_axis], homing_steps_taken[homing_axis]);
                    axes[homing_axis].position = 0;
                    axes[homing_axis].velocity = 0.0f;
                    axes[homing_axis].target_velocity = 0.0f;
                    axes[homing_axis].move_direction = 0;
                    gpio_set_level(step_pins[homing_axis], 0);

                    homing_axis++;
                    if (homing_axis >= NUM_AXES) {
                        homing_active = false;
                        ESP_LOGI(TAG, "Homing complete");
                        start_startup_preset();
                    } else {
                        homing_start_position[homing_axis] = axes[homing_axis].position;
                        homing_steps_taken[homing_axis] = 0;
                        float homing_vel = get_homing_velocity(homing_axis);
                        float homing_dir = get_homing_direction(homing_axis);
                        axes[homing_axis].target_velocity = homing_vel * homing_dir;
                        ESP_LOGI(TAG, "Homing axis %d (%s) at %.1f steps/sec, direction %.0f",
                                 homing_axis, axis_names[homing_axis], homing_vel, homing_dir);
                    }
                } else {
                    float homing_vel = get_homing_velocity(homing_axis);
                    float homing_dir = get_homing_direction(homing_axis);
                    axes[homing_axis].target_velocity = homing_vel * homing_dir;
                }
            }
        }
    }

    for (int i = 0; i < NUM_AXES; i++) {
        if (preset_move_active &&
            preset_phase[i] != PRESET_AXIS_IDLE &&
            preset_phase[i] != PRESET_AXIS_DONE) {
            axes[i].velocity = axes[i].target_velocity;
        } else {
            float vel_diff = axes[i].target_velocity - axes[i].velocity;

            if (fabsf(axes[i].target_velocity) < 0.1f) {
                axes[i].velocity = axes[i].target_velocity;
            } else {
                float max_vel_change = 2000.0f;
                float dt = 0.001f;
                float max_change = max_vel_change * dt;

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
        }

        axes[i].step_delay_us = velocity_to_step_delay(axes[i].velocity);

        bool reverse_direction = (i == AXIS_PAN || i == AXIS_TILT);

        if (axes[i].velocity > 0.1f) {
            axes[i].move_direction = 1;
            gpio_set_level(dir_pins[i], reverse_direction ? 0 : 1);
        } else if (axes[i].velocity < -0.1f) {
            axes[i].move_direction = 2;
            gpio_set_level(dir_pins[i], reverse_direction ? 1 : 0);
        } else {
            axes[i].move_direction = 0;
            gpio_set_level(step_pins[i], 0);
            continue;
        }

        if (axes[i].step_delay_us > 0) {
            int64_t time_since_last_step = now_us - axes[i].last_step_time;

            if (time_since_last_step >= axes[i].step_delay_us) {
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

void stepper_simple_set_velocities(float pan_vel, float tilt_vel, float zoom_vel)
{
    if (!initialized) {
        return;
    }

    if (homing_active) {
        ESP_LOGW(TAG, "Velocity command blocked - homing in progress");
        return;
    }

    preset_move_active = false;

    if (fabsf(pan_vel) > 0.1f) {
        if (fabsf(pan_vel) < MIN_PAN_TILT_VELOCITY) {
            pan_vel = (pan_vel > 0) ? MIN_PAN_TILT_VELOCITY : -MIN_PAN_TILT_VELOCITY;
        } else if (fabsf(pan_vel) > MAX_PAN_VELOCITY) {
            pan_vel = (pan_vel > 0) ? MAX_PAN_VELOCITY : -MAX_PAN_VELOCITY;
        }
    }
    axes[AXIS_PAN].target_velocity = pan_vel;

    if (fabsf(tilt_vel) > 0.1f) {
        if (fabsf(tilt_vel) < MIN_PAN_TILT_VELOCITY) {
            tilt_vel = (tilt_vel > 0) ? MIN_PAN_TILT_VELOCITY : -MIN_PAN_TILT_VELOCITY;
        } else if (fabsf(tilt_vel) > MAX_TILT_VELOCITY) {
            tilt_vel = (tilt_vel > 0) ? MAX_TILT_VELOCITY : -MAX_TILT_VELOCITY;
        }
    }
    axes[AXIS_TILT].target_velocity = tilt_vel;

    if (fabsf(zoom_vel) > 0.1f) {
        if (fabsf(zoom_vel) < MIN_ZOOM_VELOCITY) {
            zoom_vel = (zoom_vel > 0) ? MIN_ZOOM_VELOCITY : -MIN_ZOOM_VELOCITY;
        } else if (fabsf(zoom_vel) > MAX_ZOOM_VELOCITY) {
            zoom_vel = (zoom_vel > 0) ? MAX_ZOOM_VELOCITY : -MAX_ZOOM_VELOCITY;
        }
    }
    axes[AXIS_ZOOM].target_velocity = zoom_vel;
}

void stepper_simple_get_positions(float* pan, float* tilt, float* zoom)
{
    if (!initialized || pan == NULL || tilt == NULL || zoom == NULL) {
        if (pan) {
            *pan = 0.0f;
        }
        if (tilt) {
            *tilt = 0.0f;
        }
        if (zoom) {
            *zoom = 0.0f;
        }
        return;
    }

    *pan = (float)axes[AXIS_PAN].position;
    *tilt = (float)axes[AXIS_TILT].position;
    *zoom = (float)axes[AXIS_ZOOM].position;
}

void stepper_simple_stop(void)
{
    if (!initialized) {
        return;
    }

    for (int i = 0; i < NUM_AXES; i++) {
        axes[i].target_velocity = 0.0f;
        preset_phase[i] = PRESET_AXIS_IDLE;
    }

    preset_move_active = false;
    homing_active = false;
}

bool stepper_simple_goto_preset(uint8_t preset_index)
{
    if (!initialized) {
        return false;
    }

    preset_t preset;
    if (!preset_load(preset_index, &preset) || !preset.valid) {
        ESP_LOGE(TAG, "Preset %d not found or invalid", preset_index);
        return false;
    }

    float start_pos[NUM_AXES];
    stepper_simple_get_positions(&start_pos[0], &start_pos[1], &start_pos[2]);

    for (int i = 0; i < NUM_AXES; i++) {
        int32_t current = (int32_t)start_pos[i];
        int32_t target = (int32_t)lroundf(preset.pos[i]);

        float speed = get_default_preset_speed(i);
        if (preset.max_speed > 0.0f) {
            speed = preset.max_speed;
        }

        float max_vel = get_axis_max_velocity(i);
        if (speed > max_vel) {
            speed = max_vel;
        }

        setup_axis_preset_move(i, current, target, speed);
    }

    preset_move_active = true;
    preset_move_index = preset_index;

    for (int i = 0; i < NUM_AXES; i++) {
        axes[i].velocity = 0.0f;
        axes[i].target_velocity = 0.0f;
    }

    ESP_LOGI(TAG,
             "Preset %d: target (%ld, %ld, %ld) from (%ld, %ld, %ld) — constant velocity, uni-directional approach",
             preset_index,
             (long)preset_final_target[0], (long)preset_final_target[1], (long)preset_final_target[2],
             (long)axes[AXIS_PAN].position, (long)axes[AXIS_TILT].position, (long)axes[AXIS_ZOOM].position);

    return true;
}

bool stepper_simple_save_preset(uint8_t preset_index)
{
    if (!initialized) {
        return false;
    }

    preset_t preset;
    preset_init_default(&preset);

    stepper_simple_get_positions(&preset.pos[0], &preset.pos[1], &preset.pos[2]);

    if (!preset_save(preset_index, &preset)) {
        ESP_LOGE(TAG, "Failed to save preset %d", preset_index);
        return false;
    }

    ESP_LOGI(TAG, "Saved preset %d: (%.0f, %.0f, %.0f)",
             preset_index, preset.pos[0], preset.pos[1], preset.pos[2]);

    return true;
}

void stepper_simple_home(void)
{
    if (!initialized) {
        return;
    }

    stepper_simple_stop();

    homing_active = true;
    homing_axis = 0;

    for (int i = 0; i < NUM_AXES; i++) {
        homing_start_position[i] = axes[i].position;
        homing_steps_taken[i] = 0;
    }

    float homing_vel = get_homing_velocity(0);
    float homing_dir = get_homing_direction(0);
    axes[0].target_velocity = homing_vel * homing_dir;

    ESP_LOGI(TAG, "Homing started - axis 0 (%s) at %.1f steps/sec, direction %.0f",
             axis_names[0], homing_vel, homing_dir);
}

bool stepper_simple_is_homing(void)
{
    return homing_active;
}
