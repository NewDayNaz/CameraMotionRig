/**
 * @file stepper_simple.c
 * @brief Open-loop PTZ motion: trusted homing, constant-velocity presets
 *
 * Repeatability rules:
 * - PAN/TILT origin: magnetic sensor leading edge after debounce + pull-off
 * - ZOOM origin: PWM_SCALE_SUM rise at the wide rubber, then pull-off; 0 is the ring
 * - Homing miss faults pan/tilt; zoom faults if PWM never rises (UART-down falls back to a crawl)
 * - Arrival is counted pulses, never a software snap to target
 * - SAVE/GOTO require homed and (for SAVE) idle
 * - Zoom live stallGuard halt is off unless calibration finds a free-run vs
 *   end-stop SG gap. Soft limits use the calibrated rubber-ring span.
 * - Jog/preset slow in the last SOFT_LIMIT_APPROACH_STEPS before a software stop
 * - After IDLE_REHOME_MS with no steps: HOME, then return to the pre-home pose
 */

#include "stepper_simple.h"
#include "stepper_limits.h"
#include "board.h"
#include "preset_storage.h"
#include "tmc_driver.h"
#include "zoom_cal.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

static const char *TAG = "stepper_simple";

typedef enum {
    PRESET_AXIS_IDLE = 0,
    PRESET_AXIS_OVERSHOOT,
    PRESET_AXIS_APPROACH,
    PRESET_AXIS_DONE,
} preset_axis_phase_t;

typedef enum {
    HOME_PHASE_IDLE = 0,
    HOME_PHASE_BACKOFF,
    HOME_PHASE_FAST_SEEK,
    HOME_PHASE_PULLOFF,
    HOME_PHASE_SLOW_SEEK,
    HOME_PHASE_FINAL_PULLOFF,
    HOME_PHASE_RANGE_DRIVE, /* zoom UART-down fallback: drive N steps */
    HOME_PHASE_ZOOM_PROBE,  /* PWM high: peek toward tele to leave an end */
    HOME_PHASE_ZOOM_SEEK,   /* toward wide until PWM_SCALE_SUM rises */
    HOME_PHASE_ZOOM_PULLOFF,
    HOME_PHASE_SETTLE,
} home_phase_t;

typedef struct {
    int32_t position;
    float velocity;
    float target_velocity;
    int move_direction; /* 0 stopped, 1 positive, 2 negative */
    uint32_t step_delay_us;
    int64_t last_step_time;
    int last_dir_level;
} axis_state_t;

static axis_state_t axes[NUM_AXES];
static SemaphoreHandle_t motion_mutex;
static bool initialized = false;
static bool homed = false;
static bool homing_active = false;
static bool startup_homing = false;
static uint8_t homing_axis = 0;
static home_phase_t home_phase = HOME_PHASE_IDLE;
static uint8_t home_debounce = 0;
static uint8_t crash_debounce[NUM_AXES];
static int32_t home_phase_steps = 0;
static int32_t home_extra_start_steps = 0;
static bool home_extra_counting = false;
static int home_settle_ticks = 0;
static bool axis_fault[NUM_AXES];
static bool motors_standby = false;
static int64_t last_motion_us = 0;

static uint8_t zoom_live_sg_hits = 0;
static int zoom_live_sg_poll_ticks = 0;
static int32_t zoom_live_steps = 0;
static int zoom_live_dir = 0;

static bool idle_rehome_restore = false;
static int32_t idle_rehome_pos[NUM_AXES];

static bool preset_move_active = false;
static uint8_t preset_move_index = 0;
static int32_t preset_final_target[NUM_AXES];
static int32_t preset_immediate_goal[NUM_AXES];
static float preset_speed[NUM_AXES];
static preset_axis_phase_t preset_phase[NUM_AXES];

static char last_error[96] = "";

static bool zoom_cal_limits = false;
static bool zoom_limit_holdoff = false;
static bool zoom_sg_live = false;
static uint16_t zoom_sg_stall_max = HOME_ZOOM_SG_STALL_MAX;
static int32_t zoom_soft_min_s = 0;
static int32_t zoom_soft_max_s = MAX_ZOOM_RANGE_STEPS;
static bool zoom_origin_from_pwm = false;
static bool zoom_pwm_stop_en = false;
static uint8_t zoom_pwm_stop_thresh = HOME_ZOOM_PWM_THRESH;
static uint8_t zoom_pwm_stop_hits = 0;
static bool zoom_pwm_stop_hit = false;
static int32_t zoom_seek_ignore = HOME_ZOOM_PWM_IGNORE_STEPS;

#define MIN_STEP_DELAY_US 250
#define UPDATE_DT_S 0.001f
#define SLEW_ACCEL 2000.0f

#define MOTION_LOCK()   xSemaphoreTakeRecursive(motion_mutex, portMAX_DELAY)
#define MOTION_UNLOCK() xSemaphoreGiveRecursive(motion_mutex)

static void set_error(const char *msg)
{
    snprintf(last_error, sizeof(last_error), "%s", msg ? msg : "");
}

static float get_homing_velocity(uint8_t axis, bool slow)
{
    if (axis == AXIS_PAN) {
        return slow ? HOMING_PAN_SLOW_VELOCITY : HOMING_PAN_VELOCITY;
    }
    if (axis == AXIS_TILT) {
        return slow ? HOMING_TILT_SLOW_VELOCITY : HOMING_TILT_VELOCITY;
    }
    return HOME_ZOOM_CAL_SAMPLE_VEL;
}

static int get_homing_direction(uint8_t axis)
{
    if (axis == AXIS_PAN) {
        return HOMING_PAN_DIRECTION;
    }
    if (axis == AXIS_TILT) {
        return HOMING_TILT_DIRECTION;
    }
    return HOMING_ZOOM_DIRECTION;
}

static int get_travel_sign(uint8_t axis)
{
    if (axis == AXIS_PAN) {
        return TRAVEL_SIGN_PAN;
    }
    if (axis == AXIS_TILT) {
        return TRAVEL_SIGN_TILT;
    }
    return TRAVEL_SIGN_ZOOM;
}

static int32_t get_max_range(uint8_t axis)
{
    if (axis == AXIS_PAN) {
        return MAX_PAN_RANGE_STEPS;
    }
    if (axis == AXIS_TILT) {
        return MAX_TILT_RANGE_STEPS;
    }
    return MAX_ZOOM_RANGE_STEPS;
}

static int32_t axis_soft_min(uint8_t axis)
{
    if (axis == AXIS_ZOOM && zoom_cal_limits) {
        return zoom_soft_min_s;
    }
    return (get_travel_sign(axis) > 0) ? 0 : -get_max_range(axis);
}

static int32_t axis_soft_max(uint8_t axis)
{
    if (axis == AXIS_ZOOM && zoom_cal_limits) {
        return zoom_soft_max_s;
    }
    return (get_travel_sign(axis) > 0) ? get_max_range(axis) : 0;
}

static void reset_zoom_live_stall(void);
static bool zoom_pwm_cached(uint8_t *pwm);

static void apply_zoom_cal_locked(void)
{
    const zoom_cal_t *cal = zoom_cal_get();
    if (cal != NULL && cal->valid && cal->span_steps >= HOME_ZOOM_CAL_MIN_RANGE) {
        zoom_cal_limits = true;
        zoom_soft_min_s = cal->soft_min;
        zoom_soft_max_s = cal->soft_max;
        zoom_sg_live = cal->sg_usable != 0;
        zoom_sg_stall_max = cal->sg_stall_max != 0 ? cal->sg_stall_max : HOME_ZOOM_SG_STALL_MAX;
    } else {
        zoom_cal_limits = false;
        zoom_sg_live = false;
        zoom_sg_stall_max = HOME_ZOOM_SG_STALL_MAX;
        zoom_soft_min_s = 0;
        zoom_soft_max_s = MAX_ZOOM_RANGE_STEPS;
    }
    reset_zoom_live_stall();
}

static int32_t clamp_axis_pos(uint8_t axis, int32_t pos)
{
    int32_t mn = axis_soft_min(axis);
    int32_t mx = axis_soft_max(axis);
    if (pos < mn) {
        return mn;
    }
    if (pos > mx) {
        return mx;
    }
    return pos;
}

static int get_backlash_steps(uint8_t axis)
{
    if (axis == AXIS_ZOOM) {
        return PRESET_BACKLASH_STEPS_ZOOM;
    }
    if (axis == AXIS_TILT) {
        return PRESET_BACKLASH_STEPS_TILT;
    }
    return PRESET_BACKLASH_STEPS_PAN;
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
    return MAX_ZOOM_VELOCITY;
}

static float get_default_preset_speed(uint8_t axis)
{
    if (axis == AXIS_ZOOM) {
        return PRESET_ZOOM_VELOCITY;
    }
    return PRESET_PAN_TILT_VELOCITY;
}

static int32_t get_pulloff_steps(uint8_t axis)
{
    return (axis == AXIS_TILT) ? HOME_TILT_PULLOFF_STEPS : HOME_PULLOFF_STEPS;
}

static int32_t get_final_pulloff_steps(uint8_t axis)
{
    return (axis == AXIS_TILT) ? HOME_TILT_FINAL_PULLOFF_STEPS : HOME_FINAL_PULLOFF_STEPS;
}

static int32_t get_backoff_extra_steps(uint8_t axis)
{
    return (axis == AXIS_TILT) ? HOME_TILT_BACKOFF_EXTRA_STEPS : HOME_BACKOFF_EXTRA_STEPS;
}

static int32_t get_pulloff_max_steps(uint8_t axis)
{
    return (axis == AXIS_TILT) ? HOME_TILT_PULLOFF_MAX_STEPS : HOME_PULLOFF_MAX_STEPS;
}

static bool axis_has_endstop(uint8_t axis)
{
    return endstop_pins[axis] != GPIO_NUM_NC;
}

static bool endstop_raw(uint8_t axis)
{
    return board_get_endstop_triggered(axis);
}

static bool any_velocity_nonzero(void)
{
    for (int i = 0; i < NUM_AXES; i++) {
        if (fabsf(axes[i].velocity) > 0.1f || fabsf(axes[i].target_velocity) > 0.1f) {
            return true;
        }
    }
    return false;
}

static bool is_moving_locked(void)
{
    return preset_move_active || homing_active || any_velocity_nonzero();
}

static void halt_axis(uint8_t axis)
{
    axes[axis].velocity = 0.0f;
    axes[axis].target_velocity = 0.0f;
    axes[axis].move_direction = 0;
    axes[axis].step_delay_us = 0;
    gpio_set_level(step_pins[axis], 0);
}

static void halt_all_axes(void)
{
    for (int i = 0; i < NUM_AXES; i++) {
        halt_axis((uint8_t)i);
        preset_phase[i] = PRESET_AXIS_IDLE;
    }
    preset_move_active = false;
}

static void restore_run_current(void)
{
    if (motors_standby) {
        tmc_driver_set_standby(false);
        motors_standby = false;
    }
}

static void reset_zoom_live_stall(void)
{
    zoom_live_sg_hits = 0;
    zoom_live_sg_poll_ticks = 0;
    zoom_live_steps = 0;
    zoom_live_dir = 0;
}

static void reset_zoom_pwm_stop(void)
{
    zoom_pwm_stop_hits = 0;
    zoom_pwm_stop_hit = false;
}

static void check_zoom_pwm_stop(void)
{
    if (!zoom_pwm_stop_en || zoom_pwm_stop_hit || homing_active) {
        return;
    }
    if (!zoom_limit_holdoff) {
        return;
    }
    if (fabsf(axes[AXIS_ZOOM].velocity) < 0.1f) {
        return;
    }
    if (zoom_live_steps < HOME_ZOOM_PWM_IGNORE_STEPS) {
        return;
    }
    uint8_t pwm = 0;
    if (!zoom_pwm_cached(&pwm)) {
        return;
    }
    if (pwm >= zoom_pwm_stop_thresh) {
        zoom_pwm_stop_hits++;
        if (zoom_pwm_stop_hits >= HOME_ZOOM_PWM_HITS) {
            zoom_pwm_stop_hit = true;
            ESP_LOGI(TAG, "ZOOM cal PWM trip PWM=%u thresh=%u",
                     (unsigned)pwm, (unsigned)zoom_pwm_stop_thresh);
        }
    } else {
        zoom_pwm_stop_hits = 0;
    }
}

static void stop_locked(bool aborting_home)
{
    halt_all_axes();
    homing_active = false;
    home_phase = HOME_PHASE_IDLE;
    reset_zoom_live_stall();
    zoom_limit_holdoff = false;
    zoom_pwm_stop_en = false;
    reset_zoom_pwm_stop();
    if (aborting_home) {
        homed = false;
        startup_homing = false;
        idle_rehome_restore = false;
        ESP_LOGW(TAG, "Homing aborted — origin is untrusted");
        set_error("Homing aborted");
    }
}

static void fault_axis(uint8_t axis, const char *why)
{
    axis_fault[axis] = true;
    homed = false;
    halt_axis(axis);
    ESP_LOGE(TAG, "Axis %s fault: %s", axis_names[axis], why);
    snprintf(last_error, sizeof(last_error), "%s fault: %s", axis_names[axis], why);
}

static void setup_axis_preset_move(uint8_t axis, int32_t current, int32_t target, float speed)
{
    target = clamp_axis_pos(axis, target);
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
    preset_immediate_goal[axis] = clamp_axis_pos(axis, target - get_backlash_steps(axis));
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
        /* Arrival is counted pulses only — do not snap position */
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

static int desired_dir_level(uint8_t axis, bool positive)
{
    bool reverse = (axis == AXIS_PAN || axis == AXIS_TILT);
    if (positive) {
        return reverse ? 0 : 1;
    }
    return reverse ? 1 : 0;
}

static void set_homing_velocity(uint8_t axis, bool toward_switch, bool slow)
{
    float speed = get_homing_velocity(axis, slow);
    int dir = get_homing_direction(axis);
    axes[axis].target_velocity = toward_switch ? (speed * (float)dir) : (-speed * (float)dir);
    axes[axis].last_step_time = esp_timer_get_time();
}

static void begin_home_phase(home_phase_t phase, bool toward_switch, bool slow)
{
    home_phase = phase;
    home_debounce = 0;
    home_phase_steps = 0;
    home_extra_start_steps = 0;
    home_extra_counting = false;
    home_settle_ticks = 0;
    if (phase != HOME_PHASE_SETTLE) {
        set_homing_velocity(homing_axis, toward_switch, slow);
    } else {
        halt_axis(homing_axis);
    }
}

static void finish_axis_home_success(void)
{
    uint8_t axis = homing_axis;
    halt_axis(axis);
    if (!(axis == AXIS_ZOOM && zoom_origin_from_pwm)) {
        axes[axis].position = 0;
    }
    axis_fault[axis] = false;
    crash_debounce[axis] = 0;
    ESP_LOGI(TAG, "Homing axis %s complete — origin set after pull-off", axis_names[axis]);

    homing_axis++;
    if (homing_axis >= NUM_AXES) {
        homing_active = false;
        home_phase = HOME_PHASE_IDLE;
        homed = true;
        apply_zoom_cal_locked();
        ESP_LOGI(TAG, "Homing complete — all axes trusted");
        return;
    }

    home_phase = HOME_PHASE_IDLE;
}

static bool goto_preset_locked(uint8_t preset_index);
static void home_locked(void);

static void begin_pose_restore(const int32_t *pos)
{
    for (int i = 0; i < NUM_AXES; i++) {
        int32_t target = clamp_axis_pos((uint8_t)i, pos[i]);
        float speed = get_default_preset_speed((uint8_t)i);
        setup_axis_preset_move((uint8_t)i, axes[i].position, target, speed);
        axes[i].velocity = 0.0f;
        axes[i].target_velocity = 0.0f;
        axes[i].last_step_time = esp_timer_get_time();
    }
    preset_move_active = true;
    preset_move_index = 0;
}

static void complete_homing_maybe_preset(void)
{
    if (!homed) {
        return;
    }

    if (idle_rehome_restore) {
        idle_rehome_restore = false;
        ESP_LOGI(TAG, "Idle re-home complete — returning to (%ld, %ld, %ld)",
                 (long)idle_rehome_pos[AXIS_PAN],
                 (long)idle_rehome_pos[AXIS_TILT],
                 (long)idle_rehome_pos[AXIS_ZOOM]);
        begin_pose_restore(idle_rehome_pos);
        return;
    }

    if (!startup_homing) {
        return;
    }
    startup_homing = false;

    preset_t preset;
    if (!preset_load(1, &preset) || !preset.valid) {
        ESP_LOGI(TAG, "Startup homing complete — preset 1 is not stored");
        return;
    }
    ESP_LOGI(TAG, "Startup homing complete — recalling preset 1");
    if (!goto_preset_locked(1)) {
        ESP_LOGW(TAG, "Failed to recall preset 1 after startup homing: %s", last_error);
    }
}

static void fail_current_home(const char *why)
{
    fault_axis(homing_axis, why);
    halt_all_axes();
    homing_active = false;
    home_phase = HOME_PHASE_IDLE;
    homed = false;
    startup_homing = false;
    idle_rehome_restore = false;
    reset_zoom_live_stall();
    ESP_LOGE(TAG, "Homing failed on %s — SAVE/GOTO blocked until HOME succeeds",
             axis_names[homing_axis]);
}

static void start_homing_axis(uint8_t axis)
{
    homing_axis = axis;
    home_debounce = 0;
    home_phase_steps = 0;
    home_extra_counting = false;
    halt_axis(axis);

    if (!axis_has_endstop(axis)) {
        zoom_origin_from_pwm = false;
        if (!tmc_driver_uart_installed()) {
            ESP_LOGW(TAG, "Homing %s by step count (%ld steps) — UART down",
                     axis_names[axis], (long)HOME_ZOOM_DRIVE_STEPS);
            begin_home_phase(HOME_PHASE_RANGE_DRIVE, true, false);
            return;
        }
        ESP_LOGI(TAG, "Homing ZOOM by PWM_SCALE_SUM (thresh=%u)",
                 (unsigned)zoom_cal_pwm_thresh());
        begin_home_phase(HOME_PHASE_ZOOM_PROBE, false, false);
        return;
    }

    ESP_LOGI(TAG, "Homing %s on magnetic sensor", axis_names[axis]);
    if (endstop_raw(axis)) {
        ESP_LOGI(TAG, "%s already in magnet field — backing off", axis_names[axis]);
        begin_home_phase(HOME_PHASE_BACKOFF, false, false);
    } else {
        begin_home_phase(HOME_PHASE_FAST_SEEK, true, false);
    }
}

static bool extra_travel_done(int32_t extra_steps)
{
    if (!home_extra_counting) {
        home_extra_start_steps = home_phase_steps;
        home_extra_counting = true;
    }
    return (home_phase_steps - home_extra_start_steps) >= extra_steps;
}

static bool zoom_pwm_cached(uint8_t *pwm)
{
    uint16_t sg = 0;
    uint32_t ts = 0;
    uint8_t p = 0;
    if (!tmc_driver_get_cached_sg_tstep_pwm(&sg, &ts, &p)) {
        return false;
    }
    if (pwm != NULL) {
        *pwm = p;
    }
    return true;
}

static int32_t zoom_home_max_seek(void)
{
    const zoom_cal_t *cal = zoom_cal_get();
    if (cal != NULL && cal->valid && cal->span_steps >= HOME_ZOOM_CAL_MIN_RANGE) {
        return cal->span_steps + HOME_ZOOM_CAL_MARGIN + HOME_ZOOM_PWM_PROBE_STEPS + 80;
    }
    return HOME_ZOOM_PWM_MAX_STEPS;
}

static void update_zoom_home(uint8_t axis, int32_t range)
{
    (void)range;
    uint8_t pwm = 0;
    bool pwm_ok = zoom_pwm_cached(&pwm);
    uint8_t thresh = zoom_cal_pwm_thresh();

    if (home_phase == HOME_PHASE_RANGE_DRIVE) {
        if (home_phase_steps >= HOME_ZOOM_DRIVE_STEPS) {
            halt_axis(axis);
            ESP_LOGW(TAG,
                     "ZOOM: drove %ld steps — assuming current position as home",
                     (long)home_phase_steps);
            begin_home_phase(HOME_PHASE_SETTLE, false, true);
        }
        return;
    }

    if (home_phase == HOME_PHASE_ZOOM_PROBE) {
        if (pwm_ok && pwm < thresh) {
            home_debounce++;
            if (home_debounce >= HOME_ZOOM_PWM_HITS || home_phase_steps == 0) {
                zoom_seek_ignore = (home_phase_steps < 3)
                                       ? HOME_ZOOM_PWM_IGNORE_STEPS
                                       : 8;
                begin_home_phase(HOME_PHASE_ZOOM_SEEK, true, false);
            }
            return;
        }
        home_debounce = 0;
        if (home_phase_steps >= HOME_ZOOM_PWM_PROBE_STEPS) {
            ESP_LOGI(TAG, "ZOOM PWM still high after probe — seeking wide");
            zoom_seek_ignore = 8;
            begin_home_phase(HOME_PHASE_ZOOM_SEEK, true, false);
        }
        return;
    }

    if (home_phase == HOME_PHASE_ZOOM_SEEK) {
        if (home_phase_steps >= zoom_home_max_seek()) {
            fail_current_home("zoom PWM wide end not seen");
            return;
        }
        if (home_phase_steps < zoom_seek_ignore) {
            home_debounce = 0;
            return;
        }
        if (!pwm_ok) {
            return;
        }
        if (pwm >= thresh) {
            home_debounce++;
            if (home_debounce >= HOME_ZOOM_PWM_HITS) {
                halt_axis(axis);
                axes[axis].position = 0;
                zoom_origin_from_pwm = true;
                ESP_LOGI(TAG, "ZOOM PWM home contact PWM=%u thresh=%u after %ld steps — pull-off %d",
                         (unsigned)pwm, (unsigned)thresh, (long)home_phase_steps,
                         HOME_ZOOM_CAL_MARGIN);
                begin_home_phase(HOME_PHASE_ZOOM_PULLOFF, false, false);
            }
        } else {
            home_debounce = 0;
        }
        return;
    }

    if (home_phase == HOME_PHASE_ZOOM_PULLOFF) {
        if (home_phase_steps >= HOME_ZOOM_CAL_MARGIN) {
            halt_axis(axis);
            begin_home_phase(HOME_PHASE_SETTLE, false, true);
        }
        return;
    }
}

static void update_homing(void)
{
    if (homing_axis >= NUM_AXES) {
        homing_active = false;
        return;
    }

    uint8_t axis = homing_axis;
    bool triggered = endstop_raw(axis);
    int32_t range = get_max_range(axis);

    if (home_phase == HOME_PHASE_IDLE) {
        start_homing_axis(axis);
        return;
    }

    if (home_phase == HOME_PHASE_SETTLE) {
        halt_axis(axis);
        home_settle_ticks++;
        if (home_settle_ticks >= HOME_SETTLE_MS) {
            finish_axis_home_success();
            if (homed) {
                complete_homing_maybe_preset();
            } else if (homing_active) {
                start_homing_axis(homing_axis);
            }
        }
        return;
    }

    if (!axis_has_endstop(axis)) {
        update_zoom_home(axis, range);
        return;
    }

    if (home_phase == HOME_PHASE_BACKOFF) {
        if (home_phase_steps >= range) {
            fail_current_home("backoff: magnet never left sensor");
            return;
        }
        if (triggered) {
            home_debounce = 0;
            home_extra_counting = false;
            return;
        }
        home_debounce++;
        if (home_debounce < HOME_DEBOUNCE_SAMPLES) {
            return;
        }
        if (extra_travel_done(get_backoff_extra_steps(axis))) {
            begin_home_phase(HOME_PHASE_FAST_SEEK, true, false);
        }
        return;
    }

    if (home_phase == HOME_PHASE_FAST_SEEK || home_phase == HOME_PHASE_SLOW_SEEK) {
        if (home_phase_steps >= range) {
            fail_current_home("seek: magnet not seen within range");
            return;
        }
        if (!triggered) {
            home_debounce = 0;
            return;
        }
        home_debounce++;
        if (home_debounce < HOME_DEBOUNCE_SAMPLES) {
            return;
        }
        halt_axis(axis);
        if (home_phase == HOME_PHASE_FAST_SEEK) {
            ESP_LOGI(TAG, "%s magnet acquired after %ld steps — pulling off",
                     axis_names[axis], (long)home_phase_steps);
            begin_home_phase(HOME_PHASE_PULLOFF, false, false);
        } else {
            ESP_LOGI(TAG, "%s magnet leading edge after %ld steps — final pull-off",
                     axis_names[axis], (long)home_phase_steps);
            begin_home_phase(HOME_PHASE_FINAL_PULLOFF, false, true);
        }
        return;
    }

    if (home_phase == HOME_PHASE_PULLOFF || home_phase == HOME_PHASE_FINAL_PULLOFF) {
        if (home_phase_steps >= get_pulloff_max_steps(axis)) {
            fail_current_home("pull-off: magnet still in sensor field");
            return;
        }
        if (triggered) {
            home_debounce = 0;
            home_extra_counting = false;
            return;
        }
        home_debounce++;
        if (home_debounce < HOME_DEBOUNCE_SAMPLES) {
            return;
        }
        int32_t extra = (home_phase == HOME_PHASE_PULLOFF)
                            ? get_pulloff_steps(axis)
                            : get_final_pulloff_steps(axis);
        if (extra_travel_done(extra)) {
            if (home_phase == HOME_PHASE_PULLOFF) {
                begin_home_phase(HOME_PHASE_SLOW_SEEK, true, true);
            } else {
                begin_home_phase(HOME_PHASE_SETTLE, false, true);
            }
        }
        return;
    }
}

static void apply_soft_limits(uint8_t axis)
{
    if (!homed || homing_active) {
        return;
    }
    if (axis == AXIS_ZOOM && zoom_limit_holdoff) {
        return;
    }
    int32_t mn = axis_soft_min(axis);
    int32_t mx = axis_soft_max(axis);
    int32_t pos = axes[axis].position;
    float tv = axes[axis].target_velocity;
    int32_t remain;
    if (tv < -0.1f) {
        remain = pos - mn;
        if (remain <= 0) {
            axes[axis].target_velocity = 0.0f;
            axes[axis].velocity = 0.0f;
            return;
        }
    } else if (tv > 0.1f) {
        remain = mx - pos;
        if (remain <= 0) {
            axes[axis].target_velocity = 0.0f;
            axes[axis].velocity = 0.0f;
            return;
        }
    } else {
        return;
    }
    if (remain < SOFT_LIMIT_APPROACH_STEPS) {
        float mag = fabsf(tv) * ((float)remain / (float)SOFT_LIMIT_APPROACH_STEPS);
        float floor_v = (axis == AXIS_ZOOM) ? MIN_ZOOM_VELOCITY : MIN_PAN_TILT_VELOCITY;
        if (mag < floor_v) {
            mag = floor_v;
        }
        axes[axis].target_velocity = (tv > 0.0f) ? mag : -mag;
    }
}

static void check_endstop_crashes(void)
{
    if (homing_active) {
        return;
    }
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        if (!endstop_raw(i)) {
            crash_debounce[i] = 0;
            continue;
        }
        crash_debounce[i]++;
        if (crash_debounce[i] < HOME_CRASH_DEBOUNCE_SAMPLES) {
            continue;
        }
        crash_debounce[i] = HOME_CRASH_DEBOUNCE_SAMPLES;
        if (fabsf(axes[i].velocity) < 0.1f && fabsf(axes[i].target_velocity) < 0.1f) {
            continue;
        }
        fault_axis(i, "magnetic sensor hit while moving");
        halt_all_axes();
        reset_zoom_live_stall();
        return;
    }
}

static void check_zoom_live_stall(void)
{
    if (!zoom_sg_live) {
        return;
    }
    if (homing_active) {
        reset_zoom_live_stall();
        return;
    }

    float vel = axes[AXIS_ZOOM].velocity;
    int dir = axes[AXIS_ZOOM].move_direction;
    if (fabsf(vel) < HOME_ZOOM_LIVE_SG_MIN_VEL || dir == 0) {
        reset_zoom_live_stall();
        return;
    }

    if (dir != zoom_live_dir) {
        zoom_live_dir = dir;
        zoom_live_steps = 0;
        zoom_live_sg_hits = 0;
        zoom_live_sg_poll_ticks = 0;
    }

    if (zoom_live_steps < HOME_ZOOM_LIVE_IGNORE_STEPS) {
        return;
    }
    if (!tmc_driver_uart_installed()) {
        return;
    }

    zoom_live_sg_poll_ticks++;
    if (zoom_live_sg_poll_ticks < HOME_ZOOM_SG_POLL_MS) {
        return;
    }
    zoom_live_sg_poll_ticks = 0;

    uint16_t sg = 0;
    uint32_t tstep = 0;
    if (!tmc_driver_get_cached_sg_tstep(&sg, &tstep)) {
        return;
    }
    if (tstep > HOME_ZOOM_LIVE_TSTEP_MAX) {
        zoom_live_sg_hits = 0;
        return;
    }
    if (sg > zoom_sg_stall_max) {
        zoom_live_sg_hits = 0;
        return;
    }

    zoom_live_sg_hits++;
    if (zoom_live_sg_hits < HOME_ZOOM_SG_HITS) {
        return;
    }

    ESP_LOGE(TAG, "ZOOM live stall SG=%u TSTEP=%lu after %ld steps",
             (unsigned)sg, (unsigned long)tstep, (long)zoom_live_steps);
    fault_axis(AXIS_ZOOM, "lens stall while moving");
    halt_all_axes();
    reset_zoom_live_stall();
}

static int32_t idle_rehome_remaining_s(int64_t now_us)
{
    if (homing_active || preset_move_active || any_velocity_nonzero()) {
        return -1;
    }
    if (last_motion_us <= 0) {
        return -1;
    }
    int64_t remain_us = ((int64_t)IDLE_REHOME_MS * 1000) - (now_us - last_motion_us);
    if (remain_us < 0) {
        remain_us = 0;
    }
    return (int32_t)(remain_us / 1000000LL);
}

static void maybe_idle_rehome(int64_t now_us)
{
    if (homing_active || preset_move_active || any_velocity_nonzero()) {
        return;
    }
    if (last_motion_us <= 0) {
        return;
    }
    if ((now_us - last_motion_us) < ((int64_t)IDLE_REHOME_MS * 1000)) {
        return;
    }

    idle_rehome_restore = homed;
    for (int i = 0; i < NUM_AXES; i++) {
        idle_rehome_pos[i] = axes[i].position;
    }
    ESP_LOGI(TAG, "Idle %.1f h — re-homing to refresh origin (restore pose=%d)",
             (double)IDLE_REHOME_MS / 3600000.0, idle_rehome_restore ? 1 : 0);
    restore_run_current();
    home_locked();
}

static void slew_to_target(uint8_t axis)
{
    if (preset_move_active &&
        preset_phase[axis] != PRESET_AXIS_IDLE &&
        preset_phase[axis] != PRESET_AXIS_DONE) {
        axes[axis].velocity = axes[axis].target_velocity;
        return;
    }
    if (homing_active && axis == homing_axis) {
        axes[axis].velocity = axes[axis].target_velocity;
        return;
    }

    float vel_diff = axes[axis].target_velocity - axes[axis].velocity;
    if (fabsf(axes[axis].target_velocity) < 0.1f) {
        axes[axis].velocity = axes[axis].target_velocity;
        return;
    }
    float max_change = SLEW_ACCEL * UPDATE_DT_S;
    if (fabsf(vel_diff) > max_change) {
        axes[axis].velocity += (vel_diff > 0) ? max_change : -max_change;
    } else {
        axes[axis].velocity = axes[axis].target_velocity;
    }
}

static void generate_steps(int64_t now_us)
{
    for (int i = 0; i < NUM_AXES; i++) {
        slew_to_target((uint8_t)i);
        apply_soft_limits((uint8_t)i);
        axes[i].step_delay_us = velocity_to_step_delay(axes[i].velocity);

        if (axes[i].velocity > 0.1f) {
            axes[i].move_direction = 1;
        } else if (axes[i].velocity < -0.1f) {
            axes[i].move_direction = 2;
        } else {
            axes[i].move_direction = 0;
            gpio_set_level(step_pins[i], 0);
            continue;
        }

        int dir_level = desired_dir_level((uint8_t)i, axes[i].move_direction == 1);
        if (dir_level != axes[i].last_dir_level) {
            gpio_set_level(dir_pins[i], dir_level);
            axes[i].last_dir_level = dir_level;
            esp_rom_delay_us(DIR_SETUP_DELAY_US);
            axes[i].last_step_time = now_us;
            continue;
        }

        if (axes[i].step_delay_us == 0) {
            continue;
        }
        if ((now_us - axes[i].last_step_time) < (int64_t)axes[i].step_delay_us) {
            continue;
        }

        gpio_set_level(step_pins[i], 1);
        esp_rom_delay_us(1);
        gpio_set_level(step_pins[i], 0);

        if (axes[i].move_direction == 1) {
            axes[i].position++;
        } else {
            axes[i].position--;
        }
        axes[i].last_step_time += (int64_t)axes[i].step_delay_us;
        if ((now_us - axes[i].last_step_time) > (int64_t)axes[i].step_delay_us * 4) {
            axes[i].last_step_time = now_us;
        }

        if (motors_standby) {
            tmc_driver_set_standby(false);
            motors_standby = false;
        }
        last_motion_us = now_us;

        if (homing_active && i == homing_axis) {
            home_phase_steps++;
        }
        if (!homing_active && i == AXIS_ZOOM) {
            zoom_live_steps++;
        }
    }
}

static bool goto_preset_locked(uint8_t preset_index)
{
    if (!initialized) {
        set_error("Not initialized");
        return false;
    }
    if (homing_active) {
        set_error("Homing in progress");
        return false;
    }
    if (!homed) {
        set_error("Not homed");
        return false;
    }

    preset_t preset;
    if (!preset_load(preset_index, &preset) || !preset.valid) {
        set_error("Preset not found");
        return false;
    }

    for (int i = 0; i < NUM_AXES; i++) {
        if (axis_fault[i]) {
            set_error("Axis fault — HOME required");
            return false;
        }
    }

    for (int i = 0; i < NUM_AXES; i++) {
        int32_t current = axes[i].position;
        int32_t target = (int32_t)lroundf(preset.pos[i]);

        float speed = get_default_preset_speed((uint8_t)i);
        if (preset.max_speed > 0.0f && i != AXIS_ZOOM) {
            speed = preset.max_speed;
        }
        float max_vel = get_axis_max_velocity((uint8_t)i);
        if (speed > max_vel) {
            speed = max_vel;
        }

        setup_axis_preset_move((uint8_t)i, current, target, speed);
        axes[i].velocity = 0.0f;
        axes[i].target_velocity = 0.0f;
        axes[i].last_step_time = esp_timer_get_time();
    }

    preset_move_active = true;
    preset_move_index = preset_index;
    set_error("");

    ESP_LOGI(TAG,
             "Preset %d: target (%ld, %ld, %ld) from (%ld, %ld, %ld)",
             preset_index,
             (long)preset_final_target[0], (long)preset_final_target[1],
             (long)preset_final_target[2],
             (long)axes[AXIS_PAN].position, (long)axes[AXIS_TILT].position,
             (long)axes[AXIS_ZOOM].position);
    return true;
}

static bool save_preset_locked(uint8_t preset_index)
{
    if (!initialized) {
        set_error("Not initialized");
        return false;
    }
    if (homing_active) {
        set_error("Homing in progress");
        return false;
    }
    if (!homed) {
        set_error("Not homed");
        return false;
    }
    if (is_moving_locked()) {
        set_error("Cannot save while moving");
        return false;
    }

    preset_t preset;
    preset_init_default(&preset);
    preset.pos[AXIS_PAN] = (float)axes[AXIS_PAN].position;
    preset.pos[AXIS_TILT] = (float)axes[AXIS_TILT].position;
    preset.pos[AXIS_ZOOM] = (float)axes[AXIS_ZOOM].position;

    if (!preset_save(preset_index, &preset)) {
        set_error("NVS save failed");
        return false;
    }

    set_error("");
    ESP_LOGI(TAG, "Saved preset %d: (%ld, %ld, %ld)",
             preset_index,
             (long)axes[AXIS_PAN].position,
             (long)axes[AXIS_TILT].position,
             (long)axes[AXIS_ZOOM].position);
    return true;
}

static void home_locked(void)
{
    restore_run_current();
    stop_locked(homing_active);
    homed = false;
    for (int i = 0; i < NUM_AXES; i++) {
        axis_fault[i] = false;
        crash_debounce[i] = 0;
        halt_axis((uint8_t)i);
    }
    reset_zoom_live_stall();
    homing_active = true;
    homing_axis = 0;
    home_phase = HOME_PHASE_IDLE;
    set_error("");
    ESP_LOGI(TAG, "Homing started");
    start_homing_axis(0);
}

void stepper_simple_init(void)
{
    if (initialized) {
        return;
    }

    motion_mutex = xSemaphoreCreateRecursiveMutex();
    if (motion_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create motion mutex");
        return;
    }

    for (int i = 0; i < NUM_AXES; i++) {
        memset(&axes[i], 0, sizeof(axes[i]));
        axes[i].last_step_time = esp_timer_get_time();
        axes[i].last_dir_level = -1;
        preset_phase[i] = PRESET_AXIS_IDLE;
        axis_fault[i] = false;
        crash_debounce[i] = 0;
    }

    initialized = true;
    homed = false;
    homing_active = false;
    preset_move_active = false;
    startup_homing = true;
    last_motion_us = esp_timer_get_time();
    motors_standby = false;
    idle_rehome_restore = false;
    reset_zoom_live_stall();
    set_error("Not homed");
    apply_zoom_cal_locked();

    ESP_LOGI(TAG, "Stepper control initialized (origin untrusted until HOME)");
}

void stepper_simple_update(void)
{
    if (!initialized) {
        return;
    }

    MOTION_LOCK();

    int64_t now_us = esp_timer_get_time();

    if (homing_active) {
        update_homing();
    } else if (preset_move_active) {
        bool all_done = true;
        for (int i = 0; i < NUM_AXES; i++) {
            update_preset_axis_velocity((uint8_t)i);
            if (preset_phase[i] != PRESET_AXIS_DONE) {
                all_done = false;
            }
        }
        if (all_done) {
            preset_move_active = false;
            ESP_LOGI(TAG, "Preset %u complete (pos: %ld, %ld, %ld)",
                     (unsigned)preset_move_index,
                     (long)axes[AXIS_PAN].position,
                     (long)axes[AXIS_TILT].position,
                     (long)axes[AXIS_ZOOM].position);
        }
    }

    check_endstop_crashes();
    generate_steps(now_us);
    check_zoom_live_stall();
    check_zoom_pwm_stop();

    if (!homing_active && !preset_move_active && !any_velocity_nonzero()) {
        if (!motors_standby && last_motion_us > 0 &&
            (now_us - last_motion_us) >= ((int64_t)MOTOR_STANDBY_MS * 1000)) {
            tmc_driver_set_standby(true);
            motors_standby = true;
        }
        maybe_idle_rehome(now_us);
    } else if (motors_standby) {
        tmc_driver_set_standby(false);
        motors_standby = false;
        last_motion_us = now_us;
    }

    MOTION_UNLOCK();
}

void stepper_simple_set_velocities(float pan_vel, float tilt_vel, float zoom_vel)
{
    if (!initialized) {
        return;
    }

    MOTION_LOCK();
    if (homing_active) {
        ESP_LOGW(TAG, "Velocity command blocked — homing in progress");
        MOTION_UNLOCK();
        return;
    }

    preset_move_active = false;
    for (int i = 0; i < NUM_AXES; i++) {
        preset_phase[i] = PRESET_AXIS_IDLE;
    }

    float vels[NUM_AXES] = { pan_vel, tilt_vel, zoom_vel };

    /* Slow pan/tilt as zoom increases so framing stays controllable telephoto. */
    float z = (float)axes[AXIS_ZOOM].position / (float)MAX_ZOOM_RANGE_STEPS;
    if (z < 0.0f) {
        z = 0.0f;
    } else if (z > 1.0f) {
        z = 1.0f;
    }
    float pt_scale = 1.0f - z * (1.0f - ZOOM_PT_SCALE_MIN);
    vels[AXIS_PAN] *= pt_scale;
    vels[AXIS_TILT] *= pt_scale;

    float pt_min = MIN_PAN_TILT_VELOCITY * pt_scale;
    float mins[NUM_AXES] = { pt_min, pt_min, MIN_ZOOM_VELOCITY };
    float maxs[NUM_AXES] = { MAX_PAN_VELOCITY * pt_scale, MAX_TILT_VELOCITY * pt_scale, MAX_ZOOM_VELOCITY };

    for (int i = 0; i < NUM_AXES; i++) {
        float v = vels[i];
        if (fabsf(v) > 0.1f) {
            if (fabsf(v) < mins[i]) {
                v = (v > 0) ? mins[i] : -mins[i];
            } else if (fabsf(v) > maxs[i]) {
                v = (v > 0) ? maxs[i] : -maxs[i];
            }
        } else {
            v = 0.0f;
        }
        axes[i].target_velocity = v;
    }

    MOTION_UNLOCK();
}

void stepper_simple_get_positions(float *pan, float *tilt, float *zoom)
{
    if (pan == NULL || tilt == NULL || zoom == NULL) {
        return;
    }
    if (!initialized) {
        *pan = *tilt = *zoom = 0.0f;
        return;
    }
    MOTION_LOCK();
    *pan = (float)axes[AXIS_PAN].position;
    *tilt = (float)axes[AXIS_TILT].position;
    *zoom = (float)axes[AXIS_ZOOM].position;
    MOTION_UNLOCK();
}

void stepper_simple_get_velocities(float *pan, float *tilt, float *zoom)
{
    if (pan == NULL || tilt == NULL || zoom == NULL) {
        return;
    }
    if (!initialized) {
        *pan = *tilt = *zoom = 0.0f;
        return;
    }
    MOTION_LOCK();
    *pan = axes[AXIS_PAN].target_velocity;
    *tilt = axes[AXIS_TILT].target_velocity;
    *zoom = axes[AXIS_ZOOM].target_velocity;
    MOTION_UNLOCK();
}

void stepper_simple_stop_zoom(void)
{
    if (!initialized) {
        return;
    }
    MOTION_LOCK();
    axes[AXIS_ZOOM].target_velocity = 0.0f;
    axes[AXIS_ZOOM].velocity = 0.0f;
    axes[AXIS_ZOOM].move_direction = 0;
    reset_zoom_live_stall();
    MOTION_UNLOCK();
}

void stepper_simple_nudge_zoom(float vel)
{
    if (!initialized) {
        return;
    }
    MOTION_LOCK();
    if (!homing_active) {
        axes[AXIS_ZOOM].target_velocity = vel;
    }
    MOTION_UNLOCK();
}

void stepper_simple_set_zoom_limit_holdoff(bool holdoff)
{
    if (!initialized) {
        return;
    }
    MOTION_LOCK();
    zoom_limit_holdoff = holdoff;
    if (!holdoff) {
        zoom_pwm_stop_en = false;
        reset_zoom_pwm_stop();
    }
    MOTION_UNLOCK();
}

void stepper_simple_arm_zoom_pwm_stop(uint8_t thresh)
{
    if (!initialized) {
        return;
    }
    MOTION_LOCK();
    zoom_pwm_stop_thresh = (thresh >= 40u && thresh <= 200u) ? thresh : HOME_ZOOM_PWM_THRESH;
    zoom_pwm_stop_en = true;
    reset_zoom_pwm_stop();
    reset_zoom_live_stall();
    MOTION_UNLOCK();
}

bool stepper_simple_zoom_pwm_hit(void)
{
    if (!initialized) {
        return false;
    }
    MOTION_LOCK();
    bool hit = zoom_pwm_stop_hit;
    MOTION_UNLOCK();
    return hit;
}

void stepper_simple_shift_zoom(int32_t subtract)
{
    if (!initialized || subtract == 0) {
        return;
    }
    MOTION_LOCK();
    axes[AXIS_ZOOM].position -= subtract;
    MOTION_UNLOCK();
}

void stepper_simple_reload_zoom_cal(void)
{
    if (!initialized) {
        return;
    }
    MOTION_LOCK();
    apply_zoom_cal_locked();
    MOTION_UNLOCK();
}

void stepper_simple_get_status(motion_status_t *status)
{
    if (status == NULL) {
        return;
    }
    memset(status, 0, sizeof(*status));
    status->idle_rehome_s = -1;
    if (!initialized) {
        return;
    }
    MOTION_LOCK();
    for (int i = 0; i < NUM_AXES; i++) {
        status->position[i] = axes[i].position;
        status->velocity[i] = axes[i].target_velocity;
        status->axis_fault[i] = axis_fault[i];
        status->endstop[i] = endstop_raw((uint8_t)i);
    }
    status->homed = homed;
    status->homing = homing_active;
    status->moving = is_moving_locked();
    status->idle_rehome_s = idle_rehome_remaining_s(esp_timer_get_time());
    status->zoom_soft_min = axis_soft_min(AXIS_ZOOM);
    status->zoom_soft_max = axis_soft_max(AXIS_ZOOM);
    status->zoom_cal_valid = zoom_cal_limits;
    status->zoom_sg_live = zoom_sg_live;
    status->zoom_sg_stall_max = zoom_sg_stall_max;
    MOTION_UNLOCK();
}

void stepper_simple_stop(void)
{
    if (!initialized) {
        return;
    }
    MOTION_LOCK();
    stop_locked(homing_active);
    MOTION_UNLOCK();
}

bool stepper_simple_goto_preset(uint8_t preset_index)
{
    if (!initialized) {
        set_error("Not initialized");
        return false;
    }
    MOTION_LOCK();
    bool ok = goto_preset_locked(preset_index);
    MOTION_UNLOCK();
    return ok;
}

bool stepper_simple_save_preset(uint8_t preset_index)
{
    if (!initialized) {
        set_error("Not initialized");
        return false;
    }
    MOTION_LOCK();
    bool ok = save_preset_locked(preset_index);
    MOTION_UNLOCK();
    return ok;
}

void stepper_simple_home(void)
{
    if (!initialized) {
        return;
    }
    MOTION_LOCK();
    idle_rehome_restore = false;
    home_locked();
    MOTION_UNLOCK();
}

bool stepper_simple_is_homing(void)
{
    if (!initialized) {
        return false;
    }
    MOTION_LOCK();
    bool active = homing_active;
    MOTION_UNLOCK();
    return active;
}

bool stepper_simple_is_homed(void)
{
    if (!initialized) {
        return false;
    }
    MOTION_LOCK();
    bool ok = homed;
    MOTION_UNLOCK();
    return ok;
}

bool stepper_simple_is_moving(void)
{
    if (!initialized) {
        return false;
    }
    MOTION_LOCK();
    bool moving = is_moving_locked();
    MOTION_UNLOCK();
    return moving;
}

const char *stepper_simple_last_error(void)
{
    return last_error;
}
