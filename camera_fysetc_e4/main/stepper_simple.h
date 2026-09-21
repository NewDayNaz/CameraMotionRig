/**
 * @file stepper_simple.h
 * @brief Open-loop step/dir control with trusted origin and preset recall
 *
 * Position is counted STEP edges from a successful endstop home. SAVE/GOTO
 * are refused until all axes are homed. GOTO is also ignored while preset
 * automations are off (jog still works). Software position is never snapped
 * to a target.
 */

#ifndef STEPPER_SIMPLE_H
#define STEPPER_SIMPLE_H

#include <stdint.h>
#include <stdbool.h>
#include "board.h"

typedef struct {
    int32_t position[NUM_AXES];
    bool homed;
    bool homing;
    bool moving;
    bool axis_fault[NUM_AXES];
    bool endstop[NUM_AXES];
    /** Seconds until the 7:45 AM daily re-home, or -1 if moving/homing/no clock. */
    int32_t idle_rehome_s;
    float velocity[NUM_AXES];
    int32_t zoom_soft_min;
    int32_t zoom_soft_max;
    bool zoom_cal_valid;
    bool zoom_sg_live;
    uint16_t zoom_sg_stall_max;
    /** False: GOTO from MIDI/Companion/web/serial is ignored; jog still works. */
    bool preset_recall;
} motion_status_t;

void stepper_simple_init(void);
void stepper_simple_update(void);

void stepper_simple_set_velocities(float pan_vel, float tilt_vel, float zoom_vel);
void stepper_simple_get_positions(float *pan, float *tilt, float *zoom);
void stepper_simple_get_velocities(float *pan, float *tilt, float *zoom);
void stepper_simple_stop_zoom(void);
void stepper_simple_nudge_zoom(float vel);
void stepper_simple_set_zoom_limit_holdoff(bool holdoff);
void stepper_simple_arm_zoom_pwm_stop(uint8_t thresh);
bool stepper_simple_zoom_pwm_hit(void);
void stepper_simple_shift_zoom(int32_t subtract);
void stepper_simple_reload_zoom_cal(void);
void stepper_simple_get_status(motion_status_t *status);
void stepper_simple_stop(void);

bool stepper_simple_goto_preset(uint8_t preset_index);
bool stepper_simple_save_preset(uint8_t preset_index);

void stepper_simple_set_preset_recall(bool enabled);
bool stepper_simple_preset_recall_enabled(void);

void stepper_simple_home(void);
/** Operator is using the rig (IRUN, cal, TMC). */
void stepper_simple_touch_idle_timer(void);
bool stepper_simple_is_homing(void);
bool stepper_simple_is_homed(void);
bool stepper_simple_is_moving(void);

/** Valid until the next motion API call. */
const char *stepper_simple_last_error(void);

#endif // STEPPER_SIMPLE_H
