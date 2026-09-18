/**
 * @file stepper_simple.h
 * @brief Open-loop step/dir control with trusted origin and preset recall
 *
 * Position is counted STEP edges from a successful endstop home. SAVE/GOTO
 * are refused until all axes are homed. Software position is never snapped
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
    /** Seconds until idle re-home, or -1 if moving/homing (countdown paused). */
    int32_t idle_rehome_s;
} motion_status_t;

void stepper_simple_init(void);
void stepper_simple_update(void);

void stepper_simple_set_velocities(float pan_vel, float tilt_vel, float zoom_vel);
void stepper_simple_get_positions(float *pan, float *tilt, float *zoom);
void stepper_simple_get_status(motion_status_t *status);
void stepper_simple_stop(void);

bool stepper_simple_goto_preset(uint8_t preset_index);
bool stepper_simple_save_preset(uint8_t preset_index);

void stepper_simple_home(void);
bool stepper_simple_is_homing(void);
bool stepper_simple_is_homed(void);
bool stepper_simple_is_moving(void);

/** Valid until the next motion API call. */
const char *stepper_simple_last_error(void);

#endif // STEPPER_SIMPLE_H
