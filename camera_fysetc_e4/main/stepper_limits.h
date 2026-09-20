/**
 * @file stepper_limits.h
 * @brief Velocity, range, homing, and backlash limits for open-loop PTZ axes
 *
 * Positions are raw step pulses.
 *
 * PAN/TILT: magnetic inductive sensors. After home, origin is the pulled-off
 * leading edge of the magnet field (sensor not held). Homing direction is +
 * so useful travel is negative.
 *
 * ZOOM: no endstop. Temporary step-count home: drive HOMING_ZOOM_DIRECTION for
 * HOME_ZOOM_DRIVE_STEPS, then declare origin. Homing direction is − so useful
 * travel is positive.
 *
 * Calibrate MAX_*_RANGE_STEPS by homing, jogging to the far stop, and reading
 * STATUS. Pan/tilt homing faults if the sensor is not seen within this range.
 */

#ifndef STEPPER_LIMITS_H
#define STEPPER_LIMITS_H

#define MIN_PAN_TILT_VELOCITY 20.0f
#define MIN_ZOOM_VELOCITY 10.0f

#define MAX_PAN_VELOCITY 1000.0f
#define MAX_TILT_VELOCITY 1200.0f  /* a bit faster than pan — tilt gearing feels slower */
#define MAX_ZOOM_VELOCITY 130.0f

#define MAX_PAN_RANGE_STEPS   18400
#define MAX_TILT_RANGE_STEPS  15230
#define MAX_ZOOM_RANGE_STEPS  4000

/* Pan/tilt jog scales with zoom: wide = 1.0, full telephoto = ZOOM_PT_SCALE_MIN. */
#define ZOOM_PT_SCALE_MIN  0.5f

#define HOMING_PAN_VELOCITY  200.0f
#define HOMING_TILT_VELOCITY 200.0f
#define HOMING_ZOOM_VELOCITY 50.0f

#define HOMING_PAN_SLOW_VELOCITY  40.0f
#define HOMING_TILT_SLOW_VELOCITY 40.0f
#define HOMING_ZOOM_SLOW_VELOCITY 25.0f

#define HOMING_PAN_DIRECTION  1
#define HOMING_TILT_DIRECTION 1
#define HOMING_ZOOM_DIRECTION -1

#define TRAVEL_SIGN_PAN  -1
#define TRAVEL_SIGN_TILT -1
#define TRAVEL_SIGN_ZOOM  1

/* Magnetic sensors chatter at the field edge — debounce a bit longer than a switch */
#define HOME_DEBOUNCE_SAMPLES        12
#define HOME_CRASH_DEBOUNCE_SAMPLES   4
#define HOME_PULLOFF_STEPS           48    /* get the magnet out of the sensor lobe */
#define HOME_FINAL_PULLOFF_STEPS     24    /* origin slightly off the magnet */
#define HOME_BACKOFF_EXTRA_STEPS     48
#define HOME_PULLOFF_MAX_STEPS      600
/* Tilt magnet/sensor lobe is wider — needs more travel to leave the field. */
#define HOME_TILT_PULLOFF_STEPS          120
#define HOME_TILT_FINAL_PULLOFF_STEPS     64
#define HOME_TILT_BACKOFF_EXTRA_STEPS     96
#define HOME_TILT_PULLOFF_MAX_STEPS     1500
#define HOME_SETTLE_MS               50
#define DIR_SETUP_DELAY_US           20

/* Zoom: temporary step-count home (stallGuard disabled). Drive this many
 * steps toward wide, then set origin. Keep short so we don't grind the stop;
 * ~20 s at HOMING_ZOOM_VELOCITY. Soft travel limit stays MAX_ZOOM_RANGE_STEPS. */
#define HOME_ZOOM_DRIVE_STEPS         1000

/* Live zoom stall-stop (jog / preset). Same SG threshold as former homing.
 * Ignore a short run-up and direction changes so startup/backlash does not
 * look like a lens hit. TSTEP at 0xFFFFF means the TMC is not stepping. */
#define HOME_ZOOM_SG_POLL_MS          25
#define HOME_ZOOM_SG_HITS              3
#define HOME_ZOOM_SG_STALL_MAX        20   /* 0–1023; lower = only a hard stall */
#define HOME_ZOOM_LIVE_IGNORE_STEPS   40
#define HOME_ZOOM_LIVE_SG_MIN_VEL     25.0f
#define HOME_ZOOM_LIVE_TSTEP_MAX      0x000FFFFEu

/* Overnight origin refresh. After this idle time, HOME then return to pose.
 * 6 h is longer than a service hold, short enough for days-on drift. */
#define IDLE_REHOME_MS                (6 * 60 * 60 * 1000)

#define PRESET_PAN_TILT_VELOCITY  150.0f
#define PRESET_ZOOM_VELOCITY         40.0f
#define PRESET_BACKLASH_STEPS_PAN     8
#define PRESET_BACKLASH_STEPS_TILT    8
#define PRESET_BACKLASH_STEPS_ZOOM   16

/*
 * TMC2209 CS values 0–31. FYSETC E4 Rsense is ~0.11 Ω, vsense=0 (Vfs=0.325 V):
 * I_rms ≈ (CS+1)/32 * 0.325 / (0.11 * 1.414) → CS 16 ≈ 1.1 A, CS 4 ≈ 0.33 A.
 * Old IHOLD=8 at standstill for days is why the motors cooked.
 * Tilt keeps a little hold against gravity; zoom needs none at rest.
 * After MOTOR_STANDBY_MS with no steps, currents drop again (STANDBY_*).
 */
#define TMC_IRUN_PAN           16
#define TMC_IRUN_TILT          16
#define TMC_IRUN_ZOOM          14
#define TMC_IHOLD_PAN           3
#define TMC_IHOLD_TILT          4
#define TMC_IHOLD_ZOOM          0
#define TMC_IHOLD_STANDBY_PAN   1
#define TMC_IHOLD_STANDBY_TILT  2
#define TMC_IHOLD_STANDBY_ZOOM  0
#define TMC_IHOLDDELAY          6
#define MOTOR_STANDBY_MS        300000  /* 5 minutes */

#endif // STEPPER_LIMITS_H
