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
 * ZOOM: no endstop. PWM_SCALE_SUM home: seek HOMING_ZOOM_DIRECTION (wide)
 * until stealthChop effort rises, pull off HOME_ZOOM_CAL_MARGIN, leave 0 at
 * the rubber. Homing direction is − so useful travel is positive.
 *
 * Calibrate MAX_*_RANGE_STEPS by homing, jogging to the far stop, and reading
 * STATUS. Pan/tilt homing faults if the sensor is not seen within this range.
 */

#ifndef STEPPER_LIMITS_H
#define STEPPER_LIMITS_H

#define MIN_PAN_TILT_VELOCITY 20.0f
#define MIN_ZOOM_VELOCITY 10.0f

#define MAX_PAN_VELOCITY 722.5f    /* was 850; −15% */
#define MAX_TILT_VELOCITY 1020.0f  /* was 1200; −15%. Tilt still faster — gearing feels slower */
#define MAX_ZOOM_VELOCITY 145.0f

#define MAX_PAN_RANGE_STEPS   18400
#define MAX_TILT_RANGE_STEPS  15230
#define MAX_ZOOM_RANGE_STEPS  4000

/* Pan/tilt jog scales with zoom: wide = 1.0, full telephoto = ZOOM_PT_SCALE_MIN.
 * Scale uses the calibrated zoom span when valid, else MAX_ZOOM_RANGE_STEPS. */
#define ZOOM_PT_SCALE_MIN  0.5f

#define HOMING_PAN_VELOCITY  170.0f
#define HOMING_TILT_VELOCITY 300.0f
#define HOMING_ZOOM_VELOCITY 50.0f

#define HOMING_PAN_SLOW_VELOCITY  34.0f
#define HOMING_TILT_SLOW_VELOCITY 60.0f
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

/* Zoom UART-down fallback only. PWM homing is the normal path. */
#define HOME_ZOOM_DRIVE_STEPS         1000
/* Default PWM_SCALE_SUM trip until calibration measures free vs ends
 * (free ~76-78, rubber ~84-86 at HOME_ZOOM_CAL_SAMPLE_VEL). */
#define HOME_ZOOM_PWM_THRESH          81
#define HOME_ZOOM_CAL_PWM_GAP          4    /* free max + gap → auto-mark / home */
#define HOME_ZOOM_PWM_PROBE_STEPS     12    /* if already on an end, peek toward tele */
#define HOME_ZOOM_PWM_IGNORE_STEPS    40
#define HOME_ZOOM_PWM_HITS             3
#define HOME_ZOOM_PWM_MAX_STEPS       1200  /* first home without a saved span */
#define SOFT_LIMIT_APPROACH_STEPS     40    /* scale jog/preset speed into a soft stop */

/* Live zoom stall-stop is off unless zoom calibration finds a real SG gap.
 * UART for SG is cached off the 1 ms step task (TSTEP 3400→8600 if blocked). */
#define HOME_ZOOM_SG_POLL_MS          25
#define HOME_ZOOM_SG_HITS              3
#define HOME_ZOOM_SG_STALL_MAX        20   /* fallback; calib overwrites if usable */
#define HOME_ZOOM_LIVE_IGNORE_STEPS   40
#define HOME_ZOOM_LIVE_SG_MIN_VEL     100.0f
/* TSTEP is time between incoming 1/256 STEP edges, not rotor speed.
 * ~3400 at 120 step/s, ~7500 at 50 step/s (8 µstep). */
#define HOME_ZOOM_LIVE_TSTEP_MAX      4000u

/* User-paced zoom calibration (free-run + each rubber-ring end). */
#define HOME_ZOOM_CAL_SAMPLES         8
#define HOME_ZOOM_CAL_INTERVAL_MS     40
#define HOME_ZOOM_CAL_MARGIN          32    /* keep jog this many steps off each end */
#define HOME_ZOOM_CAL_MIN_RANGE       200
#define HOME_ZOOM_CAL_SG_GAP          20    /* free min must beat end max by this */
/* Same speed for free-run and end samples. PWM_SCALE rises at low speed even
 * in air, so mixed speeds (100 vs 40) looked like a load gap last time. */
#define HOME_ZOOM_CAL_SAMPLE_VEL      80.0f

/* Overnight origin refresh. After this idle time, HOME then return to pose.
 * 6 h is longer than a service hold, short enough for days-on drift. */
#define IDLE_REHOME_MS                (6 * 60 * 60 * 1000)

#define PRESET_PAN_TILT_VELOCITY  127.5f  /* was 150; −15% */
#define PRESET_ZOOM_VELOCITY         45.0f
#define PRESET_BACKLASH_STEPS_PAN     8
#define PRESET_BACKLASH_STEPS_TILT    8
#define PRESET_BACKLASH_STEPS_ZOOM   16
#define PRESET_MIN_DURATION_S         0.4f
#define PRESET_MAX_DURATION_S        30.0f
#define PRESET_RAMP_S                 0.4f  /* ease in/out time at cruise speed */
#define PRESET_RAMP_MIN_STEPS        24

/*
 * TMC2209 CS values 0–31. FYSETC E4 Rsense is ~0.11 Ω, vsense=0 (Vfs=0.325 V):
 * I_rms ≈ (CS+1)/32 * 0.325 / (0.11 * 1.414) → CS 16 ≈ 1.1 A, CS 11 ≈ 0.78 A.
 * Old IHOLD=8 at standstill for days is why the motors cooked.
 * Tilt keeps a little hold against gravity; zoom IHOLD=0 at rest.
 * Default pan/tilt IRUN is CS 11 (~0.78 A). Zoom defaults to CS 5 (~0.39 A)
 * so the pinion is more likely to stall at the rubber ring than skip over it.
 * CS 16 pegged PWM_SCALE_SUM (~230/255). Live pan/tilt/zoom IRUN can be
 * changed from the web UI without flashing. Hold still drops after MOTOR_STANDBY_MS.
 */
#define TMC_IRUN_PAN           11
#define TMC_IRUN_TILT          11
#define TMC_IRUN_ZOOM           5
#define TMC_IHOLD_PAN           3
#define TMC_IHOLD_TILT          4
#define TMC_IHOLD_ZOOM          0
#define TMC_IHOLD_STANDBY_PAN   1
#define TMC_IHOLD_STANDBY_TILT  2
#define TMC_IHOLD_STANDBY_ZOOM  0
#define TMC_IHOLDDELAY          6
#define MOTOR_STANDBY_MS        300000  /* 5 minutes */

#endif // STEPPER_LIMITS_H
