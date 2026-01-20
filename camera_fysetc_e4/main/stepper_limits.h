/**
 * @file stepper_limits.h
 * @brief Maximum and minimum velocity limits for stepper motors
 * 
 * This file defines the safe operating limits for all axes to prevent
 * stalling (too slow) and step skipping (too fast).
 */

#ifndef STEPPER_LIMITS_H
#define STEPPER_LIMITS_H

// Minimum velocities to prevent stalling (steps/sec)
#define MIN_PAN_TILT_VELOCITY 20.0f
#define MIN_ZOOM_VELOCITY 10.0f

// Maximum velocities to prevent step skipping (steps/sec)
#define MAX_PAN_VELOCITY 1200.0f
#define MAX_TILT_VELOCITY 1200.0f
#define MAX_ZOOM_VELOCITY 130.0f

// Maximum range of motion for homing (steps)
// These values should be calibrated by:
// 1. Starting at minimum range of motion (endstop position)
// 2. Jogging to maximum range of motion
// 3. Reading the position and updating these defines
// If endstop is not hit within this range, homing will bail out and assume current position as home
#define MAX_PAN_RANGE_STEPS   18400.0f   // Adjust after calibration
#define MAX_TILT_RANGE_STEPS  15230.0f   // Adjust after calibration
#define MAX_ZOOM_RANGE_STEPS  1000.0f    // Adjust after calibration

// Homing velocity (steps/sec) - slower for accuracy
#define HOMING_VELOCITY 200.0f

#endif // STEPPER_LIMITS_H
