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
#define MIN_PAN_TILT_VELOCITY 10.0f
#define MIN_ZOOM_VELOCITY 10.0f

// Maximum velocities to prevent step skipping (steps/sec)
#define MAX_PAN_VELOCITY 700.0f
#define MAX_TILT_VELOCITY 700.0f
#define MAX_ZOOM_VELOCITY 150.0f

#endif // STEPPER_LIMITS_H
