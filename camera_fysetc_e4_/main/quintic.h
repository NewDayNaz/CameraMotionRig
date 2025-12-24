/**
 * @file quintic.h
 * @brief Quintic polynomial interpolation for smooth motion
 * 
 * Implements minimum-jerk quintic polynomial interpolation between waypoints.
 * Boundary conditions: position at start/end, zero velocity and acceleration at endpoints.
 * 
 * x(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
 * where:
 *   a0 = x0
 *   a1 = 0
 *   a2 = 0
 *   a3 = 10*(x1-x0)/T^3
 *   a4 = -15*(x1-x0)/T^4
 *   a5 = 6*(x1-x0)/T^5
 */

#ifndef QUINTIC_H
#define QUINTIC_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Quintic polynomial coefficients
 */
typedef struct {
    float a0, a1, a2, a3, a4, a5;
    float T;  // Total duration
} quintic_coeffs_t;

/**
 * @brief Easing function type
 */
typedef enum {
    EASING_LINEAR,
    EASING_SMOOTHERSTEP,  // Quintic smootherstep
    EASING_SIGMOID
} easing_type_t;

/**
 * @brief Initialize quintic coefficients for a move
 * @param coeffs Output coefficients structure
 * @param x0 Start position
 * @param x1 End position
 * @param T Total duration (seconds)
 */
void quintic_init(quintic_coeffs_t* coeffs, float x0, float x1, float T);

/**
 * @brief Evaluate quintic polynomial at time t
 * @param coeffs Polynomial coefficients
 * @param t Time (0 to T)
 * @return Position at time t
 */
float quintic_evaluate(const quintic_coeffs_t* coeffs, float t);

/**
 * @brief Evaluate quintic polynomial with easing
 * @param coeffs Polynomial coefficients
 * @param t Time (0 to T)
 * @param easing Easing type to apply
 * @return Position at time t (with easing applied)
 */
float quintic_evaluate_eased(const quintic_coeffs_t* coeffs, float t, easing_type_t easing);

/**
 * @brief Apply easing function to normalized time u (0 to 1)
 * @param u Normalized time
 * @param easing Easing type
 * @return Eased normalized time
 */
float easing_apply(float u, easing_type_t easing);

#endif // QUINTIC_H

