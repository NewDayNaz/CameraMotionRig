/**
 * @file quintic.cpp
 * @brief Quintic polynomial implementation
 */

#include "quintic.h"
#include <math.h>

void quintic_init(quintic_coeffs_t* coeffs, float x0, float x1, float T) {
    if (T <= 0.0f) {
        T = 0.001f;  // Minimum duration to avoid division by zero
    }
    
    float dx = x1 - x0;
    float T2 = T * T;
    float T3 = T2 * T;
    float T4 = T3 * T;
    float T5 = T4 * T;
    
    coeffs->a0 = x0;
    coeffs->a1 = 0.0f;
    coeffs->a2 = 0.0f;
    coeffs->a3 = 10.0f * dx / T3;
    coeffs->a4 = -15.0f * dx / T4;
    coeffs->a5 = 6.0f * dx / T5;
    coeffs->T = T;
}

float quintic_evaluate(const quintic_coeffs_t* coeffs, float t) {
    // Clamp t to [0, T]
    if (t < 0.0f) t = 0.0f;
    if (t > coeffs->T) t = coeffs->T;
    
    float t2 = t * t;
    float t3 = t2 * t;
    float t4 = t3 * t;
    float t5 = t4 * t;
    
    return coeffs->a0 + 
           coeffs->a1 * t + 
           coeffs->a2 * t2 + 
           coeffs->a3 * t3 + 
           coeffs->a4 * t4 + 
           coeffs->a5 * t5;
}

float easing_apply(float u, easing_type_t easing) {
    // Clamp u to [0, 1]
    if (u < 0.0f) u = 0.0f;
    if (u > 1.0f) u = 1.0f;
    
    switch (easing) {
        case EASING_LINEAR:
            return u;
            
        case EASING_SMOOTHERSTEP: {
            // Quintic smootherstep: 6u^5 - 15u^4 + 10u^3
            float u2 = u * u;
            float u3 = u2 * u;
            float u4 = u3 * u;
            float u5 = u4 * u;
            return 6.0f * u5 - 15.0f * u4 + 10.0f * u3;
        }
        
        case EASING_SIGMOID: {
            // Sigmoid: 1 / (1 + exp(-12*(u-0.5)))
            // Approximate for efficiency: use tanh-based sigmoid
            float x = 12.0f * (u - 0.5f);
            // Use tanh approximation: tanh(x) ≈ x / (1 + |x|) for small x
            // For sigmoid: 0.5 + 0.5*tanh(x/2)
            if (x > 10.0f) return 1.0f;
            if (x < -10.0f) return 0.0f;
            return 0.5f + 0.5f * tanh(x * 0.5f);
        }
        
        default:
            return u;
    }
}

float quintic_evaluate_eased(const quintic_coeffs_t* coeffs, float t, easing_type_t easing) {
    // Normalize time to [0, 1]
    float u = (coeffs->T > 0.0f) ? (t / coeffs->T) : 0.0f;
    
    // Apply easing
    float u2 = easing_apply(u, easing);
    
    // Evaluate at eased time
    return quintic_evaluate(coeffs, u2 * coeffs->T);
}

