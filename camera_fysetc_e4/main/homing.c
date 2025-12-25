/**
 * @file homing.c
 * @brief Homing implementation
 */

#include "homing.h"
#include "board.h"
#if ZOOM_USE_SENSORLESS_HOMING
#include "tmc2209.h"
#endif
#include "esp_log.h"
#include <string.h>
#include <math.h>

// Import microstepping scale from board.h
#ifndef MICROSTEP_SCALE
#define MICROSTEP_SCALE 1.0f  // Default to no scaling if not defined
#endif

#if ZOOM_USE_SENSORLESS_HOMING && !defined(TMC2209_DEFAULT_SGTHRS)
#error "ZOOM_USE_SENSORLESS_HOMING requires tmc2209.h"
#endif

static const char* TAG = "homing";

static homing_status_t homing_status;

// Stall detection debouncing (require N consecutive readings)
#define STALL_DEBOUNCE_COUNT 3

// Minimum movement before trusting stall detection (steps)
// Prevents false triggers when motor first starts
#define MIN_MOVEMENT_FOR_STALL 50

void homing_init(void) {
    memset(&homing_status, 0, sizeof(homing_status));
    homing_status.state = HOMING_IDLE;
    homing_status.num_axes_to_home = 0;
    homing_status.current_axis_index = 0;
    homing_status.is_sensorless = false;
    #if ZOOM_USE_SENSORLESS_HOMING
    homing_status.stall_threshold = TMC2209_DEFAULT_SGTHRS;
    #else
    homing_status.stall_threshold = 0;
    #endif
    homing_status.stall_readings = 0;
    homing_status.stall_check_start_pos = 0.0f;
}

bool homing_start_sequential(const uint8_t* axes, uint8_t num_axes) {
    if (homing_status.state != HOMING_IDLE && homing_status.state != HOMING_COMPLETE) {
        ESP_LOGW(TAG, "Homing already in progress");
        return false;
    }
    
    // Validate axes (PAN/TILT use endstops, ZOOM uses sensorless)
    for (uint8_t i = 0; i < num_axes; i++) {
        if (axes[i] >= NUM_AXES) {
            ESP_LOGW(TAG, "Invalid axis %d", axes[i]);
            return false;
        }
    }
    
    // Setup sequential homing queue
    homing_status.num_axes_to_home = (num_axes > 3) ? 3 : num_axes;  // Limit to 3 axes
    for (uint8_t i = 0; i < homing_status.num_axes_to_home; i++) {
        homing_status.axes_to_home[i] = axes[i];
    }
    homing_status.current_axis_index = 0;
    
    // Start first axis
    if (num_axes > 0) {
        homing_status.axis = homing_status.axes_to_home[0];
        // Determine if sensorless based on configuration
        homing_status.is_sensorless = (homing_status.axis == AXIS_ZOOM && ZOOM_USE_SENSORLESS_HOMING);
        #if ZOOM_USE_SENSORLESS_HOMING
        homing_status.stall_threshold = TMC2209_DEFAULT_SGTHRS;
        #else
        homing_status.stall_threshold = 0;
        #endif
        homing_status.stall_readings = 0;
        homing_status.stall_check_start_pos = 0.0f;  // Will be set when fast approach starts
        homing_status.state = HOMING_FAST_APPROACH;
        homing_status.start_position = 0.0f;
        homing_status.endstop_triggered = false;
        homing_status.timeout_start_time = 0.0f;
        
        ESP_LOGI(TAG, "Starting sequential homing: %d axes, starting with axis %d (%s)", 
                num_axes, homing_status.axis, homing_status.is_sensorless ? "sensorless" : "endstop");
        return true;
    }
    
    return false;
}

bool homing_start(uint8_t axis) {
    // Handle "home all" case (255)
    if (axis == 255 || axis >= NUM_AXES) {
        uint8_t axes[] = {AXIS_PAN, AXIS_TILT, AXIS_ZOOM};
        return homing_start_sequential(axes, 3);
    }
    
    // Single axis homing - determine if sensorless based on configuration
    bool is_sensorless = (axis == AXIS_ZOOM && ZOOM_USE_SENSORLESS_HOMING);
    
    // Validate axis
    if (axis != AXIS_PAN && axis != AXIS_TILT && axis != AXIS_ZOOM) {
        ESP_LOGE(TAG, "Cannot home axis %d", axis);
        return false;
    }
    
    // Check if zoom axis has required hardware (endstop or sensorless configured)
    if (axis == AXIS_ZOOM) {
        if (ZOOM_USE_SENSORLESS_HOMING) {
            ESP_LOGI(TAG, "Zoom axis using sensorless homing (requires TMC2209 UART mode)");
        } else {
            // Check if physical endstop is configured
            extern const gpio_num_t endstop_pins[];
            if (endstop_pins[AXIS_ZOOM] == GPIO_NUM_NC) {
                ESP_LOGE(TAG, "Zoom axis has no endstop configured and sensorless is disabled");
                ESP_LOGE(TAG, "Either enable ZOOM_USE_SENSORLESS_HOMING or configure PIN_Z_MIN endstop");
                return false;
            }
            ESP_LOGI(TAG, "Zoom axis using physical endstop (GPIO%d)", endstop_pins[AXIS_ZOOM]);
        }
    }
    
    if (homing_status.state != HOMING_IDLE && homing_status.state != HOMING_COMPLETE) {
        ESP_LOGW(TAG, "Homing already in progress");
        return false;
    }
    
    // Setup for single axis
    homing_status.num_axes_to_home = 1;
    homing_status.axes_to_home[0] = axis;
    homing_status.current_axis_index = 0;
    homing_status.is_sensorless = is_sensorless;
    #if ZOOM_USE_SENSORLESS_HOMING
    homing_status.stall_threshold = TMC2209_DEFAULT_SGTHRS;
    #else
    homing_status.stall_threshold = 0;
    #endif
    homing_status.stall_readings = 0;
    homing_status.stall_check_start_pos = 0.0f;  // Will be set when fast approach starts
    
    homing_status.axis = axis;
    homing_status.state = HOMING_FAST_APPROACH;
    homing_status.start_position = 0.0f;
    homing_status.endstop_triggered = false;
    homing_status.timeout_start_time = 0.0f;
    
    ESP_LOGI(TAG, "Starting homing for axis %d (%s)", axis, is_sensorless ? "sensorless" : "endstop");
    return true;
}

bool homing_check_stall(uint8_t axis) {
    #if ZOOM_USE_SENSORLESS_HOMING
    if (axis == AXIS_ZOOM) {
        // Use sensorless stall detection for zoom
        uint8_t sg_result = tmc2209_get_stallguard_result(axis);
        
        // Ignore stall if motor hasn't moved enough yet (prevents false triggers at startup)
        // This is checked in homing_update() before calling this function
        
        bool stalled = (sg_result < homing_status.stall_threshold && sg_result != 255);
        
        // Debounce: require multiple consecutive readings
        if (stalled) {
            homing_status.stall_readings++;
            // Log stall detection for debugging (only occasionally to avoid spam)
            if (homing_status.stall_readings == STALL_DEBOUNCE_COUNT) {
                ESP_LOGI(TAG, "Stall detected: SG_RESULT=%d (threshold=%d)", 
                        sg_result, homing_status.stall_threshold);
            }
        } else {
            if (homing_status.stall_readings > 0) {
                // Reset counter if not stalled
                homing_status.stall_readings = 0;
            }
        }
        
        return (homing_status.stall_readings >= STALL_DEBOUNCE_COUNT);
    }
    #endif
    
    return false;
}

bool homing_update(float dt, float current_position, bool endstop_state) {
    if (homing_status.state == HOMING_IDLE || homing_status.state == HOMING_COMPLETE) {
        return false;
    }
    
    // Initialize timeout timer on first update
    if (homing_status.timeout_start_time == 0.0f) {
        homing_status.timeout_start_time = 0.001f;  // Mark as started (use small non-zero value)
    }
    
    // Accumulate timeout time
    homing_status.timeout_start_time += dt;
    
    // Check timeout
    if (homing_status.timeout_start_time > HOMING_TIMEOUT_S) {
        ESP_LOGE(TAG, "Homing timeout for axis %d", homing_status.axis);
        homing_status.state = HOMING_ERROR;
        homing_status.timeout_start_time = 0.0f;  // Reset for next attempt
        return false;
    }
    
    // Get trigger state (endstop or stall)
    bool triggered = false;
    if (homing_status.is_sensorless) {
        // For sensorless, only check for stall if motor has moved enough
        // This prevents false triggers when motor first starts
        float movement = fabsf(current_position - homing_status.stall_check_start_pos);
        
            #if ZOOM_USE_SENSORLESS_HOMING
            // Always read SG_RESULT for diagnostics, but only trigger after minimum movement
            uint8_t sg_result = tmc2209_get_stallguard_result(homing_status.axis);
            
            // Log diagnostic info occasionally (every 50 updates = ~0.5 seconds)
            static int diagnostic_count = 0;
            if ((diagnostic_count++ % 50) == 0 && movement > 0) {
                ESP_LOGI(TAG, "Sensorless homing: pos=%.1f, movement=%.1f, SG_RESULT=%d", 
                        current_position, movement, sg_result);
            }
            #endif
        
            // Scale minimum movement threshold by microstepping factor
            if (movement >= (MIN_MOVEMENT_FOR_STALL * MICROSTEP_SCALE)) {
                // Motor has moved enough - check for stall
                #if ZOOM_USE_SENSORLESS_HOMING
                // Note: If SG_RESULT is always 0, there's a configuration problem
                if (sg_result == 0) {
                    // Motor appears fully stalled - this could be real stall or config issue
                    // Only log occasionally to avoid spam
                    static int diagnostic_count_stall = 0;
                    if ((diagnostic_count_stall++ % 100) == 0) {
                        ESP_LOGW(TAG, "SG_RESULT=0 at position %.1f (movement=%.1f) - check if motor is actually stalling", 
                                current_position, movement);
                    }
                }
                #endif
                triggered = homing_check_stall(homing_status.axis);
            } else {
                // Motor hasn't moved enough yet - don't check for stall
                #if ZOOM_USE_SENSORLESS_HOMING
                // But log if we're getting SG_RESULT=0 early (indicates motor might be blocked or config issue)
                if (sg_result == 0 && movement > 0.5f) {
                    ESP_LOGW(TAG, "SG_RESULT=0 detected early (movement=%.1f) - motor may be blocked or stallGuard not configured", movement);
                }
                #endif
                triggered = false;
                homing_status.stall_readings = 0;  // Reset counter
            }
    } else {
        triggered = endstop_state;
    }
    
    switch (homing_status.state) {
        case HOMING_FAST_APPROACH:
            // Initialize stall check start position on first entry
            if (homing_status.is_sensorless && homing_status.stall_check_start_pos == 0.0f) {
                homing_status.stall_check_start_pos = current_position;
                ESP_LOGI(TAG, "Starting fast approach for axis %d (sensorless), start pos=%.1f", 
                        homing_status.axis, current_position);
            }
            
            // Check if motor is actually moving (for diagnostics)
            #if ZOOM_USE_SENSORLESS_HOMING
            if (homing_status.is_sensorless) {
                float movement = fabsf(current_position - homing_status.stall_check_start_pos);
                static float last_logged_movement = -1.0f;
                static int sg_zero_count = 0;
                
                uint8_t sg_result = tmc2209_get_stallguard_result(homing_status.axis);
                
                // Safety check: if SG_RESULT=0 continuously for too long, motor is blocked
                if (sg_result == 0) {
                    sg_zero_count++;
                    // If we see SG_RESULT=0 for more than 2 seconds without movement, abort
                    if (sg_zero_count > 200 && movement < 10.0f) {  // 200 updates * 10ms = 2 seconds
                        ESP_LOGE(TAG, "Motor appears blocked (SG_RESULT=0 for 2s, movement=%.1f) - aborting homing", movement);
                        homing_status.state = HOMING_ERROR;
                        return false;
                    }
                } else {
                    sg_zero_count = 0;  // Reset counter if not stalled
                }
                
                // Log movement progress every 100 steps
                if ((int)(movement / 100.0f) > (int)(last_logged_movement / 100.0f)) {
                    ESP_LOGI(TAG, "Fast approach: movement=%.1f, SG_RESULT=%d", movement, sg_result);
                    last_logged_movement = movement;
                }
            }
            #endif
            
            if (triggered) {
                // Endstop/stall detected - start backoff
                float movement = fabsf(current_position - homing_status.stall_check_start_pos);
                // Scale backoff distance by microstepping factor
                homing_status.backoff_target = current_position + (HOMING_BACKOFF_STEPS * MICROSTEP_SCALE);
                homing_status.state = HOMING_BACKOFF;
                homing_status.endstop_triggered = false;
                homing_status.stall_readings = 0;  // Reset stall counter
                homing_status.stall_check_start_pos = 0.0f;  // Reset for next phase
                
                if (homing_status.is_sensorless) {
                    #if ZOOM_USE_SENSORLESS_HOMING
                    uint8_t sg_result = tmc2209_get_stallguard_result(homing_status.axis);
                    ESP_LOGI(TAG, "Fast approach complete (stall detected, SG_RESULT=%d, movement=%.1f), backing off", 
                            sg_result, movement);
                    #endif
                } else {
                    ESP_LOGI(TAG, "Fast approach complete (endstop triggered), backing off");
                }
            }
            break;
            
        case HOMING_BACKOFF:
            // Move away from endstop until backoff distance reached
            if (fabsf(current_position - homing_status.backoff_target) < 10.0f) {
                // Backoff complete - start slow approach
                homing_status.state = HOMING_SLOW_APPROACH;
                ESP_LOGI(TAG, "Backoff complete, starting slow approach");
            }
            break;
            
        case HOMING_SLOW_APPROACH:
            // Initialize stall check start position on first entry to slow approach
            if (homing_status.is_sensorless && homing_status.stall_check_start_pos == 0.0f) {
                homing_status.stall_check_start_pos = current_position;
            }
            
            if (triggered) {
                // Endstop/stall triggered again - current axis homing complete
                if (homing_status.is_sensorless) {
                    #if ZOOM_USE_SENSORLESS_HOMING
                    uint8_t sg_result = tmc2209_get_stallguard_result(homing_status.axis);
                    ESP_LOGI(TAG, "Homing complete for axis %d at position %f (stall detected, SG_RESULT=%d)", 
                            homing_status.axis, current_position, sg_result);
                    #endif
                } else {
                    ESP_LOGI(TAG, "Homing complete for axis %d at position %f (endstop triggered)", 
                            homing_status.axis, current_position);
                }
                
                // Reset stall check position
                homing_status.stall_check_start_pos = 0.0f;
                
                // Reset stall counter
                homing_status.stall_readings = 0;
                
                // Check if there are more axes to home
                homing_status.current_axis_index++;
                if (homing_status.current_axis_index < homing_status.num_axes_to_home) {
                    // Start next axis in sequence
                    homing_status.axis = homing_status.axes_to_home[homing_status.current_axis_index];
                    // Determine if sensorless based on configuration
                    homing_status.is_sensorless = (homing_status.axis == AXIS_ZOOM && ZOOM_USE_SENSORLESS_HOMING);
                    #if ZOOM_USE_SENSORLESS_HOMING
                    homing_status.stall_threshold = TMC2209_DEFAULT_SGTHRS;
                    #else
                    homing_status.stall_threshold = 0;
                    #endif
                    homing_status.stall_readings = 0;
                    homing_status.stall_check_start_pos = 0.0f;  // Will be set when fast approach starts
                    homing_status.state = HOMING_FAST_APPROACH;
                    homing_status.start_position = 0.0f;
                    homing_status.endstop_triggered = false;
                    homing_status.timeout_start_time = 0.0f;  // Reset timeout for next axis
                    ESP_LOGI(TAG, "Axis %d homed, starting next axis %d (%s, %d/%d)", 
                            homing_status.axes_to_home[homing_status.current_axis_index - 1],
                            homing_status.axis,
                            homing_status.is_sensorless ? "sensorless" : "endstop",
                            homing_status.current_axis_index + 1,
                            homing_status.num_axes_to_home);
                    return true;  // Continue homing sequence
                } else {
                    // All axes homed
                    homing_status.state = HOMING_COMPLETE;
                    homing_status.timeout_start_time = 0.0f;
                    ESP_LOGI(TAG, "Sequential homing complete - all %d axes homed", 
                            homing_status.num_axes_to_home);
                    return false;
                }
            }
            break;
            
        default:
            break;
    }
    
    return true;  // Homing still in progress
}

homing_status_t homing_get_status(void) {
    return homing_status;
}

bool homing_is_active(void) {
    return homing_status.state != HOMING_IDLE && 
           homing_status.state != HOMING_COMPLETE && 
           homing_status.state != HOMING_ERROR;
}

void homing_abort(void) {
    if (homing_status.state != HOMING_IDLE) {
        ESP_LOGI(TAG, "Aborting homing sequence (was on axis %d)", homing_status.axis);
        homing_status.state = HOMING_IDLE;
        homing_status.num_axes_to_home = 0;
        homing_status.current_axis_index = 0;
        homing_status.stall_readings = 0;
    }
}

float homing_get_target_velocity(void) {
    if (homing_status.state == HOMING_IDLE || 
        homing_status.state == HOMING_COMPLETE ||
        homing_status.state == HOMING_ERROR) {
        return 0.0f;
    }
    
    // Velocity is negative (toward endstop/minimum)
    if (homing_status.state == HOMING_FAST_APPROACH) {
        // Scale homing speeds by microstepping factor
        return -HOMING_FAST_SPEED * MICROSTEP_SCALE;
    } else if (homing_status.state == HOMING_BACKOFF) {
        return HOMING_FAST_SPEED * MICROSTEP_SCALE;  // Positive - away from endstop
    } else if (homing_status.state == HOMING_SLOW_APPROACH) {
        return -HOMING_SLOW_SPEED * MICROSTEP_SCALE;
    }
    
    return 0.0f;
}

