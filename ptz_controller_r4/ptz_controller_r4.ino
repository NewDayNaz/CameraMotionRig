/**
 * @file ptz_controller_r4.ino
 * @brief Main firmware for 3-axis cinematic PTZ rig on Arduino Uno R4 Minima
 * 
 * Features:
 * - Film-quality smooth motion using quintic polynomial interpolation
 * - Segment-based deterministic step execution (5-10ms segments)
 * - Endstop-based homing for PAN and TILT
 * - Preset storage and recall with cinematic parameters
 * - Manual joystick control via USB serial from Raspberry Pi
 * - Deadband, expo, slew limiting, and soft limits
 */

#include "board.h"
#include "segment.h"
#include "quintic.h"
#include "stepper_executor.h"
#include "motion_planner.h"
#include "homing.h"
#include "preset_storage.h"
#include "usb_serial.h"

// Global instances
segment_queue_t g_segment_queue;
motion_planner_t g_motion_planner;
motion_planner_t* g_planner = &g_motion_planner;
bool g_precision_mode = false;

// Timing
unsigned long g_last_update_ms = 0;
const unsigned long UPDATE_INTERVAL_MS = 5;  // 5ms update rate (200Hz)

void setup() {
    // Initialize board
    board_init();
    board_set_enable(true);  // Enable stepper drivers
    
    // Initialize segment queue
    segment_queue_init(&g_segment_queue);
    
    // Initialize stepper executor
    stepper_executor_init(&g_segment_queue);
    stepper_executor_start();
    
    // Initialize motion planner
    motion_planner_init(&g_motion_planner, &g_segment_queue);
    
    // Initialize homing
    homing_init(&g_motion_planner);
    
    // Initialize preset storage
    preset_storage_init();
    
    // Initialize USB serial
    usb_serial_init();
    
    // Set initial positions (will be updated after homing)
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        motion_planner_set_position(&g_motion_planner, i, 0.0f);
        stepper_executor_set_position(i, 0);
    }
    
    // Set soft limits (adjust based on your mechanical limits)
    motion_planner_set_limits(&g_motion_planner, AXIS_PAN, -20000.0f, 20000.0f);
    motion_planner_set_limits(&g_motion_planner, AXIS_TILT, -10000.0f, 10000.0f);
    motion_planner_set_limits(&g_motion_planner, AXIS_ZOOM, 0.0f, 2000.0f);
    
    g_last_update_ms = millis();
    
    // Send startup message
    Serial.println("PTZ Controller Ready");
    Serial.println("Commands: VEL, GOTO, SAVE, HOME, STOP, PRECISION, POS, STATUS");
}

void loop() {
    unsigned long now_ms = millis();
    float dt = (now_ms - g_last_update_ms) / 1000.0f;  // Convert to seconds
    
    // Process USB serial commands
    usb_serial_process();
    
    // Update homing
    if (homing_is_active()) {
        homing_update(dt);
    }
    
    // Update motion planner (generates segments)
    motion_planner_update(&g_motion_planner, dt);
    
    // Update stepper executor (consumes segments and generates steps)
    stepper_executor_update();
    
    // Sync planner positions with executor positions
    // This ensures planner knows actual position after steps are executed
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        int32_t exec_pos = stepper_executor_get_position(i);
        float planner_pos = motion_planner_get_position(&g_motion_planner, i);
        
        // If positions differ significantly, sync planner to executor
        if (fabsf(planner_pos - (float)exec_pos) > 1.0f) {
            motion_planner_set_position(&g_motion_planner, i, (float)exec_pos);
        }
    }
    
    g_last_update_ms = now_ms;
    
    // Small delay to prevent tight loop (Arduino R4 can handle this, but good practice)
    delayMicroseconds(100);
}

