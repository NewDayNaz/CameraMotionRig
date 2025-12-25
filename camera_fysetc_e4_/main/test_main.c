/**
 * @file test_main.c
 * @brief Motor and Endstop Test Program for FYSETC E4 Board
 * 
 * This test program:
 * 1. Tests all motors (PAN, TILT, ZOOM) by moving them in both directions
 * 2. Tests all endstops to validate they're working
 * 
 * To use: Rename this file to main.c (backup original first), then build and flash
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

#include "board.h"
#include "stepper_executor.h"
#include "segment.h"

static const char* TAG = "test";

// Test configuration
#define TEST_STEPS 400        // Number of steps to move each motor
#define TEST_DELAY_MS 50      // Delay between direction changes
#define ENDSTOP_CHECK_DELAY_MS 100  // Delay for endstop reading

// Global segment queue for test
static segment_queue_t test_queue;

/**
 * @brief Simple function to generate step pulses directly (for basic motor test)
 */
static void test_motor_direct(uint8_t axis, int32_t steps, uint32_t delay_us) {
    gpio_num_t step_pin = step_pins[axis];
    gpio_num_t dir_pin = dir_pins[axis];
    
    // Set direction
    gpio_set_level(dir_pin, (steps > 0) ? 1 : 0);
    vTaskDelay(pdMS_TO_TICKS(10));  // Small delay for direction to settle
    
    ESP_LOGI(TAG, "Testing %s: %d steps, direction=%s", 
             axis_names[axis], 
             (steps > 0) ? steps : -steps,
             (steps > 0) ? "positive" : "negative");
    
    // Generate step pulses
    int32_t abs_steps = (steps > 0) ? steps : -steps;
    for (int32_t i = 0; i < abs_steps; i++) {
        gpio_set_level(step_pin, 1);
        // Delay using busy wait (delay_us is in microseconds)
        esp_rom_delay_us(delay_us);
        gpio_set_level(step_pin, 0);
        esp_rom_delay_us(delay_us);
        
        // Check for endstop trigger during movement (if moving toward endstop)
        if (steps < 0 && endstop_pins[axis] != GPIO_NUM_NC) {
            int level = gpio_get_level(endstop_pins[axis]);
            if (level == 0) {  // Active LOW
                ESP_LOGW(TAG, "%s endstop triggered at step %d", axis_names[axis], i);
                break;
            }
        }
    }
}

/**
 * @brief Test a single motor
 */
static void test_motor(uint8_t axis) {
    uint8_t driver_addr = board_get_tmc2209_address(axis);
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Testing %s motor (axis %d)", axis_names[axis], axis);
    ESP_LOGI(TAG, "STEP pin: GPIO%d, DIR pin: GPIO%d", 
             step_pins[axis], dir_pins[axis]);
    ESP_LOGI(TAG, "TMC2209 driver address: %d (UART1)", driver_addr);
    
    // Test positive direction
    ESP_LOGI(TAG, "Moving %s in positive direction...", axis_names[axis]);
    test_motor_direct(axis, TEST_STEPS, 500);  // 500us = 1000 steps/sec
    vTaskDelay(pdMS_TO_TICKS(TEST_DELAY_MS));
    
    // Test negative direction
    ESP_LOGI(TAG, "Moving %s in negative direction...", axis_names[axis]);
    test_motor_direct(axis, -TEST_STEPS, 500);
    vTaskDelay(pdMS_TO_TICKS(TEST_DELAY_MS));
    
    ESP_LOGI(TAG, "%s motor test complete", axis_names[axis]);
    ESP_LOGI(TAG, "========================================");
    vTaskDelay(pdMS_TO_TICKS(500));  // Pause before next motor
}

/**
 * @brief Test a single endstop
 */
static void test_endstop(uint8_t axis) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Testing %s endstop (axis %d)", axis_names[axis], axis);
    
    if (endstop_pins[axis] == GPIO_NUM_NC) {
        ESP_LOGW(TAG, "%s has no endstop configured (GPIO_NUM_NC)", axis_names[axis]);
        ESP_LOGI(TAG, "========================================");
        return;
    }
    
    gpio_num_t endstop_pin = endstop_pins[axis];
    uint8_t driver_addr = board_get_tmc2209_address(axis);
    ESP_LOGI(TAG, "Endstop pin: GPIO%d", endstop_pin);
    ESP_LOGI(TAG, "TMC2209 driver address: %d (UART1)", driver_addr);
    ESP_LOGI(TAG, "Endstops are active LOW (0 = triggered, 1 = not triggered)");
    
    // Read endstop state multiple times
    ESP_LOGI(TAG, "Reading endstop state (5 readings over 1 second)...");
    for (int i = 0; i < 5; i++) {
        int level = gpio_get_level(endstop_pin);
        bool triggered = (level == 0);  // Active LOW
        ESP_LOGI(TAG, "  Reading %d: GPIO level=%d, Endstop=%s", 
                 i + 1, level, triggered ? "TRIGGERED" : "NOT TRIGGERED");
        vTaskDelay(pdMS_TO_TICKS(ENDSTOP_CHECK_DELAY_MS));
    }
    
    ESP_LOGI(TAG, "Endstop test instructions:");
    ESP_LOGI(TAG, "  1. Manually trigger the %s endstop", axis_names[axis]);
    ESP_LOGI(TAG, "  2. Wait 2 seconds, then we'll check again...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    ESP_LOGI(TAG, "Checking endstop state after manual trigger...");
    for (int i = 0; i < 5; i++) {
        int level = gpio_get_level(endstop_pin);
        bool triggered = (level == 0);  // Active LOW
        ESP_LOGI(TAG, "  Reading %d: GPIO level=%d, Endstop=%s", 
                 i + 1, level, triggered ? "TRIGGERED" : "NOT TRIGGERED");
        vTaskDelay(pdMS_TO_TICKS(ENDSTOP_CHECK_DELAY_MS));
    }
    
    ESP_LOGI(TAG, "%s endstop test complete", axis_names[axis]);
    ESP_LOGI(TAG, "========================================");
    vTaskDelay(pdMS_TO_TICKS(1000));  // Pause before next endstop
}

/**
 * @brief Main test function
 */
static void run_tests(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "FYSETC E4 Board Test Program");
    ESP_LOGI(TAG, "Based on FluidNC configuration");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "TMC2209 Driver Addresses:");
    for (int i = 0; i < NUM_AXES; i++) {
        ESP_LOGI(TAG, "  %s (axis %d): address %d", axis_names[i], i, board_get_tmc2209_address(i));
    }
    ESP_LOGI(TAG, "UART1: TX=GPIO%d, RX=GPIO%d, Baud=%d", 
             PIN_UART1_TX, PIN_UART1_RX, TMC2209_UART_BAUD);
    ESP_LOGI(TAG, "Starting tests in 3 seconds...");
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Enable stepper drivers
    ESP_LOGI(TAG, "Enabling stepper drivers...");
    board_set_enable(true);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Test all motors
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "MOTOR TESTS");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Testing motors: %d steps each direction", TEST_STEPS);
    ESP_LOGI(TAG, "Watch the motors - they should move smoothly");
    ESP_LOGI(TAG, "");
    
    test_motor(AXIS_PAN);
    test_motor(AXIS_TILT);
    test_motor(AXIS_ZOOM);
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Motor tests complete!");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Test all endstops
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "ENDSTOP TESTS");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "You will be prompted to manually trigger each endstop");
    ESP_LOGI(TAG, "");
    
    test_endstop(AXIS_PAN);
    test_endstop(AXIS_TILT);
    test_endstop(AXIS_ZOOM);
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "ALL TESTS COMPLETE!");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Summary:");
    ESP_LOGI(TAG, "  - Motor tests: Check if all motors moved smoothly");
    ESP_LOGI(TAG, "  - Endstop tests: Check if endstops responded correctly");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "If any tests failed, check:");
    ESP_LOGI(TAG, "  1. Motor wiring (step, dir, enable, power)");
    ESP_LOGI(TAG, "  2. Endstop wiring (signal, ground, pullup resistors)");
    ESP_LOGI(TAG, "  3. GPIO pin assignments in board.h");
    ESP_LOGI(TAG, "");
    
    // Keep running and periodically check endstops
    ESP_LOGI(TAG, "Entering continuous endstop monitoring mode...");
    ESP_LOGI(TAG, "Press Ctrl+C to stop");
    
    while (1) {
        ESP_LOGI(TAG, "Endstop states: ");
        for (int axis = 0; axis < NUM_AXES; axis++) {
            if (endstop_pins[axis] != GPIO_NUM_NC) {
                int level = gpio_get_level(endstop_pins[axis]);
                bool triggered = (level == 0);
                ESP_LOGI(TAG, "  %s (GPIO%d): %s", 
                         axis_names[axis], 
                         endstop_pins[axis],
                         triggered ? "TRIGGERED" : "open");
            } else {
                ESP_LOGI(TAG, "  %s: no endstop configured", axis_names[axis]);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Initializing test program...");
    
    // Initialize board GPIO
    board_init();
    
    // Initialize segment queue (required for stepper_executor, but we won't use it)
    segment_queue_init(&test_queue);
    stepper_executor_init(&test_queue);
    
    ESP_LOGI(TAG, "Initialization complete");
    ESP_LOGI(TAG, "");
    
    // Run tests
    run_tests();
}
