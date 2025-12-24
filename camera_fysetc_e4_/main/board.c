/**
 * @file board.c
 * @brief FYSETC E4 board pin initialization and control
 */

#include "board.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

static const char* TAG = "board";

// Axis to Pin Mapping Arrays
const gpio_num_t step_pins[NUM_AXES] = {
    PIN_X_STEP,   // PAN (X-axis)
    PIN_Y_STEP,   // TILT (Y-axis)
    PIN_E0_STEP   // ZOOM (E0-axis)
};

const gpio_num_t dir_pins[NUM_AXES] = {
    PIN_X_DIR,    // PAN
    PIN_Y_DIR,    // TILT
    PIN_E0_DIR    // ZOOM
};

// Endstop pin mapping (GPIO_NUM_NC indicates no endstop)
// NOTE: GPIO15 (PIN_Z_MIN) is a strap pin - use external pullup resistor
// Alternatively, set to GPIO_NUM_NC to use sensorless homing for zoom
const gpio_num_t endstop_pins[NUM_AXES] = {
    PIN_X_MIN,    // PAN endstop
    PIN_Y_MIN,    // TILT endstop
    PIN_Z_MIN     // ZOOM endstop (GPIO15 - requires external pullup)
};

// Axis Names
const char* axis_names[NUM_AXES] = {
    "PAN",
    "TILT",
    "ZOOM"
};

void board_init(void) {
    // Configure step pins as outputs
    for (int i = 0; i < NUM_AXES; i++) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << step_pins[i]),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&io_conf);
        gpio_set_level(step_pins[i], 0);  // Start with step low
        ESP_LOGI(TAG, "Configured %s STEP pin: GPIO%d", axis_names[i], step_pins[i]);
    }

    // Configure direction pins as outputs
    for (int i = 0; i < NUM_AXES; i++) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << dir_pins[i]),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&io_conf);
        gpio_set_level(dir_pins[i], 0);  // Start with default direction
        ESP_LOGI(TAG, "Configured %s DIR pin: GPIO%d", axis_names[i], dir_pins[i]);
    }

    // Configure enable pin (shared for all axes)
    gpio_config_t en_conf = {
        .pin_bit_mask = (1ULL << PIN_X_EN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&en_conf);
    board_set_enable(false);  // Start with drivers disabled

    // Configure endstop pins as inputs
    // Note: GPIO34/35 are input-only with no internal pullups
    // External pullups must be provided in hardware
    for (int i = 0; i < NUM_AXES; i++) {
        if (endstop_pins[i] != GPIO_NUM_NC) {
            gpio_config_t endstop_conf = {
                .pin_bit_mask = (1ULL << endstop_pins[i]),
                .mode = GPIO_MODE_INPUT,
                .pull_up_en = GPIO_PULLUP_DISABLE,  // No internal pullup on GPIO34/35
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type = GPIO_INTR_DISABLE
            };
            gpio_config(&endstop_conf);
        }
    }
}

void board_set_enable(bool enable) {
    // TMC2209 enable is active LOW, so invert the logic
    gpio_set_level(PIN_X_EN, enable ? 0 : 1);
}

