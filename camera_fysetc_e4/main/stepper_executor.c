/**
 * @file stepper_executor.c
 * @brief Deterministic step generation using GPTimer ISR
 * 
 * ISR runs at 40kHz (25us period). Each segment is executed over multiple
 * ISR ticks using DDA/Bresenham style step distribution.
 */

#include "stepper_executor.h"
#include "segment.h"
#include "board.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "soc/gpio_reg.h"
#include "soc/soc.h"
#include <string.h>

// ESP32 GPIO32-39 use GPIO_OUT1 registers
// If not defined, define them based on ESP32 register layout
#ifndef GPIO_OUT1_W1TS_REG
#define GPIO_OUT1_W1TS_REG (DR_REG_GPIO_BASE + 0x0014)  // GPIO_OUT1_W1TS
#endif
#ifndef GPIO_OUT1_W1TC_REG
#define GPIO_OUT1_W1TC_REG (DR_REG_GPIO_BASE + 0x0018)  // GPIO_OUT1_W1TC
#endif

static const char* TAG = "stepper_executor";

// ISR frequency: 40kHz = 25us period
#define ISR_FREQUENCY_HZ 40000
#define ISR_PERIOD_US 25

// State for current segment execution
static struct {
    segment_queue_t* queue;
    motion_segment_t current_segment;
    bool has_segment;
    uint32_t segment_ticks_total;      // Total ticks for current segment
    uint32_t segment_ticks_remaining;
    
    // DDA accumulators for step distribution (one per axis)
    int32_t accum[NUM_AXES];
    int32_t steps_total[NUM_AXES];     // Total steps for this segment (absolute)
    int32_t steps_remaining[NUM_AXES]; // Remaining steps (absolute value)
    int32_t step_direction[NUM_AXES];  // Direction: +1 or -1
    
    // Current axis positions
    int32_t positions[NUM_AXES];
    
    // Step pulse state (toggle each ISR tick)
    bool step_pulse_state[NUM_AXES];
} executor_state;

// GPTimer handle
static gptimer_handle_t gptimer = NULL;

/**
 * @brief GPTimer ISR callback - executes step pulses
 */
static bool IRAM_ATTR timer_isr_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_data) {
    bool need_yield = false;
    
    // If no segment is active, try to get one from queue
    if (!executor_state.has_segment) {
        if (segment_queue_pop(executor_state.queue, &executor_state.current_segment)) {
            executor_state.has_segment = true;
            
            // Calculate segment duration in ISR ticks
            executor_state.segment_ticks_total = 
                executor_state.current_segment.duration_us / ISR_PERIOD_US;
            executor_state.segment_ticks_remaining = executor_state.segment_ticks_total;
            
            // Initialize step distribution for this segment
            for (int axis = 0; axis < NUM_AXES; axis++) {
                int32_t steps = executor_state.current_segment.steps[axis];
                
                // Determine direction
                if (steps > 0) {
                    executor_state.step_direction[axis] = 1;
                    executor_state.steps_total[axis] = steps;
                    executor_state.steps_remaining[axis] = steps;
                } else if (steps < 0) {
                    executor_state.step_direction[axis] = -1;
                    executor_state.steps_total[axis] = -steps;
                    executor_state.steps_remaining[axis] = -steps;
                } else {
                    executor_state.step_direction[axis] = 0;
                    executor_state.steps_total[axis] = 0;
                    executor_state.steps_remaining[axis] = 0;
                }
                
                // Reset DDA accumulator
                executor_state.accum[axis] = 0;
                executor_state.step_pulse_state[axis] = false;
            }
        } else {
            // No segment available - hold position (no steps)
            return false;
        }
    }
    
    // Execute current segment
    if (executor_state.has_segment && executor_state.segment_ticks_remaining > 0) {
        executor_state.segment_ticks_remaining--;
        
        // Process each axis using DDA/Bresenham
        for (int axis = 0; axis < NUM_AXES; axis++) {
            if (executor_state.steps_remaining[axis] > 0) {
                // DDA: accumulate steps_total and check if step should be generated
                executor_state.accum[axis] += executor_state.steps_total[axis];
                
                if (executor_state.accum[axis] >= (int32_t)executor_state.segment_ticks_total) {
                    // Time to emit a step
                    
                    // Set direction pin using direct register access (IRAM-safe)
                    // Only set direction when starting a new step (when pulse state is false)
                    if (!executor_state.step_pulse_state[axis]) {
                        gpio_num_t dir_pin = dir_pins[axis];
                        // Invert direction for PAN axis (axis 0) to fix left/right issue
                        int32_t dir = executor_state.step_direction[axis];
                        if (axis == AXIS_PAN) {
                            dir = -dir;  // Invert PAN direction
                        }
                        // GPIO32-39 use different registers (GPIO_OUT1) on ESP32
                        if (dir_pin >= 32) {
                            if (dir > 0) {
                                REG_WRITE(GPIO_OUT1_W1TS_REG, (1ULL << (dir_pin - 32)));
                            } else {
                                REG_WRITE(GPIO_OUT1_W1TC_REG, (1ULL << (dir_pin - 32)));
                            }
                        } else {
                        if (dir > 0) {
                            REG_WRITE(GPIO_OUT_W1TS_REG, (1ULL << dir_pin));
                        } else {
                            REG_WRITE(GPIO_OUT_W1TC_REG, (1ULL << dir_pin));
                            }
                        }
                    }
                    
                    // Generate step pulse (toggle) using direct register access (IRAM-safe)
                    executor_state.step_pulse_state[axis] = !executor_state.step_pulse_state[axis];
                    gpio_num_t step_pin = step_pins[axis];
                    // GPIO32-39 use different registers (GPIO_OUT1) on ESP32
                    if (step_pin >= 32) {
                        if (executor_state.step_pulse_state[axis]) {
                            REG_WRITE(GPIO_OUT1_W1TS_REG, (1ULL << (step_pin - 32)));
                        } else {
                            REG_WRITE(GPIO_OUT1_W1TC_REG, (1ULL << (step_pin - 32)));
                            // Step pulse complete - update position and decrement
                            executor_state.positions[axis] += executor_state.step_direction[axis];
                            executor_state.steps_remaining[axis]--;
                        }
                    } else {
                    if (executor_state.step_pulse_state[axis]) {
                        REG_WRITE(GPIO_OUT_W1TS_REG, (1ULL << step_pin));
                    } else {
                        REG_WRITE(GPIO_OUT_W1TC_REG, (1ULL << step_pin));
                        // Step pulse complete - update position and decrement
                        executor_state.positions[axis] += executor_state.step_direction[axis];
                        executor_state.steps_remaining[axis]--;
                        }
                    }
                    
                    // Subtract segment ticks from accumulator
                    executor_state.accum[axis] -= executor_state.segment_ticks_total;
                }
            }
        }
        
        // Segment complete
        if (executor_state.segment_ticks_remaining == 0) {
            executor_state.has_segment = false;
        }
    }
    
    return need_yield;
}

bool stepper_executor_init(segment_queue_t* queue) {
    memset(&executor_state, 0, sizeof(executor_state));
    executor_state.queue = queue;
    
    // Initialize positions
    for (int i = 0; i < NUM_AXES; i++) {
        executor_state.positions[i] = 0;
    }
    
    // Configure GPTimer
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = ISR_FREQUENCY_HZ,
    };
    
    esp_err_t ret = gptimer_new_timer(&timer_config, &gptimer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create timer: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Set alarm callback
    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_isr_callback,
    };
    ret = gptimer_register_event_callbacks(gptimer, &cbs, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register timer callback: %s", esp_err_to_name(ret));
        gptimer_del_timer(gptimer);
        return false;
    }
    
    // Enable timer
    ret = gptimer_enable(gptimer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable timer: %s", esp_err_to_name(ret));
        gptimer_del_timer(gptimer);
        return false;
    }
    
    // Set alarm to trigger every ISR period (1 tick = 25us)
    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = 1,  // Trigger every tick
        .flags = {
            .auto_reload_on_alarm = true,
        },
    };
    ret = gptimer_set_alarm_action(gptimer, &alarm_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set alarm action: %s", esp_err_to_name(ret));
        gptimer_disable(gptimer);
        gptimer_del_timer(gptimer);
        return false;
    }
    
    ESP_LOGI(TAG, "Stepper executor initialized (ISR frequency: %d Hz)", ISR_FREQUENCY_HZ);
    return true;
}

void stepper_executor_start(void) {
    if (gptimer) {
        gptimer_start(gptimer);
        ESP_LOGI(TAG, "Stepper executor started");
    }
}

void stepper_executor_stop(void) {
    if (gptimer) {
        gptimer_stop(gptimer);
        ESP_LOGI(TAG, "Stepper executor stopped");
    }
}

int32_t stepper_executor_get_position(uint8_t axis) {
    if (axis >= NUM_AXES) {
        return 0;
    }
    return executor_state.positions[axis];
}

void stepper_executor_set_position(uint8_t axis, int32_t position) {
    if (axis < NUM_AXES) {
        executor_state.positions[axis] = position;
    }
}

bool stepper_executor_is_busy(void) {
    return executor_state.has_segment || !segment_queue_is_empty(executor_state.queue);
}

