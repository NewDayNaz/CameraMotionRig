/**
 * @file segment.h
 * @brief Motion segment definition and ring buffer queue
 * 
 * Segments are fixed-duration motion blocks (5-10ms) that contain
 * step counts for each axis. The executor consumes these segments
 * deterministically in an ISR context.
 */

#ifndef SEGMENT_H
#define SEGMENT_H

#include <stdint.h>
#include <stdbool.h>
#include "board.h"

// Segment duration in microseconds (4-8ms typical)
// Shorter duration allows better fractional step accumulation for smooth low-speed motion
#define SEGMENT_DURATION_US 4000  // 4ms default (reduced from 8ms for smoother low-speed control)

// Ring buffer size (must be power of 2 for efficient masking)
#define SEGMENT_QUEUE_SIZE 32

/**
 * @brief Motion segment structure
 * 
 * Contains step counts for each axis over a fixed duration.
 * Steps are signed integers (positive = forward, negative = reverse).
 */
typedef struct {
    int32_t steps[NUM_AXES];      // Step counts per axis (can be negative)
    uint32_t duration_us;          // Segment duration in microseconds
} motion_segment_t;

/**
 * @brief Segment queue (ring buffer)
 */
typedef struct {
    motion_segment_t segments[SEGMENT_QUEUE_SIZE];
    volatile uint32_t head;        // Write position (producer)
    volatile uint32_t tail;        // Read position (consumer, ISR)
} segment_queue_t;

/**
 * @brief Initialize the segment queue
 */
void segment_queue_init(segment_queue_t* queue);

/**
 * @brief Check if queue is empty (ISR-safe)
 */
bool segment_queue_is_empty(const segment_queue_t* queue);

/**
 * @brief Check if queue is full (ISR-safe)
 */
bool segment_queue_is_full(const segment_queue_t* queue);

/**
 * @brief Add segment to queue (called from task context)
 * @return true if successful, false if queue is full
 */
bool segment_queue_push(segment_queue_t* queue, const motion_segment_t* segment);

/**
 * @brief Get next segment from queue (called from ISR)
 * @param segment Pointer to fill with segment data
 * @return true if segment was retrieved, false if queue is empty
 */
bool segment_queue_pop(segment_queue_t* queue, motion_segment_t* segment);

/**
 * @brief Get number of free slots in queue
 */
uint32_t segment_queue_free_slots(const segment_queue_t* queue);

/**
 * @brief Clear all segments from queue
 */
void segment_queue_clear(segment_queue_t* queue);

#endif // SEGMENT_H

