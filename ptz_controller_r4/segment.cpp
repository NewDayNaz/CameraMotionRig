/**
 * @file segment.cpp
 * @brief Segment queue implementation (ring buffer)
 */

#include "segment.h"

void segment_queue_init(segment_queue_t* queue) {
    queue->head = 0;
    queue->tail = 0;
    // Clear all segments
    for (uint32_t i = 0; i < SEGMENT_QUEUE_SIZE; i++) {
        for (uint8_t axis = 0; axis < NUM_AXES; axis++) {
            queue->segments[i].steps[axis] = 0;
        }
        queue->segments[i].duration_us = SEGMENT_DURATION_US;
    }
}

bool segment_queue_is_empty(const segment_queue_t* queue) {
    return (queue->head == queue->tail);
}

bool segment_queue_is_full(const segment_queue_t* queue) {
    return (((queue->head + 1) & (SEGMENT_QUEUE_SIZE - 1)) == queue->tail);
}

bool segment_queue_push(segment_queue_t* queue, const motion_segment_t* segment) {
    uint32_t next_head = (queue->head + 1) & (SEGMENT_QUEUE_SIZE - 1);
    
    if (next_head == queue->tail) {
        return false;  // Queue is full
    }
    
    // Copy segment data
    for (uint8_t axis = 0; axis < NUM_AXES; axis++) {
        queue->segments[queue->head].steps[axis] = segment->steps[axis];
    }
    queue->segments[queue->head].duration_us = segment->duration_us;
    
    // Update head (atomic write on 32-bit systems)
    queue->head = next_head;
    
    return true;
}

bool segment_queue_pop(segment_queue_t* queue, motion_segment_t* segment) {
    if (queue->head == queue->tail) {
        return false;  // Queue is empty
    }
    
    // Copy segment data
    for (uint8_t axis = 0; axis < NUM_AXES; axis++) {
        segment->steps[axis] = queue->segments[queue->tail].steps[axis];
    }
    segment->duration_us = queue->segments[queue->tail].duration_us;
    
    // Update tail (atomic write on 32-bit systems)
    queue->tail = (queue->tail + 1) & (SEGMENT_QUEUE_SIZE - 1);
    
    return true;
}

uint32_t segment_queue_free_slots(const segment_queue_t* queue) {
    uint32_t head = queue->head;
    uint32_t tail = queue->tail;
    
    if (head >= tail) {
        return SEGMENT_QUEUE_SIZE - (head - tail) - 1;
    } else {
        return tail - head - 1;
    }
}

void segment_queue_clear(segment_queue_t* queue) {
    queue->head = 0;
    queue->tail = 0;
}

