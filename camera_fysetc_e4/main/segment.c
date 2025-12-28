/**
 * @file segment.c
 * @brief Segment queue implementation (ring buffer)
 */

#include "segment.h"
#include <string.h>

void segment_queue_init(segment_queue_t* queue) {
    memset(queue, 0, sizeof(segment_queue_t));
    queue->head = 0;
    queue->tail = 0;
}

bool segment_queue_is_empty(const segment_queue_t* queue) {
    return queue->head == queue->tail;
}

bool segment_queue_is_full(const segment_queue_t* queue) {
    return ((queue->head + 1) & (SEGMENT_QUEUE_SIZE - 1)) == queue->tail;
}

bool segment_queue_push(segment_queue_t* queue, const motion_segment_t* segment) {
    if (segment_queue_is_full(queue)) {
        return false;
    }

    uint32_t next_head = (queue->head + 1) & (SEGMENT_QUEUE_SIZE - 1);
    memcpy(&queue->segments[queue->head], segment, sizeof(motion_segment_t));
    queue->head = next_head;
    return true;
}

bool segment_queue_pop(segment_queue_t* queue, motion_segment_t* segment) {
    if (segment_queue_is_empty(queue)) {
        return false;
    }

    memcpy(segment, &queue->segments[queue->tail], sizeof(motion_segment_t));
    queue->tail = (queue->tail + 1) & (SEGMENT_QUEUE_SIZE - 1);
    return true;
}

uint32_t segment_queue_free_slots(const segment_queue_t* queue) {
    uint32_t used = (queue->head - queue->tail) & (SEGMENT_QUEUE_SIZE - 1);
    return (SEGMENT_QUEUE_SIZE - 1) - used;
}

void segment_queue_clear(segment_queue_t* queue) {
    queue->head = 0;
    queue->tail = 0;
}

