#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include "app_dbg.h"
#include "accel_ring_buffer.h"

void accel_ring_buffer_init(accel_ring_buffer_t *rb)
{
    rb->head = 0;
    rb->tail = 0;
}

bool accel_ring_buffer_push(accel_ring_buffer_t *rb, int8_t x, int8_t y, int8_t z)
{
    uint32_t next = (rb->head + 1) % ACCEL_BUFFER_SIZE;
    if (next == rb->tail)
    {
        return false;
    }
    rb->buf[rb->head].x = x;
    rb->buf[rb->head].y = y;
    rb->buf[rb->head].z = z;
    rb->head = next;
    return true;
}

bool accel_ring_buffer_full(accel_ring_buffer_t *rb)
{
    return ((rb->head + 1) % ACCEL_BUFFER_SIZE) == rb->tail;
}

uint32_t accel_ring_buffer_count(accel_ring_buffer_t *rb)
{
    if (rb->head >= rb->tail)
    {
        return rb->head - rb->tail;
    }
    return ACCEL_BUFFER_SIZE - (rb->tail - rb->head);
}

void accel_ring_buffer_reset(accel_ring_buffer_t *rb)
{
    rb->head = 0;
    rb->tail = 0;
}

void accel_ring_bufffer_get(accel_ring_buffer_t *rb, accel_sample_t *accel_buff) {
    uint32_t count = accel_ring_buffer_count(rb);
    uint32_t idx = rb->tail;

    for (uint32_t i = 0; i < count; i++) {
        accel_buff[i].x = rb->buf[idx].x;
        accel_buff[i].y = rb->buf[idx].y;
        accel_buff[i].z = rb->buf[idx].z;

        idx = (idx + 1) % ACCEL_BUFFER_SIZE;
    }
}

void accel_ring_bufffer_get_x(accel_ring_buffer_t *rb, int8_t *x_buff, int len) {
    uint32_t count = accel_ring_buffer_count(rb);
    uint32_t idx = rb->tail;

    for (uint32_t i = 0; i < count && i < len; i++) {
        x_buff[i] = rb->buf[idx].x;
        idx = (idx + 1) % ACCEL_BUFFER_SIZE;
    }
}

void accel_ring_bufffer_get_y(accel_ring_buffer_t *rb, int8_t *y_buff, int len) {
    uint32_t count = accel_ring_buffer_count(rb);
    uint32_t idx = rb->tail;

    for (uint32_t i = 0; i < count && i < len; i++) {
        y_buff[i] = rb->buf[idx].y;
        idx = (idx + 1) % ACCEL_BUFFER_SIZE;
    }
}

void accel_ring_bufffer_get_z(accel_ring_buffer_t *rb, int8_t *z_buff, int len) {
    uint32_t count = accel_ring_buffer_count(rb);
    uint32_t idx = rb->tail;

    for (uint32_t i = 0; i < count && i < len; i++) {
        z_buff[i] = rb->buf[idx].z;

        idx = (idx + 1) % ACCEL_BUFFER_SIZE;
    }
}

