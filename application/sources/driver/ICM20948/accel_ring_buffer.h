#ifndef ACCEL_RING_BUFFER_H
#define ACCEL_RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ACCEL_SAMPLE_RATE       57
#define ACCEL_BUFFER_SIZE       (ACCEL_SAMPLE_RATE + 1)
#define ACCEL_TARGET_COUNT      ACCEL_SAMPLE_RATE
#define ACCEL_FEATURES_LEN  8

typedef struct accel_sample_t {
    int8_t x;
    int8_t y;
    int8_t z;
} accel_sample_t;

typedef struct accel_ring_buffer_t {
    accel_sample_t buf[ACCEL_BUFFER_SIZE];
    volatile uint32_t head;
    volatile uint32_t tail;
} accel_ring_buffer_t;

void accel_ring_buffer_init(accel_ring_buffer_t *rb);
bool accel_ring_buffer_push(accel_ring_buffer_t *rb, int8_t x, int8_t y, int8_t z);
bool accel_ring_buffer_full(accel_ring_buffer_t *rb);
uint32_t accel_ring_buffer_count(accel_ring_buffer_t *rb);
void accel_ring_buffer_reset(accel_ring_buffer_t *rb);

int32_t accel_predict_impact(accel_ring_buffer_t *rb);

#ifdef __cplusplus
}
#endif

#endif
