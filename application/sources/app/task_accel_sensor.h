#ifndef _TASK_ACCEL_SENSOR_H_
#define _TASK_ACCEL_SENSOR_H_

#include "ICM_20948.h"
#include "ring_buffer.h"

#define ACCEL_AXES_NUM                  (3)   
#define ACCEL_SAMPLE_RATE_HZ            (57)
#define ACCEL_SAMPLE_DURATION_SECONDS   (1)
#define ACCEL_WINDOW_STRIDE_SECONDS     (0.5)
#define ACCEL_BUFFER_SECONDS            (2)

typedef struct {
    uint8_t ability;
    ICM_20948_I2C icm20948;
    ring_buffer_t sample_buff;
} Accel_t;

extern Accel_t accel_sensor;

extern void accel_timer_polling(Accel_t accel);
#endif
