#ifndef _TASK_ACCEL_SENSOR_H_
#define _TASK_ACCEL_SENSOR_H_

#include "ICM_20948.h"
#include "ring_buffer.h"

#define ACCEL_AXES_NUM                  (3)   
#define ACCEL_SAMPLE_RATE_HZ            (57)
#define ACCEL_SAMPLE_DURATION_SECONDS   (1)
#define ACCEL_WINDOW_STRIDE_SECONDS     (0.5)
#define ACCEL_BUFFER_SECONDS            (1)
#define ACCEL_LSB_PER_G                 (16384.0f) /* +2g */
#define ACCEL_DELTA_THRESHOLD           (0.002f)

typedef struct {
    uint8_t bRunInfer : 1;
    uint8_t bTrigger : 1;
    uint8_t bFirstSample : 1;
    uint8_t reserve : 5;

    uint8_t ability;
    float prev_mag_g;
    ICM_20948_I2C icm20948;
    ring_buffer_t sample_buff;
} Accel_t;

extern Accel_t accel_sensor;

extern void accel_timer_polling(Accel_t *accel);
#endif
