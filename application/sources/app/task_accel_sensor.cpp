#include "ak.h"
#include "message.h"
#include "timer.h"
#include "task_list.h"
#include "app.h"
#include "app_dbg.h"

#include "task_accel_sensor.h"
#include "utils.h"

#include "ICM_20948.h"
#include "Wire.h"
#include "sys_ctrl.h"

#define EKF_N 3
#define EKF_M 3

#include "tinyekf.h"

/* acc: x, y, z (float * 3) */ 
static constexpr size_t ACCEL_DATA_SIZE = (sizeof(float) * ACCEL_AXES_NUM);
static constexpr size_t ACCEL_SAMPLE_BUFFER_SIZE = (ACCEL_BUFFER_SECONDS * ACCEL_SAMPLE_RATE_HZ * ACCEL_DATA_SIZE);

static uint8_t buffer[ACCEL_SAMPLE_BUFFER_SIZE];
Accel_t accel_sensor;

static const float EPS = 1e-4;

static const float Q[EKF_N*EKF_N] = {
    EPS, 0,   0,
    0,   EPS, 0,
    0,   0,   EPS
};

static const float R[EKF_M*EKF_M] = {
    0.01f, 0,    0,
    0,     0.01f, 0,
    0,     0,    0.01f
};

// Process model Jacobian: constant acceleration (identity)
static const float F[EKF_N*EKF_N] = {
    1, 0, 0,
    0, 1, 0,
    0, 0, 1
};

// Observation Jacobian: state directly observable
static const float H[EKF_M*EKF_N] = {
    1, 0, 0,
    0, 1, 0,
    0, 0, 1
};

static ekf_t _ekf;

void task_accel(ak_msg_t *msg)
{
    switch (msg->sig)
    {
    case AC_ACCEL_INIT:
    {
        accel_sensor.ability = AK_DISABLE;

        Wire.begin();
        Wire.setClock(50000);
        accel_sensor.icm20948.begin(Wire, true);
        APP_DBG_SIG("AC_ACCEL_INIT\n");
        APP_DBG("Sensor init: %s\n", accel_sensor.icm20948.statusString());
        if (accel_sensor.icm20948.status != ICM_20948_Stat_Ok)
        {
            timer_set(AC_TASK_ACCEL_ID, AC_ACCEL_INIT, 200, TIMER_ONE_SHOT);
            break;
        }
        task_post_pure_msg(AC_TASK_ACCEL_ID, AC_ACCEL_SET_CONFIG);
        APP_DBG("Init ring buffer: %d bytes\n", sizeof(buffer));
        ring_buffer_init(&accel_sensor.sample_buff, buffer, sizeof(buffer), ACCEL_DATA_SIZE);

        float pdiag[EKF_N] = {1.0f, 1.0f, 1.0f};
        ekf_initialize(&_ekf, pdiag);
        APP_DBG("EKF initialized: N=%d, M=%d\n", EKF_N, EKF_M);
    }
    break;

    case AC_ACCEL_SET_CONFIG:
    {
        bool success = true;
        APP_DBG_SIG("AC_ACCEL_SET_CONFIG\n");
        accel_sensor.ability = AK_DISABLE;
        if (accel_sensor.icm20948.initializeDMP() != ICM_20948_Stat_Ok) {
            APP_DBG("Init DMP failed, status: %s!\n", accel_sensor.icm20948.statusString());
            goto retry;
        }
        else
        {
            APP_DBG("Init DMP success !\n");
        }
        // DMP sensor options are defined in ICM_20948_DMP.h
        //    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
        //    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
        //    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
        //    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
        //    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
        //    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
        //    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
        //    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
        //    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
        //    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
        //    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
        //    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
        //    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
        //    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
        //    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)
        success &= (accel_sensor.icm20948.enableDMPSensor(INV_ICM20948_SENSOR_ACCELEROMETER) == ICM_20948_Stat_Ok);
        // Set to the maximum
        success &= (accel_sensor.icm20948.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok);
        success &= (accel_sensor.icm20948.enableFIFO() == ICM_20948_Stat_Ok);
        success &= (accel_sensor.icm20948.enableDMP() == ICM_20948_Stat_Ok);
        success &= (accel_sensor.icm20948.resetDMP() == ICM_20948_Stat_Ok);
        success &= (accel_sensor.icm20948.resetFIFO() == ICM_20948_Stat_Ok);
        if (!success)
        {
            APP_DBG("Config DMP failed !\n");
            goto retry;
        }
        APP_DBG("Config DMP success !\n");
        accel_sensor.ability = AK_ENABLE;
        break;

        retry:
            timer_set(AC_TASK_ACCEL_ID, AC_ACCEL_SET_CONFIG, 200, TIMER_ONE_SHOT);
    }
    break;

    case AC_ACCEL_GET_CONFIG:
    {
        APP_DBG_SIG("AC_ACCEL_GET_CONFIG\n");
    }
    break;

    default:
        break;
    }
}

void accel_timer_polling(Accel_t accel)
{
    struct icm_data_internal_t {
        float acc_x;
        float acc_y;
        float acc_z;
    };
    if (accel.ability == AK_ENABLE)
    {
        icm_20948_DMP_data_t data;
        accel_sensor.icm20948.readDMPdataFromFIFO(&data);

        float x, y, z;
        x = y = z = 0.0f;
        if ((accel_sensor.icm20948.status == ICM_20948_Stat_Ok) || (accel_sensor.icm20948.status == ICM_20948_Stat_FIFOMoreDataAvail))
        {
            if ((data.header & DMP_header_bitmap_Accel) > 0)
            {
                x = (float)data.Raw_Accel.Data.X;
                y = (float)data.Raw_Accel.Data.Y;
                z = (float)data.Raw_Accel.Data.Z;

                // EKF predict: constant acceleration model
                float fx[EKF_N] = { _ekf.x[0], _ekf.x[1], _ekf.x[2] };
                ekf_predict(&_ekf, fx, F, Q);

                // EKF update: fuse raw measurement
                float z_obs[EKF_M] = { x, y, z };
                float hx[EKF_M] = { _ekf.x[0], _ekf.x[1], _ekf.x[2] };
                ekf_update(&_ekf, z_obs, hx, H, R);

                float filt_x = _ekf.x[0];
                float filt_y = _ekf.x[1];
                float filt_z = _ekf.x[2];

                #if 0
                xfprintf((void (*)(int))sys_ctrl_shell_put_char, "%.1f,%.1f,%.1f\n", filt_x, filt_y, filt_z);
                #else
                struct icm_data_internal_t icm_data = {
                    .acc_x = filt_x,
                    .acc_y = filt_y,
                    .acc_z = filt_z
                };
                ring_buffer_put(&accel_sensor.sample_buff, &icm_data);
                #endif
            }

            
        }
        else
        {
            // APP_DBG("Status: %s\n", accel_sensor.icm20948.statusString());
            if (accel_sensor.icm20948.status == ICM_20948_Stat_FIFOIncompleteData)
            {
                task_post_pure_msg(AC_TASK_ACCEL_ID, AC_ACCEL_SET_CONFIG);
            }
        }
    }
}
