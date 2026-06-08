#ifndef __SENSOR_TASK_H__
#define __SENSOR_TASK_H__

#include <stdint.h>

/** IMU 数据结构（由 Sensor_Task 调用 Modbus 帧解析写入） */
typedef struct {
    float acc_x_g;     /* g */
    float acc_y_g;
    float acc_z_g;
    float gyr_x_dps;   /* deg/s */
    float gyr_y_dps;
    float gyr_z_dps;
    float mag_x_ut;    /* uT */
    float mag_y_ut;
    float mag_z_ut;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
} sensor_imu_t;

/**
 * 传感器融合数据（IMU + 里程计，由 rc_get_latest_odom 刷新）
 */
typedef struct {
    sensor_imu_t imu;
} sensor_task_data_t;

extern volatile sensor_task_data_t g_sensor_task_data;

#endif /* __SENSOR_TASK_H__ */
