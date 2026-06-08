#ifndef __SENSOR_TASK_H__
#define __SENSOR_TASK_H__

#include <stdint.h>

/**
 * @brief  IMU 传感器数据结构
 * @note   由 IMU_ParseFrameIfReady() 解析 Modbus 帧后写入
 */
typedef struct {
    float acc_x_g;     /**< 加速度 X 轴 (g) */
    float acc_y_g;     /**< 加速度 Y 轴 (g) */
    float acc_z_g;     /**< 加速度 Z 轴 (g) */
    float gyr_x_dps;   /**< 角速度 X 轴 (deg/s) */
    float gyr_y_dps;   /**< 角速度 Y 轴 (deg/s) */
    float gyr_z_dps;   /**< 角速度 Z 轴 (deg/s) */
    float mag_x_ut;    /**< 磁场强度 X 轴 (uT) */
    float mag_y_ut;    /**< 磁场强度 Y 轴 (uT) */
    float mag_z_ut;    /**< 磁场强度 Z 轴 (uT) */
    float roll_deg;    /**< 横滚角 (deg) */
    float pitch_deg;   /**< 俯仰角 (deg) */
    float yaw_deg;     /**< 偏航角 (deg) */
} sensor_imu_t;

/**
 * @brief 传感器融合数据
 * @note  当前仅包含 IMU 数据，后续可扩展里程计等
 */
typedef struct {
    sensor_imu_t imu;
} sensor_task_data_t;

extern volatile sensor_task_data_t g_sensor_task_data;

#endif /* __SENSOR_TASK_H__ */
