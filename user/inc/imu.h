/**
 * @file    imu.h
 * @brief   HI14 (RS-485 Modbus) IMU 驱动 + 上电初始化序列
 *
 * 说明：
 * - 使用 USART2 (RS485) 与 HI14 通讯
 * - 支持上电初始化(航向heading=0) + 复位 + 约2s静止 + PR归零
 * - TIM12 周期溢出回调中调用 IMU_RequestAndStartRx() + IMU_ParseFrameIfReady()
 */
#ifndef __IMU_H__
#define __IMU_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------- 数据结构 ---------------- */

/** IMU 传感器数据结构（由 IMU_ParseFrameIfReady() 解析 Modbus 帧后写入） */
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

/** 传感器融合数据（当前仅包含 IMU，后续可扩展里程计等） */
typedef struct {
    sensor_imu_t imu;
} sensor_task_data_t;

extern volatile sensor_task_data_t g_sensor_task_data;

/* ---------------- 函数声明 ---------------- */

/**
 * @brief HI14 上电初始化序列（开机前调用）
 * @return 0 成功，非0 表示step1(写0x0006)未通过回读确认或通讯异常
 */
int32_t IMU_HI14_PowerOnInit(void);

/** @brief 发送请求帧并启动接收（原 Sensor_Task 内部逻辑） */
void IMU_RequestAndStartRx(void);

/** @brief 检查接收完成并解析，结果写入 g_sensor_task_data.imu */
void IMU_ParseFrameIfReady(void);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_H__ */
