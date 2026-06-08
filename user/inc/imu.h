/**
 * @file    imu.h
 * @brief   HI14 (RS-485 Modbus) IMU driver + power-on init sequence
 *
 * 说明：
 * - 使用 USART2 (RS485) 与 HI14 通讯
 * - 支持上电初始化(航向heading=0) + 复位 + 约2s静止 + PR归零
 * - Sensor_Task 周期调用，发送一帧 + 解析更新 g_sensor_task_data.imu
 */
#ifndef __IMU_H__
#define __IMU_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

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
