#ifndef _ARM_H
#define _ARM_H

#include <math.h>
#include <string.h>
#include "stm32h7xx_hal.h"
#include "remote_control.h"
#include "motor.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "cmsis_os.h"
#include "motor_control.h"
#include "timer.h"

#define L_1      0.545f  // 大臂长度
#define L_2      0.440f  // 小臂长度
#define L_3      0.150f  // 吸盘臂长度
#define L_4      0.215f  // 夹爪臂长度
#define L_hold   0.300f  // 夹爪抓取枪杆时距枪杆尾部的长度
#define H0_min   0.0f    // 取放KFS的最低高度
#define H0_max   0.900f  // 取放KFS的最高度
#define H1_min   0.0f    // 捡KFS的最低高度
#define H1_max   0.400f  // 捡KFS的最高度
#define H        0.0505f   // 腰部到大臂旋转中心高度
#define S_min    0.0f    
#define S_max    0.300f    
#define S_catch  0.5f   // 夹取枪杆时机械臂底座到枪杆根部的距离
#define PI       3.1415926f
#define a_weapon 15/180*PI // 武器倾斜角

#define gravity  9.8f
#define L_c1     0.480f
#define L_c2     0.220f
#define L_c3     0.054f
#define L_c4     0.090f
#define m_1      0.468f
#define m_2      0.154f
#define m_3      0.449f
#define m_4      0.416f

typedef struct 
{
    int mode;    //0--取、放kfs  1--捡kfs  2--夹weapon
    float h;     //垂直高度
    float s;     //水平距离
}Arm_Info_TypeDef;

typedef struct 
{
    float theta_1;  // 大臂关节角度
    float theta_2;  // 大臂角度
    float theta_3;  // 小臂角度
}ArmAnglesTypeDef;

 typedef struct 
{
    float Torque_1;  // 电机1力矩
    float Torque_2;  // 电机2力矩
    float Torque_3;  // 电机3力矩
}ArmTorqueTypeDef;

extern Arm_Info_TypeDef arm;
extern ArmAnglesTypeDef angles;
extern ArmTorqueTypeDef Torque;

extern ArmAnglesTypeDef Arm_Inverse_Solution(Arm_Info_TypeDef *arm);
extern ArmTorqueTypeDef Torque_Comp_remote(ArmAnglesTypeDef *angle,Arm_Info_TypeDef *arm);
extern ArmTorqueTypeDef Torque_Comp_global(DM_MotorModule *arm_1,DM_MotorModule *arm_2,DM_MotorModule *arm_3);
extern HAL_StatusTypeDef Arm_task(DM_MotorModule *arm_1,DM_MotorModule *arm_2,DM_MotorModule *arm_3);

#endif
