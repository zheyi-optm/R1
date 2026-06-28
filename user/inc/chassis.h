#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "global.h"
#include "structure.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "remote_control.h"
#include "math.h"
#include "timer.h"

/************************底盘电机***********************/
//左前6220
#define CHASSIS_ANGLE1_ID               0x01
#define CHASSIS_ANGLE1_CMD_ID           CHASSIS_ANGLE1_ID
#define CHASSIS_ANGLE1_FEEDBACK_ID      CHASSIS_ANGLE1_ID
#define CHASSIS_ANGLE1_MASTER_ID        0x10

#define CHASSIS_ANGLE2_ID               0x02
#define CHASSIS_ANGLE2_CMD_ID           CHASSIS_ANGLE2_ID
#define CHASSIS_ANGLE2_FEEDBACK_ID      CHASSIS_ANGLE2_ID
#define CHASSIS_ANGLE2_MASTER_ID        0x10

#define CHASSIS_ANGLE3_ID               0x03
#define CHASSIS_ANGLE3_CMD_ID           CHASSIS_ANGLE3_ID
#define CHASSIS_ANGLE3_FEEDBACK_ID      CHASSIS_ANGLE3_ID
#define CHASSIS_ANGLE3_MASTER_ID        0x10

#define CHASSIS_ANGLE4_ID               0x04
#define CHASSIS_ANGLE4_CMD_ID           CHASSIS_ANGLE4_ID
#define CHASSIS_ANGLE4_FEEDBACK_ID      CHASSIS_ANGLE4_ID
#define CHASSIS_ANGLE4_MASTER_ID        0x10


//左前3508
#define CHASSIS_MOTOR1_ID          0x01
#define CHASSIS_MOTOR1_CMD_ID      0x200
#define CHASSIS_MOTOR1_FEEDBACK_ID 0x200 + CHASSIS_MOTOR1_ID

//左后
#define CHASSIS_MOTOR2_ID          0x02
#define CHASSIS_MOTOR2_CMD_ID      0x200
#define CHASSIS_MOTOR2_FEEDBACK_ID 0x200 + CHASSIS_MOTOR2_ID

//右后
#define CHASSIS_MOTOR3_ID          0x03
#define CHASSIS_MOTOR3_CMD_ID      0x200
#define CHASSIS_MOTOR3_FEEDBACK_ID 0x200 + CHASSIS_MOTOR3_ID

//右前
#define CHASSIS_MOTOR4_ID          0x04
#define CHASSIS_MOTOR4_CMD_ID      0x200
#define CHASSIS_MOTOR4_FEEDBACK_ID 0x200 + CHASSIS_MOTOR4_ID



extern float chassis_motor1_pid_param[PID_PARAMETER_NUM];   
extern float chassis_motor2_pid_param[PID_PARAMETER_NUM];
extern float chassis_motor3_pid_param[PID_PARAMETER_NUM];
extern float chassis_motor4_pid_param[PID_PARAMETER_NUM];
/*******************************************************/
//四个6220初始角
#define  ORIGIN_ANGLE1  0.0f   
#define  ORIGIN_ANGLE2  0.0f
#define  ORIGIN_ANGLE3  0.0f
#define  ORIGIN_ANGLE4  0.0f

#define R   0.55437f
#define RX  0.392f
#define RY  0.392f
#define S   0.058f

#define SQ2 (sqrtf(2.0f)/2.0f)  
#define PI  3.1415926f



typedef struct{
    float Vx_in;
    float Vy_in;
    float Vw_in;
    float Accel;
    
    float V_out[4];
}Chassis_Param;

typedef struct _Chassis_Module{
    StructureModule super_struct; 
    
    Chassis_Param param;                             
    
    void (*Chassis_Calc)(struct _Chassis_Module *chassis);
    void (*Chassis_Stop)(struct _Chassis_Module *chassis);
	  void (*chassis_mission)(struct _Chassis_Module *chassis);
} Chassis_Module;

extern Chassis_Module Chassis;
extern DM_MotorModule chassis_angle1;  // 
extern DM_MotorModule chassis_angle2;  // 
extern DM_MotorModule chassis_angle3;  // 
extern DM_MotorModule chassis_angle4;  // 

extern DJI_MotorModule chassis_motor1;  // 
extern DJI_MotorModule chassis_motor2;  // 
extern DJI_MotorModule chassis_motor3;  // 
extern DJI_MotorModule chassis_motor4;  // 

extern float SP_ACCEL;
extern float SP_X;
extern float SP_Y;
extern float SP_W;

extern float angle_now[4];
extern float auto_Vout[4];

void Chassis_Calc(Chassis_Module *chassis);
void chassis_mission(Chassis_Module *chassis);
void auto_chassis(void);
//void Chassis_Stop(Chassis_Module *chassis);

#endif
