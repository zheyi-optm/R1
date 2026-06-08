#ifndef __LIFT_H__
#define __LIFT_H__

#include "structure.h"
#include "dji_motor.h"
#include "dm_motor.h"

/************************R2Ì§Éýµç»ú***********************/
#define R2_LIFT_MOTOR_LEFT_ID           0x05
#define R2_LIFT_MOTOR_LEFT_CMD_ID       R2_LIFT_MOTOR_LEFT_ID
#define R2_LIFT_MOTOR_LEFT_FEEDBACK_ID  R2_LIFT_MOTOR_LEFT_ID
#define R2_LIFT_MOTOR_LEFT_MASTER_ID    0x10

#define R2_LIFT_MOTOR_RIGHT_ID          0x06
#define R2_LIFT_MOTOR_RIGHT_CMD_ID      R2_LIFT_MOTOR_RIGHT_ID
#define R2_LIFT_MOTOR_RIGHT_FEEDBACK_ID R2_LIFT_MOTOR_RIGHT_ID
#define R2_LIFT_MOTOR_RIGHT_MASTER_ID   0x10

//#define BALANCE_MOTOR_LEFT_ID           0x01
//#define BALANCE_MOTOR_LEFT_CMD_ID       0x200
//#define BALANCE_MOTOR_LEFT_FEEDBACK_ID  0x200 + BALANCE_MOTOR_LEFT_ID

//#define BALANCE_MOTOR_RIGHT_ID           0x02
//#define BALANCE_MOTOR_RIGHT_CMD_ID       0x200
//#define BALANCE_MOTOR_RIGHT_FEEDBACK_ID  0x200 + BALANCE_MOTOR_RIGHT_ID

//extern float balance_motor_left_pid_param[PID_PARAMETER_NUM] ;
//extern float balance_motor_right_pid_param[PID_PARAMETER_NUM];
/*******************************************************/

typedef struct _Lift_Module{
    StructureModule super_struct; 
    
                             
} Lift_Module;


extern Lift_Module Lift;
//extern DJI_MotorModule balance_motor_left;
//extern DJI_MotorModule balance_motor_right;
extern DM_MotorModule R2_lift_motor_left;
extern DM_MotorModule R2_lift_motor_right;

//extern float lift_balance_input;

//extern float r2_lift_l_position;
//extern float r2_lift_l_v;
//extern float r2_lift_l_kp;
//extern float r2_lift_l_kd;
//extern float r2_lift_l_t;
//extern float r2_lift_r_position;
//extern float r2_lift_r_v;
//extern float r2_lift_r_kp;
//extern float r2_lift_r_kd;
//extern float r2_lift_r_t;

//extern float dm4310_current_position_left; 
//extern float dm4310_pre_position_left;
//extern float dm4310_current_position_right; 
//extern float dm4310_pre_position_right;
//extern int flag_0;
//extern float Initpos_left;
//extern float Initpos_right;

void lift_mode(void);

extern float lift_left[];
extern float lift_right[];
//void Get_Initpos(void);
//void lift_stop(uint8_t flag);


#endif
