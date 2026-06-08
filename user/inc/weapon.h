#ifndef __WEAPON_H__
#define __WEAPON_H__

#include "structure.h"
#include "dji_motor.h"
#include "dm_motor.h"

/************************R2Ì§Éýµç»ú***********************/
#define WEAPON_COLLECT_MOTOR_ID           0x05
#define WEAPON_COLLECT_MOTOR_CMD_ID       WEAPON_COLLECT_MOTOR_ID
#define WEAPON_COLLECT_MOTOR_FEEDBACK_ID  WEAPON_COLLECT_MOTOR_ID
#define WEAPON_COLLECT_MOTOR_MASTER_ID    0x10

#define WEAPON_JOINT_MOTOR_ID           0x06                                                                                                                                                                        
#define WEAPON_JOINT_MOTOR_CMD_ID       0x1FF
#define WEAPON_JOINT_MOTOR_FEEDBACK_ID  0x1FF + WEAPON_JOINT_MOTOR_ID

extern float weapon_joint_motor_pid_param[PID_PARAMETER_NUM];
/*******************************************************/

typedef struct _Weapon_Module{
    StructureModule super_struct; 
    
                             
} Weapon_Module;


//extern uint16_t CH8_last;
//extern uint16_t CH8_mid;
//extern uint16_t CH8_recent;
//extern uint8_t CH8_trigger_flag0;
//extern uint8_t CH8_trigger_flag1;

extern float weapon_joint_input;
extern float hold_weapon[];
//extern float weapon_collect_input;

extern Weapon_Module Weapon;
extern DJI_MotorModule weapon_joint_motor;
extern DM_MotorModule weapon_collect_motor;

void weapon_mode (void);
void weapon_hold(void);


#endif
