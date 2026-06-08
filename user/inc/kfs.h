#ifndef __KFS_H__
#define __KFS_H__

#include "structure.h"
#include "dji_motor.h"
#include "dm_motor.h"

/************************KFS抓取机构电机***********************/
#define KFS_ARM_1_ID               0x01
#define KFS_ARM_1_CMD_ID           KFS_ARM_1_ID
#define KFS_ARM_1_FEEDBACK_ID      KFS_ARM_1_ID
#define KFS_ARM_1_MASTER_ID        0x10

#define KFS_ARM_2_ID               0x02
#define KFS_ARM_2_CMD_ID           KFS_ARM_2_ID
#define KFS_ARM_2_FEEDBACK_ID      KFS_ARM_2_ID
#define KFS_ARM_2_MASTER_ID        0x10

#define KFS_ARM_3_ID               0x03
#define KFS_ARM_3_CMD_ID           KFS_ARM_3_ID
#define KFS_ARM_3_FEEDBACK_ID      KFS_ARM_3_ID
#define KFS_ARM_3_MASTER_ID        0x10

#define KFS_CATCH_ID               0x04
#define KFS_CATCH_CMD_ID           KFS_CATCH_ID
#define KFS_CATCH_FEEDBACK_ID      KFS_CATCH_ID
#define KFS_CATCH_MASTER_ID        0x10

/*******************************************************/

typedef struct _Kfs_Module{
    StructureModule super_struct; 
    
                             
    
    void (*Set_Pos)(struct _Kfs_Module *chassis);
    void (*Gravity_Calc)(struct _Kfs_Module *chassis);
} Kfs_Module;


extern Kfs_Module  Kfs;
extern DM_MotorModule kfs_arm_1;  
extern DM_MotorModule kfs_arm_2;  
extern DM_MotorModule kfs_arm_3;  
extern DM_MotorModule kfs_catch;  

void kfs_mode(void);
void kfs_catch_mode(void);

extern float k_catch[];
//extern float kfs_lift_position;
//extern float kfs_lift_v;
//extern float kfs_lift_kp;
//extern float kfs_lift_kd;
//extern float kfs_lift_t;
//extern float kfs_flew_position;
//extern float kfs_flew_v;
//extern float kfs_flew_kp;
//extern float kfs_flew_kd;
//extern float kfs_flew_t;
//extern float position_flew;
//extern float position_lift;


#endif
