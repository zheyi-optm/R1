#include "register.h"

#include "global.h"
#include "motor.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "chassis.h"
#include "kfs.h"
#include "lift.h"
#include "weapon.h"
#include "motor_control.h"

void Chassis_Init(void) 
{
    StructureModule_Create(&Chassis.super_struct, chassis);
    Chassis.super_struct.base.Init(&Chassis.super_struct.base);
	
	  DMmotor_Create (&chassis_angle1, CHASSIS_ANGLE1_CMD_ID, CHASSIS_ANGLE1_MASTER_ID,&hfdcan1, DM_6220,MIT);
    DMmotor_Create (&chassis_angle2, CHASSIS_ANGLE2_CMD_ID, CHASSIS_ANGLE2_MASTER_ID,&hfdcan1, DM_6220,MIT);
    DMmotor_Create (&chassis_angle3, CHASSIS_ANGLE3_CMD_ID, CHASSIS_ANGLE3_MASTER_ID,&hfdcan1, DM_6220,MIT);
	  DMmotor_Create (&chassis_angle4, CHASSIS_ANGLE4_CMD_ID, CHASSIS_ANGLE4_MASTER_ID,&hfdcan1, DM_6220,MIT);
    
    DJImotor_Create(&chassis_motor1, CHASSIS_MOTOR1_CMD_ID, CHASSIS_MOTOR1_FEEDBACK_ID, &hfdcan2, DJI_3508, SPEED, PID_POSITION, chassis_motor1_pid_param);
    DJImotor_Create(&chassis_motor2, CHASSIS_MOTOR2_CMD_ID, CHASSIS_MOTOR2_FEEDBACK_ID, &hfdcan2, DJI_3508, SPEED, PID_POSITION, chassis_motor2_pid_param);
    DJImotor_Create(&chassis_motor3, CHASSIS_MOTOR3_CMD_ID, CHASSIS_MOTOR3_FEEDBACK_ID, &hfdcan2, DJI_3508, SPEED, PID_POSITION, chassis_motor3_pid_param);
    DJImotor_Create(&chassis_motor4, CHASSIS_MOTOR4_CMD_ID, CHASSIS_MOTOR4_FEEDBACK_ID, &hfdcan2, DJI_3508, SPEED, PID_POSITION, chassis_motor4_pid_param);
	
  	Chassis.super_struct.AddMotor(&Chassis.super_struct,&chassis_angle1.super_motor);
    Chassis.super_struct.AddMotor(&Chassis.super_struct,&chassis_angle2.super_motor);
  	Chassis.super_struct.AddMotor(&Chassis.super_struct,&chassis_angle3.super_motor);
    Chassis.super_struct.AddMotor(&Chassis.super_struct,&chassis_angle4.super_motor);
    
    Chassis.super_struct.AddMotor(&Chassis.super_struct, &chassis_motor1.super_motor);
    Chassis.super_struct.AddMotor(&Chassis.super_struct, &chassis_motor2.super_motor);
    Chassis.super_struct.AddMotor(&Chassis.super_struct, &chassis_motor3.super_motor);
    Chassis.super_struct.AddMotor(&Chassis.super_struct, &chassis_motor4.super_motor); 
		
	  chassis_angle1.send_cmd(&chassis_angle1, Motor_Enable);
		HAL_Delay(10);
    chassis_angle2.send_cmd(&chassis_angle2, Motor_Enable);
		HAL_Delay(10);
  	chassis_angle3.send_cmd(&chassis_angle3, Motor_Enable);
		HAL_Delay(10);
    chassis_angle4.send_cmd(&chassis_angle4, Motor_Enable);
		
//		chassis_angle1.set_mit_data(&chassis_angle1,0.f,0.f,0.f,0.f,0.f);
//  	chassis_angle2.set_mit_data(&chassis_angle2,0.f,0.f,0.f,0.f,0.f);		
//	  chassis_angle3.set_mit_data(&chassis_angle3,0.f,0.f,0.f,0.f,0.f);
//  	chassis_angle4.set_mit_data(&chassis_angle4,0.f,0.f,0.f,0.f,0.f);
    
    Chassis.Chassis_Calc = Chassis_Calc;
		Chassis.chassis_mission = chassis_mission;
//    Chassis.Chassis_Stop = Chassis_Stop;
}

void Kfs_Init(void)
{
    StructureModule_Create(&Kfs.super_struct, kfs);
    Kfs.super_struct.base.Init(&Kfs.super_struct.base);
    
    DMmotor_Create (&kfs_arm_1, KFS_ARM_1_CMD_ID, KFS_ARM_1_MASTER_ID,&hfdcan3, DM_4340,MIT);
    DMmotor_Create (&kfs_arm_2, KFS_ARM_2_CMD_ID, KFS_ARM_2_MASTER_ID,&hfdcan3, DM_4340,MIT);
    DMmotor_Create (&kfs_arm_3, KFS_ARM_3_CMD_ID, KFS_ARM_3_MASTER_ID,&hfdcan3, DM_B2325,MIT);
	  DMmotor_Create (&kfs_catch, KFS_CATCH_CMD_ID, KFS_CATCH_MASTER_ID,&hfdcan3, DM_2325,MIT);
	
    Kfs.super_struct.AddMotor(&Kfs.super_struct,&kfs_arm_1.super_motor);
    Kfs.super_struct.AddMotor(&Kfs.super_struct,&kfs_arm_2.super_motor);
  	Kfs.super_struct.AddMotor(&Kfs.super_struct,&kfs_arm_3.super_motor);
    Kfs.super_struct.AddMotor(&Kfs.super_struct,&kfs_catch.super_motor);
    
    kfs_arm_1.send_cmd(&kfs_arm_1, Motor_Enable);
    kfs_arm_2.send_cmd(&kfs_arm_2, Motor_Enable);
  	kfs_arm_3.send_cmd(&kfs_arm_3, Motor_Enable);
    kfs_catch.send_cmd(&kfs_catch, Motor_Enable);
	
  	kfs_arm_1.set_mit_data(&kfs_arm_1,0.f,0.f,0.f,0.f,0.f);
  	kfs_arm_2.set_mit_data(&kfs_arm_2,0.f,0.f,0.f,0.f,0.f);		
	  kfs_arm_3.set_mit_data(&kfs_arm_3,0.f,0.f,0.f,0.f,0.f);
  	kfs_catch.set_mit_data(&kfs_catch,0.f,0.f,0.f,0.f,0.f);	

}

void Lift_Init(void)
{
    StructureModule_Create(&Lift.super_struct, lift);
    Lift.super_struct.base.Init(&Lift.super_struct.base);
    
    DMmotor_Create(&R2_lift_motor_left, R2_LIFT_MOTOR_LEFT_CMD_ID, R2_LIFT_MOTOR_LEFT_MASTER_ID, &hfdcan1, DM_T4340, MIT);
    DMmotor_Create(&R2_lift_motor_right, R2_LIFT_MOTOR_RIGHT_CMD_ID, R2_LIFT_MOTOR_RIGHT_MASTER_ID, &hfdcan1, DM_T4340, MIT);
//    DJImotor_Create(&balance_motor_left, BALANCE_MOTOR_LEFT_CMD_ID,BALANCE_MOTOR_LEFT_FEEDBACK_ID ,&hfdcan2, DJI_2006, SPEED, PID_POSITION, balance_motor_left_pid_param);
//    DJImotor_Create(&balance_motor_right, BALANCE_MOTOR_RIGHT_CMD_ID,BALANCE_MOTOR_RIGHT_FEEDBACK_ID ,&hfdcan2, DJI_2006, SPEED, PID_POSITION, balance_motor_right_pid_param);
    
    Lift.super_struct.AddMotor(&Lift.super_struct, &R2_lift_motor_left.super_motor);
    Lift.super_struct.AddMotor(&Lift.super_struct, &R2_lift_motor_right.super_motor);
    
    R2_lift_motor_left.send_cmd(&R2_lift_motor_left,Motor_Enable);
    R2_lift_motor_right.send_cmd(&R2_lift_motor_right,Motor_Enable);
	
    R2_lift_motor_left.set_mit_data(&R2_lift_motor_left,0.f,0.f,0.f,0.f,0.f);
    R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,0.f,0.f,0.f,0.f,0.f);
	
	  //HAL_Delay(1);
	  position_origin1 = R2_lift_motor_left.position;
	  position_origin2 = R2_lift_motor_right.position;   //初始位置获取

}

void Weapon_Init(void)
{
    StructureModule_Create(&Weapon.super_struct, weapon);
    Weapon.super_struct.base.Init(&Weapon.super_struct.base);
    
    DMmotor_Create(&weapon_collect_motor, WEAPON_COLLECT_MOTOR_CMD_ID, WEAPON_COLLECT_MOTOR_FEEDBACK_ID, &hfdcan2, DM_J4310, MIT);
    DJImotor_Create(&weapon_joint_motor, WEAPON_JOINT_MOTOR_CMD_ID,WEAPON_JOINT_MOTOR_FEEDBACK_ID ,&hfdcan2, DJI_2006, SPEED, PID_POSITION, weapon_joint_motor_pid_param);
 
    Weapon.super_struct.AddMotor(&Weapon.super_struct, &weapon_collect_motor.super_motor);
    Weapon.super_struct.AddMotor(&Weapon.super_struct, &weapon_joint_motor.super_motor);
    
    weapon_collect_motor.send_cmd(&weapon_collect_motor, Motor_Enable);
}

void Structue_Init(void)
{
    Chassis_Init();
    HAL_Delay(5);
    Kfs_Init();
    HAL_Delay(5);
    Lift_Init();
    HAL_Delay(5);
    Weapon_Init();
    HAL_Delay(5);
}
