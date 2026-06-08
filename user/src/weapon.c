#include "weapon.h"
#include "remote_control.h"
#include "main.h"
#include "tim.h"
#include "timer.h"
#include "motor_control.h"
#include "lift.h"
#include "cmsis_os.h"

//#include "Motion_Task.h"


DJI_MotorModule weapon_joint_motor;
DM_MotorModule weapon_collect_motor;
Weapon_Module Weapon;

float weapon_joint_motor_pid_param[PID_PARAMETER_NUM] = {5.0f,0.05f,0.3f,1,200.0f,10000.0f}; 

float weapon_joint_input=0;
float h_compensation = 0.0f;

void weapon_mode()
{
 if(RCctrl.liftPos == 0) //按钮触发抬升1
 {
	 in_place_fast(1.0f,&lift_left[0],R2_lift_motor_left.position,&lift_left[1],8.0f,8.0f,&lift_left[2],0.0f,&lift_left[3],4.0f,0.0f,&lift_left[4],1.0f,0.0f,0.30f,1.2f);
	 in_place_fast(1.0f,&lift_right[0],R2_lift_motor_right.position,&lift_right[1],8.0f,8.0f,&lift_right[2],0.0f,&lift_right[3],4.0f,0.0f,&lift_right[4],1.0f,0.0f,0.30f,1.2f);
	 //hold_step(R2_lift_motor_left.speed_w,R2_lift_motor_right.speed_w,&lift_right[4]);
	 //side_by_side(R2_lift_motor_left.position,R2_lift_motor_right.position,&lift_right[4]);  //side函数读取初始位置的语句或许可以不要了
 }
 else if(RCctrl.liftPos == 1)  //按钮触发抬升2
 {
   in_place_fast(9.5f,&lift_left[0],R2_lift_motor_left.position,&lift_left[1],8.0f,8.0f,&lift_left[2],4.0f,&lift_left[3],4.0f,0.4f,&lift_left[4],1.0f,0.05f,0.24f,1.2f);
	 in_place_fast(9.5f,&lift_right[0],R2_lift_motor_right.position,&lift_right[1],8.0f,8.0f,&lift_right[2],4.0f,&lift_right[3],4.0f,0.4f,&lift_right[4],1.0f,0.05f,0.24f,1.2f);
//	 hold_step(R2_lift_motor_left.speed_w,R2_lift_motor_right.speed_w,&lift_right[4]);
//	 side_by_side(R2_lift_motor_left.position,R2_lift_motor_right.position,&lift_right[4]);
 }
 else if(RCctrl.liftPos == 2)  //按钮触发抬升3
 {
	 h_compensation = remote_control_read(RCctrl.accel,ACCEL_LOW,((ACCEL_HIGH + ACCEL_LOW)/2),ACCEL_HIGH,-0.8f,0,0.8f);
   in_place_fast((16.6f + h_compensation),&lift_left[0],R2_lift_motor_left.position,&lift_left[1],8.0f,8.0f,
	                             &lift_left[2],18.0f,&lift_left[3],4.0f,1.8f,&lift_left[4],1.0f,0.02f,0.35f,1.2f);
	 in_place_fast((16.6f + h_compensation),&lift_right[0],R2_lift_motor_right.position,&lift_right[1],8.0f,8.0f,
	                             &lift_right[2],18.0f,&lift_right[3],4.0f,1.8f,&lift_right[4],1.0f,0.02f,0.35f,1.2f);
//	 hold_step(R2_lift_motor_left.speed_w,R2_lift_motor_right.speed_w,&lift_right[4]);
//	 side_by_side(R2_lift_motor_left.position,R2_lift_motor_right.position,&lift_right[4]);         //初始值调好之后再解注释
 }
  weapon_joint_input = 900.0f * SPEED_X; //常驻对接2006
	 
}
//TIM1控持杆舵机，1600us张开，1450闭合；
//TIM3控换杆舵机，600us主操作位，2500次工作位；

float hold_weapon[5] = {0,0,0,0,0};
void weapon_hold()
{
 if(BUT_flag.button_new == 0)
 {
  if(RCctrl.hold ==0)  //按钮关
  {
	  hold_weapon[0] = -10.0f;
		hold_weapon[1] = 0.0f;
		hold_weapon[2] = 1.0f;
		hold_weapon[3] = 0.6f;
		hold_weapon[4] = 0.0f;
	}
	else if(RCctrl.hold ==1)  //按钮开
  {
		if(weapon_collect_motor.position >= -11.0f && weapon_collect_motor.position <= -2.0f)
	 {
	  hold_weapon[0] = 0.0f;
		hold_weapon[1] = 2.5f;
		hold_weapon[2] = 0.0f;
		hold_weapon[3] = 0.4f;
		hold_weapon[4] = 0.0f;
   }
	 if(weapon_collect_motor.position > -2.0f)
	 {
	  hold_weapon[0] = 0.0f;
		hold_weapon[1] = 0.0f;
		hold_weapon[2] = 0.0f;
		hold_weapon[3] = 0.0f;
		hold_weapon[4] = 1.0f;//参数待调！
   }
	}
 }
}



