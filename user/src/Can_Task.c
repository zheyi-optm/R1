#include "Can_Task.h"
#include "Motion_Task.h"
#include "motor.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "chassis.h"
#include "kfs.h"
#include "lift.h"
#include "weapon.h"
#include "arm.h"
#include "tim.h"
#include "cowork.h"
#include "usart.h"
#include "auto_mode.h"


//void Can_Task(void const * argument)
//{
//	for(;;)
//	{
//  if(RCctrl.rc_lost)
//{
// //   Chassis.Chassis_Stop(&Chassis);
//	  
//}

//  else//遥控在线
//{
//   if(RCctrl.CH6<1000)//主控
//{

//}
//  else//遥控	
// {
//	if(RCctrl.CH10>1400)//底盘模式，CH9换车头，CH9下尾部，双击CH8锁定
//  {
//   // Chassis_mode();
//		 Chassis.Chassis_Calc(&Chassis);

//    vTaskDelay(1);
//		
//		chassis_angle1.set_mit_data(&chassis_angle1,angle_now[0],0.0f,0.6f,0.032f,0.0f);
//		chassis_angle2.set_mit_data(&chassis_angle2,angle_now[1],0.0f,0.6f,0.032f,0.0f);
//		chassis_angle3.set_mit_data(&chassis_angle3,angle_now[2],0.0f,0.6f,0.032f,0.0f);
//		chassis_angle4.set_mit_data(&chassis_angle4,angle_now[3],0.0f,0.6f,0.032f,0.0f);
//		
//		
//  }

//	else if(RCctrl.CH5<500)//武器模式
//  {
//		weapon_mode();
//		kfs_catch_mode();
//		
//		R2_lift_motor_left.set_mit_data(&R2_lift_motor_left,lift_left[0],lift_left[1],lift_left[2],lift_left[3],lift_left[4]);
//		R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,lift_right[0],lift_right[1],lift_right[2],lift_right[3],lift_right[4]);
//		
//		kfs_catch.set_mit_data(&kfs_catch,k_catch[0],k_catch[1],k_catch[2],k_catch[3],k_catch[4]);
//		weapon_collect_motor.set_mit_data(&weapon_collect_motor,hold_weapon[0],hold_weapon[1],hold_weapon[2],hold_weapon[3],hold_weapon[4]);
//		
//		weapon_joint_motor.PID_Calculate(&weapon_joint_motor,weapon_joint_input);
//		if(ABS(weapon_joint_input) <=10){weapon_joint_motor.pid_spd.Output = 0.0f;}
//	  DJIset_motor_data(&hfdcan2, 0X1FF,0 ,weapon_joint_motor.pid_spd.Output,0,0);
//		
//		vTaskDelay(2);
//		Chassis.chassis_mission(&Chassis);
//		Arm_task(&kfs_arm_1,&kfs_arm_2,&kfs_arm_3);
//	}

//	else if(RCctrl.CH5>700 && RCctrl.CH5<1200)//kfs模式
//  {
//	  kfs_mode();
//		vTaskDelay(1);
//		Chassis.chassis_mission(&Chassis);
//		Arm_task(&kfs_arm_1,&kfs_arm_2,&kfs_arm_3);
//	}
//	
//	else if(RCctrl.CH5>1300)//抬升模式
//  {
//	  lift_mode();
//		Chassis.chassis_mission(&Chassis);//待优化，有问题，会卡死任务
//		
//		R2_lift_motor_left.set_mit_data(&R2_lift_motor_left,lift_left[0],lift_left[1],lift_left[2],lift_left[3],lift_left[4]);
//		R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,lift_right[0],lift_right[1],lift_right[2],lift_right[3],lift_right[4]);
//		
//		//后期优化加上放回枪杆的功能，复制武器模式的两行(kfs_catch)
//		weapon_joint_motor.PID_Calculate(&weapon_joint_motor,weapon_joint_input);
//		if(ABS(weapon_joint_input) <= 10){weapon_joint_motor.pid_spd.Output = 0.0f;}
//		DJIset_motor_data(&hfdcan2, 0X1FF,0 ,weapon_joint_motor.pid_spd.Output,0,0);
//		
//		vTaskDelay(2);
//		Arm_task(&kfs_arm_1,&kfs_arm_2,&kfs_arm_3);
//	}
//		
//	  chassis_motor1.PID_Calculate(&chassis_motor1, 10.0f*Chassis.param.V_out[0]);
//    chassis_motor2.PID_Calculate(&chassis_motor2, 10.0f*Chassis.param.V_out[1]);
//		chassis_motor3.PID_Calculate(&chassis_motor3, 10.0f*Chassis.param.V_out[2]);
//    chassis_motor4.PID_Calculate(&chassis_motor4, 10.0f*Chassis.param.V_out[3]);
//	
//		DJIset_motor_data(&hfdcan2, 0x200, chassis_motor1.pid_spd.Output, chassis_motor2.pid_spd.Output,chassis_motor3.pid_spd.Output,chassis_motor4.pid_spd.Output);
//   

//	 // Arm_task(&kfs_arm_1,&kfs_arm_2,&kfs_arm_3);
//   }
//  }
//    vTaskDelay(1);
//	 // osDelay(5);
// }
//}


void HF_Task()          //高频率任务，200hz
{
  if(RCctrl.rc_lost)
{
	
}

  else//遥控在线
{
	if(RCctrl.channel == 0) //遥控模式
{
	if(RCctrl.chassis == 0)//底盘模式
  {
		 Chassis.Chassis_Calc(&Chassis);

		//可能要加延时
		chassis_motor1.PID_Calculate(&chassis_motor1, 10.0f*Chassis.param.V_out[0]);
    chassis_motor2.PID_Calculate(&chassis_motor2, 10.0f*Chassis.param.V_out[1]);
		chassis_motor3.PID_Calculate(&chassis_motor3, 10.0f*Chassis.param.V_out[2]);
    chassis_motor4.PID_Calculate(&chassis_motor4, 10.0f*Chassis.param.V_out[3]);
	
		chassis_angle1.set_mit_data(&chassis_angle1,angle_now[0],0.0f,0.6f,0.032f,0.0f);
		chassis_angle2.set_mit_data(&chassis_angle2,angle_now[1],0.0f,0.6f,0.032f,0.0f);
		chassis_angle3.set_mit_data(&chassis_angle3,angle_now[2],0.0f,0.6f,0.032f,0.0f);
		chassis_angle4.set_mit_data(&chassis_angle4,angle_now[3],0.0f,0.6f,0.032f,0.0f);
	
		DJIset_motor_data(&hfdcan2, 0x200, chassis_motor1.pid_spd.Output, chassis_motor2.pid_spd.Output,chassis_motor3.pid_spd.Output,chassis_motor4.pid_spd.Output);	
  }

	else if(RCctrl.chassis == 1 && RCctrl.zone == 0)//武器模式
  {
//		weapon_mode();
//		
//		R2_lift_motor_left.set_mit_data(&R2_lift_motor_left,lift_left[0],lift_left[1],lift_left[2],lift_left[3],lift_left[4]);
//		R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,lift_right[0],lift_right[1],lift_right[2],lift_right[3],lift_right[4]);
//	  
//		weapon_joint_motor.PID_Calculate(&weapon_joint_motor,weapon_joint_input);
//		if(ABS(weapon_joint_input) <=10){weapon_joint_motor.pid_spd.Output = 0.0f;}
//	  DJIset_motor_data(&hfdcan2, 0X1FF,0 ,weapon_joint_motor.pid_spd.Output,0,0);

//		Arm_task(&kfs_arm_1,&kfs_arm_2,&kfs_arm_3);
	}

	else if(RCctrl.chassis == 1 && RCctrl.zone == 1)//kfs模式
  {
		//Arm_task(&kfs_arm_1,&kfs_arm_2,&kfs_arm_3);
	}
	
	else if(RCctrl.chassis == 1 && RCctrl.zone == 2)//抬升模式
  {		
//		lift_mode();
//		
//		R2_lift_motor_left.set_mit_data(&R2_lift_motor_left,lift_left[0],lift_left[1],lift_left[2],lift_left[3],lift_left[4]);
//		R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,lift_right[0],lift_right[1],lift_right[2],lift_right[3],lift_right[4]);
//		
//		weapon_joint_motor.PID_Calculate(&weapon_joint_motor,weapon_joint_input);
//		if(ABS(weapon_joint_input) <= 10){weapon_joint_motor.pid_spd.Output = 0.0f;}
//		DJIset_motor_data(&hfdcan2, 0X1FF,0 ,weapon_joint_motor.pid_spd.Output,0,0);

//		Arm_task(&kfs_arm_1,&kfs_arm_2,&kfs_arm_3);
	}
}
  if(RCctrl.channel == 1) //自动模式
 {
   auto_mission();
 }
}
}



void LF_Task()        //低频率任务，满速100hz
{
if(RCctrl.rc_lost)
{
	
}

  else//遥控在线
{
	if(RCctrl.channel == 0)//遥控模式
{
	if(RCctrl.chassis == 1 && RCctrl.zone == 0)//武器模式
  {
		Chassis.chassis_mission(&Chassis);
		
//		kfs_catch_mode();
//		kfs_catch.set_mit_data(&kfs_catch,k_catch[0],k_catch[1],k_catch[2],k_catch[3],k_catch[4]);//尝试降频
//	  
//    weapon_hold();
//		weapon_collect_motor.set_mit_data(&weapon_collect_motor,hold_weapon[0],hold_weapon[1],hold_weapon[2],hold_weapon[3],hold_weapon[4]);
//		
//		BUT_check();
		
	}
	
	else if(RCctrl.chassis == 1 && RCctrl.zone == 1)//kfs模式
  {
		Chassis.chassis_mission(&Chassis);
		//kfs_mode();
	}
	
	else if(RCctrl.chassis == 1 && RCctrl.zone == 2)//抬升模式
  {		
		Chassis.chassis_mission(&Chassis);
		
//		kfs_catch_mode();
//		kfs_catch.set_mit_data(&kfs_catch,k_catch[0],k_catch[1],k_catch[2],k_catch[3],k_catch[4]);//遥控支持程度低，没必要写，下面其实也是。杆功能基本不在线

//		weapon_hold();
//		weapon_collect_motor.set_mit_data(&weapon_collect_motor,hold_weapon[0],hold_weapon[1],hold_weapon[2],hold_weapon[3],hold_weapon[4]);
//		kfs_mode();
	}
  	//sense();
}
}
}

void Free_Task(void const * argument)
{
  for(;;)
 {
    if(RCctrl.chassis == 1 && RCctrl.zone == 0 && RCctrl.rodPos == 0)  //双换杆按钮静默
    {
//		  hold_weapon[0] = 0.0f;
//		  hold_weapon[1] = 2.5f;
//		  hold_weapon[2] = 0.0f;
//		  hold_weapon[3] = 0.4f;
//		  hold_weapon[4] = 0.0f;
//			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,1600);//
			vTaskDelay(1);
		}
		else if(BUT_flag.button_new ==1 && BUT_flag.wep_change %2 == 0)  //按钮触发，存（换杆）
    {
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,1400);//合舵机
			
		  hold_weapon[0] = -10.0f;
		  hold_weapon[1] = 0.0f;
		  hold_weapon[2] = 1.0f;
		  hold_weapon[3] = 0.6f;
		  hold_weapon[4] = 0.0f;//补参赋值
			vTaskDelay(800);
			weapon_collect_motor.set_mit_data(&weapon_collect_motor,-10.0f,0.0f,1.0f,0.4f,0.0f);//
			
			vTaskDelay(200);
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,600);//移动	
			vTaskDelay(1200);
			BUT_flag.button_new =0;			
		}
		else if(BUT_flag.button_new ==1 && BUT_flag.wep_change %2 == 1)  //按钮触发，取（换杆）
    {
			hold_weapon[0] = -10.0f;
		  hold_weapon[1] = 0.0f;
	  	hold_weapon[2] = 1.0f;
		  hold_weapon[3] = 0.6f;
		  hold_weapon[4] = 0.0f;//补参赋值
			weapon_collect_motor.set_mit_data(&weapon_collect_motor,-10.0f,0.0f,1.0f,0.4f,0.0f);//开2325
			vTaskDelay(800);
			
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,2500);//移动	
			vTaskDelay(1800);
			
			hold_weapon[0] = 0.0f;
	  	hold_weapon[1] = 2.5f;
	  	hold_weapon[2] = 0.0f;
	  	hold_weapon[3] = 0.4f;
	  	hold_weapon[4] = 0.0f;//补参赋值
			weapon_collect_motor.set_mit_data(&weapon_collect_motor,0.0f,2.4f,0.0f,0.5f,0.0f);
			vTaskDelay(1100);
			
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,1600);	
			BUT_flag.button_new =0;
		}
	 
	 osDelay(5);
 }
}

void Sent_Task(void const * argument)
{
	for(;;)
{
  if(RCctrl.received == true)
 {
   policy_init(kfs1, kfs2, kf);
	 vTaskDelay(1);
	 
	 bool success = policy_solve_best(
        path, &path_len,           // 输出：路径和长度
        picked_k2, &picked_cnt,    // 输出：拾取的K2和数量
        &removed_k1,               // 输出：移除的K1
        &target_k2                 // 输出：目标K2数量
    );
	 SwapNumArray(path);
   SwapNumArray(picked_k2);
	 
	 vTaskDelay(1);
	 
	 uart8_tx_init();
	 RCctrl.received = false;
	 sent_flag = true;
	 vTaskDelay(1);
 }
 if(sent_flag == true)
 {
	 if(BUT_flag.cent_count <=64)
  {
	  HAL_UART_Transmit(&huart8,uart8_tx_buf,7,100);
		BUT_flag.cent_count ++;
		vTaskDelay(1);
	}
	if(BUT_flag.cent_count >64)
  {
	  BUT_flag.cent_count =0;
		sent_flag = false;
	}
	 
 }
 
 
   osDelay(5);
}
}



