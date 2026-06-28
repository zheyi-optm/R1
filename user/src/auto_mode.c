#include "auto_mode.h"


uint8_t auto_flag = 99;


void auto_mission()  //一区对接自动流程
{
//  if(auto_flag == 0)//取杆指令触发，autoflag已经置0
// {
//    //移动到第一定点 
	     
// }
// else if()//判断里程计，到达第一定点
// {
//    //移动到第二定点，视目标杆位置而定
// }
// else if()//判断里程计，到达第二定点
// {
//    //机械臂弹出，写死姿态
//	   arm.mode=2;
//     arm.s=0.39;
//     arm.h=0.11f; 
//     angles=Arm_Inverse_Solution(&arm);             
//     Torque=Torque_Comp_global(&kfs_arm_1,&kfs_arm_2,&kfs_arm_3);
//             
//     kfs_arm_1.set_mit_data(&kfs_arm_1,angles.theta_1+0.45f,0.0f,20.0f,1.5f,-0.25*Torque.Torque_1);
//     kfs_arm_2.set_mit_data(&kfs_arm_2,-angles.theta_2,0.0f,30.0f,2.0f,-Torque.Torque_2);
//     kfs_arm_3.set_mit_data(&kfs_arm_3,-25*(-kfs_arm_2.position-kfs_arm_1.position+0.43+a_weapon+0.2*PI),0.0f,3.0f,1.5f,Torque.Torque_3);
// }
// else if()//机械臂三电机均到位
// {
//    //夹爪合上
//	  if(kfs_catch.position > 30.0f)
//   {
//	   k_catch[0] = 0.0f;
//		 k_catch[1] = -5.0f;
//		 k_catch[2] = 0.0f;
//		 k_catch[3] = 0.4f;
//		 k_catch[4] = 0.0f;
//	 }
//	 else if(kfs_catch.position < 30.0f && kfs_catch.position > 5.0f)
//   {
//	   k_catch[0] = 0.0f;
//		 k_catch[1] = -(((float)kfs_catch.position -5.0f)/(25.0f)*4.0f + 1.0f);
//		 k_catch[2] = 0.0f;
//		 k_catch[3] = 1.0f/(-k_catch[1]);
//		 k_catch[4] = 0.0f;
//	 }
//	 else if(kfs_catch.position < 5.0f)
//   {
//	   k_catch[0] = 0.0f;
//		 k_catch[1] = 0.0f;
//	   k_catch[2] = 0.0f;
//	 	 k_catch[3] = 0.0f;
//		 k_catch[4] = -1.0f;
//	 }
// }
// else if(kfs_catch.position < 3.0f)//夹爪已经合上
// {
//    //机械臂收杆
//	   float kfs_3[5]={0,0,0,0,0};
//	   Torque=Torque_Comp_global(&kfs_arm_1,&kfs_arm_2,&kfs_arm_3);
//     kfs_arm_1.set_mit_data(&kfs_arm_1,0.4f,0.0f,20.0f,2.0f,-0.35*Torque.Torque_1);
//     kfs_arm_2.set_mit_data(&kfs_arm_2,-0.39f,0.0f,30.0f,3.0f,-0.3f*Torque.Torque_2);
//     in_place(20.8,&kfs_3[0],kfs_arm_3.position,&kfs_3[1],0.1f,&kfs_3[2],1.0f,&kfs_3[3],0.8f,0.4,&kfs_3[4],0.0f,0.0f,0.30f*PI);
//		 kfs_arm_3.set_mit_data(&kfs_arm_3,kfs_3[0],kfs_3[1],kfs_3[2],kfs_3[3],kfs_3[4]);
// }
// else if()//收杆到位
// {
//    //2325夹紧固定
//	  if(weapon_collect_motor.position >= -11.0f && weapon_collect_motor.position <= -2.0f)
//	 {
//	  hold_weapon[0] = 0.0f;
//		hold_weapon[1] = 2.5f;
//		hold_weapon[2] = 0.0f;
//		hold_weapon[3] = 0.4f;
//		hold_weapon[4] = 0.0f;
//   }
//	 if(weapon_collect_motor.position > -2.0f)
//	 {
//	  hold_weapon[0] = 0.0f;
//		hold_weapon[1] = 0.0f;
//		hold_weapon[2] = 0.0f;
//		hold_weapon[3] = 0.0f;
//		hold_weapon[4] = 1.0f;
//	 }
//		weapon_collect_motor.set_mit_data(&weapon_collect_motor,hold_weapon[0],hold_weapon[1],hold_weapon[2],hold_weapon[3],hold_weapon[4]);
// }
// else if(weapon_collect_motor.position > -2.0f)//已经固定
// {
//    //移动到第三定点，准备对接
// }
// 
// if(weapon_collect_motor.position < -2.0f)//如果还未固定
// {
//    //抬升中
//   in_place_fast(11.5f,&lift_left[0],R2_lift_motor_left.position,&lift_left[1],18.0f,8.0f,&lift_left[2],4.0f,&lift_left[3],2.0f,0.5f,&lift_left[4],1.0f,0.05f,0.28f,1.2f);
//	 in_place_fast(11.5f,&lift_right[0],R2_lift_motor_right.position,&lift_right[1],18.0f,8.0f,&lift_right[2],4.0f,&lift_right[3],2.0f,0.5f,&lift_right[4],1.0f,0.05f,0.28f,1.2f);
//	 hold_step(R2_lift_motor_left.speed_w,R2_lift_motor_right.speed_w,&lift_right[4]);
//	 side_by_side(R2_lift_motor_left.position,R2_lift_motor_right.position,&lift_right[4]);
//	 
//	 R2_lift_motor_left.set_mit_data(&R2_lift_motor_left,lift_left[0],lift_left[1],lift_left[2],lift_left[3],lift_left[4]);
//	 R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,lift_right[0],lift_right[1],lift_right[2],lift_right[3],lift_right[4]);
// }
// if(weapon_collect_motor.position > -2.0f)//已经固定
// {
//    //抬升高
//    in_place_fast((19.6f),&lift_left[0],R2_lift_motor_left.position,&lift_left[1],18.0f,8.0f,
//	                             &lift_left[2],8.0f,&lift_left[3],2.0f,1.8f,&lift_left[4],1.0f,0.02f,0.35f,1.2f);
//	  in_place_fast((19.6f),&lift_right[0],R2_lift_motor_right.position,&lift_right[1],18.0f,8.0f,
//	                             &lift_right[2],8.0f,&lift_right[3],2.0f,1.8f,&lift_right[4],1.0f,0.02f,0.35f,1.2f);
//	  hold_step(R2_lift_motor_left.speed_w,R2_lift_motor_right.speed_w,&lift_right[4]);
//	  side_by_side(R2_lift_motor_left.position,R2_lift_motor_right.position,&lift_right[4]);
// }
 
}

uint8_t distance_count = 0;
float ml_dis = 1.2f;   //视情况更改，梅林边长宽度
void auto_second_inplace()  //快速抵达二区自动函数
{
//  auto_inplace(1.2f,2.0f,270); //让身位
//	if(removed_k1 == 2)
// {
// 
// }
// if(removed_k1 == 11)
// {
//   //两角落带旋转
//	 //横移到位
// }
// if(removed_k1 %3 == 1)
// {
//   //右角落带旋转
//	   distance_count = (removed_k1 /3) + 1;
//	   auto_inplace(1.2f + distance_count * ml_dis,2.0f,270);
//	 //横移到位
// }
// if(removed_k1 %3 == 0)
// {
//   //左角落带旋转
//	   distance_count = removed_k1 /3;
//	   auto_inplace(1.2f + distance_count * ml_dis,2.0f,90);
//	 //横移到位
// }
	
}
//第一步，左移一个车身位，避开R2.第二步，根据需要取走的R1KFS选定梅林角落，左或右。第三步，横移到KFS前


float mid_x = 0.0f;  //mid360数据
float mid_y = 0.0f;
float mid_yaw = 0.0f; //初始机械臂车头为0度

float mid_x_last = 0.0f;    //保存上次数据
float mid_y_last = 0.0f;
float mid_yaw_last = 0.0f; //初始机械臂车头为0度

float diff_x = 0.0f;
float diff_y = 0.0f;
float diff_w = 0.0f; //mid360世界坐标下坐标差

void auto_inplace(float target_x,float target_y,float target_w)  //一区到二区的自动定点到位函数，承担读取解析里程计，对三速度分量进行映射赋值的任务（配合auto_chassis()）
{
    diff_x = target_x - mid_x;
    diff_y = target_y - mid_y;
    diff_w = target_w - mid_yaw; //mid360世界坐标下坐标差
	
	  while(diff_w < -PI){diff_w += 2.0f*PI;}
		while(diff_w > PI){diff_w -= 2.0f*PI;}   //归化
		float auto_x = diff_x * cosf(mid_yaw) + diff_y * sinf(mid_yaw);
    float auto_y = -(diff_x * sinf(mid_yaw)) + diff_y * cosf(mid_yaw); //机器人坐标数据,具体旋转矩阵与正负需视具体情况而定
	
  	SP_ACCEL = 10.0f; //调试用10，成熟用20~25
    SP_X = auto_set(auto_x,-2.0f,2.0f,-1.0f,1.0f,0.05f) + auto_pid(diff_x);
   	SP_Y = auto_set(auto_y,-2.0f,2.0f,-1.0f,1.0f,0.05f) + auto_pid(diff_y);      
	  SP_W = - (auto_set(diff_w,-2.0f,2.0f,-1.2f,1.2f,0.02f) + auto_pid(diff_w)); 

    auto_chassis();	
	
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

//自动里程映射函数
float auto_set(float src, float src_low, float src_high, float dst_min, float dst_max, float deadzone)
{
	float persent = 0.0f;
	float res = 0.0f;
	     
	 if(src > src_high){src = src_high;}
	 if(src < src_low){src = src_low;}//范围检查，置换

	if(src < 0 + deadzone  && src > 0 - deadzone)
  {
	  persent = 0;
		res = 0;
	}
  else if(src > 0 + deadzone)
  {
	  persent = (float)(src - (0 + deadzone))/(float)(src_high - (0 + deadzone));
		res = persent * (dst_max - 0);
	}
	else if(src < 0 - deadzone)
  {
	  persent = - (float)((0 - deadzone)- src)/(float)((0 - deadzone) - src_low);
		res = persent * (0 - dst_min);
	} 
   return res;
}


float distance = 0.0f;
float distance_last = 0.0f;
float auto_i = 0.0f;
float auto_d = 0.0f;
float pi_max = 1000.0f;//积分上限
float auto_id[2] = {0.0001f,0.01};

//PID位置里程环函数
float auto_pid(float diff)
{
	if(auto_i < pi_max)
 {
   auto_i += diff;
 }
   auto_d = distance_last - diff; //
  
   float res = auto_id[0] * auto_i - auto_id[1] * auto_d;
   distance_last = diff;
 
   return res;
}

//float auto_pid()
//{
//  distance = sqrtf(diff_x * diff_x + diff_y * diff_y);
//	
//	if(auto_i < pi_max)
// {
//   auto_i += distance;
// }
//   auto_d = distance_last - distance; //
//  
//   float res = auto_id[0] * auto_i - auto_id[1] * auto_d;
//   distance_last = distance;
// 
//   return res;
//}  //distance id环







