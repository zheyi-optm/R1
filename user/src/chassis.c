#include "chassis.h"

Chassis_Module Chassis;

DM_MotorModule chassis_angle1;  // （左前）
DM_MotorModule chassis_angle2;  // （左后）
DM_MotorModule chassis_angle3;  // （右后）
DM_MotorModule chassis_angle4;  // （右前）

DJI_MotorModule chassis_motor1;  // （左前）
DJI_MotorModule chassis_motor2;  // （左后）
DJI_MotorModule chassis_motor3;  // （右后）
DJI_MotorModule chassis_motor4;  // （右前）

uint8_t mission_sign = 0;//任务状态降频发送舵向角度标志位

float SP_ACCEL = 0.0f;
float SP_X = 0.0f;
float SP_Y = 0.0f;
float SP_W = 0.0f;

float chassis_motor1_pid_param[PID_PARAMETER_NUM] = {2.5f,0.05f,0.1f,1,500.0f,10000.0f};     //KP,KI,KD,DEADBAND,LIMITINTEGRAL,LIMITOUTPUT
float chassis_motor2_pid_param[PID_PARAMETER_NUM] = {2.5f,0.05f,0.1f,1,500.0f,10000.0f};
float chassis_motor3_pid_param[PID_PARAMETER_NUM] = {2.5f,0.05f,0.1f,1,500.0f,10000.0f};
float chassis_motor4_pid_param[PID_PARAMETER_NUM] = {2.5f,0.05f,0.1f,1,500.0f,10000.0f};

float target_angle[4];//目标转向角
float angle_last[4] = {ORIGIN_ANGLE1,ORIGIN_ANGLE2,ORIGIN_ANGLE3,ORIGIN_ANGLE4};//上次转向角
float angle_diff[4];//误差
float wheel_sp[4];//目标轮向速度
float angle_now[4];//输出角度

void Chassis_Calc(Chassis_Module *chassis)
{
	mission_sign = 0;
	if(RCctrl.lock == 1)         //按钮触发，锁定状态
{
	  angle_now[0] = 0.0f - 0.785f;
	  angle_now[1] = 0.0f + 0.785f;
   	angle_now[2] = 0.0f - 0.785f;
	  angle_now[3] = 0.0f + 0.785f;
	
	  angle_last[0] = 0.0f;
	  angle_last[1] = 0.0f;
  	angle_last[2] = 0.0f;
	  angle_last[3] = 0.0f;
}
  else if (-8<RCctrl.vx && RCctrl.vx<8 && -8<RCctrl.vy && RCctrl.vy<8 && -8<RCctrl.vw && RCctrl.vw<8)  //遥控死区状态下节省算力，停运6220，锁定3508
{
    chassis->param.V_out[0] = 0.f;
    chassis->param.V_out[1] = 0.f;
    chassis->param.V_out[2] = 0.f;
    chassis->param.V_out[3] = 0.f;
}
  else  //非锁定非死区
{
	  SP_ACCEL = SPEED_ACCEL ;
    SP_X = - SPEED_X ;
    SP_Y = - SPEED_Y ;   //非触发状态，对接车头
    SP_W =   SPEED_WO ;
	
	if(RCctrl.switch_dir == 1)//按键触发状态切换机械臂车头
  {
	  SP_X = -SP_X;
		SP_Y = -SP_Y;
	}
	
    chassis->param.Accel = SP_ACCEL;
    chassis->param.Vx_in = SP_X;
    chassis->param.Vy_in = SP_Y;
    chassis->param.Vw_in = SP_W;
    
	  target_angle[0] = atan2f(SP_Y + SP_W * RX , SP_X - SP_W * RY);
	  target_angle[1] = atan2f(SP_Y + SP_W * (-RX) , SP_X - SP_W * RY);
	  target_angle[2] = atan2f(SP_Y + SP_W * (-RX) , SP_X - SP_W * (-RY));
	  target_angle[3] = atan2f(SP_Y + SP_W * RX , SP_X - SP_W * (-RY));      //基础角度解算
	
    wheel_sp[0] = - sqrtf((SP_X - SP_W * RY) * (SP_X - SP_W * RY) + (SP_Y + SP_W * RX) * (SP_Y + SP_W * RX)) / S;
	  wheel_sp[1] = - sqrtf((SP_X - SP_W * RY) * (SP_X - SP_W * RY) + (SP_Y + SP_W * (-RX)) * (SP_Y + SP_W * (-RX))) / S;
	  wheel_sp[2] = sqrtf((SP_X - SP_W * (-RY)) * (SP_X - SP_W * (-RY)) + (SP_Y + SP_W * (-RX)) * (SP_Y + SP_W * (-RX))) / S;
	  wheel_sp[3] = sqrtf((SP_X - SP_W * (-RY)) * (SP_X - SP_W * (-RY)) + (SP_Y + SP_W * RX) * (SP_Y + SP_W * RX)) / S;        //基础速度解算
		
	for(int i= 0;i<4;i++)
	{
		angle_diff[i] = target_angle[i] - angle_last[i];    //误差计算
	  while(angle_diff[i] < -PI){angle_diff[i] += 2.0f*PI;}
		while(angle_diff[i] > PI){angle_diff[i] -= 2.0f*PI;}   //归化
		
		if(PI/2.0f < angle_diff[i])
   {
	  angle_now[i] = angle_last[i] + angle_diff[i] - PI;
	  wheel_sp[i] = -wheel_sp[i];
	 }
		else if(-PI/2.0f > angle_diff[i])
   {
	  angle_now[i] = angle_last[i] + angle_diff[i] + PI;
	  wheel_sp[i] = -wheel_sp[i];
	 }
	 else if(-PI/2.0f <= angle_diff[i] && angle_diff[i] <= PI/2.0f)
   {
	  angle_now[i] = angle_last[i] + angle_diff[i];
	 }                                                    //优劣弧解算
	  angle_last[i] = angle_now[i];                       //开环赋值，循环解算
	  angle_now[i] = -angle_now[i];	                      //反转
    chassis->param.V_out[i] = SP_ACCEL * wheel_sp[i];	  //3508轮速赋值
	}

}
}


void chassis_mission(Chassis_Module *chassis)
{
	mission_sign ++;
 if(RCctrl.lock == 0)//若并非锁定
 {
  if(mission_sign %2 == 1)
 {
    chassis_angle1.set_mit_data(&chassis_angle1,1.5708f,0.0f,0.6f,0.032f,0.0f);
		chassis_angle2.set_mit_data(&chassis_angle2,1.5708f,0.0f,0.6f,0.032f,0.0f);
		chassis_angle3.set_mit_data(&chassis_angle3,-1.5708f,0.0f,0.6f,0.032f,0.0f);
		chassis_angle4.set_mit_data(&chassis_angle4,-1.5708f,0.0f,0.6f,0.032f,0.0f);
	 
	  for(int i=0;i<2;i++)
  {
	  angle_last[i] = - 1.5708f;
	}
  	for(int i=2;i<4;i++)
  {
	  angle_last[i] = 1.5708f;
	}
 }
 
   SP_ACCEL = 40.0f;
   SP_W = SPEED_WO;
 if(RCctrl.switch_dir == 1)
 {
   SP_W = -SP_W;
 }
 
   for(int i=0;i<4;i++)
 {
   chassis->param.V_out[i] = SP_ACCEL * SP_W;
 }
   chassis_motor1.PID_Calculate(&chassis_motor1, 20.0f*Chassis.param.V_out[0]);
   chassis_motor2.PID_Calculate(&chassis_motor2, 20.0f*Chassis.param.V_out[1]);
	 chassis_motor3.PID_Calculate(&chassis_motor3, 20.0f*Chassis.param.V_out[2]);
   chassis_motor4.PID_Calculate(&chassis_motor4, 20.0f*Chassis.param.V_out[3]);//从cantask集成过来实现降频同时减少代码量
 
   DJIset_motor_data(&hfdcan2, 0x200, chassis_motor1.pid_spd.Output, chassis_motor2.pid_spd.Output,chassis_motor3.pid_spd.Output,chassis_motor4.pid_spd.Output);	
 }
}

float auto_Vout[4] = {0};

void auto_chassis()
{
    target_angle[0] = atan2f(SP_Y + SP_W * RX , SP_X - SP_W * RY);
	  target_angle[1] = atan2f(SP_Y + SP_W * (-RX) , SP_X - SP_W * RY);
	  target_angle[2] = atan2f(SP_Y + SP_W * (-RX) , SP_X - SP_W * (-RY));
	  target_angle[3] = atan2f(SP_Y + SP_W * RX , SP_X - SP_W * (-RY));      //基础角度解算
	
    wheel_sp[0] = - sqrtf((SP_X - SP_W * RY) * (SP_X - SP_W * RY) + (SP_Y + SP_W * RX) * (SP_Y + SP_W * RX)) / S;
	  wheel_sp[1] = - sqrtf((SP_X - SP_W * RY) * (SP_X - SP_W * RY) + (SP_Y + SP_W * (-RX)) * (SP_Y + SP_W * (-RX))) / S;
	  wheel_sp[2] = sqrtf((SP_X - SP_W * (-RY)) * (SP_X - SP_W * (-RY)) + (SP_Y + SP_W * (-RX)) * (SP_Y + SP_W * (-RX))) / S;
	  wheel_sp[3] = sqrtf((SP_X - SP_W * (-RY)) * (SP_X - SP_W * (-RY)) + (SP_Y + SP_W * RX) * (SP_Y + SP_W * RX)) / S;        //基础速度解算
		
	for(int i= 0;i<4;i++)
	{
		angle_diff[i] = target_angle[i] - angle_last[i];    //误差计算
	  while(angle_diff[i] < -PI){angle_diff[i] += 2.0f*PI;}
		while(angle_diff[i] > PI){angle_diff[i] -= 2.0f*PI;}   //归化
		
		if(PI/2.0f < angle_diff[i])
   {
	  angle_now[i] = angle_last[i] + angle_diff[i] - PI;
	  wheel_sp[i] = -wheel_sp[i];
	 }
		else if(-PI/2.0f > angle_diff[i])
   {
	  angle_now[i] = angle_last[i] + angle_diff[i] + PI;
	  wheel_sp[i] = -wheel_sp[i];
	 }
	 else if(-PI/2.0f <= angle_diff[i] && angle_diff[i] <= PI/2.0f)
   {
	  angle_now[i] = angle_last[i] + angle_diff[i];
	 }                                                    //优劣弧解算
	  angle_last[i] = angle_now[i];                       //开环赋值，循环解算
	  angle_now[i] = -angle_now[i];	                      //反转
    auto_Vout[i] = SP_ACCEL * wheel_sp[i];	            //3508轮速赋值
	}
}

//void Chassis_Stop(Chassis_Module *chassis)
//{
//    chassis->param.V_out[0] = 0.f;
//    chassis->param.V_out[1] = 0.f;
//    chassis->param.V_out[2] = 0.f;
//    chassis->param.V_out[3] = 0.f;
//    
//}



