#include "motor_control.h"


//float ignore_diff = 0.15f;
float exceed_num = 0.5f; //针对PID算法差值供矩特性，提供超越数维持期望表现

//匀速到位函数
//t_position：期望到达位置；sent_p：传出的指令位置；c_position：电机回传位置；sent_sp：传出的指令匀速速度；u_speed：期望的匀速速度；s_kp：传出的指令KP；
//h_kp：期望的位控KP；s_kd：传出的指令KD；h_kd：期望的运动KD；hold_kd：位控时的保持KD；T：力矩，同KD逻辑；
void in_place(float t_position,float *sent_p,float c_position,float *sent_sp,float u_speed,
	            float *s_kp,float h_kp,float *s_kd,float h_kd,float hold_kd,float *s_t,float h_t,float hold_t,float ignore_diff)
{
  if(c_position < t_position - ignore_diff)
 {
   *s_kp = 0.0f;
	 *sent_sp = (u_speed + exceed_num);
	 *s_kd = h_kd;
	 *s_t = h_t;
 }
 if(c_position > t_position + ignore_diff)
 {
   *s_kp = 0.0f;
	 *sent_sp = -(u_speed + exceed_num);
	 *s_kd = h_kd; 
	 *s_t = hold_t - (h_t - hold_t);
 }
 if(t_position - ignore_diff <= c_position && c_position <= t_position + ignore_diff)
 {
	 *sent_p = t_position;
   *s_kp = h_kp;
	 *s_kd = hold_kd; 
	 *s_t = hold_t; 
 }
}         //输入参数需均为正数，下同


void in_place_1(float t_position,float *sent_p,float c_position,float *sent_sp,float u_speed,
	              float *s_kp,float h_kp,float *s_kd,float h_kd,float hold_kd,float *s_t,float h_t,float hold_t,float ignore_diff)
{
  if(c_position < t_position - ignore_diff)
 {
   *s_kp = 0.0f;
	 *sent_sp = -1.5*u_speed ;
	 *s_kd = h_kd;
	 *s_t = 1.5*h_t;
 }
 if(c_position > t_position + ignore_diff)
 {
   *s_kp = 0.0f;
	 *sent_sp = -u_speed;
	 *s_kd = h_kd; 
	 *s_t = hold_t - (h_t - hold_t);
 }
 if(t_position - ignore_diff <= c_position && c_position <= t_position + ignore_diff)
 {
	 *sent_p = t_position;
   *s_kp = h_kp;
	 *s_kd = hold_kd; 
	 *s_t = hold_t; 
 }
}

//prior_sp预先减速速度，prior_diff预减速死区
void in_place_fast(float t_position,float *sent_p,float c_position,float *sent_sp,float u_speed,float prior_sp,
	         float *s_kp,float h_kp,float *s_kd,float h_kd,float hold_kd,float *s_t,float h_t,float hold_t,float ignore_diff,float prior_diff)
{
	if(c_position < t_position - prior_diff)
 {
   *s_kp = 0.0f;
	 *sent_sp = (u_speed + exceed_num);
	 *s_kd = h_kd;
	 *s_t = h_t;
 }
 if(c_position > t_position + prior_diff)
 {
   *s_kp = 0.0f;
	 *sent_sp = -(u_speed + exceed_num);
	 *s_kd = h_kd; 
	 *s_t = hold_t - (h_t - hold_t);
 }
 
  if(c_position < t_position - ignore_diff && c_position > t_position - prior_diff)
 {
   *s_kp = 0.0f;
	 *sent_sp = prior_sp;
	 *s_kd = h_kd;
	 *s_t = h_t;
 }
 if(c_position > t_position + ignore_diff && c_position < t_position + prior_diff)
 {
   *s_kp = 0.0f;
	 *sent_sp = -prior_sp;
	 *s_kd = h_kd; 
	 *s_t = hold_t - (h_t - hold_t);
 }

 if(t_position - ignore_diff <= c_position && c_position <= t_position + ignore_diff)
 {
	 *sent_p = t_position;
	 *sent_sp = 0.0f;
   *s_kp = h_kp;
	 *s_kd = hold_kd; 
	 *s_t = hold_t; 
 }
}



float v_diff = 0.0f;    //速度差
float v_diff_last = 0.0f;
float diff_i = 0.0f;   //积分项
float i_range = 100.0f;  //积分限幅
float v_amp = 0.0f;     //amplitude,变化幅度，变化率
float vpid[3] = {1.0f,0.02f,0.2f};//kp,ki,kd参数

//速度环pid同步函数

void hold_step(float motor_1v,float motor_2v,float *T)
{
  v_diff = motor_1v - motor_2v;
	
 if(-i_range < diff_i && diff_i <i_range)
 {
	diff_i += v_diff;
 }
 
	v_amp = v_diff - v_diff_last;
 
 *T += vpid[0] * v_diff + vpid[1] * diff_i + vpid[2] * v_amp;
 v_diff_last = v_diff;

}

//位置环pid同步函数
float position_origin1;
float position_origin2;   //电机位置初始值

float p_diff = 0.0f;
float p_diff_last = 0.0f;
float p_diff_i = 0.0f;
float p_amp = 0.0f;
float ppid[3] = {3.0f,0.04f,0.4f};

void side_by_side(float motor_1p,float motor_2p,float *T)
{
  float p1 = motor_1p - position_origin1;
	float p2 = motor_2p - position_origin2;
	
	p_diff = p1 - p2;
	
	if(-i_range < p_diff_i && p_diff_i < i_range)
	{
	  p_diff_i += p_diff;
	}
	
	p_amp = p_diff - p_diff_last;
	
	*T += ppid[0] * p_diff + ppid[1] * p_diff_i + ppid[2] * p_amp;
	p_diff_last = p_diff;
}

//T加到2号电机上



