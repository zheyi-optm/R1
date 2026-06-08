#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#include "main.h"

extern void in_place(float t_position,float *sent_p,float c_position,float *sent_sp,float u_speed,
	                   float *s_kp,float h_kp,float *s_kd,float h_kd,float hold_kd,float *s_t,float h_t,float hold_t,float ignore_diff);
extern void in_place_1(float t_position,float *sent_p,float c_position,float *sent_sp,float u_speed,
	                     float *s_kp,float h_kp,float *s_kd,float h_kd,float hold_kd,float *s_t,float h_t,float hold_t,float ignore_diff);
extern void in_place_fast(float t_position,float *sent_p,float c_position,float *sent_sp,float u_speed,float prior_sp,
	                        float *s_kp,float h_kp,float *s_kd,float h_kd,float hold_kd,float *s_t,float h_t,float hold_t,float ignore_diff,float prior_diff);
extern void hold_step(float motor_1v,float motor_2v,float *T);
extern void side_by_side(float motor_1p,float motor_2p,float *T);

extern float position_origin1;
extern float position_origin2;

#endif

