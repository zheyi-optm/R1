#ifndef __AUTO_MODE_H__
#define __AUTO_MODE_H__

#include "main.h"
#include "math.h"
#include "chassis.h"
#include "lift.h"
#include "kfs.h"
#include "weapon.h"
#include "motor_control.h"
#include "arm.h"
#include "policy.h"


extern float mid_x;  //mid360Êý¾Ý
extern float mid_y;
extern float mid_yaw;

extern float diff_x;
extern float diff_y;
extern float diff_w;

extern void auto_mission();
extern void auto_second_inplace();
extern void auto_inplace(float target_x,float target_y,float target_w);
extern float auto_pid(float diff);
extern float auto_set(float src, float src_low, float src_high, float dst_min, float dst_max, float deadzone);

#endif


