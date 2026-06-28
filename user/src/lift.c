#include "lift.h"
#include "remote_control.h"
#include "main.h"
#include "tim.h"
#include "cmsis_os.h"
#include <math.h>
#include "timer.h"
#include "motor_control.h"
#include "weapon.h"

Lift_Module Lift;
DM_MotorModule R2_lift_motor_left;
DM_MotorModule R2_lift_motor_right;


float lift_left[5] = {0,0,0,0,0};
float lift_right[5] = {0,0,0,0,0};

void lift_mode()
{
  if(RCctrl.topPos == 0)  //r2Ì§Éý´¥·¢
 {
   in_place_fast(18.0f,&lift_left[0],R2_lift_motor_left.position,&lift_left[1],3.0f,3.0f,&lift_left[2],15.0f,&lift_left[3],5.0f,2.0f,&lift_left[4],6.4f,5.7f,0.30f,1.2f);
	 in_place_fast(18.0f,&lift_right[0],R2_lift_motor_right.position,&lift_right[1],3.0f,3.0f,&lift_right[2],15.0f,&lift_right[3],5.0f,2.0f,&lift_right[4],6.0f,5.5f,0.30f,1.2f);
	 hold_step(R2_lift_motor_left.speed_w,R2_lift_motor_right.speed_w,&lift_right[4]);
	 side_by_side(R2_lift_motor_left.position,R2_lift_motor_right.position,&lift_right[4]);
 }
 else if(RCctrl.topPos == 1)  //r2Ì§Éý·Ç´¥·¢
 {
   lift_left[0] = 0.0f;
	 lift_left[1] = -1.0f;
	 lift_left[2] = 0.0f;
	 lift_left[3] = 3.5f;
	 lift_left[4] = 0.0f;
	 
	 lift_right[0] = 0.0f;
	 lift_right[1] = -1.0f;
	 lift_right[2] = 0.0f;
	 lift_right[3] = 3.46f;
	 lift_right[4] = 0.0f;     //´óDÖµ×ÔÂä
	 hold_step(R2_lift_motor_left.speed_w,R2_lift_motor_right.speed_w,&lift_right[4]);
	 side_by_side(R2_lift_motor_left.position,R2_lift_motor_right.position,&lift_right[4]);
 }

 else if(RCctrl.topPos == 2)  //¹¥»÷Ì§Éý·Ç´¥·¢
 {
   in_place_fast(0.2f,&lift_left[0],R2_lift_motor_left.position,&lift_left[1],18.0f,8.0f,&lift_left[2],0.0f,&lift_left[3],2.0f,0.2f,&lift_left[4],1.0f,0.0f,0.50f,1.2f);
	 in_place_fast(0.2f,&lift_right[0],R2_lift_motor_right.position,&lift_right[1],18.0f,8.0f,&lift_right[2],0.0f,&lift_right[3],2.0f,0.2f,&lift_right[4],1.0f,0.0f,0.50f,1.2f);
	 hold_step(R2_lift_motor_left.speed_w,R2_lift_motor_right.speed_w,&lift_right[4]);
	 side_by_side(R2_lift_motor_left.position,R2_lift_motor_right.position,&lift_right[4]);
 }
 else if(RCctrl.topPos == 3)  //¹¥»÷Ì§Éý´¥·¢
 {
   in_place_fast(12.0f,&lift_left[0],R2_lift_motor_left.position,&lift_left[1],18.0f,8.0f,&lift_left[2],4.0f,&lift_left[3],2.0f,0.5f,&lift_left[4],1.0f,0.05f,0.24f,1.2f);
	 in_place_fast(12.0f,&lift_right[0],R2_lift_motor_right.position,&lift_right[1],18.0f,8.0f,&lift_right[2],4.0f,&lift_right[3],2.0f,0.5f,&lift_right[4],1.0f,0.05f,0.24f,1.2f);
	 hold_step(R2_lift_motor_left.speed_w,R2_lift_motor_right.speed_w,&lift_right[4]);
	 side_by_side(R2_lift_motor_left.position,R2_lift_motor_right.position,&lift_right[4]);
 }
 
   weapon_joint_input = 600.0f * SPEED_X;//¹¥»÷2006³£×¤
 
}




