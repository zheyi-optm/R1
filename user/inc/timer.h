#ifndef __TIMER_H__
#define __TIMER_H__

#include "main.h"
#include "remote_control.h"

typedef struct 
{
 uint8_t wep_change;
 uint8_t button_new;
 uint8_t sent_count;
	
}BUT_FLAG;

extern BUT_FLAG BUT_flag;

extern void BUT_check(void);

//typedef struct 
//{
// uint8_t C8F_chalok_calc;
// uint8_t C8F_chassis_lock;//底盘锁定标志位
// uint8_t C8F_R2_lift;
// uint8_t C8F_attack_lift;
// uint8_t C8F_ac_lift;
// uint8_t C1F_catch_flag;
// uint8_t C8F_hold_flag;
// uint8_t C8F_gas;
// uint8_t C2F_kfs3collect;
// uint8_t C2F_kfs_keep;
// uint8_t C2F_kfs_put;
// uint8_t C2F_weapon_collect;
// uint8_t C2F_weapon_change;
//	
//}CH_FLAG;

//extern CH_FLAG CH_flag;

//extern uint8_t ch2_new;

//extern void CH8_chassis_lock(void);
//extern void CH8_flag_change(void);
//extern void CH2_flag_change(void);
//extern void CH1_flag_change(void);

#endif
