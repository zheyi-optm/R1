#include "kfs.h"
#include "remote_control.h"
#include "main.h"
#include "tim.h"
#include <math.h>
#include "cmsis_os.h"
#include "timer.h"
#include "motor_control.h"

Kfs_Module  Kfs;
DM_MotorModule kfs_arm_1;  //´ó±Û
DM_MotorModule kfs_arm_2;  //Ð¡±Û
DM_MotorModule kfs_arm_3;  //Ä©¶Ë
DM_MotorModule kfs_catch;  //¼Ð×¦2325



void kfs_mode()
{
	  if(RCctrl.pump ==0)//°´Å¥´¥·¢Æø±Ã
    {
		  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_RESET);
		}
		else if(RCctrl.pump ==1)
    {
		  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_SET);
		}
}



float k_catch[5] = {0,0,0,0,0};

void kfs_catch_mode()//¼Ð×¦¿ØÖÆº¯Êý
{
  if(RCctrl.kfs_catch ==0)  //°´Å¥´¥·¢¼Ð×¦
  {
	  k_catch[0] = 35.0f;
		k_catch[1] = 0.0f;
		k_catch[2] = 1.8f;
		k_catch[3] = 1.0f;
		k_catch[4] = 0.0f;	
	}
	else if(RCctrl.kfs_catch ==1)  //°´Å¥´¥·¢¼Ð×¦
  {
		if(kfs_catch.position > 30.0f)
   {
	   k_catch[0] = 0.0f;
		 k_catch[1] = -5.0f;
		 k_catch[2] = 0.0f;
		 k_catch[3] = 0.4f;
		 k_catch[4] = 0.0f;
	 }
	 else if(kfs_catch.position < 30.0f && kfs_catch.position > 5.0f)
   {
	   k_catch[0] = 0.0f;
		 k_catch[1] = -(((float)kfs_catch.position -5.0f)/(25.0f)*4.0f + 1.0f);
		 k_catch[2] = 0.0f;
		 k_catch[3] = 1.0f/(-k_catch[1]);
		 k_catch[4] = 0.0f;
	 }
	 else if(kfs_catch.position < 5.0f)
   {
	   k_catch[0] = 0.0f;
		 k_catch[1] = 0.0f;
	   k_catch[2] = 0.0f;
	 	 k_catch[3] = 0.0f;
		 k_catch[4] = -1.0f;
	 }
	}
	
}



