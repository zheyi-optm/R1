#include "timer.h"

BUT_FLAG BUT_flag =
{
  .wep_change = 0,   //换杆计数
	.button_new = 0,   //换杆新信息标志位
	.cent_count = 0,   //发送次数计数
};

uint8_t but_read[3] = {0,0,0};//LAST,MID,CURRENT

void BUT_check()
{
  but_read[0] =but_read[1];
	but_read[1] =but_read[2];
	but_read[2] =RCctrl.rodPos;
	
	if(but_read[0] == 0 && but_read[1] == 1 && but_read[2] == 1)
 {
   BUT_flag.wep_change ++;
	 BUT_flag.button_new = 1;
 }
}

//CH_FLAG CH_flag = 
//{
//	.C8F_chalok_calc = 0,  //底盘锁定计数
//  .C8F_chassis_lock = 0, //底盘锁定标志
//	.C8F_R2_lift = 0,      //R2抬升标志
//	.C8F_attack_lift = 0,  //攻击抬升标志
//	.C8F_ac_lift = 0,      //武器对接抬升标志
//	.C1F_catch_flag = 0,   //kfs爪抓取枪杆标志
//	.C8F_hold_flag = 0,    //抓持枪杆对接标志
//	.C8F_gas = 0,
//	.C2F_kfs3collect = 0,
//  .C2F_kfs_keep = 0,
//  .C2F_kfs_put = 0,
//  .C2F_weapon_collect = 0,
//  .C2F_weapon_change = 0,
//};

//uint16_t CH8_lock_read[3] = {1694,1694,1694};//LAST,MID,CURRENT
//uint16_t CH8_read[3] = {1694,1694,1694};//LAST,MID,CURRENT
//uint16_t CH2_read[3] = {1000,1000,1000};
//uint16_t CH1_read[3] = {1000,1000,1000};//读取数值

//uint8_t CH8_calc = 0;
//uint8_t ch2_new = 0;


//void CH8_flag_change()
//{
//  CH8_read[0] = CH8_read[1];
//	CH8_read[1] = CH8_read[2];
//	CH8_read[2] = RCctrl.CH8;//ch8全局读取
// 
// if (CH8_read[0] > 1500 && CH8_read[1] < 500 && CH8_read[2] < 500)
//        {
//					if(RCctrl.CH5 > 1500 && RCctrl.CH10 < 600)//抬升
//           {
//					   if(RCctrl.CH9 < 600)
//					 {
//					 		CH_flag.C8F_R2_lift ++; 
//					 }
//				   if(RCctrl.CH9 > 1200)
//           {
//				    	CH_flag.C8F_attack_lift ++; 
//           }
//					 }
//					if(RCctrl.CH5 >500 && RCctrl.CH5 <1500 && RCctrl.CH10 < 600)//KFS模式
//					{
//					  if(RCctrl.CH9 < 600)
//            {
//						  CH_flag.C8F_gas ++;
//						}
//					}
//          if(RCctrl.CH5 < 500 &&  RCctrl.CH10 < 600)//武器模式
//         {
//				   if(RCctrl.CH9 < 600)
//					 {
//					 		CH_flag.C8F_hold_flag ++; 
//					 }
//				   if(RCctrl.CH9 > 1200)
//           {
//				    	CH_flag.C8F_ac_lift ++; 
//           }
//				 }
//					 
//				 }
//}

//void CH8_chassis_lock()//双击识别
//{
//  CH8_lock_read[0] = CH8_lock_read[1];
//	CH8_lock_read[1] = CH8_lock_read[2];
//	CH8_lock_read[2] = RCctrl.CH8;
//	
//	if (CH8_lock_read[0] > 1500 && CH8_lock_read[1] < 500 && CH8_lock_read[2] < 500 && RCctrl.CH10 >1400)
//{
//	CH_flag.C8F_chalok_calc ++;

//   if(CH_flag.C8F_chalok_calc %4 ==0 | CH_flag.C8F_chalok_calc %4 ==2)
//   {
//		 if(CH8_calc < 50)
//		{
//	   CH_flag.C8F_chassis_lock ++; 
//		}
//		CH8_calc = 0;
//	 }
//}
//	  if(CH_flag.C8F_chalok_calc %4 == 1 | CH_flag.C8F_chalok_calc %4 == 3)
// {
//   CH8_calc ++;
// }

//}

//void CH2_flag_change()
//{
//  CH2_read[0] = CH2_read[1];
//	CH2_read[1] = CH2_read[2];
//	CH2_read[2] = RCctrl.CH2;//ch2全局读取

//	if(CH2_read[0] > 800 && CH2_read[1] < 800 && CH2_read[2] < 800)
//  {
//	  if(RCctrl.CH7<500 && RCctrl.CH10 <500 && RCctrl.CH5>700 && RCctrl.CH5<1200)
//   {
//	   CH_flag.C2F_kfs3collect ++;
//	 }
//	 if(RCctrl.CH7>1500 && RCctrl.CH10 <500 && RCctrl.CH5>1500)
//   {
//	   CH_flag.C2F_kfs_put ++;
//	 }
//	 if(RCctrl.CH7<500 && RCctrl.CH10 <500 && RCctrl.CH5 < 500 && RCctrl.CH9 <500)
//   {
//	   CH_flag.C2F_weapon_collect ++;
//	 }
//	 if(RCctrl.CH7>700 && RCctrl.CH7<1200 && RCctrl.CH10 <500 && RCctrl.CH5>700 && RCctrl.CH5<1200)
//   {
//	   CH_flag.C2F_kfs_keep ++;
//	 }
//	 if(RCctrl.CH7>1500 && RCctrl.CH10 <500 && RCctrl.CH5<500 && RCctrl.CH9 <500)
//   {
//	   CH_flag.C2F_weapon_change ++;
//		 ch2_new = 1;
//	 }
//	 if(RCctrl.CH7>700 && RCctrl.CH7<1200 && RCctrl.CH10 <500 && RCctrl.CH5>1500 && RCctrl.CH9 <500)
//	 {
//	   CH_flag.C8F_gas ++;
//	 }
//	}
//}

//void CH1_flag_change()
//{
//  CH1_read[0] = CH1_read[1];
//	CH1_read[1] = CH1_read[2];
//	CH1_read[2] = RCctrl.CH1;

//	if(CH1_read[0] > 800 && CH1_read[1] < 800 && CH1_read[2] < 800)
//  {
//	  if(RCctrl.CH10 < 600 &&(RCctrl.CH5 < 500 | RCctrl.CH5 > 1500))
//    {
//		  CH_flag.C1F_catch_flag ++;
//		}
//	}
//}


