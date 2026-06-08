#ifndef __REMOTE_CONTROL_H__
#define __REMOTE_CONTROL_H__

#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include <stdio.h>
#include <string.h>
#include "policy.h"

#define SBUS_RX_BUF_NUM 50u
#define RC_FRAME_LENGTH 25u
#define RC_CH_VALUE_OFFSET 1024U
#define REMOTE_BUF_SIZE   64    // 用2的幂次方便处理，必须大于9*2//
////R_HORIZONTAL
//#define CH1_LOW     1693        //LEFT        
//#define CH1_HIGH    306         //RIGHT
//#define CH1_MID     1000        //MID
////#define LR_TRANSLATION          data_convert(RCctrl.CH1, CH1_LOW, CH1_HIGH, -ACCEL, ACCEL)
////R_UPRIGHT
//#define CH2_LOW     309         //DOWN
//#define CH2_HIGH    1686        //UP
//#define CH2_MID     1000        //MID
////#define FB_TRANSLATION          data_convert(RCctrl.CH2, CH2_LOW, CH2_HIGH, -ACCEL, ACCEL)
////#define KFS_LIFT                data_convert(RCctrl.CH2, CH2_LOW, CH2_HIGH, -3, 3)
////L_UPRIGHT
//#define CH3_LOW     299         //DOWN
//#define CH3_HIGH    1659        //UP
////#define ACCEL                   data_convert(RCctrl.CH3, CH3_LOW, CH3_HIGH, 0, 100)
////L_HORIZONTAL
//#define CH4_LOW     314         //LEFT
//#define CH4_HIGH    1700        //RIGHT
//#define CH4_MID     999         //MID
////#define ROTATION                data_convert(RCctrl.CH4, CH4_LOW, CH4_HIGH, -ACCEL, ACCEL)
////CHANNAL_C
//#define CH5_LOW     306         //DOWN
//#define CH5_HIGH    1694        //UP
//#define CH5_MID     1000        //MID
////CHANNEL_D
//#define CH6_LOW     1694        //DOWN
//#define CH6_HIGH    306         //UP
////CHANNEL_G
//#define CH7_LOW     306         //BACK
//#define CH7_HIGH    1694        //FRONT
//#define CH7_MID     1000        //MID
////CHANNEL_H
//#define CH8_LOW     306         //PRESS
//#define CH8_HIGH    1694        //RELEASE
////CHANNEL_F
//#define CH9_LOW     306         //BACK
//#define CH9_HIGH    1694        //FRONT
////CHANNEL_A
//#define CH10_LOW    1692        //DOWN
//#define CH10_HIGH   292         //UP

//accel
#define ACCEL_LOW  0
#define ACCEL_HIGH  200
//vw
#define VW_LOW  -100
#define VW_MID  0
#define VW_HIGH  100
//vx
#define VX_LOW  -100
#define VX_MID  0
#define VX_HIGH  100
//vy
#define VY_LOW  -100
#define VY_MID  0
#define VY_HIGH  100

#define DEADZONE  10  //遥控死区
#define SPEED_ACCEL   remote_control_readaccel(RCctrl.accel, ACCEL_LOW, ACCEL_HIGH, 0, 40)
#define SPEED_X       remote_control_read(RCctrl.vx, VX_LOW, VX_MID, VX_HIGH, -1, 0, 1)
#define SPEED_Y       remote_control_read(RCctrl.vy, VY_LOW, VY_MID, VY_HIGH, -1, 0, 1)
#define SPEED_WO       remote_control_read(RCctrl.vw, VW_LOW, VW_MID, VW_HIGH, -1.8, 0, 1.8)


//typedef  struct
//{
//    uint16_t CH1;  
//    uint16_t CH2;  
//    uint16_t CH3;  
//    uint16_t CH4;
//    uint16_t CH5;
//    uint16_t CH6;
//    uint16_t CH7;
//    uint16_t CH8;
//    uint16_t CH9;
//    uint16_t CH10;
//	uint16_t CH11;
//	uint16_t CH12;
//    uint16_t CH13;
//    uint16_t CH14;
//    uint16_t CH15;
//    uint16_t CH16;

//	bool rc_lost;   /*!< lost flag */
//	uint8_t online_cnt;   /*!< online count */
//} Remote_Info_Typedef;

typedef  struct
{
   // 帧头、帧尾
   uint8_t frame_head1;
   uint8_t frame_head2;
   uint8_t frame_tail;
    
   uint8_t key;         //矩阵选中键 ww
	 uint8_t takePos;     //一区取放杆
	 uint8_t liftPos;     //一区抬升
	 uint8_t rodPos;      //一区换杆
	 uint8_t topPos;      //三区抬升
 	 uint8_t bottomPos;   //三区机械臂
    
   uint8_t chassis;     //底盘模式
   uint8_t channel;     //通道
   uint8_t zone;        //区域 
    
   uint8_t pump;        //气泵
   uint8_t kfs_catch;   //夹取
   uint8_t hold;         //固定
   uint8_t led_1;
   uint8_t led_2;
   uint8_t led_3;
   uint8_t switch_dir; //切换
   uint8_t lock;        //锁定
    
   uint8_t accel;    // 油门
   int8_t  vw;          // 转向
   int8_t  vy;          // 横移
   int8_t  vx;          // 前后
	 
	 uint8_t cells[12];
	 
	 bool rc_lost;
	 bool received;
	 uint16_t online_cnt;
}Remote_Info_Typedef_New;


//extern uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM];
//extern uint8_t REMOTE_BUF[9];//
extern uint8_t REMOTE_BUF[REMOTE_BUF_SIZE];
extern Remote_Info_Typedef_New RCctrl;
//void SBUS_TO_RC(volatile const uint8_t *sbus_buf, Remote_Info_Typedef  *Remote_Ctrl);
extern void REMOTE_ParseData(volatile const uint8_t *remote_buf, Remote_Info_Typedef_New  *rc);
extern void parse_config(Remote_Info_Typedef_New *rc, volatile const uint8_t *b);
extern void Check_Cells_Status(Remote_Info_Typedef_New  *rc, int *kfs1, int *kfs2, int *g_kf);

float data_convert(int src, int src_min, int src_max, float dst_low, float dst_high);
float remote_control_read(int src, int src_low, int src_mid, int src_high, float dst_min, float dst_mid, float dst_max);
float remote_control_readaccel(int src, int src_low, int src_high, float dst_min, float dst_max);

#endif
