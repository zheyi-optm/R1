#include "remote_control.h"
#include <math.h>


//uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM];
//uint8_t REMOTE_BUF[9];
uint8_t REMOTE_BUF[REMOTE_BUF_SIZE];

#define ABS(x)  ((x) >= 0? (x) : -(x))

float data_convert(int src, int src_min, int src_max, float dst_low, float dst_high)
{
    float ratio = 0.0f;
    float dst_float = 0.0f;
    float res = 0;
    int s_low = src_min;
    int s_high = src_max;
    if (s_low > s_high) {
        int temp = s_low;
        s_low = s_high;
        s_high = temp;
    }

    if (s_high == s_low) {
        res = (int16_t)round(dst_low);
        return res;
    }
    int clamped_src = src;
    if (clamped_src < s_low) clamped_src = s_low;
    if (clamped_src > s_high) clamped_src = s_high;
    ratio = (float)(clamped_src - s_low) / (float)(s_high - s_low);
    dst_float = ratio * (dst_high - dst_low) + dst_low;

    return dst_float;
}

Remote_Info_Typedef_New RCctrl={
	.online_cnt = 0xFAU,
	.rc_lost = true,
};

/**
* @brief SBUS数据协议解析
* @param sbus_buf：接收缓冲数组
* @param Remote_Ctrl: 遥控器数据结构体
* @date&author  2025/12/25  zhouxy
*/
//void SBUS_TO_RC(volatile const uint8_t *sbus_buf, Remote_Info_Typedef  *Remote_Ctrl)
//{
//    if (sbus_buf == NULL || Remote_Ctrl == NULL) return;

//    /* Channel 0, 1, 2, 3 */
//	Remote_Ctrl->CH1 = ((((uint16_t)sbus_buf[2] << 8) | (uint16_t)sbus_buf[1]) & 0x07FF);
//	Remote_Ctrl->CH2 = ((int16_t)sbus_buf[ 2] >> 3 | ((int16_t)sbus_buf[ 3] << 5 )) & 0x07FF;
//	Remote_Ctrl->CH3 = ((int16_t)sbus_buf[ 3] >> 6 | ((int16_t)sbus_buf[ 4] << 2 ) | (int16_t)sbus_buf[ 5] << 10 ) & 0x07FF;
//    Remote_Ctrl->CH4 = ((int16_t)sbus_buf[ 5] >> 1 | ((int16_t)sbus_buf[ 6] << 7 )) & 0x07FF;
//	Remote_Ctrl->CH5 = ((int16_t)sbus_buf[ 6] >> 4 | ((int16_t)sbus_buf[ 7] << 4 )) & 0x07FF;
//	Remote_Ctrl->CH6 = ((int16_t)sbus_buf[ 7] >> 7 | ((int16_t)sbus_buf[ 8] << 1 ) | (int16_t)sbus_buf[9] << 9 ) & 0x07FF;
//	Remote_Ctrl->CH7 = ((int16_t)sbus_buf[ 9] >> 2 | ((int16_t)sbus_buf[ 10] << 6)) & 0x07FF;
//	Remote_Ctrl->CH8 = ((int16_t)sbus_buf[10] >> 5 | ((int16_t)sbus_buf[ 11] << 3)) & 0x07FF;
//	Remote_Ctrl->CH9 = ((int16_t)sbus_buf[12] << 0 | ((int16_t)sbus_buf[13] << 8 )) & 0x07FF;
//	Remote_Ctrl->CH10 = ((int16_t)sbus_buf[13] >> 3 | ((int16_t)sbus_buf[14] << 5 )) & 0x07FF;
//	Remote_Ctrl->CH11 = ((int16_t)sbus_buf[14] >> 6 | ((int16_t)sbus_buf[15] << 2 ) | (int16_t)sbus_buf[16] << 10 ) & 0x07FF;
//	Remote_Ctrl->CH12 = ((int16_t)sbus_buf[16] >> 1 | ((int16_t)sbus_buf[17] << 7 )) & 0x07FF;
//    Remote_Ctrl->CH13 = ((int16_t)sbus_buf[17] >> 4 | ((int16_t)sbus_buf[18] << 4 )) & 0x07FF;
//    Remote_Ctrl->CH14 = ((int16_t)sbus_buf[18] >> 7 | ((int16_t)sbus_buf[19] << 1 ) | (int16_t)sbus_buf[20] << 9  ) & 0x07FF;
//    Remote_Ctrl->CH15 = ((int16_t)sbus_buf[20] >> 2 | ((int16_t)sbus_buf[21] << 6 )) & 0x07FF;
//    Remote_Ctrl->CH16 = ((int16_t)sbus_buf[21] >> 5 | ((int16_t)sbus_buf[22] << 3 )) & 0x07FF;
//    

//    (sbus_buf[23] == 0x00) ? (Remote_Ctrl->rc_lost = false) : (Remote_Ctrl->rc_lost = true);
//		/* reset the online count */
//		Remote_Ctrl->online_cnt = 0xFAU;
//		
//		/* reset the lost flag */
////		Remote_Ctrl->rc_lost = false;
//}

/**
* @brief 平板遥控器的数据协议解析
* @param remote_buf：接收缓冲数组
* @param rc: 遥控器数据结构体
* @date&author  wuzhuohan
*/
void REMOTE_ParseData(volatile const uint8_t *remote_buf, Remote_Info_Typedef_New  *rc)
{
    // 帧头帧尾校验
    if (remote_buf[0] == NULL || remote_buf[1] == NULL || remote_buf[8] == NULL)
    {
			RCctrl.rc_lost = true;
        return;
    }
		if (remote_buf[0]== 0x5B && remote_buf[1]==0x5B && remote_buf[8]==0x2B) //控制指令
  {
    rc->frame_head1=remote_buf[0];
    rc->frame_head2=remote_buf[1];
    rc->frame_tail =remote_buf[8];
    
    rc->key        = (remote_buf[2] >> 4) & 0x0F;  //
    rc->chassis    = remote_buf[2] & 0x01;        
    rc->channel    = (remote_buf[2] >> 1) & 0x01; 
    rc->zone       = (remote_buf[2] >> 2) & 0x03;       
    
    rc->pump       = remote_buf[3] & 0x01;   
    rc->kfs_catch  = (remote_buf[3] >> 1) & 0x01;   
    rc->hold       = (remote_buf[3] >> 2) & 0x01;  
    rc->led_1      = (remote_buf[3] >> 3) & 0x01;         
    rc->led_2      = (remote_buf[3] >> 4) & 0x01;
    rc->led_3      = (remote_buf[3] >> 5) & 0x01;
    rc->switch_dir = (remote_buf[3] >> 6) & 0x01;
    rc->lock       = (remote_buf[3] >> 7) & 0x01;
    
    rc->accel      = (int8_t)remote_buf[4] + 100;
    rc->vw         = (int8_t)remote_buf[5];
    rc->vy         = (int8_t)remote_buf[6];
    rc->vx         = (int8_t)remote_buf[7];
		
		if(rc->chassis == 1 && rc->zone ==0)//武器模式附加解算
   {
	   rc->rodPos = (remote_buf[2] >> 4) & 0x01;
		 rc->liftPos = (remote_buf[2] >> 5) & 0x03;
		 rc->takePos = (remote_buf[2] >> 7) & 0x01;
	 }
	 if(rc->chassis == 1 && rc->zone ==2)//抬升模式附加解算
   {
	   rc->bottomPos = (remote_buf[2] >> 4) & 0x03;
		 rc->topPos = (remote_buf[2] >> 6) & 0x03;
	 }
		
		
		RCctrl.rc_lost = false;
	}
	
	else if (remote_buf[0]== 0x5C && remote_buf[1]==0x5C && remote_buf[8]==0x2C) //地图通信指令 
  {
    parse_config(rc,remote_buf);
		RCctrl.rc_lost = false;
	}
}

// 配置帧接收
void parse_config(Remote_Info_Typedef_New *rc, volatile const uint8_t *b) 
{
    for (int i = 0; i < 12; i++) {
        int byte_idx = i / 4;
        int shift    = (3 - (i % 4)) * 2;
        rc->cells[i] = (b[2 + byte_idx] >> shift) & 0x03;
    }
		//数据拉取
	  Check_Cells_Status(&RCctrl, kfs1, kfs2, &kf);
		rc->received = true;
}


/**
* @brief kfs1,kfs2,g_kf赋值函数
* @param remote_buf：接收缓冲数组
* @param rc: 遥控器数据结构体
* @date&author  wuzhuohan
*/
void Check_Cells_Status(Remote_Info_Typedef_New  *rc, int *kfs1, int *kfs2, int *g_kf)
{
    uint8_t cnt1 = 0;  // kfs1 计数
    uint8_t cnt2 = 0;  // kfs2 计数
    *g_kf = 0;         // 初始化为0
    
    for(int i = 0; i < 12; i++)
    {
        if(rc->cells[i] == 1)
        {
            kfs1[cnt1++] = i + 1;
        }
        else if(rc->cells[i] == 2)
        {
            kfs2[cnt2++] = i + 1;
        }
        else if(rc->cells[i] == 3)
        {
            *g_kf = i + 1;
        }
    }
}

/**
* @brief 遥控通道加死区占比解析
* @param sbus_buf：接收缓冲数组
* @param Remote_Ctrl: 遥控器数据结构体
* @date&author  2026/3/28  zheyi
*/

//中心映射函数，容许非对称中心映射。比例映射
float remote_control_read(int src, int src_low, int src_mid, int src_high, float dst_min, float dst_mid, float dst_max)
{
  int ac_num = src;
	int s_low = src_low;
	int s_mid = src_mid;
	int s_high = src_high;
	float d_min = dst_min;
	float d_mid = dst_mid;
	float d_max = dst_max;//拉取
	
	float persent = 0.0f;
	float res = 0.0f;
	
	
	if(s_low > s_high)
   {
	  int temp = s_low;
		 s_low = s_high;
		 s_high = temp;
	 }
	 if(d_min > d_max)//正负检查
   {
	   int tep = d_min;
		 d_min = d_max;
		 d_max = tep;
	 }                  
	 if(ac_num > s_high){ac_num = s_high;}
	 if(ac_num < s_low){ac_num = s_low;}//范围检查，置换
	 
	 if(s_low<s_mid && s_high>s_mid && d_min<=d_mid && d_max>=d_mid)
{
	if(ac_num < s_mid + DEADZONE && ac_num > s_mid - DEADZONE)
  {
	  persent = 0;
		res = 0;
	}
  else if(ac_num > s_mid + DEADZONE)
  {
	  persent = (float)(ac_num - (s_mid + DEADZONE))/(float)(s_high - (s_mid + DEADZONE));
		res = persent * (d_max - d_mid);
	}
	else if(ac_num < s_mid - DEADZONE)
  {
	  persent = - (float)((s_mid - DEADZONE)- ac_num)/(float)((s_mid - DEADZONE) - s_low);
		res = persent * (d_mid - d_min);
	}
    
}
else
{
   res = 0;
}
   return res;
}

/**
* @brief 遥控通道油门占比解析
* @param sbus_buf：接收缓冲数组
* @param Remote_Ctrl: 遥控器数据结构体
* @date&author  2026/3/28  zheyi
*/
float remote_control_readaccel(int src, int src_low, int src_high, float dst_min, float dst_max)
{
  int ac_num = src;
	int s_low = src_low;
	int s_high = src_high;
	float d_min = dst_min;
	float d_max = dst_max;//拉取
	
	float persent = 0.0f;
	float res = 0.0f;
	
	
	if(s_low > s_high)
   {
	  int temp = s_low;
		 s_low = s_high;
		 s_high = temp;
	 }
	 if(d_min > d_max)//正负检查
   {
	   int tep = d_min;
		 d_min = d_max;
		 d_max = tep;
	 }                  
	 if(ac_num > s_high){ac_num = s_high;}
	 if(ac_num < s_low){ac_num = s_low;}//范围检查，置换
	 
	 persent = (float)(ac_num - (s_low + DEADZONE))/(float)(s_high - (s_low + DEADZONE));
	 res = persent * (d_max - d_min) + d_min;
	 if(res < d_min){res = d_min;}
	 
   return res;
}



