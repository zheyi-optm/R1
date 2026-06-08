#include "cowork.h"

bool sent_flag = 0;           //发送准许标志位

uint8_t uart8_tx_buf[7];

void uart8_tx_init()
{
	memset(uart8_tx_buf, 0x00, 7);

  uart8_tx_buf[0] = 0xAA;  // 帧头
  uart8_tx_buf[6] = 0xBB;  // 帧尾
	
	 uint8_t path_data[7] = {0};
    for (int i = 0; i < 7; i++)
    {
        if (i < path_len)
        {
            path_data[i] = path[i]; // 填入有效路径
        }
        // 无路径则保持0
    }

		uint8_t k2_data[3] = {0};
    for (int i = 0; i < 3; i++)
    {
        if (i < picked_cnt)
        {
            k2_data[i] = picked_k2[i]; // 填入有效K2
        }
        // 无K2则保持0
    }
		
    uart8_tx_buf[1] = combine_4bit(path_data[0], path_data[1]);
    uart8_tx_buf[2] = combine_4bit(path_data[2], path_data[3]);
    uart8_tx_buf[3] = combine_4bit(path_data[4], path_data[5]);
    
    uart8_tx_buf[4] = combine_4bit(path_data[6], k2_data[0]); // 低4bit=K2_1
    uart8_tx_buf[5] = combine_4bit(k2_data[1], k2_data[2]);  // 高4bit=K2_2，低4bit=K2_3
	
	
}

static uint8_t combine_4bit(uint8_t high_4bit, uint8_t low_4bit)//位合并函数
{
    high_4bit &= 0x0F;
    low_4bit  &= 0x0F;
    // 高4位左移4位 + 低4位
    return (high_4bit << 4) | low_4bit;
}

/**
* @brief 路径镜像函数
* @param buf:需要被镜像的数组
* @date&author  wuzhuohan
*/
 void SwapNumArray(int *buf)
{
    uint16_t len = sizeof(buf) / sizeof(buf[0]);
    
    for(uint16_t i = 0; i < len; i++)
    {
        switch(buf[i])
        {
            case 1:  buf[i] = 3; break;
            case 3:  buf[i] = 1; break;
            case 4:  buf[i] = 6; break;
            case 6:  buf[i] = 4; break;
            case 7:  buf[i] = 9; break;
            case 9:  buf[i] = 7; break;
            case 10: buf[i] = 12; break;
            case 12: buf[i] = 10; break;
            default: break; 
        }
    }
}

void sense()
{
	if(RCctrl.switch_dir == 0)
 {
   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
 }
 	else if(RCctrl.switch_dir == 1)
 {
   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
	 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);
 }
  if(RCctrl.led_1 == 0)
 {
   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_RESET);
 }
  else if(RCctrl.led_1 == 1)
 {
   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_SET);
 }
 if(RCctrl.led_2 == 0)
 {
   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_RESET);
 }
 else if(RCctrl.led_2 == 1)
 {
   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_SET);
 }
}



