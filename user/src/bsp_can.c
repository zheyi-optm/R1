#include "bsp_can.h"

#include "dji_motor.h"
#include "dm_motor.h"
#include "chassis.h"
#include "kfs.h"
#include "lift.h"
#include "weapon.h"

//FDCAN_FilterTypeDef  can1_filter;     //can1滤波器
//FDCAN_FilterTypeDef  can2_filter;     //can2滤波器

//	HAL_StatusTypeDef flag1=HAL_ERROR;
//	HAL_StatusTypeDef flag2=HAL_ERROR;

FDCAN_RxFrame_TypeDef FDCAN1_RxFrame;
FDCAN_RxFrame_TypeDef FDCAN2_RxFrame;


FDCAN_TxFrame_TypeDef SteerTxFrame = {
  .hcan = &hfdcan2,
  .Header.IdType = FDCAN_STANDARD_ID, 
  .Header.TxFrameType = FDCAN_DATA_FRAME,
  .Header.DataLength = 8,
	.Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE,
  .Header.BitRateSwitch = FDCAN_BRS_OFF,
  .Header.FDFormat =  FDCAN_CLASSIC_CAN,           
  .Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS,  
  .Header.MessageMarker = 0,
};
FDCAN_TxFrame_TypeDef WheelTxFrame = {
  .hcan = &hfdcan1,
  .Header.IdType = FDCAN_STANDARD_ID, 
  .Header.TxFrameType = FDCAN_DATA_FRAME,
  .Header.DataLength = 8,
	.Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE,
  .Header.BitRateSwitch = FDCAN_BRS_OFF,
  .Header.FDFormat =  FDCAN_CLASSIC_CAN,           
  .Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS,  
  .Header.MessageMarker = 0,
};

/**
* @brief can通信滤波器配置
* @param hcanx：can句柄
* @param can_filter: 滤波器句柄
* @date&author  2025/12/24  zhouxy
*/
//static void Can_Filter_Config(CAN_HandleTypeDef hcanx, CAN_FilterTypeDef *can_filter)
//{
//	                      // filter 0
//    can_filter->FilterMode =  CAN_FILTERMODE_IDMASK;  // mask mode
//    can_filter->FilterScale = CAN_FILTERSCALE_32BIT;
//    can_filter->FilterIdHigh = 0;
//    can_filter->FilterIdLow  = 0;
//    can_filter->FilterMaskIdHigh = 0;
//    can_filter->FilterMaskIdLow  = 0;                // set mask 0 to receive all can id
//    can_filter->FilterFIFOAssignment = CAN_RX_FIFO0; // assign to fifo0
//    can_filter->FilterActivation = ENABLE;           // enable can filter
//	if(&hcanx == &hcan1)
//    {
//        can_filter->FilterBank = 0; 
//        can_filter->SlaveStartFilterBank  = 14;
//    }
//    if(&hcanx == &hcan2)
//    {
//        can_filter->FilterBank = 14; 
//    }
//    
//	HAL_CAN_ConfigFilter(&hcanx, can_filter);
//	HAL_CAN_Start(&hcanx);
//  HAL_CAN_ActivateNotification(&hcanx, CAN_IT_RX_FIFO0_MSG_PENDING);

//}


/**
* @brief can初始化
* @date&author  2025/12/24  zhouxy
*/

void BSP_CAN_Init(void)
{

  FDCAN_FilterTypeDef FDCAN1_FilterConfig;
	
	FDCAN1_FilterConfig.IdType = FDCAN_STANDARD_ID;
  FDCAN1_FilterConfig.FilterIndex = 0;
  FDCAN1_FilterConfig.FilterType = FDCAN_FILTER_MASK;
  FDCAN1_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  FDCAN1_FilterConfig.FilterID1 = 0x00000000; 
  FDCAN1_FilterConfig.FilterID2 = 0x00000000; 

  
	if(HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN1_FilterConfig) != HAL_OK)
	{
		Error_Handler();
	}
		
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
 
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  
	
	 FDCAN_FilterTypeDef FDCAN2_FilterConfig;
	
	FDCAN2_FilterConfig.IdType = FDCAN_STANDARD_ID;
  FDCAN2_FilterConfig.FilterIndex = 0;
  FDCAN2_FilterConfig.FilterType = FDCAN_FILTER_MASK;
  FDCAN2_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;//MX配置的是FIFO0，待议
  FDCAN2_FilterConfig.FilterID1 = 0x00000000; 
  FDCAN2_FilterConfig.FilterID2 = 0x00000000; 

  
	if(HAL_FDCAN_ConfigFilter(&hfdcan2, &FDCAN2_FilterConfig) != HAL_OK)
	{
		Error_Handler();
	}
		
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
 
  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }

FDCAN_FilterTypeDef FDCAN3_FilterConfig;//FDCAN3预配置，可行性待议————
	
	FDCAN3_FilterConfig.IdType = FDCAN_STANDARD_ID;
  FDCAN3_FilterConfig.FilterIndex = 0;
  FDCAN3_FilterConfig.FilterType = FDCAN_FILTER_MASK;
  FDCAN3_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;//MX配置的是FIFO0，待议
  FDCAN3_FilterConfig.FilterID1 = 0x00000000; 
  FDCAN3_FilterConfig.FilterID2 = 0x00000000; 

  
	if(HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN3_FilterConfig) != HAL_OK)
	{
		Error_Handler();
	}
		
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
 
  if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK)
  {
    Error_Handler();
  }
                       //有修改
}


//uint32_t a;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{ 
  
	FDCAN_RxHeaderTypeDef rx_header;
	uint8_t             rx_data[8];
    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
//     a =HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0);
    if(hfdcan->Instance == FDCAN1)
	{
        switch (rx_header.Identifier){
         
			
				}
			
		
        switch(rx_data[0] & 0x0F)
        {
            case CHASSIS_ANGLE1_FEEDBACK_ID:
                DMget_motor_measure(&chassis_angle1,rx_data);
                break;
            case CHASSIS_ANGLE2_FEEDBACK_ID:
                DMget_motor_measure(&chassis_angle2,rx_data);
                break;
            case CHASSIS_ANGLE3_FEEDBACK_ID:
                DMget_motor_measure(&chassis_angle3,rx_data);
                break;
            case CHASSIS_ANGLE4_FEEDBACK_ID:
                DMget_motor_measure(&chassis_angle4,rx_data);
                break;
						case R2_LIFT_MOTOR_LEFT_FEEDBACK_ID:{
								DMget_motor_measure(&R2_lift_motor_left,rx_data);
								break;
						}
            case R2_LIFT_MOTOR_RIGHT_FEEDBACK_ID:{
                DMget_motor_measure(&R2_lift_motor_right,rx_data);
                break;
        }
    }
	}
    else if(hfdcan->Instance == FDCAN2)
    {
        switch (rx_header.Identifier){
					 case CHASSIS_MOTOR1_FEEDBACK_ID:{
				DJIget_motor_measure(&chassis_motor1,rx_data);
				break;
			}
			case CHASSIS_MOTOR2_FEEDBACK_ID:{
				DJIget_motor_measure(&chassis_motor2,rx_data);
				break;
			}
            case CHASSIS_MOTOR3_FEEDBACK_ID:{
				DJIget_motor_measure(&chassis_motor3,rx_data);
				break;
			}
			case CHASSIS_MOTOR4_FEEDBACK_ID:{
				DJIget_motor_measure(&chassis_motor4,rx_data);
				break;
			}
            case WEAPON_JOINT_MOTOR_FEEDBACK_ID:{
								DJIget_motor_measure(&weapon_joint_motor,rx_data);
								break;
						}
//            case BALANCE_MOTOR_LEFT_FEEDBACK_ID:{
//                DJIget_motor_measure(&balance_motor_left,rx_data);
//                break;
//						}
//            case BALANCE_MOTOR_RIGHT_FEEDBACK_ID:{
//                DJIget_motor_measure(&balance_motor_right,rx_data);
//                break;
//						}
        }
        switch(rx_data[0] & 0x0F)
        {
            case WEAPON_COLLECT_MOTOR_FEEDBACK_ID:{
                DMget_motor_measure(&weapon_collect_motor,rx_data);
                break;
						}
        }
        
    }
		  //具体待改
	    if(hfdcan->Instance == FDCAN3)
	{
        switch (rx_header.Identifier){
         
			
				}
			
		
        switch(rx_data[0] & 0x0F)
        {
            case KFS_ARM_1_FEEDBACK_ID:
                DMget_motor_measure(&kfs_arm_1,rx_data);
                break;
            case KFS_ARM_2_FEEDBACK_ID:
                DMget_motor_measure(&kfs_arm_2,rx_data);
                break;
            case KFS_ARM_3_FEEDBACK_ID:
                DMget_motor_measure(&kfs_arm_3,rx_data);
                break;
            case KFS_CATCH_FEEDBACK_ID:
                DMget_motor_measure(&kfs_catch,rx_data);
                break;

        }
    }
	}
	//FDCAN3未加，可视需求

	

	
	
//原码用于参考实际应用逻辑
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{	
//    CAN_RxHeaderTypeDef rx_header;
//	uint8_t             rx_data[8];
//    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
//    
//    if(hcan->Instance == CAN1)
//	{
//        switch (rx_header.StdId){
//            case CHASSIS_MOTOR1_FEEDBACK_ID:{
//				DJIget_motor_measure(&chassis_motor1,rx_data);
//				break;
//			}
//			case CHASSIS_MOTOR2_FEEDBACK_ID:{
//				DJIget_motor_measure(&chassis_motor2,rx_data);
//				break;
//			}
//            case CHASSIS_MOTOR3_FEEDBACK_ID:{
//				DJIget_motor_measure(&chassis_motor3,rx_data);
//				break;
//			}
//			case CHASSIS_MOTOR4_FEEDBACK_ID:{
//				DJIget_motor_measure(&chassis_motor4,rx_data);
//				break;
//			}
//			
//		}
//        switch(rx_data[0] & 0x0F)
//        {
//            case R2_LIFT_MOTOR_LEFT_FEEDBACK_ID:
//                DMget_motor_measure(&R2_lift_motor_left,rx_data);
//                break;
//            case R2_LIFT_MOTOR_RIGHT_FEEDBACK_ID:
//                DMget_motor_measure(&R2_lift_motor_right,rx_data);
//                break;
//            case KFS_LIFT_MOTOR_FEEDBACK_ID:
//                DMget_motor_measure(&kfs_lift_motor,rx_data);
//                break;
//            case KFS_FLEX_MOTOR_FEEDBACK_ID:
//                DMget_motor_measure(&kfs_flex_motor,rx_data);
//                break;
//        }
//    }
//    else if(hcan->Instance == CAN2)
//    {
//        switch (rx_header.StdId){
//            case WEAPON_JOINT_MOTOR_FEEDBACK_ID:{
//								DJIget_motor_measure(&weapon_joint_motor,rx_data);
//								break;
//						}
//            case BALANCE_MOTOR_LEFT_FEEDBACK_ID:{
//                DJIget_motor_measure(&balance_motor_left,rx_data);
//                break;
//						}
//            case BALANCE_MOTOR_RIGHT_FEEDBACK_ID:{
//                DJIget_motor_measure(&balance_motor_right,rx_data);
//                break;
//						}
//        }
//        switch(rx_data[0] & 0x0F)
//        {
//            case WEAPON_COLLECT_MOTOR_FEEDBACK_ID:{
//                DMget_motor_measure(&weapon_collect_motor,rx_data);
//                break;
//						}
//        }
//        
//    }

//}
