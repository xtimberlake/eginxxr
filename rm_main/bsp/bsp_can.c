/**
	* @file bap_can.c
	* @version 1.1
	* @date Oct,19th 2018
  *
  * @brief  底层can通讯的配置，包括发送数据帧，CAN滤波组，接受中断回调函数
  *
  *	@author lin kr
  *
  */
#define __BSP_CAN_GLOBALS
#include "bsp_can.h"
#include "pid.h"
#include "bsp_JY901.h"
#include "bsp_motor.h"
#include "chassis_task.h"
#include "status_task.h"
uint32_t j;

CAN_TxHeaderTypeDef Tx1Message;
CAN_RxHeaderTypeDef Rx1Message;

CAN_TxHeaderTypeDef Tx2Message;
CAN_RxHeaderTypeDef Rx2Message;

uint8_t CAN_Rx_data[8];

/*********************************CAN信息接收**************************************/

/**
  * @brief     CAN接受中断回调函数
  * @param     CAN_Rx_data ：CAN节点反馈的数据帧
  * @attention 
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	__HAL_CAN_DISABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	if(hcan==&hcan1)
	{
		HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&Rx1Message, CAN_Rx_data);
		switch (Rx1Message.StdId)
		{
			case CAN_3508_M1_ID:
			case CAN_3508_M2_ID:
			case CAN_3508_M3_ID:
			case CAN_3508_M4_ID:
			{
				static uint8_t i;
				i = Rx1Message.StdId - CAN_3508_M1_ID;
				moto_chassis[i].msg_cnt++ <= 50 ? 
				get_moto_offset(&moto_chassis[i], &hcan1,CAN_Rx_data) : encoder_data_handler(&moto_chassis[i], &hcan1,CAN_Rx_data);
				status.chassis_status[i] = 1;
			}
			break;
			case CAN_3508_UP1_ID:
			case CAN_3508_UP2_ID:
			{
				static uint8_t i;
				i = Rx1Message.StdId - CAN_3508_UP1_ID;
				moto_uplift[i].msg_cnt++ <= 50 ? 
				get_moto_offset(&moto_uplift[i], &hcan1,CAN_Rx_data) : encoder_data_handler(&moto_uplift[i], &hcan1,CAN_Rx_data);
				status.uplift_status[i] = 1;
			}
			break;
			default: {}break;
		};
	}
	if(hcan==&hcan2)
	{
		HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&Rx2Message, CAN_Rx_data);
		switch (Rx2Message.StdId)
		{

			case CAN_GM3510_PIT_ID:
			{
				moto_pit.msg_cnt++ <= 50 ? 
				get_moto_offset(&moto_pit, &GIMBAL_CAN,CAN_Rx_data) : encoder_data_handler(&moto_pit, &GIMBAL_CAN,CAN_Rx_data);
				moto_pit.speed_rpm = moto_pit.total_ecd - moto_pit.last_total_ecd;//速度解算
				moto_pit.last_total_ecd = moto_pit.total_ecd;
			}				
			break;
			case CAN_GM3510_YAW_ID:
			{
				moto_yaw.msg_cnt++ <= 50 ? 
				get_moto_offset(&moto_yaw, &GIMBAL_CAN,CAN_Rx_data) : encoder_data_handler(&moto_yaw, &GIMBAL_CAN,CAN_Rx_data);
				moto_yaw.speed_rpm = moto_yaw.total_ecd - moto_yaw.last_total_ecd;
				moto_yaw.last_total_ecd = moto_yaw.total_ecd;
			}
			break;


			default: {}break;
		}
	}
	__HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

}


/*********************************CAN信息发送**************************************/
//CAN1 底盘电机+抬升2电机
//CAN2 GM3510
/**
  * @brief  send calculated current to motor
  * @param  3508 motor current
  */
void send_chassis_cur(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
	uint8_t FreeTxNum = 0;  
	
	Tx1Message.StdId = 0x200;
	Tx1Message.IDE 	 = CAN_ID_STD;
	Tx1Message.RTR   = CAN_RTR_DATA;
  Tx1Message.DLC   = 0x08;
	
	chassis_can_tx_data.CAN_Tx_data[0] = iq1 >> 8;
	chassis_can_tx_data.CAN_Tx_data[1] = iq1;
	chassis_can_tx_data.CAN_Tx_data[2] = iq2 >> 8;
	chassis_can_tx_data.CAN_Tx_data[3] = iq2;
	chassis_can_tx_data.CAN_Tx_data[4] = iq3 >> 8;
	chassis_can_tx_data.CAN_Tx_data[5] = iq3;
	chassis_can_tx_data.CAN_Tx_data[6] = iq4 >> 8;
	chassis_can_tx_data.CAN_Tx_data[7] = iq4;
	
	//查询发送邮箱是否为空
	FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);  
	while(FreeTxNum == 0) 
	{  
    FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);  
  }
	
	HAL_CAN_AddTxMessage(&CHASSIS_CAN, &Tx1Message,chassis_can_tx_data.CAN_Tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}

/**
  * @brief  send calculated current to motor
  * @param  3508 motor current
  */

void send_uplift_cur(int16_t up1_iq, int16_t up2_iq)
{
	uint8_t FreeTxNum = 0;  
	
	Tx1Message.StdId = 0x1ff;
	Tx1Message.IDE 	 = CAN_ID_STD;
	Tx1Message.RTR   = CAN_RTR_DATA;
  Tx1Message.DLC   = 0x08;
	
	uplift_can_tx_data.CAN_Tx_data[0] = up1_iq >> 8;
	uplift_can_tx_data.CAN_Tx_data[1] = up1_iq;
	uplift_can_tx_data.CAN_Tx_data[2] = up2_iq >> 8;
	uplift_can_tx_data.CAN_Tx_data[3] = up2_iq;
	uplift_can_tx_data.CAN_Tx_data[4] = 0;
	uplift_can_tx_data.CAN_Tx_data[5] = 0;
	uplift_can_tx_data.CAN_Tx_data[6] = 0;
	uplift_can_tx_data.CAN_Tx_data[7] = 0;
	
	//查询发送邮箱是否为空
	FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);  
	while(FreeTxNum == 0) 
	{  
    FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);  
  }
	
	HAL_CAN_AddTxMessage(&UPLIFT_CAN, &Tx1Message,uplift_can_tx_data.CAN_Tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}


/**
  * @brief  send calculated current to motor
  * @param  gm3510 motor current
  */
void send_gimbal_cur(int16_t yaw_iq,int16_t pit_iq)
{
	uint8_t FreeTxNum = 0;  
	
	Tx2Message.StdId = 0x1ff;
	Tx2Message.IDE 	 = CAN_ID_STD;
	Tx2Message.RTR   = CAN_RTR_DATA;
  Tx2Message.DLC   = 0x08;
	
	gimbal_tx_data.CAN_Tx_data[0] = yaw_iq >> 8;
	gimbal_tx_data.CAN_Tx_data[1] = yaw_iq;
	gimbal_tx_data.CAN_Tx_data[2] = pit_iq >> 8;
	gimbal_tx_data.CAN_Tx_data[3] = pit_iq ;
	gimbal_tx_data.CAN_Tx_data[4] = 0;
	gimbal_tx_data.CAN_Tx_data[5] = 0;
	gimbal_tx_data.CAN_Tx_data[6] = 0;
	gimbal_tx_data.CAN_Tx_data[7] = 0;
	
	//查询发送邮箱是否为空
	FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan2);  
	while(FreeTxNum == 0) 
	{  
    FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan2);  
  }
	
	HAL_CAN_AddTxMessage(&GIMBAL_CAN, &Tx2Message,gimbal_tx_data.CAN_Tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}


/*********************************CAN底层初始化**************************************/
/**
  * @brief   can filter initialization
  * @param   CAN_HandleTypeDef
  * @retval  None
  */
void my_can_filter_init_recv_all(void)
{
  //can1 filter config
  CAN_FilterTypeDef  can_filter;

	can_filter.FilterBank           = 0;
  can_filter.FilterMode           = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale          = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh         = 0x0000;
  can_filter.FilterIdLow          = 0x0000;
  can_filter.FilterMaskIdHigh     = 0x0000;
  can_filter.FilterMaskIdLow      = 0x0000;
  can_filter.FilterFIFOAssignment = CAN_FilterFIFO0;
	can_filter.SlaveStartFilterBank = 0;  
  can_filter.FilterActivation     = ENABLE;

  HAL_CAN_ConfigFilter(&hcan1, &can_filter);
  while (HAL_CAN_ConfigFilter(&hcan1, &can_filter) != HAL_OK);
	
	can_filter.FilterBank           = 14;
  HAL_CAN_ConfigFilter(&hcan2, &can_filter);  
	while (HAL_CAN_ConfigFilter(&hcan2, &can_filter) != HAL_OK);
}


/**
  * @brief  init the can transmit and receive
  * @param  None
  */
void can_device_init(void)
{
  my_can_filter_init_recv_all();
	HAL_Delay(100);
	HAL_CAN_Start(&hcan1);
 	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(&hcan2);
 	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
	
}
