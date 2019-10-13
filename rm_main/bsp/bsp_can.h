/** 
  * @file bap_can.h
  * @version 1.0
  * @date Oct,15th 2018
  *
  * @brief  
  *
  * @author lin kr
  *
  */
 
#ifndef _BSP_CAN_H_
#define _BSP_CAN_H_

#ifdef  __BSP_CAN_GLOBALS
#define __BSP_CAN_EXT
#else
#define __BSP_CAN_EXT extern
#endif
#include "bsp_JY901.h"
#include "can.h"
#include "comm_task.h"
#define CHASSIS_CAN   hcan1
#define UPLIFT_CAN    hcan1
#define GIMBAL_CAN    hcan2
/* CAN send and receive ID */
typedef enum
{
  CAN_3508_M1_ID       = 0x201,
  CAN_3508_M2_ID       = 0x202,
  CAN_3508_M3_ID       = 0x203,
  CAN_3508_M4_ID       = 0x204,
  CAN_3508_UP1_ID      = 0x205,
  CAN_3508_UP2_ID      = 0x206, 

	CAN_CHASSIS_ALL_ID   = 0x200,
  CAN_GIMBAL_ALL_ID    = 0x1ff,
	
	//can2
	CAN_GM3510_YAW_ID 	 = 0x205,
	CAN_GM3510_PIT_ID 	 = 0x206,
} can_msg_id_e;

typedef struct
{
  uint8_t CAN_Tx_data[8];
} CAN_SendMsg;

/* can receive motor parameter structure */

__BSP_CAN_EXT CAN_SendMsg chassis_can_tx_data;
__BSP_CAN_EXT CAN_SendMsg uplift_can_tx_data;
__BSP_CAN_EXT CAN_SendMsg gimbal_tx_data;
__BSP_CAN_EXT CAN_SendMsg relay_tx_data;

extern uint8_t Rx1Data[8];


void can_device_init(void);
void my_can_filter_init_recv_all(void);

void send_uplift_cur(int16_t up1_iq, int16_t up2_iq);
void send_chassis_cur(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void send_gimbal_cur(int16_t yaw_iq,int16_t pit_iq);

#endif
