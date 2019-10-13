/**
  * @file relay_task.h
  * @version 1.0
  * @date March,24 2019
  *
  * @brief  
  *
  *	@author li zh
  *
  */
#ifndef __RELAY_TASK_H__
#define __RELAY_TASK_H__
#ifdef  __RELAY_TASK_GLOBALS
#define __RELAY_TASK_EXT
#else
#define __RELAY_TASK_EXT extern
#endif

#include "stm32f4xx_hal.h"
#include "bsp_pump.h"

#define RELAY_TASK_PERIOD 5



typedef struct
{
	uint8_t gas_status;//Æø·§¿ØÖÆÎ»
	uint16_t dis1;
	uint16_t dis2;
	uint8_t bullet_left;
	uint8_t bullet_right;
	int16_t enc;
	
} relay_t;

__RELAY_TASK_EXT relay_t relay;

void relay_task(void const *argu);
void relay_init(void);
__RELAY_TASK_EXT void get_relay_message(uint8_t CAN_Rx_data[8]);

#endif
