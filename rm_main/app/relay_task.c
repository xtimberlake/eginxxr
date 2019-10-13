/** 
  * @file relay_task.c
  * @version 1.1
  * @date Mar,24th 2019
	*
  * @brief  与气阀测距扩展板的通讯任务
	*
  *	@author li zh
  *
  */
#define __RELAY_TASK_GLOBALS
#include "relay_task.h"
#include "cmsis_os.h"
#include "usart.h"
#include "comm_task.h"
#include "stdlib.h"
#include "bsp_can.h"
#include "bsp_pump.h"
#include "uplift_task.h"

extern TaskHandle_t can_msg_send_task_t;

/**
  * @brief realy_task
  * @param     
  * @attention  
	* @note  
  */
void relay_task(void const *argu)
{
	uint32_t relay_wake_time = osKernelSysTick();
	for(;;)
	{
		claw_executed();
		bracket_executed();
		help_executed();
		press_executed();
		magazine_executed();
		rotate_executed();		
		camera_executed();
		HAL_UART_Transmit(&huart6, (uint8_t *)&relay.gas_status, 1, 5);

		osDelayUntil(&relay_wake_time, RELAY_TASK_PERIOD);
	}

}




