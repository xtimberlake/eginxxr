/** 
  * @file climb_task.c
  * @version 1.1
  * @date Mar,6th 2019
	*
  * @brief  µÇµºÈÎÎñ
	*
  *	@author li zh
  *
  */
#define __CLIMB_TASK_GLOBALS
#include "climb_task.h"
#include "cmsis_os.h"
#include "usart.h"
#include "comm_task.h"
#include "string.h"	
#include "modeswitch_task.h"
#include "remote_msg.h"
#include "pid.h"
#include "stdlib.h"
#include "bsp_can.h"
#include "data_processing.h"
#include "math_calcu.h"
#include "bsp_motor.h"

extern TaskHandle_t can_msg_send_task_t;

/**
  * @brief climb_task
  * @param     
  * @attention  
	* @note  
  */
void climb_task(void const *argu)
{
	uint32_t climb_wake_time = osKernelSysTick();
	for(;;)
	{
		//climb.ecd_ref[0] = 
		//climb.ecd_ref[1] = 
//		pid_calc(&pid_climb_ecd[0],moto_climb[0].total_ecd,climb.ecd_ref[0]);
//		pid_calc(&pid_climb_spd[0],moto_climb[0].speed_rpm,pid_climb_ecd[0].pos_out);
		
//		pid_calc(&pid_climb_ecd[1],moto_climb[1].total_ecd,climb.ecd_ref[1]);
//		pid_calc(&pid_climb_spd[1],moto_climb[1].speed_rpm,pid_climb_ecd[1].pos_out);
		
//		climb.current[0] = pid_climb_spd[0].pos_out;	
//		climb.current[1] = pid_climb_spd[1].pos_out;
		climb.current[0] = 0;	
		climb.current[1] = 0;

		osDelayUntil(&climb_wake_time, CLIMB_TASK_PERIOD);
	}

}

void climb_init()
{

}
