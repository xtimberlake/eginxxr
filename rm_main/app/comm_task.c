/** 
  * @file comm_task.c
  * @version 1.1
  * @date Oct,23th 2018
	*
  * @brief  交互任务文件
	*
  *	@author lin zh
  *
  */
#include "comm_task.h"
#include "bsp_can.h"
#include "cmsis_os.h"
#include "pid.h"
#include "relay_task.h"
motor_current_t motor_cur;
/**
  * @brief can_msg_send_task
  * @param 
  * @attention  
  * @note  
  */
void can_msg_send_task(void const *argu)
{
	osEvent event;
	for(;;)
  {
		event = osSignalWait(UPLIFT_MOTOR_MSG_SEND | \
													GIMBAL_MOTOR_MSG_SEND|	\
													CHASSIS_MOTOR_MSG_SEND, osWaitForever);
		
		if (event.status == osEventSignal)
    {
			if (event.value.signals & CHASSIS_MOTOR_MSG_SEND)
      {
        send_chassis_motor_ctrl_message(motor_cur.chassis_cur);
			}
      if (event.value.signals & UPLIFT_MOTOR_MSG_SEND)
      {
//        can_time_ms = HAL_GetTick() - can_time_last;
//        can_time_last = HAL_GetTick();
        send_uplift_motor_ctrl_message(motor_cur.uplift_cur);
      }
			if (event.value.signals & GIMBAL_MOTOR_MSG_SEND)
      {
        send_gimbal_motor_ctrl_message(motor_cur.gimbal_cur);
			}
			
		}
	}
}

/**
  * @brief send_chassis_motor_ctrl_message
  * @param 
  * @attention  
  * @note  
  */
void send_chassis_motor_ctrl_message(int16_t chassis_cur[])
{
  send_chassis_cur(chassis_cur[0], chassis_cur[1], 
                   chassis_cur[2], chassis_cur[3]);
}

void send_uplift_motor_ctrl_message(int16_t uplift_cur[])
{
  /* 0: uplift1 motor current
     1: uplift2 motor current*/
	  send_uplift_cur(uplift_cur[0], uplift_cur[1]);
}

void send_gimbal_motor_ctrl_message(int16_t gimbal_cur[])
{
	  send_gimbal_cur(gimbal_cur[0], gimbal_cur[1]);

}


void send_chassis_Judgement_power_message(uint16_t chassis_power,uint16_t chassis_power_buffer)
{
  //send_chassis_power(chassis_power,chassis_power_buffer);
}
