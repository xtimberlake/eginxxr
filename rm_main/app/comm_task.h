/** 
  * @file comm_task.h
  * @version 1.0
  * @date Oct,19th 2018
	*
  * @brief  
	*
  *	@author lin kr
  *
  */
#ifndef __COMM_TASK_H__
#define __COMM_TASK_H__

#include "stm32f4xx_hal.h"

#define GIMBAL_MOTOR_MSG_SEND   ( 1 << 2 )
#define UPLIFT_MOTOR_MSG_SEND   ( 1 << 3 )
#define CHASSIS_MOTOR_MSG_SEND  ( 1 << 4 )

/* motor current parameter structure */
typedef struct
{
  /* 4 chassis motor current */
  int16_t chassis_cur[4];
  int16_t uplift_cur[2];
  int16_t gimbal_cur[2];
} motor_current_t;

void send_chassis_Judgement_power_message(uint16_t chassis_power,uint16_t chassis_power_buffer);

void can_msg_send_task(void const *argu);
void send_chassis_motor_ctrl_message(int16_t chassis_cur[]);
void send_uplift_motor_ctrl_message(int16_t uplift_cur[]);
void send_gimbal_motor_ctrl_message(int16_t gimbal_cur[]);
extern motor_current_t motor_cur;
#endif
