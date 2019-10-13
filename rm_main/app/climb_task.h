/**
  * @file climb_task.h
  * @version 1.0
  * @date March,1 2019
  *
  * @brief  
  *
  *	@author lin zh
  *
  */
#ifndef __CLIMB_TASK_H__
#define __CLIMB_TASK_H__
#ifdef  __CLIMB_TASK_GLOBALS
#define __CLIMB_TASK_EXT
#else
#define __CLIMB_TASK_EXT extern
#endif

#include "stm32f4xx_hal.h"
#include "bsp_pump.h"

#define CLIMB_TASK_PERIOD 20

typedef enum
{

	CLIMB_ADJUST,
} climb_mode_e;//Ì§Éý»ú¹¹



__CLIMB_TASK_EXT climb_t climb;

void climb_task(void const *argu);
void climb_init(void);

#endif
