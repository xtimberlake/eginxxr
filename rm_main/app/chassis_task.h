/**
  * @file chassis_task.h
  * @version 1.0
  * @date Oct,19th 2018
  *
  * @brief  
  *
  *	@author lin kr
  *
  */
#ifndef __CLASSIS_TASK_H__
#define __CLASSIS_TASK_H__
#ifdef  __CHASSIS_TASK_GLOBALS
#define __CHASSIS_TASK_EXT
#else
#define __CHASSIS_TASK_EXT extern
#endif

#include "stm32f4xx_hal.h"
#include "bsp_pump.h"
#define CHASSIS_PERIOD 10
#define MAX_WHEEL_RPM   9000		//底盘电机转速限制极限

/* chassis parameter structure */
typedef enum
{
  CHASSIS_STOP   ,
  CHASSIS_REMOTE_NORMAL ,
	CHASSIS_REMOTE_SLOW		,
	
  CHASSIS_KB_NEAR   ,//贴墙自动模式
	CHASSIS_KB_MANUAL	 ,
	
  CHASSIS_DODGE  ,
  CHASSIS_KB  ,
} chassis_mode_e;

typedef enum
{
	NEAR_AIMING		,
	NEAR_DONE			,
  LEFT_AIMING   ,
  LEFT_DONE 		,
  RIGHT_AIMING  ,//贴墙自动模式
	RIGHT_AIMIGN	,
  NO_AIMING  ,
} chassis_aim_mode_e;
typedef struct
{
  float           vx; // forward/back
  float           vy; // left/right
  float           vw; // rotate
  chassis_mode_e  ctrl_mode;
  int16_t         current[4];
	chassis_aim_mode_e aim_ctrl_mode;
  int16_t         wheel_spd_fdb[4];//反馈值
  int16_t         wheel_spd_ref[4];//目标值

  int16_t         position_ref;
	float 					position_error;
	float 					dis_ref;
	
  float           target_vx; // forward/back
  float           target_vy; // left/right
  float           target_vw; // rotate

} chassis_t;

__CHASSIS_TASK_EXT chassis_t chassis;

void chassis_task(void const *argu);
void chassis_init(void);
void mecanum_calc(float vx, float vy, float vw, int16_t speed[]);

#endif
