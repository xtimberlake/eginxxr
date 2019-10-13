/**
  * @file uplift_task.h
  * @version 1.0
  * @date March,1 2019
  *
  * @brief  
  *
  *	@author lin zh
  *
  */
#ifndef __UPLIFT_TASK_H__
#define __UPLIFT_TASK_H__
#ifdef  __UPLIFT_TASK_GLOBALS
#define __UPLIFT_TASK_EXT
#else
#define __UPLIFT_TASK_EXT extern
#endif

#include "stm32f4xx_hal.h"
#include "bsp_pump.h"

#define UPLIFT_PERIOD 10

#define HEIGHT_DOWN_LIMIT		0			//初始化高度
#define HEIGHT_UP_LIMIT			92.0f		//上限的高度
#define HEIGHT_GIVE					39.1f		//传输子弹的高度.
#define HEIGHT_GET					60.0f		//取弹机构取出弹药箱的高度.
#define HEIGHT_AIM					31.0f		//对位的高度		对位是否就是取弹高度
#define HEIGHT_NORMAL					0.0f		//普通

#define HEIGHT_CLIMB_UP_HANG			91.5f				//登岛对位
#define HEIGHT_CLIMB_UP_ZHUZI			89.3f				//上岛抱柱
#define HEIGHT_CLIMB_DOWN_HANG		6.0f			//下岛对位
//#define HEIGHT_CLIMB_DOWN_ZHUZI		10.3f			//下岛抱柱
#define HEIGHT_CLIMB_ON_TOP				4.2f				//登岛台上
#define HEIGHT_CLIMB_RETRACT			-1.0f				//登岛缩起

typedef enum
{
	UPLIFT_UNKNOWN,		//未知态
	UPLIFT_CALIBRA,		//校准	
	UPLIFT_KNOWN			//校准完成
} uplift_mode_e;//抬升机构


typedef enum
{
	UPLIFT_ADJUST,		//微调
	UPLIFT_AUTO,			
	UPLIFT_STOP,
} uplift_ctrl_mode_e;//抬升机构

typedef struct
{
  uplift_ctrl_mode_e   ctrl_mode;
  uplift_ctrl_mode_e   last_ctrl_mode;
	
  uplift_mode_e   mode;	
  int16_t         current[2];
	float						height_ref[2];	//抬升机构高度设定值
	float						height_fdb[2];	//抬升机构高度反馈值
	float						height_offset[2];//接触到光电开关后动态补偿
	int16_t					spd_ref[2];
	uint8_t 				down_limit1;
	uint8_t 				down_limit2;
	
	uint8_t 				last_down_limit1;
	uint8_t 				last_down_limit2;
	
	uint8_t  				offset_flag;//自动模式下校准抬升机构，其他模式可不理

	float height_up_limit;
	float height_give;
	float height_get;
	float height_aim;
	float height_normal;
	float height_climb_up_hang;
	float height_climb_up_zhuzi;
	float height_climb_down_hang;
//	float height_climb_down_zhuzi;
	float height_climb_on_top;
	float height_climb_retract;

} uplift_t;

__UPLIFT_TASK_EXT uplift_t uplift;
void uplift_task(void const *argu);
void uplift_init(void);

#endif
