/** 
  * @file modeswitch_task.h
  * @version 1.0
  * @date Mar,05th 2018
	*
  * @brief  模式选择
	*
  *	@author lin zh
  *
  */
	
#ifndef __MODESWITCH_TASK_H__
#define __MODESWITCH_TASK_H__

#ifdef  __MODESW_TASK_GLOBALS
#define __MODESW_TASK_EXT
#else
#define __MODESW_TASK_EXT extern
#endif
#include "stm32f4xx_hal.h"

#define INFO_GET_PERIOD 5

//工程整机模式
typedef enum
{
  SAFETY_MODE,//保护模式
	RC_MOVE_MODE,//遥控器底盘移动模式（抬升机构可微调）
	RC_BULLET_MODE,//遥控器各组件测试模式
	RC_CLIMB_MODE,//遥控器自动执行功能模式
	KB_MODE,//键盘模式
} engineer_mode_e;

//功能模式
typedef enum
{
  GET_BULLET1_MODE,
	GET_BULLET2_MODE,
	GIVE_BULLET_MODE,
	MOVE_MODE,
	
} func_mode_e;

typedef enum 
{
	CAMERA_NORMAL_MODE,
	CAMERA_CLIMB_MODE,
	CAMERA_BULLET_MODE,	
	CAMERA_HELP_MODE,
}camera_mode_e;

typedef enum
{
	BULLET_N,
	BULLET_AIM,	//对位高度，若取后排则伸出悬架   对位在刚好取弹高度
	BULLET_STICK,
	BULLET_ROTATE_OUT,//转动爪子出去
	BULLET_PRESS,//夹取弹药箱	
	BULLET_GET,	//抬升弹药箱取出
	BULLET_WITHDRAW,//（取第二排模式）收回悬架
	BULLET_PUSH,		//倒子弹
	BULLET_THROW,		//丢弹药箱
	BULLET_LOOSE,		//松开
	BULLET_DONE_STAY,		//取弹完成停着
	BULLET_DONE_LEFT,
	BULLET_DONE_RIGHT,
	BULLET_RESET,		//校准

} get_bullet_mode_e;//用状态描述整个取弹流程


typedef enum
{
	CLIMB_UP_HANG,	//升到比柱子高
	CLIMB_UP_AIM,
	CLIMB_UP_ZHUZI,	//降到柱子高度
	CLIMB_UP_CLAW,
	CLIMB_UP_RETRACT,
	CLIMB_UP_TURBINE,
	CLIMB_UP_ON,			//到岛上
	CLIMB_UP_LOOSE,
	CLIMB_UP_DONE,
	
} climb_up_mode_e;//用状态描述整个上岛流程

typedef enum
{
	CLIMB_DOWN_HANG,
	CLIMB_DOWN_AIM,
	CLIMB_DOWN_ZHUZI,	//降到柱子高度
	CLIMB_DOWN_CLAW,
	CLIMB_DOWN_RETRACT,
	CLIMB_DOWN_TURBINE,
	CLIMB_DOWN_ON,		//到岛下
	CLIMB_DOWN_LOOSE,
	CLIMB_DOWN_DONE,
	
} climb_down_mode_e;//用状态描述整个下岛流程


typedef struct
{
	get_bullet_mode_e ctrl_mode;
	uint8_t step_flag;		//单步执行的flag
	uint8_t reset_flag;
	uint8_t loosetime;
	float dis_ref1;
	float dis_ref2;
} get_bullet_t;

typedef struct
{
//	get_bullet_mode_e ctrl_mode;
	uint8_t step_flag;		//单步执行的flag
	uint8_t reset_flag;
	uint8_t jdq;
	uint16_t pwm;
	uint8_t flag;
	climb_up_mode_e climb_up_mode;
	climb_down_mode_e climb_down_mode;
	uint8_t up_or_down;
} climb_t;

__MODESW_TASK_EXT get_bullet_t bullet;
__MODESW_TASK_EXT climb_t climb;

__MODESW_TASK_EXT engineer_mode_e glb_ctrl_mode;
__MODESW_TASK_EXT engineer_mode_e last_glb_ctrl_mode;

__MODESW_TASK_EXT func_mode_e func_mode;
__MODESW_TASK_EXT func_mode_e last_func_mode;
__MODESW_TASK_EXT camera_mode_e camera_mode;

__MODESW_TASK_EXT int camera_flag;	//决定车前后方向
void mode_switch_task(void const *argu);

void get_main_ctrl_mode(void);

void get_global_last_mode(void);

void rc_move_handle(void);
void rc_bullet_handle(void);
void rc_climb_handle(void);
void kb_handle(void);
void get_bullet_ctrl_mode(void);
void safety_mode_handle(void);
void climb_ctrl_mode(void);

#endif
