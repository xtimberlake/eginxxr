/** 
  * @file modeswitch_task.h
  * @version 1.0
  * @date Mar,05th 2018
	*
  * @brief  ģʽѡ��
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

//��������ģʽ
typedef enum
{
  SAFETY_MODE,//����ģʽ
	RC_MOVE_MODE,//ң���������ƶ�ģʽ��̧��������΢����
	RC_BULLET_MODE,//ң�������������ģʽ
	RC_CLIMB_MODE,//ң�����Զ�ִ�й���ģʽ
	KB_MODE,//����ģʽ
} engineer_mode_e;

//����ģʽ
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
	BULLET_AIM,	//��λ�߶ȣ���ȡ�������������   ��λ�ڸպ�ȡ���߶�
	BULLET_STICK,
	BULLET_ROTATE_OUT,//ת��צ�ӳ�ȥ
	BULLET_PRESS,//��ȡ��ҩ��	
	BULLET_GET,	//̧����ҩ��ȡ��
	BULLET_WITHDRAW,//��ȡ�ڶ���ģʽ���ջ�����
	BULLET_PUSH,		//���ӵ�
	BULLET_THROW,		//����ҩ��
	BULLET_LOOSE,		//�ɿ�
	BULLET_DONE_STAY,		//ȡ�����ͣ��
	BULLET_DONE_LEFT,
	BULLET_DONE_RIGHT,
	BULLET_RESET,		//У׼

} get_bullet_mode_e;//��״̬��������ȡ������


typedef enum
{
	CLIMB_UP_HANG,	//���������Ӹ�
	CLIMB_UP_AIM,
	CLIMB_UP_ZHUZI,	//�������Ӹ߶�
	CLIMB_UP_CLAW,
	CLIMB_UP_RETRACT,
	CLIMB_UP_TURBINE,
	CLIMB_UP_ON,			//������
	CLIMB_UP_LOOSE,
	CLIMB_UP_DONE,
	
} climb_up_mode_e;//��״̬���������ϵ�����

typedef enum
{
	CLIMB_DOWN_HANG,
	CLIMB_DOWN_AIM,
	CLIMB_DOWN_ZHUZI,	//�������Ӹ߶�
	CLIMB_DOWN_CLAW,
	CLIMB_DOWN_RETRACT,
	CLIMB_DOWN_TURBINE,
	CLIMB_DOWN_ON,		//������
	CLIMB_DOWN_LOOSE,
	CLIMB_DOWN_DONE,
	
} climb_down_mode_e;//��״̬���������µ�����


typedef struct
{
	get_bullet_mode_e ctrl_mode;
	uint8_t step_flag;		//����ִ�е�flag
	uint8_t reset_flag;
	uint8_t loosetime;
	float dis_ref1;
	float dis_ref2;
} get_bullet_t;

typedef struct
{
//	get_bullet_mode_e ctrl_mode;
	uint8_t step_flag;		//����ִ�е�flag
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

__MODESW_TASK_EXT int camera_flag;	//������ǰ����
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
