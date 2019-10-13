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

#define HEIGHT_DOWN_LIMIT		0			//��ʼ���߶�
#define HEIGHT_UP_LIMIT			92.0f		//���޵ĸ߶�
#define HEIGHT_GIVE					39.1f		//�����ӵ��ĸ߶�.
#define HEIGHT_GET					60.0f		//ȡ������ȡ����ҩ��ĸ߶�.
#define HEIGHT_AIM					31.0f		//��λ�ĸ߶�		��λ�Ƿ����ȡ���߶�
#define HEIGHT_NORMAL					0.0f		//��ͨ

#define HEIGHT_CLIMB_UP_HANG			91.5f				//�ǵ���λ
#define HEIGHT_CLIMB_UP_ZHUZI			89.3f				//�ϵ�����
#define HEIGHT_CLIMB_DOWN_HANG		6.0f			//�µ���λ
//#define HEIGHT_CLIMB_DOWN_ZHUZI		10.3f			//�µ�����
#define HEIGHT_CLIMB_ON_TOP				4.2f				//�ǵ�̨��
#define HEIGHT_CLIMB_RETRACT			-1.0f				//�ǵ�����

typedef enum
{
	UPLIFT_UNKNOWN,		//δ֪̬
	UPLIFT_CALIBRA,		//У׼	
	UPLIFT_KNOWN			//У׼���
} uplift_mode_e;//̧������


typedef enum
{
	UPLIFT_ADJUST,		//΢��
	UPLIFT_AUTO,			
	UPLIFT_STOP,
} uplift_ctrl_mode_e;//̧������

typedef struct
{
  uplift_ctrl_mode_e   ctrl_mode;
  uplift_ctrl_mode_e   last_ctrl_mode;
	
  uplift_mode_e   mode;	
  int16_t         current[2];
	float						height_ref[2];	//̧�������߶��趨ֵ
	float						height_fdb[2];	//̧�������߶ȷ���ֵ
	float						height_offset[2];//�Ӵ�����翪�غ�̬����
	int16_t					spd_ref[2];
	uint8_t 				down_limit1;
	uint8_t 				down_limit2;
	
	uint8_t 				last_down_limit1;
	uint8_t 				last_down_limit2;
	
	uint8_t  				offset_flag;//�Զ�ģʽ��У׼̧������������ģʽ�ɲ���

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
