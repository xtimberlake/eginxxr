/**
  * @file bsp_gy53.h
  * @version 1.0
  * @date Mar,3 2019
  * @brief  
  * @author li zh
  */
#ifndef __BSP_GY53_H__
#define __BSP_GY53_H__

#ifdef  __BSP_GY53_GLOBALS
#define __BSP_GY53_EXT
#else
#define __BSP_GY53_EXT extern
#endif

#include "stm32f4xx_hal.h"

#define CatahTime				0.125		//����ĵ�λʱ�䣬��λus
//#define CatahTime				0.012		//����ĵ�λʱ�䣬��λus

typedef struct cap_t
{
	float us;								//���񵽵ĸߵ�ƽus����ʱ��
	uint16_t Capture_State;//����λΪ������ɱ�־������λΪ�ߵ�ƽ������ɱ�־������λΪ�������
	uint32_t cntStart;
	uint32_t cntContin;			//���񵽵ĸߵ�ƽ������
	
	uint32_t Tick;								//�����жϵĵ�ǰʱ��

}__capture_t;

typedef struct gy53_t
{
	float dist_left;				
	float dist_right;				
	uint32_t Tick;					
	float last_dist_left;				
	float last_dist_right;				
	uint32_t last_Tick;	
}__gy53_t;

__BSP_GY53_EXT __capture_t cap1,cap2;
__BSP_GY53_EXT __gy53_t gy53;

void gy53_init(void);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

#endif

