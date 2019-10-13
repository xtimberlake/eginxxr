/** 
  * @file bsp_gy53.c
  * @version 1.0
  * @date Mar,3 2018
  * @brief  ���⴫����gy53
  *	@author li zh
  *
  */
#define  __BSP_GY53_GLOBALS                                                                                             
#include "string.h"
#include "stdlib.h"
#include "bsp_uart.h"
#include "usart.h"
#include "main.h"
#include "tim.h"
#include "bsp_gy53.h"
void gy53_init(void)
{
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_3);	//�������벶���ж�
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_4);	//�������벶���ж�
	cap1.cntStart = 0;
	cap1.cntContin = 0;
	cap1.Capture_State = 0;
	cap1.us = 0;
	cap2.cntStart = 0;
	cap2.cntContin = 0;
	cap2.Capture_State = 0;
	cap2.us = 0;
}


/************************�����Ϣ��ȡ*************************/
/**
  * @brief  HAL�����벶��ص������е��Ӻ���
  * @param  
  * @retval ��
  */
void _CaptureCallBack(__capture_t * cap)
{
	cap->Tick=HAL_GetTick();
	TIM_HandleTypeDef * timHandle;
	uint32_t tim_channel;
	
	timHandle = &htim1;
	
	if(cap==&cap1)
		tim_channel = TIM_CHANNEL_3;
	if(cap==&cap2)
		tim_channel = TIM_CHANNEL_4;
	
	//δ��ɲ���
	if(!(cap->Capture_State & 0x80))
	{
		//���񵽵͵�ƽ
		if(cap->Capture_State & 0x40)
		{
			cap->Capture_State = 0;//�������
			
			cap->cntContin = HAL_TIM_ReadCapturedValue(timHandle,tim_channel) - cap->cntStart;
			
			TIM_RESET_CAPTUREPOLARITY(timHandle,tim_channel);
			TIM_SET_CAPTUREPOLARITY(timHandle,tim_channel,TIM_ICPOLARITY_RISING);

			if(cap->cntContin*CatahTime>100&&cap->cntContin*CatahTime<8000)
				cap->us=cap->cntContin*CatahTime;

		}//���񵽸ߵ�ƽ
		else
		{
			cap->Capture_State |= 0x40;
			cap->cntStart = HAL_TIM_ReadCapturedValue(timHandle,tim_channel);
			TIM_RESET_CAPTUREPOLARITY(timHandle,tim_channel); 
			TIM_SET_CAPTUREPOLARITY(timHandle,tim_channel,TIM_ICPOLARITY_FALLING);	
		}
	}
}


/**
  * @brief  HAL�����벶��ص�����
  * @param  htim TIM����ṹ��ָ��
  * @retval ��
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
		if(TIM1==htim->Instance)
		{
			if(HAL_TIM_ACTIVE_CHANNEL_3 == htim->Channel)
			{
				_CaptureCallBack(&cap1);
				gy53.dist_left = cap1.us/10.0f;
			}
			if(HAL_TIM_ACTIVE_CHANNEL_4 == htim->Channel)
			{
				_CaptureCallBack(&cap2);
				gy53.dist_right = cap2.us/10.0f;
			}
			
		}
	
}


	