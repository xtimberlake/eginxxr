
/**
  ******************************************************************************
  * @file			data_processing.c
  * @version		V1.0.0
  * @date			2018年12月5日
  * @brief   		一些数据处理函数
  *******************************************************************************/
  
  
  #define __DATA_PROCESSING_GLOBALS
/* Includes ------------------------------------------------------------------*/
#include "mytype.h"
#include "data_processing.h"
#include <math.h>
//#include "cmsis_os.h"


void abs_limit(float *a, float ABS_MAX,float offset){
    if(*a > ABS_MAX+offset)
        *a = ABS_MAX+offset;
    if(*a < -ABS_MAX+offset)
        *a = -ABS_MAX+offset;}


/**
	*@func   float Circle_error(float set ,float get ,float circle_para)
	*@bref		环形数据计算偏差值
	*@param[in] set 设定值 get采样值 circle_para 一圈数值
	*@note	环形数据下，直接计算出PID中的偏差值
*/
float circle_error(float set ,float get ,float circle_para)
{
	float error;
	if(set > get)
	{
		if(set - get> circle_para/2)
			error = set - get - circle_para;
		else
			error = set - get;
	}
	else if(set < get)
	{
		if(set - get<-1*circle_para/2)
			error = set - get +circle_para;
		else
			error = set - get;
	}
	else	error = 0;

	return error;
}


/**
	* @func		void slope_processing(slope_t *V,float target,float last_target,float target_scale)
  * @brief  数据斜坡处理函数
  * @param  slope_t *V 要设置的变量指针
  * @param  float target 目标值
  * @param 	float target_scale 每个单位时间的变化值（ms）
  * @retval void
  */

void slope_processing(slope_t *V,float target,float target_scale)
{
//	float target_scale;//每ms增加或减少的量
//	V->ticks = osKernelSysTick();
	V->ticks = HAL_GetTick();
	V->last_target = V->target;
	V->target = target;
//	if(last_inital!=inital)	*set = inital;		//初始值变化时，赋初值
	if(V->last_target!=V->target)
	{
		V->inital = V->value;
		V->real_target = V->target;
	}
//	target_scale = (V->real_target-V->inital)/slopetime;	//每个单位时间的变化
	if(V->real_target>V->inital)		//加操作
	{
		if(V->value + target_scale>=V->real_target)
		{V->value = V->real_target;}//斜坡启动完成保持为目标值
		else						//未完成进行斜坡处理
		{V->value += target_scale;}
	}
	else if(V->real_target<=V->inital)	//减操作
	{
		if(V->value - target_scale<=V->real_target)
		{V->value = V->real_target;}//斜坡启动完成保持为目标值
		else						
		{V->value -= target_scale;}
	}
		
	V->last_ticks = V->ticks;

}

