#ifndef __DATA_PROCESSING_H
#define __DATA_PROCESSING_H

#ifdef  __DATA_PROCESSING_GLOBALS
#define __DATA_PROCESSING_EXT
#else
#define __DATA_PROCESSING_EXT extern
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#define ABS(x)		((x>0)? (x): (-x)) 

float circle_error(float set ,float get ,float circle_para);
void abs_limit(float *a, float ABS_MAX,float offset);

typedef struct
{
	float value;
	float inital;		//≥ı º÷µ
	float real_target;
	uint32_t ticks;
	uint32_t last_ticks;
	float target;
	float last_target;
//	TicksTypedef *SlopeTick;
}slope_t;

__DATA_PROCESSING_EXT slope_t slope_chassis_vx,slope_chassis_vy,slope_chassis_vw;

__DATA_PROCESSING_EXT slope_t slope_climb_pwm;
void slope_processing(slope_t *V,float target,float target_scale);


#endif


