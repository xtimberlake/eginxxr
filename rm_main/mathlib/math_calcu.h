#ifndef _MATH_CALCU_H_
#define _MATH_CALCU_H_

#include "stm32f4xx_hal.h"

typedef struct
{
    float input;        //��������
    float out;          //�������
    float min_value;    //�޷���Сֵ
    float max_value;    //�޷����ֵ
    float frame_period; //ʱ����
} ramp_function_source_t;

extern ramp_function_source_t chassis_x_ramp;
extern ramp_function_source_t chassis_y_ramp;

void ramp_calc(ramp_function_source_t *ramp_source_type, float frame_period, float input, float max, float min);
void chassis_ramp(void);
#endif
