/**
  * @file bsp_motor.h
  * @version 1.0
  * @date Mar,28 2019
  *
  * @brief
  *
  * @author li zh
  *
  */

#ifndef __BSP_MOTOR_H__
#define __BSP_MOTOR_H__

#ifdef  __BSP_MOTOR_GLOBALS
#define __BSP_MOTOR_EXT
#else
#define __BSP_MOTOR_EXT extern
#endif


#include "stm32f4xx_hal.h"
#define FILTER_BUF 5


typedef struct
{
  uint16_t ecd;
  uint16_t last_ecd;
  
  int16_t  speed_rpm;
  int16_t  given_current;

  int32_t  round_cnt;
  int32_t  total_ecd;
  int32_t  last_total_ecd;
  
  uint16_t offset_ecd;
  uint32_t msg_cnt;
  
  int32_t  ecd_raw_rate;
  int32_t  rate_buf[FILTER_BUF];
  uint8_t  buf_cut;
  int32_t  filter_rate;
} moto_measure_t;

__BSP_MOTOR_EXT moto_measure_t moto_chassis[4] ;
__BSP_MOTOR_EXT moto_measure_t moto_uplift[2];//Ì§Éý
__BSP_MOTOR_EXT moto_measure_t moto_pit ;
__BSP_MOTOR_EXT moto_measure_t moto_yaw ;

__BSP_MOTOR_EXT void encoder_data_handler(moto_measure_t* ptr, CAN_HandleTypeDef* hcan,uint8_t CAN_Rx_data[8]);
__BSP_MOTOR_EXT void get_moto_offset(moto_measure_t* ptr, CAN_HandleTypeDef* hcan,uint8_t CAN_Rx_data[8]);

#endif

