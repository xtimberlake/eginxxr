/**
  * @file bap_imu.h
  * @version 1.0
  * @date Dec,4th 2018
  *
  * @brief  维特智能陀螺仪JY901的数据解算
  *
  * @author li zh
  *
  */

#ifndef __BSP_JY901_H__
#define __BSP_JY901_H__

#ifdef  __BSP_JY901_GLOBALS
#define __BSP_JY901_EXT
#else
#define __BSP_JY901_EXT extern
#endif


#include "stm32f4xx_hal.h"

#include "JY901.h"

#define IMU_BUFLEN  44
#define IMU_MAX_LEN 50
typedef struct imu_typedef1
{
	float pitch;
	float roll;
	float yaw;
	float wx;
	float wy;
	float wz;
	float ax;
	float ay;
	float az;
}imu_typedef;

__BSP_JY901_EXT imu_typedef imu_data;

void imu_decoding(uint8_t imu_buf[],uint8_t buflen);

#endif

