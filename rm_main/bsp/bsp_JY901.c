/** 
  * @file bsp_JY901.c
  * @version 1.0
  * @date Dec,04th 2018
	*
  * @brief  维特智能陀螺仪JY901数据读取
	*
  *	@author li zh
  *
  */
#define  __BSP_JY901_GLOBALS                                                                                             
#include "string.h"
#include "stdlib.h"
#include "bsp_uart.h"
#include "usart.h"
#include "main.h"
#include "JY901.h"
#include "bsp_JY901.h"

struct STime		stcTime;
struct SAcc 		stcAcc;
struct SGyro 		stcGyro;
struct SAngle 	stcAngle;
struct SMag 		stcMag;
struct SDStatus stcDStatus;
struct SPress 	stcPress;
struct SLonLat 	stcLonLat;
struct SGPSV 		stcGPSV;
struct SQ       stcQ;

//imu_typedef imu_gimbal;


void imu_decoding(uint8_t imu_buf[],uint8_t buflen)
{
  
			uint8_t i=0;
			while(i< buflen)
			{
			if(imu_buf[i]==0x55)
			{
				switch(imu_buf[i+1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
				{
					case 0x50:	memcpy(&stcTime,&imu_buf[i+2],8);break;//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。
					case 0x51:	memcpy(&stcAcc,&imu_buf[i+2],8);break;
					case 0x52:	memcpy(&stcGyro,&imu_buf[i+2],8);break;
					case 0x53:	memcpy(&stcAngle,&imu_buf[i+2],8);break;
					case 0x54:	memcpy(&stcMag,&imu_buf[i+2],8);break;
					case 0x55:	memcpy(&stcDStatus,&imu_buf[i+2],8);break;
					case 0x56:	memcpy(&stcPress,&imu_buf[i+2],8);break;
					case 0x57:	memcpy(&stcLonLat,&imu_buf[i+2],8);break;
					case 0x58:	memcpy(&stcGPSV,&imu_buf[i+2],8);break;
					case 0x59:	memcpy(&stcQ,&imu_buf[i+2],8);break;
				}		
				i+=11;
			}
			else i++;
		}i=0;
		imu_data.roll = stcAngle.Angle[0]/32768.0f*180.0f;
		imu_data.pitch = stcAngle.Angle[1]/32768.0f*180.0f;
		imu_data.yaw = stcAngle.Angle[2]/32768.0f*180.0f +180.0f;
		
		imu_data.wx = stcGyro.w[0]/32768.0f*2000.0f;
		imu_data.wy = stcGyro.w[1]/32768.0f*2000.0f;
		imu_data.wz = stcGyro.w[2]/32768.0f*2000.0f;
		
}
