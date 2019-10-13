#include "status_task.h"
#include "tim.h"

uint8_t serial_send[25];
uint8_t sum;
status_t status;
uint32_t pwm1,pwm2;
/**
  * @brief status_task
  * @param     
  * @attention  
	* @note  
  */
void status_task(void const *argu)
{
  for(;;)
	{
//			TIM3->CCR3 = pwm1;

//		HAL_GPIO_TogglePin(GPIOD, LED_A_Pin);
//		if(status.imu_status == 1)
//		{
//			status.imu_status = 0;
//		}
//		else
//		{
//			status.imu_status = 2;
//			HAL_TIM_PWM_Start(&htim13,TIM_CHANNEL_1);
//			osDelay(400);
//			HAL_TIM_PWM_Stop(&htim13,TIM_CHANNEL_1);
//		}
//		if(status.dbus_status == 1)
//		{
//			status.dbus_status = 0;
//		}
//		else
//		{
//			HAL_TIM_PWM_Start(&htim13,TIM_CHANNEL_1);
//			osDelay(100);
//			HAL_TIM_PWM_Stop(&htim13,TIM_CHANNEL_1);
//		}
//		if(status.chassis_status[0] == 1&& status.chassis_status[1] == 1 
//			  && status.chassis_status[2] == 1 && status.chassis_status[3] == 1)
//		{
//			for(int i = 0;i<4;i++)
//			{
//		   status.chassis_status[i] = 0;
//			}
//		}
//		else
//		{
//			HAL_TIM_PWM_Start(&htim13,TIM_CHANNEL_1);
//			osDelay(100);
//			HAL_TIM_PWM_Stop(&htim13,TIM_CHANNEL_1);
//			osDelay(100);
//			HAL_TIM_PWM_Start(&htim13,TIM_CHANNEL_1);
//			osDelay(100);
//			HAL_TIM_PWM_Stop(&htim13,TIM_CHANNEL_1);
//			osDelay(100);
//		}
//		if(status.gimbal_status[0] == 1 && status.gimbal_status[1] == 1)
//		{
//		  status.gimbal_status[0] = 0;
//			status.gimbal_status[1] = 0;
//		}
//		else
//		{
//			HAL_TIM_PWM_Start(&htim13,TIM_CHANNEL_1);
//			osDelay(100);
//			HAL_TIM_PWM_Stop(&htim13,TIM_CHANNEL_1);
//			osDelay(100);
//			HAL_TIM_PWM_Start(&htim13,TIM_CHANNEL_1);
//			osDelay(100);
//			HAL_TIM_PWM_Stop(&htim13,TIM_CHANNEL_1);
//			osDelay(100);
//			HAL_TIM_PWM_Start(&htim13,TIM_CHANNEL_1);
//			osDelay(100);
//			HAL_TIM_PWM_Stop(&htim13,TIM_CHANNEL_1);
//			osDelay(100);
//		}

		osDelay(100);
	}
}

void status_init()
{
	HAL_TIM_PWM_Start(&htim13,TIM_CHANNEL_1);
	TIM13->CCR1 = 83;
}
void status_init_success()
{
  HAL_TIM_PWM_Stop(&htim13,TIM_CHANNEL_1);
	status.dbus_status = 0;
	status.imu_status = 0;
	for(int i = 0;i<4;i++)
	{	
	  status.chassis_status[i] = 0;
	}
	status.gimbal_status[0] = 0;
	status.gimbal_status[1] = 0;
}
//void blue_display()
//{
//	serial_send[0] = 0xAA;
//	serial_send[1] = 0xAA;
//	serial_send[2] = 0x06;
//	serial_send[3] = 8;
//	serial_send[4] = chassis.wheel_spd_fdb[0] >> 8;
//	serial_send[5] = chassis.wheel_spd_fdb[0];
//	serial_send[6] = chassis.wheel_spd_fdb[1] >> 8;
//	serial_send[7] = chassis.wheel_spd_fdb[1];
//	serial_send[8] = chassis.wheel_spd_fdb[2] >> 8;
//	serial_send[9] = chassis.wheel_spd_fdb[2];
//	serial_send[10] = chassis.wheel_spd_fdb[3] >> 8;
//	serial_send[11] = chassis.wheel_spd_fdb[3];
////	serial_send[12] = 0x00;
////	serial_send[13] = 0x00;
////	serial_send[14] = 0x00;//02Z
////	serial_send[15] = 0x00;
////	serial_send[16] = 0x00;//03X
////	serial_send[17] = 0x00;
////	serial_send[18] = 0x00;//03Y
////	serial_send[19] = 0x00;
////	serial_send[20] = 0x00;//03Z
////	serial_send[21] = 0x00;	
//	sum = 0;
//	for(uint8_t i=0;i<12;i++)
//	{
//		sum += serial_send[i];
//	}
//	serial_send[12]= sum ;
//	
//	HAL_UART_Transmit(&huart2, serial_send,13, 100);
//}
