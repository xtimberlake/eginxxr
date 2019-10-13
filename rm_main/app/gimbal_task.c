/** 
  * @file gimbal_task.c
  * @version 1.1
  * @date July,24th 2019
	*
  * @brief  GM3510��̨
	*
  *	@author li zh
  *
  */
#define __GIMBAL_TASK_GLOBALS
#include "gimbal_task.h"
#include "cmsis_os.h"
#include "usart.h"
#include "comm_task.h"
#include "string.h"	
#include "modeswitch_task.h"
#include "remote_msg.h"
#include "pid.h"
#include "stdlib.h"
#include "bsp_can.h"
#include "data_processing.h"
#include "math_calcu.h"
#include "bsp_motor.h"

extern TaskHandle_t can_msg_send_task_t;

#define  yaw_range_ecd  4000
#define  pit_range_ecd  1000


void gimbal_param_init(void)
{
  memset(&gimbal, 0, sizeof(gimbal_t));
  /* pitch axis motor pid parameter */
	//pit����̨û�̶��õ�ʱ���ã�
  PID_struct_init(&pid_pit_ecd, POSITION_PID, 29000, 5000,
                  12.0f, 0.0f, 50.0f); //����
//  PID_struct_init(&pid_pit_ecd, POSITION_PID, 500, 300,
//                  0.4, 0.0, 0.0); 
  PID_struct_init(&pid_pit_spd, POSITION_PID, 29000, 5000,
                  20.0f, 0.001f, 0.0);
  PID_struct_init(&pid_pit_gyro, POSITION_PID, 29000, 5000,
                  2.0f, 0.0f, 0.0);
	
  /* yaw axis motor pid parameter */
  PID_struct_init(&pid_yaw_ecd, POSITION_PID, 29000, 5000,
                  15.0f, 0.0f, 30.0f); //����
//  PID_struct_init(&pid_yaw_ecd, POSITION_PID, 500, 300,
//                  0.4f, 0.0, 0.0f); 
  PID_struct_init(&pid_yaw_spd, POSITION_PID, 29000, 1000,
                  20.0f, 0.001, 0);
  PID_struct_init(&pid_yaw_gyro, POSITION_PID, 29000, 5000,
                  4.0f, 0.1f, 0.0);
	
	
	gimbal.pit_center_offset = 4420;
	gimbal.yaw_center_offset = 4300;
	gimbal.yaw_ecd_ref = gimbal.yaw_center_offset;
	gimbal.pid.pit_ecd_ref = gimbal.pit_center_offset;
}

void gimbal_control(gimbal_mode_e gimbal_mode)
{

  switch(gimbal_mode)
	{
	  case GIMBAL_REMOTER:
		{
			//pit ecd
			gimbal.pid.pit_relative_ecd += rc.ch4 * -0.025f;//ң�����ı����ֵ����
			//yaw angle
			gimbal.pid.yaw_relative_ecd += rc.ch3 * -0.025f ;	//�Ƕ��趨
		}break;
		case GIMBAL_KEYBOARD:
		{	
			//pit ecd
			gimbal.pid.pit_relative_ecd += rc.mouse.y * -2.0f;//ң�����ı����ֵ����
			//yaw angle
			gimbal.pid.yaw_relative_ecd += rc.mouse.x * -2.05f;	//�Ƕ��趨
		}break;
		default:
		{
		}break;
	}
	
	abs_limit(&gimbal.pid.pit_relative_ecd , pit_range_ecd , 0);	//�Ա���ֵ���������޷�
	gimbal.pid.pit_ecd_ref = gimbal.pid.pit_relative_ecd + gimbal.pit_center_offset;	//����+�е�=Ŀ��
	gimbal.pid.pit_error_ecd = circle_error(gimbal.pid.pit_ecd_ref,gimbal.pid.pit_ecd_fdb,8191);//����������
	gimbal.pid.pit_ecd_fdb = moto_pit.ecd;	//�������ֵ����
	pid_calc(&pid_pit_ecd, gimbal.pid.pit_ecd_fdb, gimbal.pid.pit_ecd_fdb + gimbal.pid.pit_error_ecd);//λ�û�
	//pit spd
	gimbal.pid.pit_spd_ref = pid_pit_ecd.pos_out; 	//λ�û����Ϊ�ٶȻ��趨
	
//	gimbal.pid.pit_spd_ref = 0; 
	//�ڻ��õ�������ٶ�
	gimbal.pid.pit_spd_fdb = moto_pit.speed_rpm;	//Ҫ�ĳ��Լ�����ĵ���ٶȣ�
	pid_calc(&pid_pit_spd, gimbal.pid.pit_spd_fdb, gimbal.pid.pit_spd_ref);//�ٶȻ�
	//�ڻ��ý��ٶ�
//	gimbal.pid.pit_spd_fdb = imu_data.wx;	
//	pid_calc(&pid_pit_gyro, gimbal.pid.pit_spd_fdb, gimbal.pid.pit_spd_ref);//�ٶȻ�

	
	abs_limit(&gimbal.pid.yaw_relative_ecd , yaw_range_ecd , 0);	//�Ա���ֵ���������޷�
	gimbal.pid.yaw_ecd_ref = gimbal.pid.yaw_relative_ecd + gimbal.yaw_center_offset;	//����+�е�=Ŀ��
	gimbal.pid.yaw_error_ecd = circle_error(gimbal.pid.yaw_ecd_ref,gimbal.pid.yaw_ecd_fdb,8191);//����������
	gimbal.pid.yaw_ecd_fdb = moto_yaw.ecd;	//�������ֵ����
	pid_calc(&pid_yaw_ecd, gimbal.pid.yaw_ecd_fdb, gimbal.pid.yaw_ecd_fdb + gimbal.pid.yaw_error_ecd);//λ�û�
	//yaw spd
	gimbal.pid.yaw_spd_ref = pid_yaw_ecd.pos_out; 	//λ�û����Ϊ�ٶȻ��趨
//	gimbal.pid.yaw_spd_ref = 0; 
//�ڻ��õ�������ٶ�
	gimbal.pid.yaw_spd_fdb = moto_yaw.speed_rpm;	//Ҫ�ĳ��Լ�����ĵ���ٶȣ�
	pid_calc(&pid_yaw_spd, gimbal.pid.yaw_spd_fdb, gimbal.pid.yaw_spd_ref);//�ٶȻ�
//�ڻ��ý��ٶ�
//	gimbal.pid.yaw_spd_fdb = imu_data.wz;	
//	pid_calc(&pid_yaw_gyro, gimbal.pid.yaw_spd_fdb, gimbal.pid.yaw_spd_ref);//�ٶȻ�



//	gimbal.current[0] = pid_yaw_gyro.pos_out;	
//	gimbal.current[1] = pid_pit_gyro.pos_out;

//	gimbal.current[0] = pid_yaw_spd.pos_out;	
//	gimbal.current[1] = pid_pit_spd.pos_out;
	
	//����PID
	gimbal.current[0] = pid_yaw_ecd.pos_out;
//	gimbal.current[0] = 0;	
//	gimbal.current[1] = 0;

	gimbal.current[1] = pid_pit_ecd.pos_out;
}

/**
  * @brief get_bullet_task
  * @param     
  * @attention  
	* @note  
  */
void gimbal_task(void const *argu)
{
	uint32_t bullet_wake_time = osKernelSysTick();
	for(;;)
	{		

	switch(glb_ctrl_mode)
	{
		case SAFETY_MODE:
		{
			 taskENTER_CRITICAL();
			
			gimbal.current[0] = 0;
			gimbal.current[1] = 0;
			
			 taskEXIT_CRITICAL();

		}break;			
		case RC_MOVE_MODE:
		{
			 taskENTER_CRITICAL();
			
			gimbal_control(GIMBAL_REMOTER);
			
			 taskEXIT_CRITICAL();
		}break;		
		case KB_MODE:
		{
			 taskENTER_CRITICAL();
			
			gimbal_control(GIMBAL_KEYBOARD);
			
			 taskEXIT_CRITICAL();
		}break;		

		default:
		{
		}break;
	}		

	memcpy(motor_cur.gimbal_cur, gimbal.current, sizeof(gimbal.current));
  osSignalSet(can_msg_send_task_t, GIMBAL_MOTOR_MSG_SEND);

		
	osDelayUntil(&bullet_wake_time, GIMBAL_TASK_PERIOD);
	}

}

