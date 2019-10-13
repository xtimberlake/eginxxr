/** 
  * @file uplift_task.c
  * @version 1.1
  * @date Mar,2 2019
	*
  * @brief  抬升机构任务
	*
  *	@author li zh
  *
  */
#define __UPLIFT_TASK_GLOBALS
#include "uplift_task.h"
#include "cmsis_os.h"
#include "usart.h"
#include "comm_task.h"
#include "string.h"	
#include "modeswitch_task.h"
#include "remote_msg.h"
#include "pid.h"
#include "stdlib.h"
#include "bsp_can.h"
#include "bsp_motor.h"
#include "data_processing.h"
#include "math_calcu.h"

#define uplift_ratio (8192.0f)

uint16_t uplift_test;
extern TaskHandle_t can_msg_send_task_t;

/**
  * @brief uplift_task
  * @param     
  * @attention  
	* @note  软件定时器
  */
void uplift_task(void const *argu)
{
	taskENTER_CRITICAL();
	uplift.height_fdb[0] =  moto_uplift[0].total_ecd/uplift_ratio  -  uplift.height_offset[0];//具体需要测试
	uplift.height_fdb[1] = -moto_uplift[1].total_ecd/uplift_ratio  -  uplift.height_offset[1];
	uplift.down_limit1 = !HAL_GPIO_ReadPin(GPIOA,DOWN_LIMIT1_Pin);
	uplift.down_limit2 = !HAL_GPIO_ReadPin(GPIOB,DOWN_LIMIT2_Pin);

	if(uplift.down_limit1==1)		//接触到下限后给补偿值
	{
		uplift.height_offset[0] = moto_uplift[0].total_ecd/uplift_ratio ;	
//		uplift.spd_ref[0] =0;
		if(uplift.last_down_limit1==0){
			uplift.height_ref[0] = HEIGHT_DOWN_LIMIT;		//设置一次目标值归零
		}
	}
	if(uplift.down_limit2==1)		//接触到下限后给补偿值
	{
		uplift.height_offset[1] = -moto_uplift[1].total_ecd/uplift_ratio ;
//		pid_uplift_height[1].iout = 0;
//		uplift.spd_ref[1] =0;
		if(uplift.last_down_limit2==0){
			uplift.height_ref[1] = HEIGHT_DOWN_LIMIT;
		}
	}
	uplift.last_down_limit1 = uplift.down_limit1;
	uplift.last_down_limit2 = uplift.down_limit2;
	
	if(uplift.down_limit1==1&&uplift.down_limit2==1)
		uplift.mode = UPLIFT_KNOWN;
 	
	if(uplift.ctrl_mode==UPLIFT_AUTO&&uplift.mode!=UPLIFT_KNOWN)	//自动模式下无位置状态才开始自动校准
	{
		switch(uplift.mode)
		{
			case UPLIFT_UNKNOWN:
				uplift.mode = UPLIFT_CALIBRA;
				break;
			case UPLIFT_CALIBRA:
				if(uplift.down_limit1==0)		//接触到下限后给补偿值
					uplift.current[0] =  pid_calc(&pid_calibre_spd[0],moto_uplift[0].speed_rpm,-400);
				else
					uplift.current[0] =0;
				if(uplift.down_limit2==0)		//接触到下限后给补偿值				
				uplift.current[1] =  pid_calc(&pid_calibre_spd[1],moto_uplift[1].speed_rpm,400);	
				else
					uplift.current[1] =0;					
				break;
		}
	}
	else	//
	{
		if(uplift.ctrl_mode==UPLIFT_ADJUST&&uplift.last_ctrl_mode!=UPLIFT_ADJUST)
		{
			uplift.height_ref[0] = uplift.height_fdb[0];
			uplift.height_ref[1] = uplift.height_fdb[1];
		}
		switch(uplift.ctrl_mode)
		{
			case UPLIFT_STOP:
				uplift.current[0] = 0;	
				uplift.current[1] = 0;
				uplift.height_ref[0] = uplift.height_fdb[0];
				uplift.height_ref[1] = uplift.height_fdb[1];

				break;
			
			case UPLIFT_ADJUST:
			{
				if((uplift.down_limit1==1||uplift.down_limit2==1)&&rc.ch5>0)	rc.ch5=0;
			
				uplift.height_ref[0] -= 0.00056*rc.ch5;
				uplift.height_ref[1] -= 0.00056*rc.ch5;
				if(uplift.height_ref[0]>uplift.height_up_limit)
					uplift.height_ref[0] = uplift.height_up_limit;
				if(uplift.height_ref[1]>uplift.height_up_limit)
					uplift.height_ref[1] = uplift.height_up_limit;
				pid_calc(&pid_uplift_height[0],uplift.height_fdb[0],uplift.height_ref[0]);
				pid_calc(&pid_uplift_height[1],uplift.height_fdb[1],uplift.height_ref[1]);	
				uplift.spd_ref[0] =  pid_calc(&pid_uplift_height[0],uplift.height_fdb[0],uplift.height_ref[0]);
				uplift.spd_ref[1] =  -pid_calc(&pid_uplift_height[1],uplift.height_fdb[1],uplift.height_ref[1]);
//				uplift.spd_ref[0] = -rc.ch5*3;
//				uplift.spd_ref[1] = rc.ch5*3;

				uplift_test++;
				uplift.current[0] = pid_calc(&pid_uplift_spd[0],moto_uplift[0].speed_rpm,uplift.spd_ref[0]);
				uplift.current[1] = pid_calc(&pid_uplift_spd[1],moto_uplift[1].speed_rpm,uplift.spd_ref[1]);
				if(uplift.down_limit1==1||uplift.down_limit2==1)	
				{
					pid_uplift_spd[0].iout = 0;
					pid_uplift_spd[0].pos_out = 0;
					pid_uplift_spd[1].iout = 0;
					pid_uplift_spd[1].pos_out = 0;			
				}			
			}
			break;
				
			case UPLIFT_AUTO:
				if(climb.flag==0)
				{
					uplift.spd_ref[0] =  pid_calc(&pid_uplift_height[0],uplift.height_fdb[0],uplift.height_ref[0]);
					uplift.spd_ref[1] =  -pid_calc(&pid_uplift_height[1],uplift.height_fdb[1],uplift.height_ref[1]);				
				}
				else if(climb.flag==1)
				{
					uplift.spd_ref[0] =  pid_calc(&pid_retract_height[0],uplift.height_fdb[0],uplift.height_ref[0]);
					uplift.spd_ref[1] =  -pid_calc(&pid_retract_height[1],uplift.height_fdb[1],uplift.height_ref[1]);				
				}
				
				uplift.current[0] = pid_calc(&pid_uplift_spd[0],moto_uplift[0].speed_rpm,uplift.spd_ref[0]);
				uplift.current[1] = pid_calc(&pid_uplift_spd[1],moto_uplift[1].speed_rpm,uplift.spd_ref[1]);
				
			break;
		}
	}
	uplift.last_ctrl_mode = uplift.ctrl_mode;
	taskEXIT_CRITICAL();
	if(uplift.ctrl_mode==UPLIFT_STOP){
		uplift.current[0] = 0;	
		uplift.current[1] = 0;
	}
	memcpy(motor_cur.uplift_cur, uplift.current, sizeof(uplift.current));
	osSignalSet(can_msg_send_task_t, UPLIFT_MOTOR_MSG_SEND);
}

void uplift_init()
{
	//对位校准的PID
		PID_struct_init(&pid_calibre_spd[0], POSITION_PID, 15000, 4000,
									10.0f,	0.03f,	0.0f	);  
		PID_struct_init(&pid_calibre_spd[1], POSITION_PID, 15000, 4000,
									10.0f,	0.03f,	0.0f	); 
//		PID_struct_init(&pid_calibre_height[0], POSITION_PID, 3000, 500,
//									2500.0f,	5.0f,	0.0f	);  
//		PID_struct_init(&pid_calibre_height[1], POSITION_PID, 3000, 500,
//									2500.0f,	5.0f,	0.0f	);  

		//抬升的PID 
		PID_struct_init(&pid_uplift_spd[0], POSITION_PID, 15000, 3000,
									10.0f,	0.02f,	0.0f	);  
		PID_struct_init(&pid_uplift_spd[1], POSITION_PID, 15000, 3000,
									10.0f,	0.02f,	0.0f	);  
		PID_struct_init(&pid_uplift_height[0], POSITION_PID, 2800, 500,
									800.0f,	0.5f,	0.0f	);  
		PID_struct_init(&pid_uplift_height[1], POSITION_PID, 2800, 500,
									800.0f,	0.5f,	0.0f	);  
	
		//收腿的PID 
		PID_struct_init(&pid_retract_spd[0], POSITION_PID, 15000, 15000,
									10.0f,	0.02f,	0.0f	);  
		PID_struct_init(&pid_retract_spd[1], POSITION_PID, 15000, 15000,
									10.0f,	0.02f,	0.0f	);  
		PID_struct_init(&pid_retract_height[0], POSITION_PID, 3000, 500,
									800.0f,	5.0f,	0.0f	);  
		PID_struct_init(&pid_retract_height[1], POSITION_PID, 3000, 500,
									800.0f,	5.0f,	0.0f	);  
	
	uplift.mode = UPLIFT_UNKNOWN;
//	uplift.mode = UPLIFT_KNOWN;

	uplift.height_up_limit = HEIGHT_UP_LIMIT;
	uplift.height_give = HEIGHT_GIVE;
	uplift.height_get = HEIGHT_GET;
	uplift.height_aim = HEIGHT_AIM;
	uplift.height_normal = HEIGHT_NORMAL;
	
	uplift.height_climb_up_hang = 	HEIGHT_CLIMB_UP_HANG;
	uplift.height_climb_up_zhuzi = HEIGHT_CLIMB_UP_ZHUZI;
	uplift.height_climb_on_top = HEIGHT_CLIMB_ON_TOP;
	uplift.height_climb_down_hang = HEIGHT_CLIMB_DOWN_HANG;
//	uplift.height_climb_down_zhuzi = HEIGHT_CLIMB_ON_TOP;
	uplift.height_climb_retract = HEIGHT_CLIMB_RETRACT;
}


			//设置上下限
//			if(uplift.height_ref[0]<=HEIGHT_DOWN_LIMIT)	uplift.height_ref[0] = HEIGHT_DOWN_LIMIT;
//			if(uplift.height_ref[1]<=HEIGHT_DOWN_LIMIT)	uplift.height_ref[1] = HEIGHT_DOWN_LIMIT;
//			if(uplift.height_ref[0]>=HEIGHT_UP_LIMIT)	uplift.height_ref[0] = uplift.height_up_limit;
//			if(uplift.height_ref[1]>=HEIGHT_UP_LIMIT)	uplift.height_ref[1] = uplift.height_up_limit;

