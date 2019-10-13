/** 
  * @file bsp_pump.c
  * @version 1.0
  * @date Mar,2 2018
	*
  * @brief  气动元件。救援机构。抱柱爪子，悬架，夹取
	*
  *	@author li zh
  *
  */
#define  __BSP_PUMP_GLOBALS                                                                                             
#include "string.h"
#include "stdlib.h"
#include "bsp_uart.h"
#include "usart.h"
#include "main.h"
#include "bsp_pump.h"
#include "chassis_task.h"
#include "gimbal_task.h"

void claw_executed()
{
	if(pump.claw_ctrl_mode ==ON_MODE)	claw_on();
	else	claw_off();
}

void bracket_executed()
{
	if(pump.bracket_ctrl_mode ==ON_MODE)	bracket_on();
	else	bracket_off();
}

void press_executed()
{
	if(pump.press_ctrl_mode ==ON_MODE)	press_on();
	else	press_off();
}

void help_executed()
{
	if(pump.help_ctrl_mode ==ON_MODE)	help_on();
	else	help_off();
}

void rotate_executed()
{
	if(pump.rotate_ctrl_mode ==ON_MODE)	rotate_on();
	else	rotate_off();
}

void magazine_executed()
{
	if(pump.magazine_ctrl_mode ==ON_MODE)	magazine_on();
	else	magazine_off();
}

void camera_executed()
{
	if(pump.camera_ctrl_mode ==ON_MODE)	camera_on();
	else	camera_off();
}
void pump_init()
{
	pump.claw_ctrl_mode = OFF_MODE;
	pump.bracket_ctrl_mode = OFF_MODE;
	pump.press_ctrl_mode = OFF_MODE;
	pump.help_ctrl_mode = OFF_MODE;
	claw_executed();
	bracket_executed();
	press_executed();
	help_executed();
	camera_executed();
}


