/**
  * @file bsp_pump.h
  * @version 1.0
  * @date Mar,2 2019
  *
  * @brief  
  *
  * @author li zh
  *
  */

#ifndef __BSP_PUMP_H__
#define __BSP_PUMP_H__

#ifdef  __BSP_PUMP_GLOBALS
#define __BSP_PUMP_EXT
#else
#define __BSP_PUMP_EXT extern
#endif

#include "stm32f4xx_hal.h"
#include "relay_task.h"
#define help_on()			 relay.gas_status |=  (0x01 <<0)
#define help_off()		 relay.gas_status &= ~(0x01 <<0)
#define press_on()		 relay.gas_status |=  (0x01 <<1)
#define press_off()		 relay.gas_status &= ~(0x01 <<1)
#define claw_on() 		 relay.gas_status |=  (0x01 <<2)
#define claw_off()		 relay.gas_status &= ~(0x01 <<2)
#define bracket_on()	 relay.gas_status |=  (0x01 <<3)
#define bracket_off()	 relay.gas_status &= ~(0x01 <<3)
#define rotate_on()		 relay.gas_status |=  (0x01 <<4)
#define rotate_off()   relay.gas_status &= ~(0x01 <<4)
#define magazine_on()	 relay.gas_status |=  (0x01 <<5)
#define magazine_off() relay.gas_status &= ~(0x01 <<5)
#define exb_on()			 relay.gas_status |=  (0x01 <<6)
#define exb_off() 		 relay.gas_status &= ~(0x01 <<6)
#define camera_on()		 relay.gas_status |=  (0x01 <<7)
#define camera_off()	 relay.gas_status &= ~(0x01 <<7)

typedef enum
{
	OFF_MODE,
	ON_MODE,
} pump_mode_e;//气缸


typedef struct
{
  pump_mode_e  press_ctrl_mode;
  pump_mode_e  bracket_ctrl_mode;
  pump_mode_e  claw_ctrl_mode;	//抱柱的气动爪
	pump_mode_e  help_ctrl_mode;
	pump_mode_e  magazine_ctrl_mode;
	pump_mode_e  rotate_ctrl_mode;
	pump_mode_e  camera_ctrl_mode;

} pump_t;
__BSP_PUMP_EXT void rotate_executed(void);
__BSP_PUMP_EXT void claw_executed(void);
__BSP_PUMP_EXT void help_executed(void);
__BSP_PUMP_EXT void bracket_executed(void);
__BSP_PUMP_EXT void press_executed(void);
__BSP_PUMP_EXT void pump_init(void);
__BSP_PUMP_EXT void camera_executed(void);
__BSP_PUMP_EXT void magazine_executed(void);
__BSP_PUMP_EXT pump_t pump;
#endif

