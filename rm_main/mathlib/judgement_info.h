/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
/** @file judgement_info.h
 *  @version 1.0
 *  @date Feb 2019
 *
 *  @brief the information from judgement system
 *
 *  @copyright 2019 DJI RoboMaster. All rights reserved.
 *
 */

#ifndef __JUDGEMENT_INFO_H__
#define __JUDGEMENT_INFO_H__

#include "stm32f4xx_hal.h"

#define JUDGE_FIFO_BUFLEN 500

/** 
  * @brief  judgement data command id
  */
typedef enum
{
  GAME_STATE_INFO_ID      = 0x0001, //比赛状态数据，1Hz 周期发送
  GAME_RESULT_ID          = 0x0002, //比赛结果数据，比赛结束后
  GAME_ROBOT_LIVE_ID      = 0x0003, //比赛机器人存活数据，1Hz 发送
	
  FIELD_EVENT_DATA_ID     = 0x0101, //场地事件数据，事件改变后
	FIELD_SUPPY_ACTION_ID   = 0x0102, //场地补给站动作标识数据，动作改变后发送
	FIELD_SUPPY_BOOKING_ID  = 0X0103, //场地补给站预约子弹数据，由参赛队发送
	
  ROBOT_STATE_DATA_ID     = 0x0201, //机器人状态数据,10Hz 周期发送
  REAL_POWER_HEAT_DATA_ID = 0x0202, //实时功率热量数据，50Hz 周期发送
	ROBOT_POS_DATA_ID       = 0x0203, //机器人位置数据，10Hz 发送
	ROBOT_GAIN_BUFF_ID      = 0x0204, //机器人增益数据,增益状态改变后发送
	DRONE_ENERGY_DATA_ID    = 0X0205, //空中机器人能量状态数据，10Hz 周期发送，只有空中机器人主控发送
	REAL_BLOOD_DATA_ID      = 0X0206, //伤害状态数据，伤害发生后发送
  REAL_SHOOT_DATA_ID      = 0X0207, //实时射击数据，子弹发射后发送
	
  ROBOT_INTERACTIVE_ID    = 0x0301, //机器人间交互数据 、客户端自定义数据，上限 10Hz
	STU_CUSTOM_DATA_CMD_ID  = 0xD180, //客户端自定义数据内容ID
} judge_data_id_e;

typedef enum
{
	//Red
  Hero_R1     = 1,
	Engineer_R2 = 2,
	Standard_R3 = 3,
	Standard_R4 = 4,
	Standard_R5 = 5,
	Aerial_R6   = 6,
	Sentry_R7   = 7,
	//Blue
	Hero_B1     = 11,
	Engineer_B2 = 12,
	Standard_B3 = 13,
	Standard_B4 = 14,
	Standard_B5 = 15,
	Aerial_B6   = 16,
	Sentry_B7   = 17,
} robot_id_e;

typedef enum
{
	//Red
  Hero_CLient_R1     = 0x0101,
	Engineer_CLient_R2 = 0x0102,
	Standard_CLient_R3 = 0x0103,
	Standard_CLient_R4 = 0x0104,
	Standard_CLient_R5 = 0x0105,
	Aerial_CLient_R6   = 0x0106,
	//Blue
	Hero_CLient_B1     = 0x0111,
	Engineer_CLient_B2 = 0x0112,
	Standard_CLient_B3 = 0x0113,
	Standard_CLient_B4 = 0x0114,
	Standard_CLient_B5 = 0x0115,
	Aerial_CLient_B6   = 0x0116,
} stu_client_id_e;

//0x001
typedef __packed struct
{
 uint8_t game_type : 4;
 uint8_t game_progress : 4;
 uint16_t stage_remain_time;
} ext_game_state_t;

//0x002
typedef __packed struct
{
 uint8_t winner;
} ext_game_result_t;

//0x003
typedef __packed struct
{
 uint16_t robot_legion;
} ext_game_robot_survivors_t;

//0x0101
typedef __packed struct
{
 uint32_t event_type;
} ext_event_data_t;


//0x0102
typedef __packed struct
{
 uint8_t supply_projectile_id; 
 uint8_t supply_robot_id; 
 uint8_t supply_projectile_step; 
 uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;


//0x0103
typedef __packed struct
{
  uint8_t supply_projectile_id;
  uint8_t supply_robot_id; 
  uint8_t supply_num;
} ext_supply_projectile_booking_t;


typedef __packed struct
{
 uint8_t robot_id;
 uint8_t robot_level;
 uint16_t remain_HP;
 uint16_t max_HP;
 uint16_t shooter_heat0_cooling_rate;
 uint16_t shooter_heat0_cooling_limit;
 uint16_t shooter_heat1_cooling_rate;
 uint16_t shooter_heat1_cooling_limit;
 uint8_t mains_power_gimbal_output : 1;
 uint8_t mains_power_chassis_output : 1;
 uint8_t mains_power_shooter_output : 1;
} ext_game_robot_state_t;

//0x0202
typedef __packed struct
{
 uint16_t chassis_volt; 
 uint16_t chassis_current; 
 float chassis_power; 
 uint16_t chassis_power_buffer; 
 uint16_t shooter_heat0; 
 uint16_t shooter_heat1; 
} ext_power_heat_data_t;

//0x0203
typedef __packed struct
{
 float x;
 float y;
 float z;
 float yaw;
} ext_game_robot_pos_t;

//0x0204
typedef __packed struct
{ 
 uint8_t power_rune_buff;    // bit 1：枪口热量冷却加速
}ext_buff_musk_t;

//0x0205
typedef __packed struct
{
 uint8_t energy_point;
 uint8_t attack_time;
} aerial_robot_energy_t;

//0x0206
typedef __packed struct
{
 uint8_t armor_id : 4;
 uint8_t hurt_type : 4;
} ext_robot_hurt_t;

//0x0207
typedef __packed struct
{
 uint8_t bullet_type;
 uint8_t bullet_freq;
 float bullet_speed;
} ext_shoot_data_t;

//0x0301
typedef __packed struct
{
 uint16_t data_cmd_id;
 uint16_t send_ID;
 uint16_t receiver_ID;
}ext_student_interactive_header_data_t;

//cmd_id:0x0301。内容ID:0xD180
typedef __packed struct
{
 float data1;
 float data2;
 float data3;
 uint8_t masks;
}client_custom_data_t;



typedef __packed struct
{
 uint16_t data_cmd_id;
 uint16_t send_id;
	uint16_t receiver_id;
	
}ext_student_interative_header_data_t;
/** 
  * @brief  the data structure receive from judgement
  */
typedef struct
{
  ext_game_state_t                game_information;      //0x0001
	ext_game_result_t               game_result_data;      //0x0002
	ext_game_robot_survivors_t      robot_survivors_data;  //0x0003
	
	ext_event_data_t                field_event_data;      //0x0101
	ext_supply_projectile_action_t  supply_action_data;    //0x0102
	ext_supply_projectile_booking_t supply_booking_data;   //0x0103

	ext_game_robot_state_t          robot_state_data;      //0x0201
	ext_power_heat_data_t           power_heat_data;       //0x0202
	ext_game_robot_pos_t            robot_pos_data;        //0x0203
	ext_buff_musk_t                 get_buff_data;         //0x0204
	aerial_robot_energy_t           drone_energy_data;     //0x0205
	ext_robot_hurt_t                blood_changed_data;    //0x0206
	ext_shoot_data_t                real_shoot_data;       //0x0207     
	
  ext_student_interactive_header_data_t  stu_interactive_data; //0x0301
	
} receive_judge_t;
/** 
  * @brief  the data structure send to judgement
  */
typedef struct
{
	client_custom_data_t                   show_in_client_data ;
	
  ext_student_interactive_header_data_t  stu_interactive_data;
} send_judge_t;

/* data send (forward) */
/* data receive */
extern receive_judge_t                       judge_recv_mesg;
extern client_custom_data_t                  client_custom_data;
extern ext_student_interactive_header_data_t student_interactive_header_id;
extern send_judge_t                          judge_send_mesg;
extern uint16_t                              Stu_Client_ID;
void  judgement_data_handler(uint8_t *p_frame);

#endif
