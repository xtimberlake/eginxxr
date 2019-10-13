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
/** @file judgement_info.c
 *  @version 1.0
 *  @date June 2017
 *
 *  @brief the information from judgement system
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#include "judgement_info.h"
#include "comm_task.h"
#include "protocol.h"
#include "bsp_uart.h"
#include "data_fifo.h"
#include "string.h"



/* data send (forward) */
/* data receive */
receive_judge_t judge_recv_mesg;
send_judge_t    judge_send_mesg;
uint16_t Stu_Client_ID;

/**
  * @brief    get judgement system message
  */
extern TaskHandle_t pc_unpack_task_t;
void judgement_data_handler(uint8_t *p_frame)
{
  frame_header_t *p_header = (frame_header_t*)p_frame;
  memcpy(p_header, p_frame, HEADER_LEN);

  uint16_t data_length = p_header->data_length;
  uint16_t cmd_id      = *(uint16_t *)(p_frame + HEADER_LEN);
  uint8_t *data_addr   = p_frame + HEADER_LEN + CMD_LEN;
  

  
  switch (cmd_id)
  {
    case GAME_STATE_INFO_ID:
		{
      memcpy(&judge_recv_mesg.game_information, data_addr, data_length);
		}
    break;
		
    case GAME_RESULT_ID:
		{
      memcpy(&judge_recv_mesg.game_result_data, data_addr, data_length);
		}
		break;
		
		case  GAME_ROBOT_LIVE_ID:
		{
      memcpy(&judge_recv_mesg.robot_survivors_data, data_addr, data_length);
		}
		break;
    
    case FIELD_EVENT_DATA_ID:
		{
      memcpy(&judge_recv_mesg.field_event_data, data_addr, data_length);
		}
		break;
		
		case FIELD_SUPPY_ACTION_ID:
		{
      memcpy(&judge_recv_mesg.supply_action_data, data_addr, data_length);
		}
		break;
		
		case FIELD_SUPPY_BOOKING_ID:
		{
      memcpy(&judge_recv_mesg.supply_booking_data, data_addr, data_length);
		}
		break;

		case ROBOT_STATE_DATA_ID:
		{
      memcpy(&judge_recv_mesg.robot_state_data, data_addr, data_length);
		}
		break;
    
    case REAL_POWER_HEAT_DATA_ID:
		{
      memcpy(&judge_recv_mesg.power_heat_data, data_addr, data_length);
		}
		break;
		
    case ROBOT_POS_DATA_ID:
		{
      memcpy(&judge_recv_mesg.robot_pos_data, data_addr, data_length);
		}
		break;
    
		case ROBOT_GAIN_BUFF_ID:
		{
      memcpy(&judge_recv_mesg.get_buff_data, data_addr, data_length);
		}
		break;
		
		case DRONE_ENERGY_DATA_ID:
		{
      memcpy(&judge_recv_mesg.drone_energy_data, data_addr, data_length);
		}
		break;
		
		case REAL_BLOOD_DATA_ID:
		{
      memcpy(&judge_recv_mesg.blood_changed_data, data_addr, data_length);
		}
		break;

    case REAL_SHOOT_DATA_ID:
		{
      memcpy(&judge_recv_mesg.real_shoot_data, data_addr, data_length);
		}
		break;
		
    default:
		{
			break;
		}
		
  }
  
	
	switch (judge_recv_mesg.robot_state_data.robot_id)
	{
		 case Hero_R1:
		 {
			 Stu_Client_ID=Hero_CLient_R1;
		 }
		 break;
		 case Engineer_R2:
		 {
			 Stu_Client_ID=Engineer_CLient_R2;
		 }
		 break;
		 case Standard_R3:
		 {
			 Stu_Client_ID=Standard_CLient_R3;
		 }
		 break;
		 case Standard_R4:
		 {
			 Stu_Client_ID=Standard_CLient_R4;
		 }
		 break;
		 case Standard_R5:
		 {
			 Stu_Client_ID=Standard_CLient_R5;
		 }
		 break;
		 case Aerial_R6:
		 {
			 Stu_Client_ID=Aerial_CLient_R6;
		 }
		 break;
		 case Hero_B1:
		 {
			 Stu_Client_ID=Hero_CLient_B1;
		 }
		 break;
		 case Engineer_B2:
		 {
			 Stu_Client_ID=Engineer_CLient_B2;
		 }
		 break;
		 case Standard_B3:
		 {
			 Stu_Client_ID=Standard_CLient_B3;
		 }
		 break;
		 case Standard_B4:
		 {
			 Stu_Client_ID=Standard_CLient_B4;
		 }
		 break;
		 case Standard_B5:
		 {
			 Stu_Client_ID=Standard_CLient_B5;
		 }
		 break;
		 case Aerial_B6:
		 {
			 Stu_Client_ID=Aerial_CLient_B6;
		 }
		 break;
		 default:
		 {
			break;
		 }
		}
		

}

