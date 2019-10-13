/**
  * @file judge_unpack_task.h
  * @version 1.0
  * @date 
  *
  * @brief  
  *
  *	@author link
  *
  */
#ifndef __JUDGE_SEND_TASK_H__
#define __JUDGE_SEND_TASK_H__

#include "usart.h"
#include "data_fifo.h"
#include "protocol.h"



#define JUDGE_SEND_PERIOD 150



void judge_send_task(void const *argu);
void data_packet_pack(uint16_t cmd_id,uint16_t data_cmd_id,uint16_t robot_id,uint16_t client_id,
                      uint8_t *p_data, uint16_t len, uint8_t sof);
uint32_t send_packed_fifo_data(fifo_s_t *pfifo, uint8_t sof);


#endif

