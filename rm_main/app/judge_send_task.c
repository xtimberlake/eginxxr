/** 
  * @file judge_send_task.c
  * @version 1.0
  * @date    2019.5.4
  * @brief 
  *	@author link
  */
#include "judge_send_task.h"
#include "judge_unpack_task.h"
#include "comm_task.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "judgement_info.h"
#include "protocol.h"
#include "data_fifo.h"
#include "string.h"
#include "stdio.h"


      //send data to judge system
void judge_send_task(void const *argu)
{

		data_packet_pack(ROBOT_INTERACTIVE_ID,
				             STU_CUSTOM_DATA_CMD_ID,
				             judge_recv_mesg.robot_state_data.robot_id,
				             Stu_Client_ID,
				             (uint8_t *)&judge_send_mesg.show_in_client_data,
				             sizeof(client_custom_data_t),
										 DN_REG_ID);
    send_packed_fifo_data(&judge_txdata_fifo, DN_REG_ID);
		judge_send_mesg.show_in_client_data.data1 =  0;
		judge_send_mesg.show_in_client_data.data2 =  0;
	  judge_send_mesg.show_in_client_data.data3 =  0;
		judge_send_mesg.show_in_client_data.masks =  0;

}
uint8_t* protocol_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof, uint8_t *tx_buf)
{
  //memset(tx_buf, 0, 100);
  static uint8_t seq;
  
  uint16_t frame_length = HEADER_LEN + CMD_LEN + len + CRC_LEN;
  frame_header_t *p_header = (frame_header_t*)tx_buf;
  
  p_header->sof          = sof;
  p_header->data_length  = len;
  
  
  if (sof == UP_REG_ID)
  {
    if (seq++ >= 255)
      seq = 0;
    
    p_header->seq = seq;
  }
  else
  {
    p_header->seq = 0;
  }
  
  
  memcpy(&tx_buf[HEADER_LEN], (uint8_t*)&cmd_id, CMD_LEN);
  append_crc8_check_sum(tx_buf, HEADER_LEN);
  memcpy(&tx_buf[HEADER_LEN + CMD_LEN], p_data, len);
  append_crc16_check_sum(tx_buf, frame_length);
  
  return tx_buf;
}

void data_packet_pack(uint16_t cmd_id,uint16_t data_cmd_id,uint16_t robot_id,uint16_t client_id,
                      uint8_t *p_data, uint16_t len, uint8_t sof)
{
  uint8_t tx_buf[PROTOCAL_FRAME_MAX_SIZE];
  uint8_t data_buf[DATA_MAX_SIZE];
  uint16_t frame_length = HEADER_LEN + CMD_LEN + DATA_HEADER_LEN + len + CRC_LEN;
	

  memcpy(&data_buf[0], (uint8_t*)&data_cmd_id, 2);
	memcpy(&data_buf[2], (uint8_t*)&robot_id, 2);
	memcpy(&data_buf[4], (uint8_t*)&client_id, 2);
	memcpy(&data_buf[6], p_data, len);
	
  protocol_packet_pack(cmd_id, data_buf, DATA_HEADER_LEN+len, sof, tx_buf);
  
  /* use mutex operation */

  if (sof == DN_REG_ID)
	{
    fifo_s_puts(&judge_txdata_fifo, tx_buf, frame_length);
	}
    return ;
}

uint32_t send_packed_fifo_data(fifo_s_t *pfifo, uint8_t sof)
{
  uint8_t  tx_buf[JUDGE_FIFO_BUFLEN];

  
  uint32_t fifo_count = fifo_used_count(pfifo);
  
  if (fifo_count)
  {
    fifo_s_gets(pfifo, tx_buf, fifo_count);
    
 if (sof == DN_REG_ID)
	    HAL_UART_Transmit(&JUDGE_HUART,tx_buf,fifo_count, 100);
    else
      return 0;
  }
  
  return fifo_count;
}

