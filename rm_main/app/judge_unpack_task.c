/** 
  * @file judge_unpack_task.c
  * @version 1.0
  * @date    2019.3.6
  * @brief 
  *	@author link
  */

#include "judge_unpack_task.h"
#include "comm_task.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "judgement_info.h"
#include "protocol.h"
#include "data_fifo.h"
#include "string.h"
#include "stdio.h"

#define UP_REG_ID    0xA0  //up layer regional id
#define DN_REG_ID    0xA5  //down layer regional id
#define HEADER_LEN   sizeof(frame_header_t)
#define CMD_LEN      2    //cmdid bytes
#define CRC_LEN      2    //crc16 bytes

/* judge system receive data fifo and buffer*/
static osMutexId judge_rxdata_mutex;
static fifo_s_t  judge_rxdata_fifo;
static uint8_t   judge_rxdata_buf[JUDGE_FIFO_BUFLEN];
/* judge system send data fifo and buffer*/
osMutexId judge_txdata_mutex;
fifo_s_t  judge_txdata_fifo;
uint8_t   judge_txdata_buf[JUDGE_FIFO_BUFLEN];

extern osThreadId can_msg_send_task_t;
	
uart_dma_rxdata_t judge_rx_obj;
unpack_data_t judge_unpack_obj;


int dma_write_len = 0;
int fifo_overflow = 0;

void judge_unpack_task_Init(void)
{
	/* create the judge_rxdata_mutex mutex  */
  osMutexDef(judge_rxdata_mutex);
  judge_rxdata_mutex = osMutexCreate(osMutex(judge_rxdata_mutex));
  
  /* create the judge_txdata_mutex mutex  */
  osMutexDef(judge_txdata_mutex);
  judge_txdata_mutex = osMutexCreate(osMutex(judge_txdata_mutex));
	
	/* judge data fifo init */
  fifo_s_init(&judge_rxdata_fifo, judge_rxdata_buf, JUDGE_FIFO_BUFLEN, judge_rxdata_mutex);
  fifo_s_init(&judge_txdata_fifo, judge_txdata_buf, JUDGE_FIFO_BUFLEN, judge_txdata_mutex);
	
	/* initial judge data dma receiver object */
  judge_rx_obj.huart = &JUDGE_HUART;
  judge_rx_obj.data_fifo = &judge_rxdata_fifo;
  judge_rx_obj.buff_size = UART_RX_DMA_SIZE;
  judge_rx_obj.buff[0] = judge_dma_rxbuff[0];
  judge_rx_obj.buff[1] = judge_dma_rxbuff[1];
	
	/* initial judge data unpack object */
  judge_unpack_obj.data_fifo = &judge_rxdata_fifo;
  judge_unpack_obj.p_header = (frame_header_t *)judge_unpack_obj.protocol_packet;
  judge_unpack_obj.index = 0;
  judge_unpack_obj.data_len = 0;
  judge_unpack_obj.unpack_step = STEP_HEADER_SOF;
}
void judge_unpack_task(void const *argu)
{
  osEvent event;
  
  /* open judge uart receive it */
  judgement_uart_init();
  
  while (1)
  {
    event = osSignalWait(JUDGE_UART_TX_SIGNAL | \
                         JUDGE_UART_IDLE_SIGNAL, osWaitForever);
    
    if (event.status == osEventSignal)
    {
      //receive judge data puts fifo
      if (event.value.signals & JUDGE_UART_IDLE_SIGNAL)
      {
        dma_buffer_to_unpack_buffer(&judge_rx_obj, UART_IDLE_IT);
        unpack_fifo_data(&judge_unpack_obj, DN_REG_ID);
				
				send_chassis_Judgement_power_message((uint16_t)judge_recv_mesg.power_heat_data.chassis_power,
				                                     (uint16_t)judge_recv_mesg.power_heat_data.chassis_power_buffer);
				
      }
      
      //send data to judge system
      if (event.value.signals & JUDGE_UART_TX_SIGNAL)
      {
        
      }
      
    }
    
  }
}

void unpack_fifo_data(unpack_data_t *p_obj, uint8_t sof)
{
  uint8_t byte = 0;
  
  while ( fifo_used_count(p_obj->data_fifo) )
  {
    byte = fifo_s_get(p_obj->data_fifo);
    switch(p_obj->unpack_step)
    {
      case STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          p_obj->unpack_step = STEP_LENGTH_LOW;
          p_obj->protocol_packet[p_obj->index++] = byte;
        }
        else
        {
          p_obj->index = 0;
        }
      }break;
      
      case STEP_LENGTH_LOW:
      {
        p_obj->data_len = byte;
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      case STEP_LENGTH_HIGH:
      {
        p_obj->data_len |= (byte << 8);
        p_obj->protocol_packet[p_obj->index++] = byte;

        if(p_obj->data_len < (PROTOCAL_FRAME_MAX_SIZE - HEADER_LEN - CRC_LEN))
        {
          p_obj->unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }break;
    
      case STEP_FRAME_SEQ:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_HEADER_CRC8;
      }break;

      case STEP_HEADER_CRC8:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;

        if (p_obj->index == HEADER_LEN)
        {
          if ( verify_crc8_check_sum(p_obj->protocol_packet, HEADER_LEN) )
          {
            p_obj->unpack_step = STEP_DATA_CRC16;
          }
          else
          {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
          }
        }
      }break;  

      case STEP_DATA_CRC16:
      {
        if (p_obj->index < (HEADER_LEN + CMD_LEN + p_obj->data_len + CRC_LEN))
        {
           p_obj->protocol_packet[p_obj->index++] = byte;  
        }
        if (p_obj->index >= (HEADER_LEN + CMD_LEN + p_obj->data_len + CRC_LEN))
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;

          if ( verify_crc16_check_sum(p_obj->protocol_packet, HEADER_LEN + CMD_LEN + p_obj->data_len + CRC_LEN) )
          {
            if (sof == UP_REG_ID)
            {
            }
            else  //DN_REG_ID
            {
              judgement_data_handler(p_obj->protocol_packet);
            }
          }
        }
      }break;

      default:
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;
      }break;
    }
  }
}


void dma_buffer_to_unpack_buffer(uart_dma_rxdata_t *dma_obj, uart_it_type_e it_type)
{
  int16_t  tmp_len;
  uint8_t  current_memory_id;
  uint16_t remain_data_counter;
  uint8_t  *pdata = dma_obj->buff[0];
  
  get_dma_memory_msg(dma_obj->huart->hdmarx->Instance, &current_memory_id, &remain_data_counter);
  
  if (UART_IDLE_IT == it_type)
  {
    if (current_memory_id)
    {
      dma_obj->write_index = dma_obj->buff_size*2 - remain_data_counter;
    }
    else
    {
      dma_obj->write_index = dma_obj->buff_size - remain_data_counter;
    }
  }
  else if (UART_DMA_FULL_IT == it_type)
  {
#if 0
    if (current_memory_id)
    {
      dma_obj->write_index = dma_obj->buff_size;
    }
    else
    {
      dma_obj->write_index = dma_obj->buff_size*2;
    }
#endif
  }
  
  if (dma_obj->write_index < dma_obj->read_index)
  {
    dma_write_len = dma_obj->buff_size*2 - dma_obj->read_index + dma_obj->write_index;
    
    tmp_len = dma_obj->buff_size*2 - dma_obj->read_index;
    if (tmp_len != fifo_s_puts(dma_obj->data_fifo, &pdata[dma_obj->read_index], tmp_len))
      fifo_overflow = 1;
    else
      fifo_overflow = 0;
    dma_obj->read_index = 0;
    
    tmp_len = dma_obj->write_index;
    if (tmp_len != fifo_s_puts(dma_obj->data_fifo, &pdata[dma_obj->read_index], tmp_len))
      fifo_overflow = 1;
    else
      fifo_overflow = 0;
    dma_obj->read_index = dma_obj->write_index;
  }
  else
  {
    dma_write_len = dma_obj->write_index - dma_obj->read_index;
    
    tmp_len = dma_obj->write_index - dma_obj->read_index;
    if (tmp_len != fifo_s_puts(dma_obj->data_fifo, &pdata[dma_obj->read_index], tmp_len))
      fifo_overflow = 1;
    else
      fifo_overflow = 0;
    dma_obj->read_index = (dma_obj->write_index) % (dma_obj->buff_size*2);
  }
}

void get_dma_memory_msg(DMA_Stream_TypeDef *dma_stream, uint8_t *mem_id, uint16_t *remain_cnt)
{
  *mem_id     = dma_current_memory_target(dma_stream);
  *remain_cnt = dma_current_data_counter(dma_stream);
}

