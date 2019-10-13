/** 
  * @file bsp_uart.c
  * @version 1.0
  * @date Nov,05th 2018
	*
  * @brief  底层串口配置，
	*
  *	@author lin kr
  *
  */
#define __BSP_UART_GLOBALS                                                                                                    
#include "string.h"
#include "stdlib.h"
#include "bsp_uart.h"
#include "usart.h"
#include "main.h"
#include "remote_msg.h"
#include "JY901.h"
#include "bsp_JY901.h"
#include "status_task.h"
#include "relay_task.h"
#include "cmsis_os.h"
#include "judge_unpack_task.h"



uint8_t judge_dma_rxbuff[2][UART_RX_DMA_SIZE];
uint8_t judge_dma_txbuff[UART_TX_DMA_SIZE];
extern TaskHandle_t judge_unpack_task_t;

uint16_t test;

/**
  * @brief      enable global uart it and do not use DMA transfer done it
  * @param[in]  huart: uart IRQHandler id
  * @param[in]  pData: receive buff 
  * @param[in]  Size:  buff size
  * @retval     set success or fail
  */
static int uart_receive_dma_no_it(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{
  uint32_t tmp1 = 0;

  tmp1 = huart->RxState;
	
	if (tmp1 == HAL_UART_STATE_READY)
	{
		if ((pData == NULL) || (Size == 0))
		{
			return HAL_ERROR;
		}

		huart->pRxBuffPtr = pData;
		huart->RxXferSize = Size;
		huart->ErrorCode  = HAL_UART_ERROR_NONE;

		/* Enable the DMA Stream */
		HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);

		/* 
		 * Enable the DMA transfer for the receiver request by setting the DMAR bit
		 * in the UART CR3 register 
		 */
		SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

		return HAL_OK;
	}
	else
	{
		return HAL_BUSY;
	}
}

/**
  * @brief      returns the number of remaining data units in the current DMAy Streamx transfer.
  * @param[in]  dma_stream: where y can be 1 or 2 to select the DMA and x can be 0
  *             to 7 to select the DMA Stream.
  * @retval     The number of remaining data units in the current DMAy Streamx transfer.
  */
uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream)
{
  /* Return the number of remaining data units for DMAy Streamx */
  return ((uint16_t)(dma_stream->NDTR));
}


/**
  * @brief      clear idle it flag after uart receive a frame data
  * @param[in]  huart: uart IRQHandler id
  * @retval  
  */
static void uart_rx_idle_callback(UART_HandleTypeDef* huart)
{
	
  
	/* clear idle it flag avoid idle interrupt all the time */
	__HAL_UART_CLEAR_IDLEFLAG(huart);

	/* handle received data in idle interrupt */

		/* clear DMA transfer complete flag */
	__HAL_DMA_DISABLE(huart->hdmarx);
	uint32_t DMA_FLAGS = __HAL_DMA_GET_TC_FLAG_INDEX(huart->hdmarx);
  __HAL_DMA_DISABLE(huart->hdmarx);
	__HAL_DMA_CLEAR_FLAG(huart->hdmarx, DMA_FLAGS);
	/* handle received data in idle interrupt */

	if (huart == &DBUS_HUART)
	{		
		/* restart dma transmission */
		__HAL_DMA_SET_COUNTER(huart->hdmarx, DBUS_MAX_LEN);
	} 	
	else if(huart == &RELAY_HUART)
  {
		__HAL_DMA_SET_COUNTER(huart->hdmarx, RELAY_MAX_LEN);
  }	
	else if(huart == &IMU_HUART)
  {
		__HAL_DMA_SET_COUNTER(huart->hdmarx, IMU_MAX_LEN);
  }		
		__HAL_DMA_ENABLE(huart->hdmarx);
 

}

/**
  * @brief      callback this function when uart interrupt 
  * @param[in]  huart: uart IRQHandler id
  * @retval  
  */
void uart_receive_handler(UART_HandleTypeDef *huart)
{  
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) && 
			__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
	{
		if (huart == &DBUS_HUART)
		{
			rc_callback_handler(&rc, dbus_buf);
			status.dbus_status = 1;
			uart_rx_idle_callback(huart);

		}
		if (huart == &IMU_HUART)
		{
			imu_decoding(imu_buf,IMU_BUFLEN);
			uart_rx_idle_callback(huart);

		}	
		if (huart == &RELAY_HUART)
		{
			relay_decoding();
//			test++;
//			imu_decoding(imu_buf,IMU_BUFLEN);		//陀螺仪串口读取
//			status.imu_status = 1;
			uart_rx_idle_callback(huart);
		}
		if (huart == &DEBUG_HUART)
		{
			test++;

			uart_rx_idle_callback(huart);
		}
    if (huart == &JUDGE_HUART)
		{
    osSignalSet(judge_unpack_task_t, JUDGE_UART_IDLE_SIGNAL);
		}
	}
}

/**
  * @brief   initialize dbus/imu uart device 
  * @param   
  * @retval  
  */
void uart_init(void)
{
	/* open uart idle it */
	__HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);
	__HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);

	uart_receive_dma_no_it(&DBUS_HUART, dbus_buf, DBUS_MAX_LEN);
	
	__HAL_UART_CLEAR_IDLEFLAG(&RELAY_HUART);
	__HAL_UART_ENABLE_IT(&RELAY_HUART, UART_IT_IDLE);

	uart_receive_dma_no_it(&RELAY_HUART, relay_buf, RELAY_MAX_LEN);

	__HAL_UART_CLEAR_IDLEFLAG(&DEBUG_HUART);
	__HAL_UART_ENABLE_IT(&DEBUG_HUART, UART_IT_IDLE);

	uart_receive_dma_no_it(&DEBUG_HUART, debug_buf, DEBUG_MAX_LEN);

	__HAL_UART_CLEAR_IDLEFLAG(&IMU_HUART);
	__HAL_UART_ENABLE_IT(&IMU_HUART, UART_IT_IDLE);

	uart_receive_dma_no_it(&IMU_HUART, imu_buf, IMU_MAX_LEN);

}


void relay_decoding(void)
{
	relay.dis1 = relay_buf[0]<<8|relay_buf[1];
	relay.dis2 = relay_buf[2]<<8|relay_buf[3];
	if(relay_buf[4]==0x00)	{relay.bullet_left = 0;relay.bullet_right = 0;}
	else if(relay_buf[4]==0x40){relay.bullet_left = 0;relay.bullet_right = 1;}
	else if(relay_buf[4]==0x80){relay.bullet_left = 1;relay.bullet_right = 0;}
	else if(relay_buf[4]==0xC0){relay.bullet_left = 1;relay.bullet_right = 1;}
	
//	relay.bullet_left  = relay_buf[4]&0x80>>7;
//	relay.bullet_right = relay_buf[4]&0x40>>6;
}



void uart_dma_full_signal(UART_HandleTypeDef *huart)
{
  if (huart == &JUDGE_HUART)
  {
    /* remove DMA buffer full interrupt handler */
    osSignalSet(judge_unpack_task_t, JUDGE_DMA_FULL_SIGNAL);
  }
}

static void dma_m1_rxcplt_callback(DMA_HandleTypeDef *hdma)
{
  UART_HandleTypeDef* huart = ( UART_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
  uart_dma_full_signal(huart);
}
/* Current memory buffer used is Memory 1 */
static void dma_m0_rxcplt_callback(DMA_HandleTypeDef *hdma)
{
  UART_HandleTypeDef* huart = ( UART_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
  uart_dma_full_signal(huart);
}
static HAL_StatusTypeDef DMAEx_MultiBufferStart_IT(DMA_HandleTypeDef *hdma, \
                                                   uint32_t SrcAddress, \
                                                   uint32_t DstAddress, \
                                                   uint32_t SecondMemAddress, \
                                                   uint32_t DataLength)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  /* Memory-to-memory transfer not supported in double buffering mode */
  if (hdma->Init.Direction == DMA_MEMORY_TO_MEMORY)
  {
    hdma->ErrorCode = HAL_DMA_ERROR_NOT_SUPPORTED;
    return HAL_ERROR;
  }
  
  /* Set the UART DMA transfer complete callback */
  /* Current memory buffer used is Memory 1 callback */
  hdma->XferCpltCallback   = dma_m0_rxcplt_callback;
  /* Current memory buffer used is Memory 0 callback */
  hdma->XferM1CpltCallback = dma_m1_rxcplt_callback;
  
  /* Check callback functions */
  if ((NULL == hdma->XferCpltCallback) || (NULL == hdma->XferM1CpltCallback))
  {
    hdma->ErrorCode = HAL_DMA_ERROR_PARAM;
    return HAL_ERROR;
  }
  
  /* Process locked */
  __HAL_LOCK(hdma);
  
  if(HAL_DMA_STATE_READY == hdma->State)
  {
    /* Change DMA peripheral state */
    hdma->State = HAL_DMA_STATE_BUSY;
    /* Initialize the error code */
    hdma->ErrorCode = HAL_DMA_ERROR_NONE;
    /* Enable the Double buffer mode */
    hdma->Instance->CR |= (uint32_t)DMA_SxCR_DBM;
    /* Configure DMA Stream destination address */
    hdma->Instance->M1AR = SecondMemAddress;
    
    /* Configure DMA Stream data length */
    hdma->Instance->NDTR = DataLength;
    /* Configure the source, destination address */
    if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
    {
      hdma->Instance->PAR = DstAddress;
      hdma->Instance->M0AR = SrcAddress;
    }
    else
    {
      hdma->Instance->PAR = SrcAddress;
      hdma->Instance->M0AR = DstAddress;
    }
    
    /* Clear TC flags */
    __HAL_DMA_CLEAR_FLAG (hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma));
    /* Enable TC interrupts*/
    hdma->Instance->CR  |= DMA_IT_TC;
    
    /* Enable the peripheral */
    __HAL_DMA_ENABLE(hdma);
    
    /* Change the DMA state */
    hdma->State = HAL_DMA_STATE_READY;
  }
  else
  {
    /* Return error status */
    status = HAL_BUSY;
  }
  
  /* Process unlocked */
  __HAL_UNLOCK(hdma);
  
  return status; 
}

/**
  * @brief  Returns the current memory target used by double buffer transfer.
  * @param  dma_stream: where y can be 1 or 2 to select the DMA and x can be 0
  *          to 7 to select the DMA Stream.
  * @retval The memory target number: 0 for Memory0 or 1 for Memory1. 
  */
uint8_t dma_current_memory_target(DMA_Stream_TypeDef *dma_stream)
{
  uint8_t tmp = 0;

  /* Get the current memory target */
  if ((dma_stream->CR & DMA_SxCR_CT) != 0)
  {
    /* Current memory buffer used is Memory 1 */
    tmp = 1;
  }
  else
  {
    /* Current memory buffer used is Memory 0 */
    tmp = 0;
  }
  return tmp;
}


/**
  * @brief   initialize uart device 
  */

void judgement_uart_init(void)
{
  //open uart idle it
  __HAL_UART_CLEAR_IDLEFLAG(&JUDGE_HUART);
  __HAL_UART_ENABLE_IT(&JUDGE_HUART, UART_IT_IDLE);
  
  // Enable the DMA transfer for the receiver request
  SET_BIT(JUDGE_HUART.Instance->CR3, USART_CR3_DMAR); //串口接收数据使能
  
  DMAEx_MultiBufferStart_IT(JUDGE_HUART.hdmarx, \
                           (uint32_t)&JUDGE_HUART.Instance->DR, \
                           (uint32_t)judge_dma_rxbuff[0], \
                           (uint32_t)judge_dma_rxbuff[1], \
                           UART_RX_DMA_SIZE);

}

