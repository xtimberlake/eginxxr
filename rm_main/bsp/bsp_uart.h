/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       bsp_uart.h
 * @brief      this file contains the common defines and functions prototypes for 
 *             the bsp_uart.c driver
 * @note         
 * @Version    V1.0.0
 * @Date       Jan-30-2018      
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */

#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#ifdef  __BSP_UART_GLOBALS
#define __BSP_UART_EXT
#else
#define __BSP_UART_EXT extern
#endif

#include "usart.h"
#include "remote_msg.h"
#include "bsp_JY901.h"
#include "debug_task.h"

#define UART_RX_DMA_SIZE (1024)
#define DBUS_HUART       huart1 /* for dji remote controler reciever */
#define IMU_HUART				 huart2

#define RELAY_HUART      huart6

#define DEBUG_HUART      huart5
#define RELAY_BUFLEN  6
#define RELAY_MAX_LEN 6
__BSP_UART_EXT uint8_t   dbus_buf[DBUS_BUFLEN];
__BSP_UART_EXT uint8_t   imu_buf[IMU_BUFLEN];
__BSP_UART_EXT uint8_t   debug_buf[DEBUG_BUFLEN];
__BSP_UART_EXT uint8_t   relay_buf[RELAY_BUFLEN];

#define UART_TX_DMA_SIZE       28

#define JUDGE_HUART        huart3

void uart_receive_handler(UART_HandleTypeDef *huart);
void uart_init(void);
void USART_SendData(USART_TypeDef* USARTx, uint16_t Data);
void relay_decoding(void);

void judgement_uart_init(void);
uint8_t dma_current_memory_target(DMA_Stream_TypeDef *dma_stream);
uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream);

extern uint8_t judge_dma_rxbuff[2][UART_RX_DMA_SIZE];
extern uint8_t judge_dma_txbuff[UART_TX_DMA_SIZE];

#endif

