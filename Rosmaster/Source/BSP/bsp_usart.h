#ifndef __BSP_USART_H__
#define __BSP_USART_H__


#include "stm32f10x.h"


/* 宏定义 --------------------------------------------------------------------*/
#define ENABLE_USART2                  1
#define ENABLE_SBUS                    1
#define ENABLE_USART3                  1
#define ENABLE_UART4                   0

#define USART1_BAUDRATE                115200
#if ENABLE_SBUS
#define USART2_BAUDRATE                100000
#else
#define USART2_BAUDRATE                115200
#endif
#define USART3_BAUDRATE                115200
#define UART4_BAUDRATE                 115200


//调试串口-USART
#if ENABLE_UART4
#define  DEBUG_USARTx                   UART4
#else
#define  DEBUG_USARTx                   USART1
#endif

/* 定义变量 --------------------------------------------------------------------*/


/* 函数申明 ------------------------------------------------------------------*/
void USART1_Init(uint32_t baudrate);
void USART1_Send_U8(uint8_t ch);
void USART1_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length);

#if ENABLE_USART2
void USART2_Init(uint32_t baudrate);
void USART2_Send_U8(uint8_t Data);
void USART2_Send_ArrayU8(uint8_t *pData, uint16_t Length);
void USART2_Connect_BT(void);
#endif

#if ENABLE_USART3
void USART3_Init(uint32_t baudrate);
void USART3_Send_U8(uint8_t Data);
void USART3_Send_ArrayU8(uint8_t *pData, uint16_t Length);
#endif

#if ENABLE_UART4
void UART4_Init(uint32_t baudrate);
#endif

#endif /* __BSP_USART_H__ */
