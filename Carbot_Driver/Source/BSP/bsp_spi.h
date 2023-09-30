#ifndef __BSP_SPI_H__
#define __BSP_SPI_H__

#include "stm32f10x.h"

#define SPI_CS_PORT         GPIOB
#define SPI_CS_PIN          GPIO_Pin_12

#define SPI_CS_ASSERT()     GPIO_ResetBits(SPI_CS_PORT, SPI_CS_PIN)
#define SPI_CS_DEASSERT()   GPIO_SetBits(SPI_CS_PORT, SPI_CS_PIN)

#define SPI_TIMEOUT_COUNT         200
#define SPI_TRANSFER_OK           0
#define SPI_TRANSMIT_TIMEOUT      -1
#define SPI_RECEIVE_TIMEOUT       -2


void SPI2_Init(void);
uint8_t SPI2_ReadWriteByte(uint8_t TxData, uint8_t* RxData);

#endif /* __BSP_SPI_H__ */
