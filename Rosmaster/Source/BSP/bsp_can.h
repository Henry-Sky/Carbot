#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#include "stdint.h"

#define CAN_D_GPIO_PORT         GPIOB
#define CAN_D_GPIO_PIN          GPIO_Pin_9
#define CAN_D_GPIO_CLK          RCC_APB2Periph_GPIOB
#define CAN_R_GPIO_PORT         GPIOB
#define CAN_R_GPIO_PIN          GPIO_Pin_8
#define CAN_R_GPIO_CLK          RCC_APB2Periph_GPIOB


#define CAN_TX_TIMEOUT          1000
#define CAN_STANDARD_ID         (0x000F)
#define CAN_EXTENDED_ID         (0x00AA0000)

#define CAN1_IRQ_PREEMPT_PRIORITY  (0)
#define CAN1_IRQ_SUB_PRIORITY      (0)


typedef enum CAN_BAUDRATE 
{
    CAN_BAUD_50Kbps = 5,
    CAN_BAUD_100Kbps = 10,
    CAN_BAUD_250Kbps = 25,
    CAN_BAUD_500Kbps = 50,
    CAN_BAUD_800Kbps = 80,
    CAN_BAUD_1000Kbps = 100,

} EN_CAN_BAUDRATE;


typedef struct _CAN_Data_t
{
    uint8_t data[8];
} CAN_Data_t;


void CAN_Config_Init(EN_CAN_BAUDRATE baudrate);
void CAN_Send_Data(CAN_Data_t* data);
void CAN_Receive_Data(CAN_Data_t* Can);

void CAN_Test_Send(void);

void CAN_RX_Beep(uint8_t enable);

#endif /* __BSP_CAN_H__ */
