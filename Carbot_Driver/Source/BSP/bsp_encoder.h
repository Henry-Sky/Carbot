#ifndef __BSP_ENCODER_H__
#define __BSP_ENCODER_H__

#include "stm32f10x.h"

#define HAL_1A_PIN GPIO_Pin_0
#define HAL_1A_PORT GPIOA
#define HAL_1A_CLK RCC_APB2Periph_GPIOA
#define HAL_1B_PIN GPIO_Pin_1
#define HAL_1B_PORT GPIOA
#define HAL_1B_CLK RCC_APB2Periph_GPIOA

#define HAL_2A_PIN GPIO_Pin_7
#define HAL_2A_PORT GPIOA
#define HAL_2A_CLK RCC_APB2Periph_GPIOA
#define HAL_2B_PIN GPIO_Pin_6
#define HAL_2B_PORT GPIOA
#define HAL_2B_CLK RCC_APB2Periph_GPIOA

#define HAL_3A_PIN GPIO_Pin_15
#define HAL_3A_PORT GPIOA
#define HAL_3A_CLK RCC_APB2Periph_GPIOA
#define HAL_3B_PIN GPIO_Pin_3
#define HAL_3B_PORT GPIOB
#define HAL_3B_CLK RCC_APB2Periph_GPIOB

#define HAL_4A_PIN GPIO_Pin_7
#define HAL_4A_PORT GPIOB
#define HAL_4A_CLK RCC_APB2Periph_GPIOB
#define HAL_4B_PIN GPIO_Pin_6
#define HAL_4B_PORT GPIOB
#define HAL_4B_CLK RCC_APB2Periph_GPIOB


// 不可大于65535 因为F103的定时器是16位的。
#define ENCODER_TIM_PERIOD       (uint16_t)(65535)


void Encoder_Init(void);

int Encoder_Get_Count_Now(uint8_t Motor_id);
void Encoder_Update_Count(void);
void Encoder_Get_ALL(int* Encoder_all);

void Encoder_Send_Count_Now(void);

#endif

