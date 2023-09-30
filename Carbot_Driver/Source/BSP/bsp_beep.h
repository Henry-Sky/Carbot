#ifndef __BEEP_H__
#define __BEEP_H__
#include "stm32f10x.h"

/* ������������ӵ�GPIO�˿�*/
// #define BEEP_GPIO_PORT GPIOA                /* GPIO�˿� */
// #define BEEP_GPIO_CLK  RCC_APB2Periph_GPIOA /* GPIO�˿�ʱ�� */
// #define BEEP_GPIO_PIN  GPIO_Pin_8           /* ���ӵ���������GPIO */

#define BEEP_GPIO_PORT GPIOC                /* GPIO�˿� */
#define BEEP_GPIO_CLK  RCC_APB2Periph_GPIOC /* GPIO�˿�ʱ�� */
#define BEEP_GPIO_PIN  GPIO_Pin_5           /* ���ӵ���������GPIO */

#define BEEP_ON()  GPIO_SetBits(BEEP_GPIO_PORT, BEEP_GPIO_PIN)
#define BEEP_OFF() GPIO_ResetBits(BEEP_GPIO_PORT, BEEP_GPIO_PIN)

#define BEEP_STATE_OFF       0
#define BEEP_STATE_ON_ALWAYS 1
#define BEEP_STATE_ON_DELAY  2

void Beep_GPIO_Init(void);

void Beep_On_Time(uint16_t time);
void Beep_Timeout_Close_Handle(void);

#endif /* __BEEP_H__ */
