#ifndef __BSP_ADC_H__
#define __BSP_ADC_H__

#include "stm32f10x.h"

#define BAT_GPIO_CLK    RCC_APB2Periph_GPIOC
#define BAT_GPIO_PORT   GPIOC
#define BAT_GPIO_PIN    GPIO_Pin_4


#define BAT_ADC         ADC1
#define BAT_ADC_CH      ADC_Channel_14
#define BAT_ADC_CLK     RCC_APB2Periph_ADC1

void Adc_Init(void);
float Adc_Get_Measure_Volotage(void);
float Adc_Get_Battery_Volotage(void);

uint16_t Adc_Get_Average(uint8_t ch, uint8_t times);



#endif /* __BSP_ADC_H__ */
