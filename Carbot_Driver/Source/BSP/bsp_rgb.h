#ifndef __BSP_RGB_H__
#define __BSP_RGB_H__

#include "stm32f10x.h"

// SPI驱动开关, 为0则使用PWM
#define RGB_DRV_SPI     1

#define RGB_CTRL_ALL    0xFF
#define MAX_RGB         14


#if RGB_DRV_SPI
#define Colorful_PORT   GPIOB
#define Colorful_PIN    GPIO_Pin_5
#define Colorful_RCC    RCC_APB2Periph_GPIOB
#else
#define Colorful_PORT   GPIOA
#define Colorful_PIN    GPIO_Pin_8
#define Colorful_RCC    RCC_APB2Periph_GPIOA
#endif

void RGB_Init(void);
void RGB_Update(void);

void RGB_Set_Color(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
void RGB_Set_Color_U32(uint8_t index, uint32_t color);
void RGB_Clear(void);

#endif /* __BSP_RGB_H__ */
