
#ifndef __BSP_MOTOR_H__
#define __BSP_MOTOR_H__

#include "stm32f10x.h"

#define MOTOR_A_IN1_PORT  GPIOB                /* GPIO端口 */
#define MOTOR_A_IN1_PIN   GPIO_Pin_0          /* GPIO编号 */
#define MOTOR_A_IN1_CLK   RCC_APB2Periph_GPIOB /* GPIO端口时钟 */
#define MOTOR_A_IN2_PORT  GPIOB
#define MOTOR_A_IN2_PIN   GPIO_Pin_1
#define MOTOR_A_IN2_CLK   RCC_APB2Periph_GPIOB


#define MOTOR_B_IN1_PORT  GPIOB
#define MOTOR_B_IN1_PIN   GPIO_Pin_5
#define MOTOR_B_IN1_CLK   RCC_APB2Periph_GPIOB
#define MOTOR_B_IN2_PORT  GPIOB
#define MOTOR_B_IN2_PIN   GPIO_Pin_4
#define MOTOR_B_IN2_CLK   RCC_APB2Periph_GPIOB


#define MOTOR_C_IN1_PORT  GPIOD
#define MOTOR_C_IN1_PIN   GPIO_Pin_2
#define MOTOR_C_IN1_CLK   RCC_APB2Periph_GPIOD
#define MOTOR_C_IN2_PORT  GPIOC
#define MOTOR_C_IN2_PIN   GPIO_Pin_12
#define MOTOR_C_IN2_CLK   RCC_APB2Periph_GPIOC


#define MOTOR_D_IN1_PORT  GPIOC
#define MOTOR_D_IN1_PIN   GPIO_Pin_11
#define MOTOR_D_IN1_CLK   RCC_APB2Periph_GPIOC
#define MOTOR_D_IN2_PORT  GPIOC
#define MOTOR_D_IN2_PIN   GPIO_Pin_10
#define MOTOR_D_IN2_CLK   RCC_APB2Periph_GPIOC


#define M1A_PORT  GPIOA
#define M1A_PIN   GPIO_Pin_11
#define M1A_CLK   RCC_APB2Periph_GPIOA
#define M1B_PORT  GPIOA
#define M1B_PIN   GPIO_Pin_8
#define M1B_CLK   RCC_APB2Periph_GPIOA

#define M2A_PORT  GPIOB
#define M2A_PIN   GPIO_Pin_0
#define M2A_CLK   RCC_APB2Periph_GPIOB
#define M2B_PORT  GPIOB
#define M2B_PIN   GPIO_Pin_1
#define M2B_CLK   RCC_APB2Periph_GPIOB

#define M3A_PORT  GPIOC
#define M3A_PIN   GPIO_Pin_6
#define M3A_CLK   RCC_APB2Periph_GPIOC
#define M3B_PORT  GPIOC
#define M3B_PIN   GPIO_Pin_7
#define M3B_CLK   RCC_APB2Periph_GPIOC

#define M4A_PORT  GPIOC
#define M4A_PIN   GPIO_Pin_8
#define M4A_CLK   RCC_APB2Periph_GPIOC
#define M4B_PORT  GPIOC
#define M4B_PIN   GPIO_Pin_9
#define M4B_CLK   RCC_APB2Periph_GPIOC


#define PWM_M1_A  TIM8->CCR1
#define PWM_M1_B  TIM8->CCR2

#define PWM_M2_A  TIM8->CCR3
#define PWM_M2_B  TIM8->CCR4

#define PWM_M3_A  TIM1->CCR4
#define PWM_M3_B  TIM1->CCR1

#define PWM_M4_A  TIM1->CCR2
#define PWM_M4_B  TIM1->CCR3


#define MOTOR_ENABLE_A      (0x01)
#define MOTOR_ENABLE_B      (0x02)
#define MOTOR_ENABLE_C      (0x04)
#define MOTOR_ENABLE_D      (0x08)


#define MOTOR_SUNRISE_IGNORE_PULSE  (2000)
#define MOTOR_IGNORE_PULSE  (1600)
#define MOTOR_MAX_PULSE     (3600)
#define MOTOR_FREQ_DIVIDE   (0)


// MOTOR: M1 M2 M3 M4
// MOTOR: L1 L2 R1 R2
typedef enum {
    MOTOR_ID_M1 = 0,
    MOTOR_ID_M2,
    MOTOR_ID_M3,
    MOTOR_ID_M4,
    MAX_MOTOR
} Motor_ID;


void Motor_PWM_Init(uint16_t arr, uint16_t psc);

void Motor_Set_Pwm(uint8_t id, int16_t speed);
void Motor_Stop(uint8_t brake);
void MOTOR_GPIO_Init(void);

uint8_t Motor_Get_Enable_State(uint8_t id);

void Motor_Check_Start(void);
int Motor_Check_Result(uint8_t Encoder_id);

void Motor_Close_Brake(void);

#endif
