#ifndef __BSP_PWM_SERVO_H__
#define __BSP_PWM_SERVO_H__


#include "bsp_common.h"



#define USE_SERVO_J1
#define USE_SERVO_J2
#define USE_SERVO_J3
#define USE_SERVO_J4
#define USE_SERVO_J5_
#define USE_SERVO_J6_

#define Servo_J1_PORT	GPIOC
#define Servo_J2_PORT	GPIOC
#define Servo_J3_PORT	GPIOC
#define Servo_J4_PORT	GPIOC
#define Servo_J5_PORT	GPIOC
#define Servo_J6_PORT	GPIOC

#define Servo_J1_PIN	GPIO_Pin_3
#define Servo_J2_PIN	GPIO_Pin_2
#define Servo_J3_PIN	GPIO_Pin_1
#define Servo_J4_PIN	GPIO_Pin_0
#define Servo_J5_PIN	GPIO_Pin_0
#define Servo_J6_PIN	GPIO_Pin_0

#define Servo_J1_CLK	RCC_APB2Periph_GPIOC
#define Servo_J2_CLK	RCC_APB2Periph_GPIOC
#define Servo_J3_CLK    RCC_APB2Periph_GPIOC
#define Servo_J4_CLK	RCC_APB2Periph_GPIOC
#define Servo_J5_CLK	RCC_APB2Periph_GPIOC
#define Servo_J6_CLK	RCC_APB2Periph_GPIOC


#define SERVO_1_HIGH()  PCout(3) = 1
#define SERVO_1_LOW()   PCout(3) = 0

#define SERVO_2_HIGH()  PCout(2) = 1
#define SERVO_2_LOW()   PCout(2) = 0

#define SERVO_3_HIGH()  PCout(1) = 1
#define SERVO_3_LOW()   PCout(1) = 0

#define SERVO_4_HIGH()  PCout(0) = 1
#define SERVO_4_LOW()   PCout(0) = 0

#define SERVO_5_HIGH()  PCout(0) = 1
#define SERVO_5_LOW()   PCout(0) = 0

#define SERVO_6_HIGH()  PCout(0) = 1
#define SERVO_6_LOW()   PCout(0) = 0


typedef enum _PWMServo_ID
{
    PWMServo_ID_S1 = 0,
    PWMServo_ID_S2,
    PWMServo_ID_S3,
    PWMServo_ID_S4,
    
    MAX_PWM_SERVO
} PWMServo_ID;

void PwmServo_Init(void);
void PwmServo_Set_Angle(uint8_t index, uint8_t angle);
void PwmServo_Set_Angle_All(uint8_t angle_s1, uint8_t angle_s2, uint8_t angle_s3, uint8_t angle_s4);
void PwmServo_Handle(void);
uint8_t* PwmServo_Get_Angle(void);

#endif
