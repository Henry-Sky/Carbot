#ifndef __APP_OLED_H__
#define __APP_OLED_H__
#include "stdbool.h"
#include "stm32f10x.h"

typedef enum en_OLED_FLAG{
    OLED_FLAG_NO_DISPLAY = 0,
    OLED_FLAG_IMU,
    OLED_FLAG_VOLTAGE,
    OLED_FLAG_MOTOR_SPEED,
    OLED_FLAG_UART_SERVO,

    OLED_MAX_FLAG
} OLED_FLAG;


void OLED_Clear(void);
void OLED_Refresh(void);
void OLED_Draw_String(char *data, uint8_t x, uint8_t y, bool clear, bool refresh);
void OLED_Draw_Line(char *data, uint8_t line, bool clear, bool refresh);


void OLED_Show_CarType(uint8_t car_type, uint8_t v_major, uint8_t v_minor, uint8_t v_patch);
void OLED_Show_Voltage(uint16_t bat_voltage);
void OLED_Show_IMU_Attitude(float yaw, float roll, float pitch);
void OLED_Show_YAW(float yaw);
void OLED_Show_Test_Mode(void);
void OLED_Show_Test_Mode_Error(void);
void OLED_Show_Motor_Speed(float m1, float m2, float m3, float m4);
void OLED_Show_UART_Servo_Read(uint16_t read_value);
void OLED_Show_Waiting(void);
void OLED_Show_Error(void);

#endif
