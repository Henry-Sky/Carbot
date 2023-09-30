#ifndef __APP_BAT_H__
#define __APP_BAT_H__

#include "stm32f10x.h"

typedef enum Battery_State
{
    BATTERY_LOW,          // 电池电压过低
    BATTERY_NORMAL,       // 电池电压正常
    BATTERY_OVER_VOLTAGE  // 电池电压过高
} Battery_State_t;


uint8_t System_Enable(void);
uint8_t Bat_State(void);
int Bat_Voltage_Z10(void);
uint8_t Bat_Get_Low_Voltage(void);
uint8_t Bat_Get_Over_Voltage(void);
uint8_t Bat_Show_LED_Handle(uint8_t enable_beep);

#endif /* __APP_BAT_H__ */
