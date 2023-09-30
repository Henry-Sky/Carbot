#ifndef __APP_H__
#define __APP_H__

#include "stdio.h"

#include "config.h"
#include "yb_debug.h"
#include "protocol.h"

#include "tool_pid.h"

void App_Init(void);
void App_Loop(void);
void App_Start_FreeRTOS(void);

void App_Delay_ms(uint16_t ms);
void App_Send_Version(void);

void Set_Auto_Report(uint16_t enable);


void App_Set_OLED_Flag(uint8_t flag);
void App_Clear_Yaw(void);
void App_Test_Mode_Init(void);
uint8_t App_FreeRTOS_Enable(void);

#endif
