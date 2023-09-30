#include "app_bat.h"
#include "app.h"
#include "app_oled.h"
#include "app_motion.h"
#include "app_rgb.h"

#include "bsp.h"

// 电池电压检测异常计数阈值，乘100毫秒就是延迟时间，单位为毫秒。
// 例如：100*10=1000，即1秒。
#define BAT_CHECK_COUNT        20

uint8_t g_system_enable = 1;      // 系统功能状态。检测到电压过低或者过高后为0。只能通过复位恢复1
uint8_t g_bat_state = 1;          // 电池低电压状态。默认为1，检测到电压过低为0，检测到电压过高为2。只能通过复位恢复1。
int Voltage_Z10 = 0;              // 电池电压值
int Voltage_Unusual_Count = 0;    // 电压异常计数


// 检查电池电压状态，返回值：1=电压正常，0=电压过低，2=电压过高。
static uint8_t Bat_Check_Voltage(int Voltage)
{
	if (Motion_Get_Car_Type() == CAR_SUNRISE)
	{
		if (Voltage > Bat_Get_Over_Voltage())
		{
			Voltage_Unusual_Count++;
			if(Voltage_Unusual_Count > BAT_CHECK_COUNT)
			{
				return BATTERY_OVER_VOLTAGE;
			}
		}
		else if (Voltage < Bat_Get_Low_Voltage())
		{
			Voltage_Unusual_Count++;
			if(Voltage_Unusual_Count > BAT_CHECK_COUNT)
			{
				return BATTERY_LOW;
			}
		}
		else
		{
			Voltage_Unusual_Count = 0;
		}
	}
	else
	{
		if (Voltage > Bat_Get_Over_Voltage())
		{
			Voltage_Unusual_Count++;
			if(Voltage_Unusual_Count > BAT_CHECK_COUNT)
			{
				return BATTERY_OVER_VOLTAGE;
			}
		}
		else if (Voltage < Bat_Get_Low_Voltage())
		{
			// 过滤6.5-8.5之间的低电压报警功能。
			if (Voltage > 85 || Voltage < 65)
			{
				Voltage_Unusual_Count++;
				if(Voltage_Unusual_Count > BAT_CHECK_COUNT)
				{
					return BATTERY_LOW;
				}
			}
		}
		else
		{
			Voltage_Unusual_Count = 0;
		}
	}
	return BATTERY_NORMAL;
}

// Carbot智能物流小车使用的是12.6V电池组，低电压报警阈值为9.6V。
// Rosmaster系列小车使用的是12.6V电池组，低电压报警阈值为9.6V。
// sunrise系列小车使用的是8.4V电池则，低电压报警阈值为6.5V。
uint8_t Bat_Get_Low_Voltage(void)
{
	if (Motion_Get_Car_Type() == CAR_SUNRISE)
	{
		return 65;
	}
	return 96;
}

// Carbot智能物流小车使用的是12.6V电池组，电压过高报警阈值为13.0V。
// Rosmaster系列小车使用的是12.6V电池组，电压过高报警阈值为13.0V。
// sunrise系列小车使用的是8.4V电池则，电压过高报警阈值为8.5V。
uint8_t Bat_Get_Over_Voltage(void)
{
	if (Motion_Get_Car_Type() == CAR_SUNRISE)
	{
		return 85;
	}
	return 130;
}


// 查询电池电压状态，连续几秒读到低于9.6V返回0，高于9.6V返回1
uint8_t Bat_State(void)
{
	if (g_bat_state == BATTERY_NORMAL)
	{
		Voltage_Z10 = (int) (Adc_Get_Battery_Volotage() * 10);
		#if ENABLE_LOW_BATTERY_ALARM
		g_bat_state = Bat_Check_Voltage(Voltage_Z10);
		if(g_bat_state != BATTERY_NORMAL)
		{
			g_system_enable = 0;
		}
		#endif
	}
    // DEBUG("BAT:%d, %d", g_bat_state, Voltage_Z10);
	return g_bat_state;
}

int Bat_Voltage_Z10(void)
{
	return Voltage_Z10;
}


// 返回系统是否进入供电正常，正常返回1，不正常返回0
uint8_t System_Enable(void)
{
	return g_system_enable;
}

// 10毫秒调用一次，根据电池显示LED状态，返回系统状态。
uint8_t Bat_Show_LED_Handle(uint8_t enable_beep)
{
	static uint16_t bat_led_state = 0;
	bat_led_state++;
	if (bat_led_state >= 10)
	{
		static uint8_t alarm = 1;
		uint8_t battery_state = Bat_State();
		bat_led_state = 0;
		if (battery_state == BATTERY_LOW)
		{
			Bsp_Led_Show_Low_Battery(enable_beep);
			if (alarm)
			{
				alarm = 0;
				OLED_Draw_Line("Battery Low", 2, true, true);
			}
			Motion_Stop(STOP_BRAKE);
		}
		else if (battery_state == BATTERY_OVER_VOLTAGE)
		{
			Bsp_Led_Show_Overvoltage_Battery(enable_beep);
			if (alarm)
			{
				alarm = 0;
				app_rgb_set_effect(0, 0);
				RGB_Set_Color(RGB_CTRL_ALL, 255, 0, 0);
				RGB_Update();
				OLED_Draw_Line("Battery", 2, true, false);
				OLED_Draw_Line("Over Voltage", 3, false, true);
			}
			Motion_Stop(STOP_BRAKE);
		}
		else
		{
			Bsp_Led_Show_State();
		}
	}
	return g_system_enable;
}

