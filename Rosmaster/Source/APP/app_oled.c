#include "app_oled.h"
#include "bsp_ssd1306.h"
#include "app_motion.h"
#include "config.h"

#include "stdio.h"
#include "bsp.h"



/* OLED清除屏幕 */
void OLED_Clear(void)
{
    SSD1306_Fill(SSD1306_COLOR_BLACK);
}

/* 刷新OLED屏幕 */
void OLED_Refresh(void)
{
    SSD1306_UpdateScreen();
}

/* 写入字符 */
void OLED_Draw_String(char *data, uint8_t x, uint8_t y, bool clear, bool refresh)
{
#if ENABLE_OLED
    if (clear) OLED_Clear();
    SSD1306_GotoXY(x, y);
    SSD1306_Puts(data, &Font_7x10, SSD1306_COLOR_WHITE);
    if (refresh) OLED_Refresh();
#endif
}

/* 写入一行字符 */
void OLED_Draw_Line(char *data, uint8_t line, bool clear, bool refresh)
{
    if (line > 0 && line <= 3)
    {
        OLED_Draw_String(data, 0, 10 * (line - 1), clear, refresh);
    }
}


/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////

// OLED显示小车类型和版本号
void OLED_Show_CarType(uint8_t car_type, uint8_t v_major, uint8_t v_minor, uint8_t v_patch)
{
	char text[20];
    switch (car_type)
    {
			case CAR_CARBOT:
					OLED_Draw_Line("CARBOT_SIT", 1, true, false);
					break;
			case CAR_MECANUM:
					OLED_Draw_Line("ROSMASTER_X3", 1, true, false);
					break;
			case CAR_MECANUM_MAX:
					OLED_Draw_Line("ROSMASTER_X3-Plus", 1, true, false);
					break;
			case CAR_FOURWHEEL:
					OLED_Draw_Line("ROSMASTER_X1", 1, true, false);
					break;
			case CAR_ACKERMAN:
					OLED_Draw_Line("ROSMASTER_R2", 1, true, false);
					break;
			case CAR_SUNRISE:
					OLED_Draw_Line("SUNRISE_CAR", 1, true, false);
					break;
			default:
					OLED_Draw_Line("None Car Type", 1, true, false);
					break;
    }
    #if APP_RELEASE
	sprintf(text, "Version:V%d.%d.%d", v_major, v_minor, v_patch);
    #else
    sprintf(text, "Version-D:V%d.%d.%d", v_major, v_minor, v_patch);
    #endif
	OLED_Draw_Line(text, 2, false, false);

    sprintf(text, "%s  %s", __DATE__, __TIME__);
	OLED_Draw_Line(text, 3, false, true);
}


// OLED显示电池电压
void OLED_Show_Voltage(uint16_t bat_voltage)
{
    char text[20];
    sprintf(text, "Voltage:%.1fV", bat_voltage/10.0);
    OLED_Draw_Line(text, 2, true, true);
}

// OLED显示姿态角
void OLED_Show_IMU_Attitude(float yaw, float roll, float pitch)
{
    char text[20];
    sprintf(text, "YAW:%.2f", yaw);
    OLED_Draw_Line(text, 1, true, false);
    sprintf(text, "ROLL:%.2f", roll);
    OLED_Draw_Line(text, 2, false, false);
    sprintf(text, "PITCH:%.2f", pitch);
    OLED_Draw_Line(text, 3, false, true);
}

// OLED显示偏航角
void OLED_Show_YAW(float yaw)
{
    char text[20];
    if (yaw > 180) yaw = yaw - 360;
    int yaw_int = (int)(yaw);
    sprintf(text, "YAW:%d", yaw_int);
    // sprintf(text, "YAW:%.1f", yaw);
    OLED_Draw_Line(text, 2, true, true);
}

// OLED显示测试模式提示
void OLED_Show_Test_Mode(void)
{
    char text[20];
    #if APP_RELEASE
	sprintf(text, "Version:V%d.%d.%d", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);
    #else
    sprintf(text, "Version-D:V%d.%d.%d", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);
    #endif
	OLED_Draw_Line(text, 2, true, false);

    sprintf(text, "Test Mode");
	OLED_Draw_Line(text, 3, false, true);
}

// OLED显示模式错误提示
void OLED_Show_Test_Mode_Error(void)
{
    char text[20];
    sprintf(text, "Mode Error");
	OLED_Draw_Line(text, 2, true, true);
}

// OLED显示电机速度
void OLED_Show_Motor_Speed(float m1, float m2, float m3, float m4)
{
    char text[20];
    
    sprintf(text, "M1:%d, M2:%d", (int)(m1/10), (int)(m2/10));
	OLED_Draw_Line(text, 2, true, false);

    sprintf(text, "M3:%d, M4:%d", (int)(m3/10), (int)(m4/10));
	OLED_Draw_Line(text, 3, false, true);
}

// OLED显示串口舵机读取状态
void OLED_Show_UART_Servo_Read(uint16_t read_value)
{
    char text[20];
    if (read_value > 0) // read_value > 1800 && read_value < 2200
    {
        sprintf(text, "Servo Read OK!");
    }
    else
    {
        sprintf(text, "Servo Read Fail!");
    }
	OLED_Draw_Line(text, 2, true, true);
}

// OLED显示等待中
void OLED_Show_Waiting(void)
{
    char text[20];
    
    #if APP_RELEASE
	sprintf(text, "----LOADING----");
    #else
    sprintf(text, "Version-D:V%d.%d.%d", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);
    #endif
	OLED_Draw_Line(text, 2, true, false);

    sprintf(text, "Please Waiting..");
	OLED_Draw_Line(text, 3, false, true);
}

// OLED显示IMU初始化错误
void OLED_Show_Error(void)
{
    LED_OFF();
    LED_SW_OFF();
    BEEP_OFF();
    char text[20];

    #if APP_RELEASE
	sprintf(text, "Version:V%d.%d.%d", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);
    #else
    sprintf(text, "Version-D:V%d.%d.%d", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);
    #endif
	OLED_Draw_Line(text, 2, true, false);

    sprintf(text, "IMU ERROR!!!");
	OLED_Draw_Line(text, 3, false, true);
}
