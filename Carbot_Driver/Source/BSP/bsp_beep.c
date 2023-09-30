#include "bsp_beep.h"

uint16_t beep_on_time = 0;
uint8_t beep_state = 0;

void Beep_GPIO_Init(void)
{
	/*定义一个GPIO_InitTypeDef类型的结构体*/
	GPIO_InitTypeDef GPIO_InitStructure;
	/*开启控制蜂鸣器的GPIO的端口时钟*/
	RCC_APB2PeriphClockCmd(BEEP_GPIO_CLK, ENABLE);
	/*选择要控制蜂鸣器的GPIO*/
	GPIO_InitStructure.GPIO_Pin = BEEP_GPIO_PIN;
	/*设置GPIO模式为通用推挽输出*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	/*设置GPIO速率为50MHz */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	/*调用库函数，初始化控制蜂鸣器的GPIO*/
	GPIO_Init(BEEP_GPIO_PORT, &GPIO_InitStructure);
	/* 关闭蜂鸣器*/
	// GPIO_ResetBits(BEEP_GPIO_PORT, BEEP_GPIO_PIN);
	/* 打开蜂鸣器*/
	GPIO_SetBits(BEEP_GPIO_PORT, BEEP_GPIO_PIN);
}

// 刷新蜂鸣器打开的时间
static void Beep_Set_Time(uint16_t time)
{
	beep_on_time = time;
}

// 获取当前蜂鸣器打开的剩余时间
static uint16_t Beep_Get_Time(void)
{
	return beep_on_time;
}

// 刷新蜂鸣器的状态
static void Beep_Set_State(uint8_t state)
{
	beep_state = state;
}

// 获取蜂鸣器的状态
static uint8_t Beep_Get_State(void)
{
	return beep_state;
}

// 设置蜂鸣器开启时间，time=0时关闭，time=1时一直响，time>=10，延迟xx毫秒后自动关闭
void Beep_On_Time(uint16_t time)
{
	if (time == BEEP_STATE_ON_ALWAYS)
	{
		Beep_Set_State(BEEP_STATE_ON_ALWAYS);
		Beep_Set_Time(0);
		BEEP_ON();
	}
	else if (time == BEEP_STATE_OFF)
	{
		Beep_Set_State(BEEP_STATE_OFF);
		Beep_Set_Time(0);
		BEEP_OFF();
	}
	else
	{
		if (time >= 10)
		{
			Beep_Set_State(BEEP_STATE_ON_DELAY);
			Beep_Set_Time(time / 10);
			BEEP_ON();
		}
	}
}

/* 蜂鸣器超时自动关闭程序, 10毫秒调用一次 */
void Beep_Timeout_Close_Handle(void)
{
	if (Beep_Get_State() == BEEP_STATE_ON_DELAY)
	{
		if (Beep_Get_Time())
		{
			beep_on_time--;
		}
		else
		{
			BEEP_OFF();
			Beep_Set_State(BEEP_STATE_OFF);
		}
	}
}
