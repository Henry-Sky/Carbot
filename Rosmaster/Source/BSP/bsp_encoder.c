#include "bsp_encoder.h"
#include "bsp.h"

#include "protocol.h"
#include "app_motion.h"


int g_Encoder_M1_Now = 0;
int g_Encoder_M2_Now = 0;
int g_Encoder_M3_Now = 0;
int g_Encoder_M4_Now = 0;


// TIM2初始化为编码器接口模式， 3A 3B
static void Encoder_Init_TIM2(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	Bsp_JTAG_Set(SWD_ENABLE);    //=====打开SWD接口,关闭JTAG接口 可以利用主板的SWD接口调试
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  // 使能AFIO时钟
	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);    // remap TIM2引脚CH1 CH2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); // 使能定时器2的时钟


	RCC_APB2PeriphClockCmd(HAL_3A_CLK, ENABLE);			  //使能端口时钟
	GPIO_InitStructure.GPIO_Pin = HAL_3A_PIN;			  //端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
	GPIO_Init(HAL_3A_PORT, &GPIO_InitStructure);		  //根据设定参数初始化GPIO

	RCC_APB2PeriphClockCmd(HAL_3B_CLK, ENABLE);			  //使能端口时钟
	GPIO_InitStructure.GPIO_Pin = HAL_3B_PIN;			  //端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
	GPIO_Init(HAL_3B_PORT, &GPIO_InitStructure);		  //根据设定参数初始化GPIO

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;					//预分频器
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;		//设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//选择时钟分频：不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //使用编码器模式3

	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 10;
    
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);                  //清除TIM的更新标志位
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	//Reset counter
	TIM_SetCounter(TIM2, 0);
	//===============================================
	TIM2->CNT = 0x7fff;
	//===============================================
	TIM_Cmd(TIM2, ENABLE);
}

// 定时器3通道1通道2连接编码器2A 2B
static void Encoder_Init_TIM3(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //使能定时器4的时钟

	RCC_APB2PeriphClockCmd(HAL_2A_CLK, ENABLE);			  //使能端口时钟
	GPIO_InitStructure.GPIO_Pin = HAL_2A_PIN;			  //端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
	GPIO_Init(HAL_2A_PORT, &GPIO_InitStructure);		  //根据设定参数初始化GPIO

	RCC_APB2PeriphClockCmd(HAL_2B_CLK, ENABLE);			  //使能端口时钟
	GPIO_InitStructure.GPIO_Pin = HAL_2B_PIN;			  //端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
	GPIO_Init(HAL_2B_PORT, &GPIO_InitStructure);		  //根据设定参数初始化GPIO

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;					// 预分频器
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;		//设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//选择时钟分频：不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //使用编码器模式3
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 10;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);                   //清除TIM的更新标志位
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	//Reset counter
	TIM_SetCounter(TIM3, 0);
	//===============================================
	TIM3->CNT = 0x7fff;
	//===============================================
	TIM_Cmd(TIM3, ENABLE);
}


// TIM4初始化为编码器接口模式, 4A 4B
static void Encoder_Init_TIM4(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //使能定时器4的时钟

	RCC_APB2PeriphClockCmd(HAL_4A_CLK, ENABLE);			  //使能端口时钟
	GPIO_InitStructure.GPIO_Pin = HAL_4A_PIN;			  //端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
	GPIO_Init(HAL_4A_PORT, &GPIO_InitStructure);		  //根据设定参数初始化GPIO

	RCC_APB2PeriphClockCmd(HAL_4B_CLK, ENABLE);			  //使能端口时钟
	GPIO_InitStructure.GPIO_Pin = HAL_4B_PIN;			  //端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
	GPIO_Init(HAL_4B_PORT, &GPIO_InitStructure);		  //根据设定参数初始化GPIO

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;					// 预分频器
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;		//设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//选择时钟分频：不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //使用编码器模式3
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 10;
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);                   //清除TIM的更新标志位
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	//Reset counter
	TIM_SetCounter(TIM4, 0);
	//===============================================
	TIM4->CNT = 0x7fff;
	//===============================================
	TIM_Cmd(TIM4, ENABLE);
}

// 定时器5通道1通道2连接编码器1A 1B
static void Encoder_Init_TIM5(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); //使能定时器5的时钟

	RCC_APB2PeriphClockCmd(HAL_1A_CLK, ENABLE);			  //使能端口时钟
	GPIO_InitStructure.GPIO_Pin = HAL_1A_PIN;			  //端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
	GPIO_Init(HAL_1A_PORT, &GPIO_InitStructure);		  //根据设定参数初始化GPIO

	RCC_APB2PeriphClockCmd(HAL_1B_CLK, ENABLE);			  //使能端口时钟
	GPIO_InitStructure.GPIO_Pin = HAL_1B_PIN;			  //端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
	GPIO_Init(HAL_1A_PORT, &GPIO_InitStructure);		  //根据设定参数初始化GPIO

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;					//预分频器
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;		//设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//选择时钟分频：不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //使用编码器模式3

	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 10;
	TIM_ICInit(TIM5, &TIM_ICInitStructure);
	TIM_ClearFlag(TIM5, TIM_FLAG_Update);                  //清除TIM的更新标志位
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
	//Reset counter
	TIM_SetCounter(TIM5, 0);
	//===============================================
	TIM5->CNT = 0x7fff;
	//===============================================
	TIM_Cmd(TIM5, ENABLE);
}

/**
 * @Brief: 10毫秒更新一次，读取编码器计数
 * @Note: 
 * @Parm: 电机的ID号:MOTOR_ID_M1, MOTOR_ID_M2, MOTOR_ID_M3, MOTOR_ID_M4
 * @Retval: 返回编码器计数数据
 */
static int16_t Encoder_Read_CNT(uint8_t Motor_id)
{
	int16_t Encoder_TIM = 0;
	switch(Motor_id)
	{
	case MOTOR_ID_M1:  Encoder_TIM = 0x7fff - (short)TIM2 -> CNT; TIM2 -> CNT = 0x7fff; break;
	case MOTOR_ID_M2:  Encoder_TIM = 0x7fff - (short)TIM4 -> CNT; TIM4 -> CNT = 0x7fff; break;
	case MOTOR_ID_M3:  Encoder_TIM = 0x7fff - (short)TIM5 -> CNT; TIM5 -> CNT = 0x7fff; break;
	case MOTOR_ID_M4:  Encoder_TIM = 0x7fff - (short)TIM3 -> CNT; TIM3 -> CNT = 0x7fff; break;
	default:  break;
	}
	return Encoder_TIM;
}


// 返回开机到现在总共统计的编码器的计数（单路）。
int Encoder_Get_Count_Now(uint8_t Motor_id)
{
	if (Motor_id == MOTOR_ID_M1) return g_Encoder_M1_Now;
	if (Motor_id == MOTOR_ID_M2) return g_Encoder_M2_Now;
	if (Motor_id == MOTOR_ID_M3) return g_Encoder_M3_Now;
	if (Motor_id == MOTOR_ID_M4) return g_Encoder_M4_Now;
	return 0;
}

// 获取开机到现在总共的四路编码器计数。
void Encoder_Get_ALL(int* Encoder_all)
{
	Encoder_all[0] = g_Encoder_M1_Now;
	Encoder_all[1] = g_Encoder_M2_Now;
	Encoder_all[2] = g_Encoder_M3_Now;
	Encoder_all[3] = g_Encoder_M4_Now;
}

// 更新编码器的计数总值。
void Encoder_Update_Count(void)
{
	if (Motion_Get_Car_Type() == CAR_MECANUM_MAX)
	{
		g_Encoder_M1_Now -= Encoder_Read_CNT(MOTOR_ID_M1);
		// g_Encoder_M1_Now += Encoder_Read_CNT(MOTOR_ID_M1);

		// g_Encoder_M2_Now -= Encoder_Read_CNT(MOTOR_ID_M2);
		g_Encoder_M2_Now += Encoder_Read_CNT(MOTOR_ID_M2);

		// g_Encoder_M3_Now -= Encoder_Read_CNT(MOTOR_ID_M3);
		g_Encoder_M3_Now += Encoder_Read_CNT(MOTOR_ID_M3);

		// g_Encoder_M4_Now += Encoder_Read_CNT(MOTOR_ID_M4);
		g_Encoder_M4_Now -= Encoder_Read_CNT(MOTOR_ID_M4);
	}
	else if (Motion_Get_Car_Type() == CAR_CARBOT)
	{
		g_Encoder_M1_Now -= Encoder_Read_CNT(MOTOR_ID_M1);
		// g_Encoder_M1_Now += Encoder_Read_CNT(MOTOR_ID_M1);

		// g_Encoder_M2_Now -= Encoder_Read_CNT(MOTOR_ID_M2);
		g_Encoder_M2_Now += Encoder_Read_CNT(MOTOR_ID_M2);

		// g_Encoder_M3_Now -= Encoder_Read_CNT(MOTOR_ID_M3);
		g_Encoder_M3_Now += Encoder_Read_CNT(MOTOR_ID_M3);

		// g_Encoder_M4_Now += Encoder_Read_CNT(MOTOR_ID_M4);
		g_Encoder_M4_Now -= Encoder_Read_CNT(MOTOR_ID_M4);
	}
	// 其他车型
	else
	{
		// g_Encoder_M1_Now -= Encoder_Read_CNT(MOTOR_ID_M1);
		g_Encoder_M1_Now += Encoder_Read_CNT(MOTOR_ID_M1);

		g_Encoder_M2_Now -= Encoder_Read_CNT(MOTOR_ID_M2);
		// g_Encoder_M2_Now += Encoder_Read_CNT(MOTOR_ID_M2);

		g_Encoder_M3_Now -= Encoder_Read_CNT(MOTOR_ID_M3);
		// g_Encoder_M3_Now += Encoder_Read_CNT(MOTOR_ID_M3);

		g_Encoder_M4_Now += Encoder_Read_CNT(MOTOR_ID_M4);
		// g_Encoder_M4_Now -= Encoder_Read_CNT(MOTOR_ID_M4);
	}
}


// 初始化编码器GPIO和定时器捕获脉冲
void Encoder_Init(void)
{
	Encoder_Init_TIM2();       // M1 TIM2 PWMC HAL3
	Encoder_Init_TIM4();       // M2 TIM4 PWMD HAL4
	Encoder_Init_TIM5();       // M3 TIM5 PWMA HAL1
	Encoder_Init_TIM3();       // M4 TIM3 PWMB HAL2
}


// 发送当前的编码器数据到主控上
void Encoder_Send_Count_Now(void)
{
    #define LEN        21
	uint8_t data_buffer[LEN] = {0};
	uint8_t i, checknum = 0;
	data_buffer[0] = PTO_HEAD;
	data_buffer[1] = PTO_DEVICE_ID-1;
	data_buffer[2] = LEN-2; // 数量
	data_buffer[3] = FUNC_REPORT_ENCODER; // 功能位
	data_buffer[4] = g_Encoder_M1_Now & 0xff;
	data_buffer[5] = (g_Encoder_M1_Now >> 8) & 0xff;
	data_buffer[6] = (g_Encoder_M1_Now >> 16) & 0xff;
	data_buffer[7] = (g_Encoder_M1_Now >> 24) & 0xff;
	data_buffer[8] = g_Encoder_M2_Now & 0xff;
	data_buffer[9] = (g_Encoder_M2_Now >> 8) & 0xff;
	data_buffer[10] = (g_Encoder_M2_Now >> 16) & 0xff;
	data_buffer[11] = (g_Encoder_M2_Now >> 24) & 0xff;
	data_buffer[12] = g_Encoder_M3_Now & 0xff;
	data_buffer[13] = (g_Encoder_M3_Now >> 8) & 0xff;
	data_buffer[14] = (g_Encoder_M3_Now >> 16) & 0xff;
	data_buffer[15] = (g_Encoder_M3_Now >> 24) & 0xff;
	data_buffer[16] = g_Encoder_M4_Now & 0xff;
	data_buffer[17] = (g_Encoder_M4_Now >> 8) & 0xff;
	data_buffer[18] = (g_Encoder_M4_Now >> 16) & 0xff;
	data_buffer[19] = (g_Encoder_M4_Now >> 24) & 0xff;

	for (i = 2; i < LEN-1; i++)
	{
		checknum += data_buffer[i];
	}
	data_buffer[LEN-1] = checknum;
	USART1_Send_ArrayU8(data_buffer, sizeof(data_buffer));
}
