#include "bsp_adc.h"
#include "bsp.h"

// ADC初始化接口
void Adc_Init(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(BAT_GPIO_CLK | BAT_ADC_CLK, ENABLE); //使能 BAT_ADC 通道时钟
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);							//设置 ADC 分频因子 6

	//72M/6=12, ADC 最大输入时钟不能超过14M
	//PC4 作为模拟通道输入引脚
	GPIO_InitStructure.GPIO_Pin = BAT_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;  //模拟输入
	GPIO_Init(BAT_GPIO_PORT, &GPIO_InitStructure); //初始化 GPIOC.4

	ADC_DeInit(BAT_ADC);												//复位 BAT_ADC,将外设 BAT_ADC 的全部寄存器重设为缺省值
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;					//ADC 独立模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;						//单通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;					//单次转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;				//ADC 数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;								//顺序进行规则转换的 ADC 通道的数目
	ADC_Init(BAT_ADC, &ADC_InitStructure);								//根据指定的参数初始化外设 ADCx
	ADC_Cmd(BAT_ADC, ENABLE);											//使能指定的 BAT_ADC
	ADC_ResetCalibration(BAT_ADC);										//开启复位校准
	while (ADC_GetResetCalibrationStatus(BAT_ADC))
		;						   //等待复位校准结束
	ADC_StartCalibration(BAT_ADC); //开启 AD 校准
	while (ADC_GetCalibrationStatus(BAT_ADC))
		; //等待校准结束
}


// 获得 ADC 值, ch:通道值 0~3
static uint16_t Adc_Get(uint8_t ch)
{
	uint16_t timeout = 1000;
	//设置指定 ADC 的规则组通道，设置它们的转化顺序和采样时间
	ADC_RegularChannelConfig(BAT_ADC, ch, 1, ADC_SampleTime_239Cycles5);
	//通道 1,规则采样顺序值为 1,采样时间为 239.5 周期
	ADC_SoftwareStartConvCmd(BAT_ADC, ENABLE); //使能软件转换功能
	while (!ADC_GetFlagStatus(BAT_ADC, ADC_FLAG_EOC) && timeout--)
		;									//等待转换结束
	return ADC_GetConversionValue(BAT_ADC); //返回最近一次 BAT_ADC 规则组的转换结果
}


// 获得 ADC 多次测量平均值, ch:通道值 ; times:测量次数
uint16_t Adc_Get_Average(uint8_t ch, uint8_t times)
{
	uint16_t temp_val = 0;
	uint8_t t;
	for (t = 0; t < times; t++)
	{
		temp_val += Adc_Get(ch);
	}
	if (times == 4)
	{
		temp_val = temp_val >> 2;
	}
	else
	{
		temp_val = temp_val / times;
	}
	return temp_val;
}


// 获得测得原始电压值
float Adc_Get_Measure_Volotage(void)
{
	uint16_t adcx;
	float temp;
	adcx = Adc_Get_Average(BAT_ADC_CH, 4);
	temp = (float)adcx * (3.30f / 4096);
	return temp;
}


// 获得实际电池分压前电压
float Adc_Get_Battery_Volotage(void)
{
	float temp;
	temp = Adc_Get_Measure_Volotage();
	// 实际测量的值比计算得出的值低一点点。
	temp = temp * 4.03f;    //temp*(10+3.3)/3.3; 
	return temp;
}


