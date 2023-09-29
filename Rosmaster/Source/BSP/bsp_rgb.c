#include "bsp_rgb.h"


#if RGB_DRV_SPI
#define TIMING_ONE           0xF8
#define TIMING_ZERO          0x80
#else
#define TIMING_ONE           61
#define TIMING_ZERO          29
#endif
#define BIT_LEN              24
#define BUFF_SIZE            (MAX_RGB * BIT_LEN + 2)


// 存储每颗灯的颜色值，范围：[0, 0xffffff].
uint32_t led_buf[MAX_RGB] = {0};
uint8_t RGB_Byte_Buffer[BUFF_SIZE] = {0};

// GPIO初始化
static void RGB_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(Colorful_RCC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = Colorful_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(Colorful_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(Colorful_PORT, Colorful_PIN);
}

// 初始化RGB灯条的DMA通道。
static void RGB_DMA_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
#if RGB_DRV_SPI
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
    DMA_DeInit(DMA2_Channel2);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&SPI3->DR);           // 外设地址： SPIx  DR
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)RGB_Byte_Buffer;           // 待发送数据的地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                          // 传送方向，从内存到寄存器
    DMA_InitStructure.DMA_BufferSize = 0;                                       // 发送的数据长度，初始化可设置为0，发送时修改
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;            // 外设地址不增加
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                     // 内存地址自动增加1
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;     // 外设数据宽度
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;             // 内存数据宽度
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                               // 发送模式，只发一次
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;                         // DMA传送优先级为高
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                // 关闭内存到内存
    DMA_Init(DMA2_Channel2, &DMA_InitStructure);
#else
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_DeInit(DMA1_Channel5);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&TIM1->CCR1);         // 外设地址： TIMx  CCRx
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)RGB_Byte_Buffer;           // 待发送数据的地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                          // 传送方向，从内存到寄存器
    DMA_InitStructure.DMA_BufferSize = 0;                                       // 发送的数据长度，初始化可设置为0，发送时修改
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;            // 外设地址不增加
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                     // 内存地址自动增加1
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // 外设数据宽度
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;             // 内存数据宽度
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                               // 发送模式，只发一次
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;                         // DMA传送优先级为高
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                // 关闭内存到内存
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);
    /* TIM1 CC1 DMA Request enable */
    TIM_DMACmd(TIM1, TIM_DMA_Update, ENABLE);
#endif
}

#if RGB_DRV_SPI
// 初始化RGB灯条的SPI设备
static void RGB_Spi_Init(void)
{
    SPI_InitTypeDef SPIInitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
    SPIInitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
    SPIInitStructure.SPI_Mode = SPI_Mode_Master;
    SPIInitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPIInitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPIInitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPIInitStructure.SPI_NSS = SPI_NSS_Soft;
    SPIInitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    SPIInitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPIInitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI3, &SPIInitStructure);
    SPI_Cmd(SPI3, ENABLE);
    SPI_CalculateCRC(SPI3, DISABLE);
    SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Tx, ENABLE);
}
#else
// 初始化RGB灯条的定时器PWM输出。
static void RGB_Timer_Pwm_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    //重新将Timer设置为缺省值
    TIM_DeInit(TIM1);
    //预分频系数为0，即不进行预分频，此时TIMER的频率为72MHzre.TIM_Prescaler =0;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    //设置计数溢出大小，每计90个数就产生一个更新事件，72000000/90=800k
    TIM_TimeBaseStructure.TIM_Period = 90 - 1;
    //设置时钟分割
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    //设置计数器模式为向上计数模式
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; //不使用重复计数功能，不可缺少
    //将配置应用到TIM1中
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    //设置缺省值
    TIM_OCStructInit(&TIM_OCInitStructure);

    //TIM1的CH1输出
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             //设置是PWM模式还是比较模式
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能，使能PWM输出到端口
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //设置极性是高还是低
    //设置占空比，占空比=(CCRx/ARR)*100%或(TIM_Pulse/TIM_Period)*100%
    TIM_OCInitStructure.TIM_Pulse = 30;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    //设置TIM1的PWM输出为使能
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    //TIM_Cmd(TIM1,ENABLE);
    /* 定时器1使能DMA*/
    TIM_SelectCCDMA(TIM1, ENABLE);
}
#endif

// RGB灯条驱动初始化
static void RGB_Driver_Init(void)
{
#if RGB_DRV_SPI
    RGB_Spi_Init();
#else
    RGB_Timer_Pwm_Init();
#endif
    RGB_DMA_Init();
}

// 初始化灯条
void RGB_Init(void)
{
    RGB_GPIO_Init();
    RGB_Driver_Init();
}

// 更新RGB灯条
void RGB_Update(void)
{
    uint8_t i, j;
    for (j = 0; j < MAX_RGB; j++)
    {
        for (i = 0; i < BIT_LEN; i++)
        {
            RGB_Byte_Buffer[i + j * 24 + 1] = ((led_buf[j] >> (23 - i)) & 0x01) ? TIMING_ONE : TIMING_ZERO;
        }
    }
#if RGB_DRV_SPI
    DMA_SetCurrDataCounter(DMA2_Channel2, BUFF_SIZE); // 更新传输的数据量
    DMA_Cmd(DMA2_Channel2, ENABLE);                   // 使能DMA通道，开始传输数据
    while (!DMA_GetFlagStatus(DMA2_FLAG_TC2))         // 等待传输完成
        ;
    DMA_Cmd(DMA2_Channel2, DISABLE); // 关闭DMA通道
    DMA_ClearFlag(DMA2_FLAG_TC2);    // 清除DMA通道状态
#else
    DMA_SetCurrDataCounter(DMA1_Channel5, BUFF_SIZE); // 更新传输的数据量
    DMA_Cmd(DMA1_Channel5, ENABLE);                   // 使能DMA通道5，开始传输数据
    TIM_Cmd(TIM1, ENABLE);                            // 使能定时器1
    while (!DMA_GetFlagStatus(DMA1_FLAG_TC5))         // 等待传输完成
        ;
    TIM_Cmd(TIM1, DISABLE); // 关闭定时器1，并清除定时器参数
    TIM1->CCR1 = 0;
    TIM1->CNT = 0;
    TIM1->SR = 0;
    DMA_Cmd(DMA1_Channel5, DISABLE); // 关闭DMA通道5
    DMA_ClearFlag(DMA1_FLAG_TC5);    // 清除DMA通道状态
#endif
}

// 设置颜色，index=[0, MAX_RGB]控制对应灯珠颜色, index=0xFF控制所有灯珠颜色。
void RGB_Set_Color(uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
    uint32_t color = g << 16 | r << 8 | b;
    if (index < MAX_RGB)
    {
        led_buf[index] = color;
        return;
    }
    if (index == RGB_CTRL_ALL)
    {
        for (uint8_t i = 0; i < MAX_RGB; i++)
        {
            led_buf[i] = color;
        }
    }
}

// 设置RGB灯条颜色值，index=[0, 16]控制对应灯珠颜色, index=255控制所有灯珠颜色。color=0xggrrbb
void RGB_Set_Color_U32(uint8_t index, uint32_t color)
{
    if (index < MAX_RGB)
    {
        led_buf[index] = color;
        return;
    }
    if (index == RGB_CTRL_ALL)
    {
        for (uint8_t i = 0; i < MAX_RGB; i++)
        {
            led_buf[i] = color;
        }
    }
}

// 清除颜色（熄灭）
void RGB_Clear(void)
{
    for (uint8_t i = 0; i < MAX_RGB; i++)
    {
        led_buf[i] = 0x0;
    }
}
