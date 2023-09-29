#include "bsp_spi.h"
#include "bsp.h"

static void Spi_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //PORTB时钟使能

    // CLK  MOSI
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // CS
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB, GPIO_Pin_12);

    // MISO
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //上拉输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

// SPI初始化，SPI2,全双工，软件NSS
void SPI2_Init(void)
{
    Spi_gpio_init();
    SPI_InitTypeDef SPI_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE); //SPI2时钟使能
    //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    //设置SPI工作模式:设置为主SPI
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    //设置SPI的数据大小:SPI发送接收8位帧结构
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    //串行同步时钟的空闲状态为高电平
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    //串行同步时钟的第二个跳变沿（上升或下降）数据被采样
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    //NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    //定义波特率预分频的值:波特率预分频值为256
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
    //指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    //CRC值计算的多项式
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI2, &SPI_InitStructure);
    //使能SPI外设
    SPI_Cmd(SPI2, ENABLE);
}


//SPIx 读写一个字节
//TxData:要写入的字节
uint8_t SPI2_ReadWriteByte(uint8_t TxData, uint8_t* RxData)
{
    uint8_t retry = 0;

    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) //检查指定的SPI标志位设置与否:发送缓存空标志位
    {
        retry++;
        if(retry > 200)return 1;
    }

    SPI_I2S_SendData(SPI2, TxData); //通过外设SPIx发送一个数据
    retry = 0;

    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET) //检查指定的SPI标志位设置与否:接受缓存非空标志位
    {
        retry++;

        if(retry > 200)return 1;
    }

	*RxData = SPI_I2S_ReceiveData(SPI2);
	
    return 0; 
}
