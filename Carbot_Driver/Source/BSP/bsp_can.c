#include "bsp_can.h"
#include "bsp.h"
#include "app_motion.h"
#include "app.h"


CAN_Data_t g_can_rx = {0};

// 接收信息蜂鸣器响一声的开关，默认为关，系统初始化完成后再自动打开。
uint8_t g_can_beep_state = 0;


// 解析CAN接收的数据 
// 协议格式：ID|FUNC|①|②|③|④|⑤|CHECK
static void CAN_Parse_Data(uint8_t* data)
{
    uint8_t device_id = data[0];
    uint8_t checknum = data[7];
    uint8_t checksum = 0;
    if (device_id != PTO_DEVICE_ID) return;
    for (int i = 1; i < 7; i++)
    {
        checksum = checksum + data[i];
    }
    if (checksum != checknum) return;
    
    uint8_t func = data[1];
    uint8_t parm[5] = {data[2], data[3], data[4], data[5], data[6]};
    Upper_CAN_Execute_Command(func, parm);
}


// 获取对应波特率的分频系数
static uint8_t CAN_Get_Prescaler(EN_CAN_BAUDRATE baudrate)
{
    uint8_t prescaler = 4; // 默认是1000Kbps
    if (baudrate == CAN_BAUD_1000Kbps) prescaler = 4;
    if (baudrate == CAN_BAUD_800Kbps) prescaler = 5;
    if (baudrate == CAN_BAUD_500Kbps) prescaler = 8;
    if (baudrate == CAN_BAUD_250Kbps) prescaler = 16;
    if (baudrate == CAN_BAUD_100Kbps) prescaler = 40;
    if (baudrate == CAN_BAUD_50Kbps) prescaler = 80;
    return prescaler;
}

// 初始化CAN的引脚
static void CAN_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(CAN_R_GPIO_CLK | CAN_D_GPIO_CLK, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);
    GPIO_InitStructure.GPIO_Pin = CAN_R_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(CAN_R_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = CAN_D_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(CAN_D_GPIO_PORT, &GPIO_InitStructure);
}

// 配置CAN的过滤器
static void CAN_Filter_Init(void)
{
    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    // 配置过滤器
    CAN_FilterInitStructure.CAN_FilterNumber = 0;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);
}



// 配置CAN的中断配置
static void CAN_NVIC_Init(void)
{
    NVIC_InitTypeDef      NVIC_InitStructure;
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
    NVIC_InitStructure.NVIC_IRQChannel                   = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


// 初始化CAN1
void CAN_Config_Init(EN_CAN_BAUDRATE baudrate)
{
    CAN_InitTypeDef CAN_InitStructure;
    CAN_GPIO_Init();

    CAN_DeInit(CAN1);
    CAN_StructInit(&CAN_InitStructure);
    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = DISABLE;
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = DISABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = DISABLE;
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; 
    // CAN_Mode_Normal; CAN_Mode_LoopBack;

    // CAN波特率计算方式：baud=Fpclk/(BRP+1)/(SJW_+BS1_+BS2_)
    // Fpclk=36M, CAN_Prescaler=BRP+1, CAN_BS1=BS1_-1.
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
    CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
    CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
    CAN_InitStructure.CAN_Prescaler = CAN_Get_Prescaler(baudrate); // (BRP+1)
    CAN_Init(CAN1, &CAN_InitStructure);

    CAN_Filter_Init();

    CAN_NVIC_Init();
}


// CAN FIFO0 接收中断函数
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    CAN_Receive_Data(&g_can_rx);
    if (App_FreeRTOS_Enable())
    {
        CAN_Parse_Data(g_can_rx.data);
    }
}


// CAN发送数据
void CAN_Send_Data(CAN_Data_t* data)
{
    int i;
    uint8_t TxMailbox = 0;
    CanTxMsg TxMsg;
    TxMsg.StdId = CAN_STANDARD_ID;
    // TxMsg.ExtId = CAN_EXTENDED_ID;
    TxMsg.RTR = CAN_RTR_DATA;  // 帧类型：数据帧、远程帧
    TxMsg.IDE = CAN_ID_STD;    // 帧格式：标准帧、扩展帧
    TxMsg.DLC = 8;
    for (i = 0; i < 8; i++)
    {
        TxMsg.Data[i] = data->data[i];
    }
    TxMailbox = CAN_Transmit(CAN1, &TxMsg);
    for (i = 0; i < CAN_TX_TIMEOUT; i++)
    {
        if ((CAN_TransmitStatus(CAN1, TxMailbox)) == CANTXOK)
        {
            break;
        }
    }
    if (i >= CAN_TX_TIMEOUT)
    {
        i = 0;
        DEBUG("CAN SEND ERROR\n");
    }
}

// 接收信息蜂鸣器响一声开关
void CAN_RX_Beep(uint8_t enable)
{
    if (enable)
    {
        g_can_beep_state = 1;
    }
    else
    {
        g_can_beep_state = 0;
    }
}

// 接收CAN数据
void CAN_Receive_Data(CAN_Data_t* Can)
{
    CanRxMsg RxMsg;
    if (CAN_MessagePending(CAN1, CAN_FIFO0))
    {
        RxMsg.StdId = 0x00;
        RxMsg.IDE = CAN_ID_STD;
        RxMsg.DLC = 8;
        for (int i = 0; i < 8; i++)
        {
            RxMsg.Data[i] = 0x00;
        }
        CAN_Receive(CAN1, CAN_FIFO0, &RxMsg);
        DEBUG("Can->data RX:");
        for (int i = 0; i < 8; i++)
        {
            Can->data[i] = RxMsg.Data[i];
            DEBUG("%02X,", Can->data[i]);
        }
        DEBUG("\n");
        if (g_can_beep_state) Beep_On_Time(50);
    }
    else
    {
        for (int i = 0; i < 8; i++)
        {
            Can->data[i] = 0;
        }
        DEBUG("Can->data RX Error\n");
    }
}


// 测试发送数据
void CAN_Test_Send(void)
{
    CAN_Data_t can_data;
    for (int i = 0; i < 8; i++)
    {
        can_data.data[i] = 1 << i;
    }
    static uint8_t can_count = 0;
    can_count++;
    can_data.data[0] = can_count;
    CAN_Send_Data(&can_data);
}


