#include "app_uart_servo.h"
#include "bsp_usart.h"
#include "app.h"
#include "bsp.h"
#include "app_flash.h"
#include "app_oled.h"

uint16_t sync_servo[] = {MEDIAN_VALUE, MEDIAN_VALUE, MEDIAN_VALUE, MEDIAN_VALUE, MID_VAL_ID5, MID_VAL_ID6};

uint8_t RecvFlag = 0;
uint8_t Rx_Data[RX_MAX_BUF] = {0};
uint8_t Rx_index = 0;
uint8_t Rx_Flag = 0;

// 中位偏差值
uint16_t g_arm_median_value[] = {MEDIAN_VALUE, MEDIAN_VALUE, MEDIAN_VALUE, MEDIAN_VALUE, MID_VAL_ID5, MID_VAL_ID6};
// 读取数据的状态和数值
uint16_t servo_read_state = 0;
uint16_t g_arm_read_value[6] = {0};

uint8_t g_test_read_ok = 0;


/* 控制舵机 */
void UartServo_Ctrl(uint8_t id, uint16_t value, uint16_t time)
{
    uint8_t head1 = 0xff;
    uint8_t head2 = 0xff;
    uint8_t s_id = id & 0xff;
    uint8_t len = 0x07;
    uint8_t cmd = 0x03;
    uint8_t addr = 0x2a;

    value = value + UartServo_Get_Median_Offset(s_id);

    if (value > ARM_MAX_VALUE)
        value = ARM_MAX_VALUE;
    else if (value < ARM_MIN_VALUE)
        value = ARM_MIN_VALUE;

    uint8_t pos_H = (value >> 8) & 0xff;
    uint8_t pos_L = value & 0xff;

    uint8_t time_H = (time >> 8) & 0xff;
    uint8_t time_L = time & 0xff;

    uint8_t checknum = (~(s_id + len + cmd + addr +
                          pos_H + pos_L + time_H + time_L)) &
                       0xff;
    uint8_t data[] = {head1, head2, s_id, len, cmd,
                      addr, pos_H, pos_L, time_H, time_L, checknum};

    USART3_Send_ArrayU8(data, sizeof(data));
}

/* 设置同步写的缓存值 */
void UartServo_Set_Snyc_Buffer(uint16_t s1, uint16_t s2, uint16_t s3, uint16_t s4, uint16_t s5, uint16_t s6)
{
    sync_servo[0] = s1 + UartServo_Get_Median_Offset(1);
    sync_servo[1] = s2 + UartServo_Get_Median_Offset(2);
    sync_servo[2] = s3 + UartServo_Get_Median_Offset(3);
    sync_servo[3] = s4 + UartServo_Get_Median_Offset(4);
    sync_servo[4] = s5 + UartServo_Get_Median_Offset(5);
    sync_servo[5] = s6 + UartServo_Get_Median_Offset(6);

    for (int i = 0; i < 6; i++)
    {
        if (sync_servo[i] > ARM_MAX_VALUE)
        {
            sync_servo[i] = MEDIAN_VALUE;
        }
        else if (sync_servo[i] < ARM_MIN_VALUE)
        {
            sync_servo[i] = MEDIAN_VALUE;
        }
    }
}

/* 同时向多个舵机写入不同的参数 */
void UartServo_Sync_Write(uint16_t sync_time)
{
    uint8_t head1 = 0xff;
    uint8_t head2 = 0xff;
    uint8_t s_id = 0xfe;
    uint8_t len = 0x22; /* 数据长度，需要根据实际ID个数修改：len= sizeof(data)-4*/
    uint8_t cmd = 0x83;
    uint8_t addr = 0x2a;
    uint8_t data_len = 0x04; /* 实际单个写入舵机的字节数，不需要修改 */

    uint8_t ctrl_id[MAX_SERVO_NUM] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};

    uint8_t pos_n_H[MAX_SERVO_NUM] = {0};
    uint8_t pos_n_L[MAX_SERVO_NUM] = {0};

    uint8_t pos_sum = 0;
    uint8_t time_H = (sync_time >> 8) & 0xff;
    uint8_t time_L = sync_time & 0xff;

    for (uint8_t i = 0; i < MAX_SERVO_NUM; i++)
    {
        pos_n_H[i] = (sync_servo[i] >> 8) & 0xff;
        pos_n_L[i] = sync_servo[i] & 0xff;
        pos_sum = (pos_sum + ctrl_id[i] + pos_n_H[i] + pos_n_L[i] + time_H + time_L) & 0xff;
    }

    uint8_t checknum = (~(s_id + len + cmd + addr + data_len + pos_sum)) & 0xff;

    uint8_t data[] = {head1, head2, s_id, len, cmd, addr, data_len,
                      ctrl_id[0], pos_n_H[0], pos_n_L[0], time_H, time_L,
                      ctrl_id[1], pos_n_H[1], pos_n_L[1], time_H, time_L,
                      ctrl_id[2], pos_n_H[2], pos_n_L[2], time_H, time_L,
                      ctrl_id[3], pos_n_H[3], pos_n_L[3], time_H, time_L,
                      ctrl_id[4], pos_n_H[4], pos_n_L[4], time_H, time_L,
                      ctrl_id[5], pos_n_H[5], pos_n_L[5], time_H, time_L,
                      checknum};
    USART3_Send_ArrayU8(data, sizeof(data));
}

/* 设置总线舵机的扭矩开关,0为关闭，1为开启*/
void UartServo_Set_Torque(uint8_t enable)
{
    uint8_t on_off = 0x00; /* 扭矩参数，0为关闭，1为开启 */
    if (enable)
    {
        on_off = 0x01;
    }
    uint8_t head1 = 0xff;
    uint8_t head2 = 0xff;
    uint8_t s_id = 0xfe; /* 发送广播的ID */
    uint8_t len = 0x04;
    uint8_t cmd = 0x03;
    uint8_t addr = 0x28;

    uint8_t checknum = (~(s_id + len + cmd + addr + on_off)) & 0xff;
    uint8_t data[] = {head1, head2, s_id, len, cmd,
                      addr, on_off, checknum};
    USART3_Send_ArrayU8(data, sizeof(data));
}

/* 写入目标ID(1~250) */
void UartServo_Set_ID(uint8_t id)
{
    if ((id >= 1) && (id <= 250))
    {
        uint8_t head1 = 0xff;
        uint8_t head2 = 0xff;
        uint8_t s_id = 0xfe; /* 发送广播的ID */
        uint8_t len = 0x04;
        uint8_t cmd = 0x03;
        uint8_t addr = 0x05;
        uint8_t set_id = id; /* 实际写入的ID */

        uint8_t checknum = (~(s_id + len + cmd + addr + set_id)) & 0xff;
        uint8_t data[] = {head1, head2, s_id, len, cmd,
                          addr, set_id, checknum};
        USART3_Send_ArrayU8(data, sizeof(data));
    }
}

/*****************************读数据*********************************/
/* 读取舵机当前角度 */
void UartServo_Get_Angle(uint8_t id)
{
    uint8_t head1 = 0xff;
    uint8_t head2 = 0xff;
    uint8_t s_id = id & 0xff;
    uint8_t len = 0x04;
    uint8_t cmd = 0x02;
    uint8_t param_H = 0x38;
    uint8_t param_L = 0x02;

    uint8_t checknum = (~(s_id + len + cmd + param_H + param_L)) & 0xff;
    uint8_t data[] = {head1, head2, s_id, len, cmd, param_H, param_L, checknum};
    USART3_Send_ArrayU8(data, sizeof(data));
}

/* 接收串口数据 */
void UartServo_Revice(uint8_t Rx_Temp)
{
    switch (Rx_Flag)
    {
    case 0:
        if (Rx_Temp == 0xff)
        {
            Rx_Data[0] = 0xff;
            Rx_Flag = 1;
        }
        break;

    case 1:
        if (Rx_Temp == 0xf5)
        {
            Rx_Data[1] = 0xf5;
            Rx_Flag = 2;
            Rx_index = 2;
        }
        else
        {
            Rx_Flag = 0;
            Rx_Data[0] = 0x0;
        }
        break;

    case 2:
        Rx_Data[Rx_index] = Rx_Temp;
        Rx_index++;
        if (Rx_index >= RX_MAX_BUF)
        {
            Rx_Flag = 0;
            RecvFlag = FLAG_RECV;
        }
        break;
    default:
        break;
    }
}

/* 读取总线舵机标志位 */
uint8_t UartServo_Get_Flag(uint8_t flag)
{
    if (flag == FLAG_RECV)
    {
        return RecvFlag;
    }
    return 0;
}

/* 清空总线舵机标志位和缓存数据 */
void UartServo_Clear_Flag(uint8_t flag)
{
    RecvFlag = 0;
    if (flag == FLAG_RXBUFF)
    {
        for (uint8_t i = 0; i < RX_MAX_BUF; i++)
        {
            Rx_Data[i] = 0;
        }
    }
}

// 发送串口舵机的位置数据到串口
void UartServo_Send_ARM_Angle(uint8_t id, uint16_t value)
{
    #define LEN 8
    uint8_t checknum = 0;
    uint8_t data[LEN] = {0};
    uint16_t send_value = value;
    send_value = value - UartServo_Get_Median_Offset(id);

    data[0] = PTO_HEAD;
    data[1] = PTO_DEVICE_ID - 1;
    data[2] = LEN - 2;             // 数量
    data[3] = FUNC_UART_SERVO;     // 功能字
    data[4] = id;                  // ID
    data[5] = send_value & 0xff;        // 位置低位  //小端通讯
    data[6] = (send_value >> 8) & 0xff; // 位置高位
    for (uint8_t i = 2; i < LEN - 1; i++)
    {
        checknum += data[i];
    }
    data[LEN - 1] = checknum;
    USART1_Send_ArrayU8(data, LEN);
    // DEBUG("uartServo send ok\n");
}

// 发送机械臂的六个关节位置到上位机
void UartServo_Send_ARM_Angle_Array(void)
{
    #define LEN_A    17
    uint8_t checknum = 0, i = 0;
    uint8_t data[LEN_A] = {0};
    uint16_t send_val[6] = {0};
    for (i = 0; i < 6; i++)
    {
        send_val[i] = g_arm_read_value[i] - UartServo_Get_Median_Offset(i+1);
    }
    data[0] = PTO_HEAD;
    data[1] = PTO_DEVICE_ID - 1;
    data[2] = LEN_A - 2;                   // 数量
    data[3] = FUNC_ARM_CTRL;              // 功能字
    data[4] = send_val[0] & 0xff;         // 位置低位  //小端通讯
    data[5] = (send_val[0] >> 8) & 0xff;  // 位置高位
    data[6] = send_val[1] & 0xff;         // 位置低位  //小端通讯
    data[7] = (send_val[1] >> 8) & 0xff;  // 位置高位
    data[8] = send_val[2] & 0xff;         // 位置低位  //小端通讯
    data[9] = (send_val[2] >> 8) & 0xff;  // 位置高位
    data[10] = send_val[3] & 0xff;        // 位置低位  //小端通讯
    data[11] = (send_val[3] >> 8) & 0xff; // 位置高位
    data[12] = send_val[4] & 0xff;        // 位置低位  //小端通讯
    data[13] = (send_val[4] >> 8) & 0xff; // 位置高位
    data[14] = send_val[5] & 0xff;        // 位置低位  //小端通讯
    data[15] = (send_val[5] >> 8) & 0xff; // 位置高位
    for (i = 2; i < LEN_A - 1; i++)
    {
        checknum += data[i];
    }
    data[LEN_A - 1] = checknum;
    USART1_Send_ArrayU8(data, LEN_A);
    UartServo_Set_Read_State(ARM_READ_TO_UART);
}

// 清空串口读取机械臂的缓存数据, 设置为中位偏差的值，避免读不到数据返回数据不是0。
void UartServo_Clear_Arm_Read_Vlaue(void)
{
    for (int i = 0; i < 6; i++)
    {
        // g_arm_read_value[i] = 0;
        g_arm_read_value[i] = UartServo_Get_Median_Offset(i+1);
    }
}


/* 解析串口数据, 并发送到串口, 发送成功返回1， 否则返回0 */
uint8_t UartServo_Rx_Parse(uint8_t request_id)
{
    uint8_t result = 0;
    if (UartServo_Get_Flag(FLAG_RECV))
    {
        result = 1;
        UartServo_Clear_Flag(FLAG_RECV);
        uint8_t checknum = (~(Rx_Data[2] + Rx_Data[3] + Rx_Data[4] + Rx_Data[5] + Rx_Data[6])) & 0xff;
        if (checknum == Rx_Data[7])
        {
            uint8_t s_id = Rx_Data[2];
            uint16_t read_value = Rx_Data[5] << 8 | Rx_Data[6];
            // if (request_id != s_id)
            //     return 0;
            DEBUG("read arm value:%d, %d\n", s_id, read_value);
            if (UartServo_Get_Read_State() == ARM_READ_TO_FLASH) // 保存到flash
            {
                UartServo_Verify_Offset(s_id, read_value);
            }
            else if (UartServo_Get_Read_State() == ARM_READ_TO_ARM) // 保存到机械臂读取所有角度的临时值
            {
                if (s_id > 0 && s_id <= 6)
                {
                    g_arm_read_value[s_id-1] = read_value;
                }
            }
            else // 发送到串口
            {
                UartServo_Send_ARM_Angle(s_id, read_value);
            }
            if (Bsp_Get_TestMode())
            {
                g_test_read_ok = 1;
            }
        }
    }
    return result;
}

uint16_t UartServo_Test_Read_State(void)
{
    return g_test_read_ok;
}


// /* 解析串口数据,并发送到串口, 发送成功返回1， 否则返回0 */
// uint8_t UartServo_Rx_Parse(uint8_t request_id)
// {
//     uint8_t result = 0;
//     if (UartServo_Get_Flag(FLAG_RECV))
//     {
//         uint8_t checknum = (~(Rx_Data[2] + Rx_Data[3] + Rx_Data[4] + Rx_Data[5] + Rx_Data[6])) & 0xff;
//         if (checknum == Rx_Data[7])
//         {
//             uint8_t s_id = Rx_Data[2];
//             uint16_t read_value = Rx_Data[5] << 8 | Rx_Data[6];
//             if (request_id != s_id)
//                 return 0;
//             DEBUG("read arm value:%d, %d\n", s_id, read_value);
//             if (UartServo_Get_Read_State()) // 保存到flash
//             {
//                 UartServo_Verify_Offset(s_id, read_value);
//             }
//             else // 发送到串口
//             {
//                 UartServo_Send_ARM_Angle(s_id, read_value);
//             }
//             result = 1;
//         }
//         UartServo_Clear_Flag(FLAG_RECV);
//     }
//     return result;
// }

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

// 中位偏差相关

/* 设置中位偏差的全局变量的值 */
// id=1~6, value=ARM_MIN_VALUE~ARM_MAX_VALUE
void UartServo_Set_Median_Value(uint8_t id, uint16_t value)
{
    g_arm_median_value[id - 1] = value;
}

/* 读取中位偏差的全局变量的值 */
int16_t UartServo_Get_Median_Offset(uint8_t id)
{
    int16_t value = 0;
    if ((id >= 1) && (id <= MAX_SERVO_NUM))
    {
        if (id == 5)
        {
            value = g_arm_median_value[id-1] - MID_VAL_ID5;
        }
        else if (id == 6)
        {
            value = g_arm_median_value[id-1] - MID_VAL_ID6;
        }
        else
        {
            value = g_arm_median_value[id-1] - MEDIAN_VALUE;
        }
    }
    return value;
}

/* 从flash读取所有舵机中位偏差 */
void UartServo_Read_All_Median_Value(void)
{
    for (uint8_t i = 0; i < MAX_SERVO_NUM; i++)
    {
        uint8_t id = i + 1;
        uint16_t value = 0;
        Flash_Read_ARM_Median_Value(id, &value);
        g_arm_median_value[i] = value;
    }
}

/* 中位偏移复位，设置flash的值为默认 */
void UartServo_Offset_Reset(void)
{
    Flash_Reset_ARM_Median_Value();
    UartServo_Read_All_Median_Value();
}

/* 设置读取舵机位置后的状态，0：发送到串口，1：保存中位偏差 */
void UartServo_Set_Read_State(uint16_t state)
{
    servo_read_state = state;
}

/* 返回读取舵机位置后的状态，0：发送到串口，1：保存中位偏差 */
uint16_t UartServo_Get_Read_State(void)
{
    return servo_read_state;
}

// 发送机械臂偏移状态到串口
void UartServo_Send_Offset_State(uint8_t id, uint8_t state)
{
    #define LENS       7
    uint8_t checknum = 0;
    uint8_t data[LENS] = {0};
    data[0] = PTO_HEAD;
    data[1] = PTO_DEVICE_ID - 1;
    data[2] = LENS - 2;             // 数量
    data[3] = FUNC_ARM_OFFSET;     // 功能字
    data[4] = id;                  // ID
    data[5] = state;               // 状态
    for (uint8_t i = 2; i < LENS - 1; i++)
    {
        checknum += data[i];
    }
    data[LENS - 1] = checknum;
    USART1_Send_ArrayU8(data, LENS);
    UartServo_Set_Read_State(ARM_READ_TO_UART);
}

// 验证读取到的机械臂中位偏移量是否正常
void UartServo_Verify_Offset(uint8_t id, uint16_t value)
{
    if (id > 0 && id < 7)
    {
        if (id == 5)
        {
            // MID_VAL_ID5 ± 300
            if (value >= 1186 && value <= 1786)
            {
                Flash_Set_ARM_Median_Value(id, value);
                UartServo_Send_Offset_State(id, FLAG_OFFSET_OK);
            }
            else
            {
                UartServo_Send_Offset_State(id, FLAG_OFFSET_OVER);
            }
        }
        else if (id == 6)
        {
            if (value >= 2700 && value <= 3500)
            {
                Flash_Set_ARM_Median_Value(id, value);
                UartServo_Send_Offset_State(id, FLAG_OFFSET_OK);
            }
            else
            {
                UartServo_Send_Offset_State(id, FLAG_OFFSET_OVER);
            }
        }
        else
        {
            if (value >= 1600 && value <= 2400)
            {
                Flash_Set_ARM_Median_Value(id, value);
                UartServo_Send_Offset_State(id, FLAG_OFFSET_OK);
            }
            else
            {
                UartServo_Send_Offset_State(id, FLAG_OFFSET_OVER);
            }
        }
    }
}
