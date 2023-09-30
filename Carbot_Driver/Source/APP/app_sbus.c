#include "app_sbus.h"
#include "app.h"
#include "app_motion.h"
#include "app_mecanum.h"
#include "app_ackerman.h"
#include "app_fourwheel.h"
#include "app_math.h"

#include "string.h"
#include "bsp_usart.h"


#define SBUS_RECV_MAX    25
#define SBUS_START       0x0F
#define SBUS_END         0x00


uint8_t SBUS_RX_BUF[SBUS_RECV_MAX]; //接收缓冲,最大USART_REC_LEN个字节.


uint8_t sbus_start = 0;
uint8_t sbus_buf_index = 0;
uint8_t sbus_new_cmd = 0;

// SUBUS原始数据缓冲区
uint8_t inBuffer[SBUS_RECV_MAX] = {0};
uint8_t failsafe_status = SBUS_SIGNAL_FAILSAFE;

uint8_t sbus_data[SBUS_RECV_MAX] = {0};
int16_t g_sbus_channels[18] = {0};

int16_t g_sbus_speed_X = 0;
int16_t g_sbus_speed_Y = 0;
int16_t g_sbus_speed_Z = 0;


// 解析SBUS的数据，转化成数值。
void SBUS_Parse_Data(void)
{
    g_sbus_channels[0]  = ((sbus_data[1] | sbus_data[2] << 8) & 0x07FF);
    g_sbus_channels[1]  = ((sbus_data[2] >> 3 | sbus_data[3] << 5) & 0x07FF);
    g_sbus_channels[2]  = ((sbus_data[3] >> 6 | sbus_data[4] << 2 | sbus_data[5] << 10) & 0x07FF);
    g_sbus_channels[3]  = ((sbus_data[5] >> 1 | sbus_data[6] << 7) & 0x07FF);
    g_sbus_channels[4]  = ((sbus_data[6] >> 4 | sbus_data[7] << 4) & 0x07FF);
    g_sbus_channels[5]  = ((sbus_data[7] >> 7 | sbus_data[8] << 1 | sbus_data[9] << 9) & 0x07FF);
    g_sbus_channels[6]  = ((sbus_data[9] >> 2 | sbus_data[10] << 6) & 0x07FF);
    g_sbus_channels[7]  = ((sbus_data[10] >> 5 | sbus_data[11] << 3) & 0x07FF);
    #ifdef ALL_CHANNELS
    g_sbus_channels[8]  = ((sbus_data[12] | sbus_data[13] << 8) & 0x07FF);
    g_sbus_channels[9]  = ((sbus_data[13] >> 3 | sbus_data[14] << 5) & 0x07FF);
    g_sbus_channels[10] = ((sbus_data[14] >> 6 | sbus_data[15] << 2 | sbus_data[16] << 10) & 0x07FF);
    g_sbus_channels[11] = ((sbus_data[16] >> 1 | sbus_data[17] << 7) & 0x07FF);
    g_sbus_channels[12] = ((sbus_data[17] >> 4 | sbus_data[18] << 4) & 0x07FF);
    g_sbus_channels[13] = ((sbus_data[18] >> 7 | sbus_data[19] << 1 | sbus_data[20] << 9) & 0x07FF);
    g_sbus_channels[14] = ((sbus_data[20] >> 2 | sbus_data[21] << 6) & 0x07FF);
    g_sbus_channels[15] = ((sbus_data[21] >> 5 | sbus_data[22] << 3) & 0x07FF);
    #endif

    // Failsafe
    failsafe_status = SBUS_SIGNAL_OK;
    if (sbus_data[23] & (1 << 2))
    {
        failsafe_status = SBUS_SIGNAL_LOST;
        g_sbus_channels[0]  = SBUS_MIDDLE_VALUE;
        g_sbus_channels[1]  = SBUS_MIDDLE_VALUE;
        g_sbus_channels[2]  = SBUS_MIDDLE_VALUE;
        g_sbus_channels[3]  = SBUS_MIDDLE_VALUE;
        DEBUG("SBUS_SIGNAL_LOST");
    }
    else if (sbus_data[23] & (1 << 3))
    {
        failsafe_status = SBUS_SIGNAL_FAILSAFE;
        g_sbus_channels[0]  = SBUS_MIDDLE_VALUE;
        g_sbus_channels[1]  = SBUS_MIDDLE_VALUE;
        g_sbus_channels[2]  = SBUS_MIDDLE_VALUE;
        g_sbus_channels[3]  = SBUS_MIDDLE_VALUE;
        DEBUG("SBUS_SIGNAL_FAILSAFE");
    }
}

// 接收SBUS的数据
void SBUS_Reveive(uint8_t data)
{
    if (sbus_start == 0 && data == SBUS_START)
    {
        sbus_start = 1;
        sbus_new_cmd = 0;
        sbus_buf_index = 0;
        inBuffer[sbus_buf_index] = data;
        inBuffer[SBUS_RECV_MAX - 1] = 0xff;
    }
    else if (sbus_start)
    {
        sbus_buf_index++;
        inBuffer[sbus_buf_index] = data;
    }

    if (sbus_start & sbus_buf_index >= (SBUS_RECV_MAX - 1))
    {
        sbus_start = 0;
        if (inBuffer[SBUS_RECV_MAX - 1] == SBUS_END)
        {
            memcpy(sbus_data, inBuffer, SBUS_RECV_MAX);
            sbus_new_cmd = 1;
            // DEBUG("XX-%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X-XX\r\n",
            //     sbus_data[0],  sbus_data[1],  sbus_data[2],  sbus_data[3],  sbus_data[4],
            //     sbus_data[5],  sbus_data[6],  sbus_data[7],  sbus_data[8],  sbus_data[9],
            //     sbus_data[10], sbus_data[11], sbus_data[12], sbus_data[13], sbus_data[14],
            //     sbus_data[15], sbus_data[16], sbus_data[17], sbus_data[18], sbus_data[19],
            //     sbus_data[20], sbus_data[21], sbus_data[22], sbus_data[23], sbus_data[24]);
        }
    }
}



// SBUS接收处理数据句柄
void SBUS_Handle(void)
{
    static uint16_t stop_flag = 0;
    static uint16_t stop_count = 100;
    static uint8_t brake = 0;
    if (sbus_new_cmd)
    {
        sbus_new_cmd = 0;
        SBUS_Parse_Data();
        #if ENABLE_DEBUG_SBUS
        #ifdef ALL_CHANNELS
        printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
               g_sbus_channels[0], g_sbus_channels[1], g_sbus_channels[2], g_sbus_channels[3], g_sbus_channels[4],
               g_sbus_channels[5], g_sbus_channels[6], g_sbus_channels[7], g_sbus_channels[8], g_sbus_channels[9],
               g_sbus_channels[10], g_sbus_channels[11], g_sbus_channels[12], g_sbus_channels[13], g_sbus_channels[14], g_sbus_channels[15]);
        #else
        printf("%d,%d,%d,%d,%d,%d,%d,%d\r\n",
               g_sbus_channels[0], g_sbus_channels[1], g_sbus_channels[2], g_sbus_channels[3], g_sbus_channels[4],
               g_sbus_channels[5], g_sbus_channels[6], g_sbus_channels[7]);
        #endif
        #else
        
        // 控制小车运动
        int16_t value_x = -(g_sbus_channels[2] - SBUS_MIDDLE_VALUE);
        int16_t value_y = (g_sbus_channels[3] - SBUS_MIDDLE_VALUE);
        int16_t value_z = (g_sbus_channels[0] - SBUS_MIDDLE_VALUE);
        if (value_x > -60 && value_x < 60) value_x = 0;
        if (value_y > -60 && value_y < 60) value_y = 0;
        if (value_z > -60 && value_z < 60) value_z = 0;
        
        if (value_x == 0 && value_y == 0 && value_z == 0)
        {
            if (stop_flag < stop_count)
            {
                stop_flag++;
                brake = g_sbus_channels[5]<500?STOP_FREE:STOP_BRAKE;
                Motion_Stop(brake);
                if (Motion_Get_Car_Type() == CAR_ACKERMAN)
                {
                    Ackerman_Steering(0);
                }
            }
            if (stop_flag == stop_count)
            {
                // 刹车超时切换到自由停止。
                stop_flag = stop_count + 1;
                Motion_Stop(STOP_FREE);
                DEBUG("SBUS STOP TIMEOUT\n");
            }
        }
        else
        {
            switch (Motion_Get_Car_Type())
            {
							case CAR_CARBOT:
							{
									g_sbus_speed_X = value_x * 1.2;
									g_sbus_speed_Y = value_y * 1.2;
									g_sbus_speed_Z = value_z * 5;
									Mecanum_Ctrl(g_sbus_speed_X, g_sbus_speed_Y, g_sbus_speed_Z, 0);
									break;
							}
							case CAR_MECANUM:
							{
									g_sbus_speed_X = value_x * 1.2;
									g_sbus_speed_Y = value_y * 1.2;
									g_sbus_speed_Z = value_z * 5;
									Mecanum_Ctrl(g_sbus_speed_X, g_sbus_speed_Y, g_sbus_speed_Z, 0);
									break;
							}
							case CAR_MECANUM_MAX:
							{
									g_sbus_speed_X = value_x * 1.2;
									g_sbus_speed_Y = value_y * 1.2;
									g_sbus_speed_Z = value_z * 5;
									Mecanum_Ctrl(g_sbus_speed_X, g_sbus_speed_Y, g_sbus_speed_Z, 0);
									break;
							}
							case CAR_FOURWHEEL:
							{
									g_sbus_speed_X = value_x * 1.2;
									g_sbus_speed_Y = 0;
									g_sbus_speed_Z = value_z * 6;
									Fourwheel_Ctrl(g_sbus_speed_X, g_sbus_speed_Y, g_sbus_speed_Z, 0);
									break;
							}
							case CAR_ACKERMAN:
							{
									g_sbus_speed_X = value_x * 2.3;
									g_sbus_speed_Y = value_z / 18;
									g_sbus_speed_Z = 0;
									Ackerman_Ctrl(g_sbus_speed_X, g_sbus_speed_Y, g_sbus_speed_Z, 0);
									DEBUG("SBUS Akm Ctrl:%d, %d, %d\n", g_sbus_speed_X, g_sbus_speed_Y, g_sbus_speed_Z);
									break;
							}
							case CAR_SUNRISE:
							{
									g_sbus_speed_X = value_x * 1.2;
									g_sbus_speed_Y = value_y * 1.2;
									g_sbus_speed_Z = value_z * 5;
									Mecanum_Ctrl(g_sbus_speed_X, g_sbus_speed_Y, g_sbus_speed_Z, 0);
									break;
							}
							default:
									break;
            }
            stop_flag = 0;
        }
        #endif
    }
}
