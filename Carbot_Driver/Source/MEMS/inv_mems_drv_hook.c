#include <stdio.h>
#include <string.h>

#include "inv_mems_hw_config.h"
#include "invn_types.h"

#include "icm20948.h"
#include "app.h"
#include "bsp.h"
#include "bsp_spi.h"


int inv_serial_interface_write_hook(uint16_t reg, uint32_t length, uint8_t *data)
{
    unsigned char rx;
    int result = 0, i = 0;

    ICM20948_CS = 0;

    result |= SPI2_ReadWriteByte((unsigned char)reg, &rx);

    for(; i < length; i++)
    {
        result |= SPI2_ReadWriteByte( data[i], &rx);
    }

    ICM20948_CS = 1;

    return result;
}

int inv_serial_interface_read_hook(uint16_t reg, uint32_t length, uint8_t *data)
{
    unsigned char rx;
    int result = 0, i = 0;

    ICM20948_CS = 0;
    reg = reg | 0x80;
    result |= SPI2_ReadWriteByte((unsigned char)reg, &rx);

    for(; i < length; i++)
    {
        result |= SPI2_ReadWriteByte(0xff, data++);
    }

    ICM20948_CS = 1;

    return result;
}

//外部中断服务程序
#if ENABLE_ICM20948
void EXTI4_IRQHandler(void)
{
    if(EXTI_GetITStatus(INVEN_INT_EXTI_LINE) == 1)
    {
        // fifo_handler();    //开启usart_dbg后不宜在此处处理数据
        gyro_data_ready_cb();

        EXTI_ClearITPendingBit(INVEN_INT_EXTI_LINE);
    }
}
#endif

/**
 *  @brief  Sleep function.
**/
void inv_sleep(unsigned long mSecs)
{
    if (ICM20948_Get_Init_State())
    {
        App_Delay_ms(1);
    }
    else
    {
        delay_ms(mSecs);
    }
}

void inv_sleep_100us(unsigned long nHowMany100MicroSecondsToSleep)
{
    if (ICM20948_Get_Init_State())
    {
        inv_sleep(nHowMany100MicroSecondsToSleep);
    }
    else
    {
        delay_us(100 * nHowMany100MicroSecondsToSleep);
    }
}

/**
 *  @brief  get system's internal tick count.
 *          Used for time reference.
 *  @return current tick count.
**/
long long inv_get_tick_count(void)
{
    long long count;

    get_tick_count(&count);

    return (long long)count;
}

