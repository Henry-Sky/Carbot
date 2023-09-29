#ifndef __ICM_20948_H
#define __ICM_20948_H
#include "bsp_common.h"

#define	ICM20948_CS  PBout(12)   //片选信号


//外部中断配置
#define INVEN_INT_PIN                         GPIO_Pin_4
#define INVEN_INT_GPIO_PORT                   GPIOA
#define INVEN_INT_GPIO_CLK                    RCC_APB2Periph_GPIOA
#define INVEN_INT_EXTI_PORT                   GPIO_PortSourceGPIOA
#define INVEN_INT_EXTI_PIN                    GPIO_PinSource4
#define INVEN_INT_EXTI_LINE                   EXTI_Line4
#define INVEN_INT_EXTI_IRQ                    EXTI4_IRQn     


struct hal_s_
{
    long report;          // What to report?
    unsigned char  debug_reg_on;     // with '\' as a command this turns ON
    int expected_data;
    volatile unsigned char new_gyro;
};

// 滚转角Φ（Roll）,  范围为 (-90 to 90)
// 俯仰角θ（Pitch）, 范围为 (-180 to 180)
// 偏航角ψ（Yaw）,   范围为 (-180 to 180)
typedef struct
{
	float accel_float[3];
	float gyro_float[3];
	float compass_float[3];
	float orientation[3];    // 弧度[Roll, Pitch, Yaw]
	float temperature;
}icm20948_data_t;

int ICM_20948_Init(void);
void fifo_handler(void);
int handle_char_input(char c);
void gyro_data_ready_cb(void);

void self_test(void);

float ICM20948_Get_Temperature(void);
void ICM20948_Get_Data(icm20948_data_t *data);
uint8_t ICM20948_Get_Init_State(void);

int get_tick_count(long long *count);
void ICM20948_Read_Data_Handle(void);
void ICM20948_Send_Raw_Data(void);
void ICM20948_Send_Attitude_Data(void);


float ICM20948_Get_Yaw_Now(void);
float ICM20948_Get_Roll_Now(void);
float ICM20948_Get_Pitch_Now(void);


#endif	/* __ICM_20948_H */





