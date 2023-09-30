#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include "stm32f10x.h"

#define PTO_MAX_BUF_LEN           (20)


#define PTO_HEAD                  (0xFF)
#define PTO_DEVICE_ID             (0xFC)

/* 功能名称定义 */
#define FUNC_AUTO_REPORT          (0x01)
#define FUNC_BEEP                 (0x02)
#define FUNC_PWM_SERVO            (0x03)
#define FUNC_PWM_SERVO_ALL        (0x04)
#define FUNC_RGB                  (0x05)
#define FUNC_RGB_EFFECT           (0x06)

#define FUNC_REPORT_SPEED         (0x0A)
#define FUNC_REPORT_MPU_RAW       (0x0B)
#define FUNC_REPORT_IMU_ATT       (0x0C)
#define FUNC_REPORT_ENCODER       (0x0D)
#define FUNC_REPORT_ICM_RAW       (0x0E)
#define FUNC_RESET_STATE          (0x0F)


/* 控制小车运动 */
#define FUNC_MOTOR                (0x10)
#define FUNC_CAR_RUN              (0x11)
#define FUNC_MOTION               (0x12)
#define FUNC_SET_MOTOR_PID        (0x13)
#define FUNC_SET_YAW_PID          (0x14)
#define FUNC_CAR_TYPE             (0x15)


/* 总线舵机相关变量 */
#define FUNC_UART_SERVO           (0x20)
#define FUNC_UART_SERVO_ID        (0x21)
#define FUNC_UART_SERVO_TORQUE    (0x22)
#define FUNC_ARM_CTRL             (0x23)
#define FUNC_ARM_OFFSET           (0x24)


/* 阿克曼类型小车相关变量 */
#define FUNC_AKM_DEF_ANGLE        (0x30)
#define FUNC_AKM_STEER_ANGLE      (0x31)


/* 请求数据 */
#define FUNC_REQUEST_DATA         (0x50)
#define FUNC_VERSION              (0x51)
#define FUNC_NOW_YAW              (0x52)

/* CAN相关变量 */
#define FUNC_CAN_RX_BEEP          (0x60)


/* OLED相关变量 */
#define FUNC_OLED_FLAG            (0x70)



/* 重置FLASH数据 */
#define FUNC_RESET_FLASH          (0xA0)

/* 测试模式 */
#define FUNC_TESE_MODE            (0xA1)
#define FUNC_TEST_CLEAR_YAW       (0xA2)


/* 其他参数 */
#define SAVE_VERIFY                (0x5F)


void Upper_Data_Receive(uint8_t data);
void Upper_Data_Parse(uint8_t *data_buf, uint8_t num);
void Upper_Data_Parse_Low_Battery(uint8_t *data_buf, uint8_t num);


void Clear_CMD_Flag(void);
void Clear_RxBuffer(void);
uint8_t* Get_RxBuffer(void);
uint8_t Get_CMD_Length(void);
uint8_t Get_CMD_Flag(void);

uint8_t Get_Request_Flag(void);
void Request_Data(uint8_t request, uint8_t parm);
void Send_Request_Data(void);

void Upper_CAN_Execute_Command(uint8_t func, uint8_t* parm);


#endif
