#ifndef __APP_PID_H__
#define __APP_PID_H__

#include "stdint.h"

#define PID_SUNRISE_KP  (1.5f)
#define PID_SUNRISE_KI  (0.08f)
#define PID_SUNRISE_KD  (0.5f)

#define PID_DEF_KP      (0.8f)
#define PID_DEF_KI      (0.06f)
#define PID_DEF_KD      (0.5f)

#define PID_YAW_DEF_KP  (0.4)
#define PID_YAW_DEF_KI  (0.0)
#define PID_YAW_DEF_KD  (0.1)


typedef struct _pid
{
    float target_val;               //目标值
    float output_val;               //输出值
    float pwm_output;        		//PWM输出值
    float Kp,Ki,Kd;          		//定义比例、积分、微分系数
    float err;             			//定义偏差值
    float err_last;          		//定义上一个偏差值

    float err_next;                 //定义下一个偏差值, 增量式
    float integral;          		//定义积分值，位置式
} pid_t;

typedef struct _motor_data_t
{
    float speed_mm_s[4];        // 输入值，编码器计算速度
    float speed_pwm[4];         // 输出值，PID计算出PWM值
    int16_t speed_set[4];       // 速度设置值
} motor_data_t;


typedef struct
{
    float SetPoint;   // 设定目标Desired value
    float Proportion; // 比例常数Proportional Const
    float Integral;   // 积分常数Integral Const
    float Derivative; // 微分常数Derivative Const
    float LastError;  // Error[-1]
    float PrevError;  // Error[-2]
    float SumError;   // Sums of Errors
} PID;



void PID_Param_Init(void);

float PID_Location_Calc(pid_t *pid, float actual_val);
void PID_Calc_Motor(motor_data_t* motor);
float PID_Calc_One_Motor(uint8_t motor_id, float now_speed);
void PID_Set_Motor_Target(uint8_t motor_id, float target);
void PID_Clear_Motor(uint8_t motor_id);
void PID_Set_Motor_Parm(uint8_t motor_id, float kp, float ki, float kd);
void PID_Send_Parm_Active(uint8_t index);



void PID_Yaw_Reset(float yaw);
float PID_Yaw_Calc(float NextPoint);
void PID_Yaw_Set_Parm(float kp, float ki, float kd);

#endif /* __APP_PID_H__ */
