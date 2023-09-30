#ifndef __APP_MECANUM_H__
#define __APP_MECANUM_H__

#include "stdint.h"


// 智能物流车间距
#define CARBOT_WIDTH								 (150.0f)    // 毫米
#define CARBOT_LENGTH								 (158.0f)    // 毫米

// 智能物流车间距之和的一半
#define CARBOT_APB									 ((CARBOT_WIDTH + CARBOT_LENGTH)/2.0f)

// 智能物流车转一整圈的位移，单位为mm
#define CARBOT_CIRCLE_MM             (251.327f)

// 智能物流车速度限制
#define CAR_CARBOT_MAX_SPEED        (500)


// 小麦轮车底盘间距
#define ROBOT_WIDTH                  (169.0f)    // 毫米
#define ROBOT_LENGTH                 (160.11f)   // 毫米

// 小麦轮底盘电机间距之和的一半
// #define MECANUM_APB               ((ROBOT_WIDTH + ROBOT_LENGTH)/2.0f)
#define MECANUM_APB                  (164.555f)

// 小麦轮转一整圈的位移，单位为mm
#define MECANUM_CIRCLE_MM            (204.203f)
#define MECANUM_MINI_CIRCLE_MM       (204.203f)


// 大麦轮底盘电机间距之和的一半
#define MECANUM_MAX_APB              (214.1f)
// 大麦轮转一整圈的位移，单位为MM
#define MECANUM_MAX_CIRCLE_MM        (251.327f)

// 小麦轮大底盘电机间距之和的一半
#define MECANUM_MINI_APB             (174.5f)

// 旭日派小车底盘电机间距之和的一半
#define MECANUM_SUNRISE_APB          (143.68f)

// X3 PLUS小车速度限制
#define CAR_X3_PLUS_MAX_SPEED        (700)




void Mecanum_Ctrl(int16_t V_x, int16_t V_y, int16_t V_z, uint8_t adjust);
void Mecanum_State(uint8_t state, uint16_t speed, uint8_t adjust);

void Mecanum_Yaw_Calc(float yaw);

#endif /* __APP_MECANUM_H__ */
