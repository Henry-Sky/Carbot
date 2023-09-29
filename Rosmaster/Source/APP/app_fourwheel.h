#ifndef __APP_FOURWHEEL_H__
#define __APP_FOURWHEEL_H__

#include "stdint.h"


// 四轮车底盘电机间距之和的一半
// #define FOURWHEEL_APB               ((ROBOT_WIDTH + ROBOT_LENGTH)/2.0f)
#define FOURWHEEL_APB                  (164.555f)


// 四轮小车轮子转一整圈的位移，单位为MM
#define FOURWHEEL_CIRCLE_MM            (215.2f)


void Fourwheel_Ctrl(int16_t V_x, int16_t V_y, int16_t V_z, uint8_t adjust);
void Fourwheel_State(uint8_t state, uint16_t speed, uint8_t adjust);
void Fourwheel_Yaw_Calc(float yaw);

#endif /*__APP_FOURWHEEL_H__*/
