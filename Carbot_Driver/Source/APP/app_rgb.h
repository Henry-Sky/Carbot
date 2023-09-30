#ifndef __APP_RGB_H__
#define __APP_RGB_H__

#include "stdint.h"



typedef enum _rgb_effect_t
{
    RGB_NO_EFFECT = 0,

    RGB_WATERFALL,  // 流水灯
    RGB_MARQUEE,    // 跑马灯
    RGB_BREATHING,  // 呼吸灯
    RGB_GRADIENT,   // 渐变灯 
    RGB_STARBRIGHT, // 星光点点
    RGB_BATTERY,    // 电池电量显示

    RGB_MAX_EFFECT
} rgb_effect_t;


void app_rgb_init(void);
void app_rgb_effects_handle(void);

void app_rgb_set_breathing_color(uint8_t color);
void app_rgb_set_effect(uint8_t effect, uint8_t speed);


uint8_t app_rgb_get_speed(void);
rgb_effect_t app_rgb_get_effect(void);


#endif /* __APP_RGB_H__ */
