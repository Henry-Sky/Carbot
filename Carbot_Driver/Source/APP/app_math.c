#include "app_math.h"


const float RtA = 57.2957795f;   //弧度（radian）转角度（angle）的常数
const float AtR = 0.0174532925f; //角度转弧度的常数



// int类型数值输入限制
int Math_Limit_int(int input, int val_min, int val_max)
{
    if (input < val_min) return val_min;
    if (input > val_max) return val_max;
    return input;
}


// float类型数值输入限制
float Math_Limit_float(float input, float val_min, float val_max)
{
    if (input < val_min) return val_min;
    if (input > val_max) return val_max;
    return input;
}

// 映射数值,整数类型。
int Math_Map(int x, int in_min, int in_max, int out_min, int out_max)
{
    int res = (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
    if (res < out_min) res = out_min;
    else if (res > out_max) res = out_max;
    return res;
}
