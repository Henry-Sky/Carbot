#ifndef __APP_MATH_H__
#define __APP_MATH_H__

#include "stdint.h"
#include "math.h"

#define M_PI 3.14159265358979f

extern const float RtA;
extern const float AtR;


int Math_Limit_int(int input, int val_min, int val_max);
float Math_Limit_float(float input, float val_min, float val_max);
int Math_Map(int x, int in_min, int in_max, int out_min, int out_max);




#endif /* __APP_MATH_H__ */
