#ifndef __IMU_VECTORS_H__
#define __IMU_VECTORS_H__

#include "stm32f4xx.h"
#include "bool.h"
#include <Math.h>
#include <stdlib.h>

float Vector_Dot_Product(float vector1[3], float vector2[3]);
void Vector_Cross_Product(float vectorOut[3], float v1[3], float v2[3]);
void Vector_Scale(float vectorOut[3], float vectorIn[3], float scale2);
void Vector_Add(float vectorOut[3], float vectorIn1[3], float vectorIn2[3]);
void Matrix_Multiply(float a[3][3], float b[3][3],float mat[3][3]);
void init_rotation_matrix(float m[3][3], float yaw, float pitch, float roll);

float constrain(float x, float a, float b);
float findMedianFloat(float *data, int arraySize);
float filterSmooth(float currentData, float previousData, float smoothFactor);
float low_pass_filter(float vNew,float vPrev,float factor);

#endif /* __IMU_VECTORS_H__ */
