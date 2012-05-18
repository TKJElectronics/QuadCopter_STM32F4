#ifndef __ULTRASONIC_SENSOR_H__
#define __ULTRASONIC_SENSOR_H__

#include "stm32f4xx.h"
#include "Delay.h"

/*
 * TRIG (OUTPUT): PE12
 * ECHO (INPUT):  PE14 (TIM1, CH4) - Use Input Capture (see example)
 */
int Ultrasonic_Init(void);
void Ultrasonic_TIM_Config(void);
void Ultrasonic_InitOutput(void);
void TIM1_UP_TIM10_IRQHandler(void);



#endif /* __ULTRASONIC_SENSOR_H__ */
