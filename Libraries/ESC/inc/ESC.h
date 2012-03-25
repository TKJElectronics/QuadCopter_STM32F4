#ifndef __ESC_H__
#define __ESC_H__

#include "stm32f4xx.h"
#include "Delay.h"
#include "bool.h"

#define ESC_LEFT 2
#define ESC_RIGHT 1
#define ESC_FRONT 0
#define ESC_BACK 3

#define ESC_THROTTLE_MIN 400
#define ESC_THROTTLE_MAX 800
#define ESC_THROTTLE_RANGE 	ESC_THROTTLE_MAX-ESC_THROTTLE_MIN	// 400

#define ESC_WITH_CALIBRATION true
#define ESC_WITHOUT_CALIBRATION false

extern u16 ESC_Motor_Values[4];

void ESC_InitAll(bool CalibrationEnabled);
void ESC_Outputs_Init(void);
void ESC_Timer_Init(void);

void ESC_SetSpeed(u8 ESC_Number, u16 Speed);
void ESC_UpdateAll(void);
void ESC_ThrottleProgramming(void);

void ESC_MotorOrientationTest(void);
void ESC_MotorSpeedTest(void);


#endif /* __ESC_H__ */
