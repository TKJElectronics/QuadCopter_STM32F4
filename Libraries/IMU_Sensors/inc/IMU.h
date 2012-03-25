#ifndef __IMU_H__
#define __IMU_H__

#include "stm32f4xx.h"
#include "bool.h"

#include "IMU_DCM.h"
#include "IMU_I2C.h"
#include "IMU_Sensors.h"
#include "IMU_Vectors.h"
#include "IMU_Calibration.h"
#include "PID.h"

typedef struct
{
	float Pitch;
	float Roll;
	float Yaw;
	float Omega_Vector[3];
} AverageMeasurements;
extern AverageMeasurements IMU_Average;

void IMU_InitTimer(u16 IMU_Freq);
void IMU_Task(void);
void IMU_Timer_Configuration(u16 IMU_Freq);
void IMU_Timer_Interrupt_Configuration(void);
void TIM2_IRQHandler(void);



#endif /* __IMU_H__ */
