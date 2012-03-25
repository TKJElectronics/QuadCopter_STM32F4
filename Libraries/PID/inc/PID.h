#ifndef __PID_H__
#define __PID_H__

#include "stm32f4xx.h"
#include "bool.h"

#include "ESC.h"
#include "IMU.h"


#define PID_Pitch_Kp_Init 0.5000f // 0.5000
#define PID_Pitch_Ki_Init 0.0000f
#define PID_Pitch_Kd_Init 0.0000f // 0.2000

#define PID_Roll_Kp_Init 0.5000f
#define PID_Roll_Ki_Init 0.0f
#define PID_Roll_Kd_Init 0.0f

#define PID_Yaw_Kp_Init 0.0f
#define PID_Yaw_Ki_Init 0.0f
#define PID_Yaw_Kd_Init 0.0f


/// Low pass filter cut frequency for derivative calculation.
///
/// 20 Hz because anything over that is probably noise, see
/// http://en.wikipedia.org/wiki/Low-pass_filter.
///
#define PID_fCut		20	// Hertz


typedef struct
{
	u16 Throttle;
	float Pitch;
	float Roll;
	float Yaw;
} QuadCopter_Settings;

typedef struct
{
	u32 TimeStamp;
	u32 TimeStamp_Old;
	float Dt;

	float Error_Before;
	float Error_Current;

	float Kp; // Proportional constant
	float Proportional;

	float Ki; // Integral constant
	float Integral;

	float Kd; // Derivative constant
	float Derivative; // Differentiation
	float Derivative_Before;

	float Control;
} PID_Regulators;

extern QuadCopter_Settings QuadCopter;

extern PID_Regulators PID_PitchVar;
extern PID_Regulators PID_RollVar;
extern PID_Regulators PID_YawVar;

extern bool PID_Enabled;


void PID_Init(void);
void PID_ResetRegulator(PID_Regulators *Regulator);
void PID_SetConstants(PID_Regulators *Regulator, float Kp, float Ki, float Kd);

void PID_Task(float SetPitch, float SetRoll, float SetYaw, u16 Throttle);
void PID_Roll(float Set, float Current);
void PID_Pitch(float Set, float Current);


#endif /* __PID_H__ */
