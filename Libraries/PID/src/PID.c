/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "PID.h"

QuadCopter_Settings QuadCopter;

PID_Regulators PID_PitchVar;
PID_Regulators PID_RollVar;
PID_Regulators PID_YawVar;

bool PID_Enabled = false;
float PID_RC;


void PID_Init(void)
{
	PID_SetConstants(&PID_PitchVar, PID_Pitch_Kp_Init, PID_Pitch_Ki_Init, PID_Pitch_Kd_Init);
	PID_SetConstants(&PID_RollVar, PID_Roll_Kp_Init, PID_Roll_Ki_Init, PID_Roll_Kd_Init);
	PID_SetConstants(&PID_YawVar, PID_Yaw_Kp_Init, PID_Yaw_Ki_Init, PID_Yaw_Kd_Init);

	PID_ResetRegulator(&PID_PitchVar);
	PID_ResetRegulator(&PID_RollVar);
	PID_ResetRegulator(&PID_YawVar);

	PID_Enabled = false;
}

void PID_ResetRegulator(PID_Regulators *Regulator)
{
	(*Regulator).Proportional = 0;
	(*Regulator).Integral = 0;
	(*Regulator).Derivative = 0;

	(*Regulator).Control = 0;

	(*Regulator).TimeStamp = Micros();
}

void PID_SetConstants(PID_Regulators *Regulator, float Kp, float Ki, float Kd)
{
	(*Regulator).Kp = Kp;
	(*Regulator).Ki = Ki;
	(*Regulator).Kd = Kd;
}

void PID_Task(float SetPitch, float SetRoll, float SetYaw, u16 Throttle)
{
	if (PID_Enabled)
	{
		//PID_Pitch(TO_DEG(SetPitch), TO_DEG(IMU_Pitch_Final));
		PID_Roll(TO_DEG(SetRoll), TO_DEG(IMU_Roll_Final));
		//PID_Yaw(TO_DEG(SetYaw), TO_DEG(IMU_Yaw_Final));

		ESC_Motor_Values[ESC_RIGHT] = ESC_THROTTLE_MIN + Throttle - PID_RollVar.Control + PID_YawVar.Control;
		ESC_Motor_Values[ESC_LEFT] = ESC_THROTTLE_MIN + Throttle + PID_RollVar.Control + PID_YawVar.Control;
		ESC_Motor_Values[ESC_FRONT] = ESC_THROTTLE_MIN + Throttle + PID_PitchVar.Control - PID_YawVar.Control;
		ESC_Motor_Values[ESC_BACK] = ESC_THROTTLE_MIN + Throttle - PID_PitchVar.Control - PID_YawVar.Control;

		ESC_UpdateAll();
	}
}

void PID_Roll(float Set, float Current)
{
	PID_RollVar.TimeStamp_Old = PID_RollVar.TimeStamp;
	PID_RollVar.TimeStamp = Micros();

    if (PID_RollVar.TimeStamp > PID_RollVar.TimeStamp_Old)
    	PID_RollVar.Dt = (float)(PID_RollVar.TimeStamp - PID_RollVar.TimeStamp_Old) / 1000000.0f; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else PID_RollVar.Dt = 0;

    PID_RollVar.Error_Before = PID_RollVar.Error_Current;
    PID_RollVar.Error_Current = Set - Current;

    PID_RollVar.Proportional = PID_RollVar.Kp * PID_RollVar.Error_Current;
    PID_RollVar.Integral += PID_RollVar.Ki * PID_RollVar.Error_Current * PID_RollVar.Dt;
    PID_RollVar.Derivative = PID_RollVar.Kd * ((PID_RollVar.Error_Current - PID_RollVar.Error_Before) / PID_RollVar.Dt);

    PID_RollVar.Control = PID_RollVar.Proportional + PID_RollVar.Integral + PID_RollVar.Derivative;

}

void PID_Pitch(float Set, float Current)
{
	PID_PitchVar.TimeStamp_Old = PID_PitchVar.TimeStamp;
	PID_PitchVar.TimeStamp = Micros();

    if (PID_PitchVar.TimeStamp > PID_PitchVar.TimeStamp_Old)
    	PID_PitchVar.Dt = (float)(PID_PitchVar.TimeStamp - PID_PitchVar.TimeStamp_Old) / 1000000.0f; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else PID_PitchVar.Dt = 0;

    PID_PitchVar.Error_Before = PID_PitchVar.Error_Current;
    PID_PitchVar.Error_Current = Set - Current;
    //PID_PitchVar.Error_Current = low_pass_filter(Set - Current, PID_PitchVar.Error_Before, 25.0); // Try this filtered value

    PID_PitchVar.Proportional = PID_PitchVar.Kp * PID_PitchVar.Error_Current;
    PID_PitchVar.Integral += PID_PitchVar.Ki * PID_PitchVar.Error_Current * PID_PitchVar.Dt;

    /*if (PID_PitchVar.Dt > 0)
    {
    	PID_PitchVar.Derivative = (PID_PitchVar.Error_Current - PID_PitchVar.Error_Before) / PID_PitchVar.Dt;

    	// discrete low pass filter, cuts out the
    	// high frequency noise that can drive the controller crazy
    	PID_RC = 1/(2*PI*PID_fCut);
    	PID_PitchVar.Derivative = PID_PitchVar.Derivative_Before + (PID_PitchVar.Dt / (PID_RC + PID_PitchVar.Dt)) * (PID_PitchVar.Derivative - PID_PitchVar.Derivative_Before);
    	PID_PitchVar.Derivative_Before = PID_PitchVar.Derivative;

    	// Multiply with the constant
    	PID_PitchVar.Derivative *= PID_PitchVar.Kd;
    }*/
    PID_PitchVar.Derivative = PID_PitchVar.Kd * ((PID_PitchVar.Error_Current - PID_PitchVar.Error_Before) / PID_PitchVar.Dt);

    PID_PitchVar.Control = PID_PitchVar.Proportional + PID_PitchVar.Integral + PID_PitchVar.Derivative;

}


