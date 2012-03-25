/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "IMU_Calibration.h"

/* Offset/Calibration routines */
s32 AccelerometerOffsetX, AccelerometerOffsetY, AccelerometerOffsetZ;
s32 GyroOffsetX, GyroOffsetY, GyroOffsetZ;
s32 MagnetometerOffsetX, MagnetometerOffsetY, MagnetometerOffsetZ;
void IMU_InitCalibrateAll(void)
{
	AccelerometerOffsetX = 0;
	AccelerometerOffsetY = 0;
	AccelerometerOffsetZ = 0;

	MagnetometerOffsetX = 0;
	MagnetometerOffsetY = 0;
	MagnetometerOffsetZ = 0;

	GyroOffsetX = 0;
	GyroOffsetY = 0;
	GyroOffsetZ = 0;

	for (u8 i = 0; i < 100; i++)
	{
		AccelerometerOffsetX += IMU_Accelerometer_X();
		AccelerometerOffsetY += IMU_Accelerometer_Y();
#ifdef UPSIDE_DOWN_CONFIGURATION
		AccelerometerOffsetZ -= IMU_Accelerometer_Z();
#else
		AccelerometerOffsetZ += IMU_Accelerometer_Z();
#endif

		GyroOffsetX += IMU_Gyro_X();
		GyroOffsetY += IMU_Gyro_Y();
		GyroOffsetZ += IMU_Gyro_Z();

		MagnetometerOffsetX += IMU_Magnetometer_X();
		MagnetometerOffsetY += IMU_Magnetometer_Y();
#ifdef UPSIDE_DOWN_CONFIGURATION
		MagnetometerOffsetZ -= IMU_Magnetometer_Z();
#else
		MagnetometerOffsetZ += IMU_Magnetometer_Z();
#endif

		Delay_Ms(50);
	}

	AccelerometerOffsetX /= 100;
	AccelerometerOffsetY /= 100;
	AccelerometerOffsetZ /= 100;
	/* The accelerometer is hold vertically when being calibrated, therefor Z-axis is 1g */
	AccelerometerOffsetZ -= ACCELEROMETER_SENSITIVITY; // Substract 1g from the offset

	GyroOffsetX /= 100;
	GyroOffsetY /= 100;
	GyroOffsetZ /= 100;

	MagnetometerOffsetX /= 100;
	MagnetometerOffsetY /= 100;
	MagnetometerOffsetZ /= 100;

	// Try with this instead: findMedianFloat();
}


void IMU_AfterInitCalibration(void)
{
	u16 i;

	IMU_Pitch_Offset = 0;
	IMU_Roll_Offset = 0;
	IMU_Yaw_Offset = 0;

	/* First let filter stabilize - for 2 seconds */
	for (i = 0; i < 200; i++)
	{
		IMU_SampleAll();
		Delay_Ms(1);
		IMU_SampleAll();
		Delay_Ms(1);
		IMU_SampleAll();
		Delay_Ms(1);
		IMU_SampleAll();
		Delay_Ms(1);
		IMU_Task();
		Delay_Ms(6);
	}

	/* Then take 100 measurements */
	for (i = 0; i < 100; i++)
	{
		IMU_SampleAll();
		Delay_Ms(1);
		IMU_SampleAll();
		Delay_Ms(1);
		IMU_SampleAll();
		Delay_Ms(1);
		IMU_SampleAll();
		Delay_Ms(1);
		IMU_Task();

		IMU_Pitch_Offset += IMU_Pitch;
		IMU_Roll_Offset += IMU_Roll;
		IMU_Yaw_Offset += IMU_Yaw;

		Delay_Ms(6);
	}

	IMU_Pitch_Offset /= 100;
	IMU_Roll_Offset /= 100;
	IMU_Yaw_Offset /= 100;


}

void IMU_SetPredefinedCalibration(void)
{
	u16 i;

	/* First let filter stabilize - for 2 seconds */
	for (i = 0; i < 100; i++)
	{
		IMU_SampleAll();
		Delay_Ms(1);
		IMU_SampleAll();
		Delay_Ms(1);
		IMU_SampleAll();
		Delay_Ms(1);
		IMU_SampleAll();
		Delay_Ms(1);
		IMU_Task();
		Delay_Ms(16);
	}

	IMU_Pitch_Offset = IMU_PREDEFINED_PITCH_OFFSET;
	IMU_Roll_Offset = IMU_PREDEFINED_ROLL_OFFSET;
	IMU_Yaw_Offset = IMU_PREDEFINED_YAW_OFFSET;
}

bool CalibrationPaused;
u8 CalibrationStep;
s16 accel_min[3];
s16 accel_max[3];
s16 magnetom_min[3];
s16 magnetom_max[3];
float gyro_average[3];
u32 gyro_num_samples;

void IMU_SerialMaxValuesCalibration(void)
{
	CalibrationStep = 1;
	CalibrationPaused = false;

	while (CalibrationStep > 0)
	{
		if (CalibrationStep == 1 && !CalibrationPaused)
		{
			IMU_GetAccelerometer();

			// Output MIN/MAX values
			Serial_Print("accel x,y,z (min/max) = ");
			for (int i = 0; i < 3; i++) {
				if (IMU_accel[i] < accel_min[i]) accel_min[i] = IMU_accel[i];
				if (IMU_accel[i] > accel_max[i]) accel_max[i] = IMU_accel[i];
				sprintf(GlobalStringBuffer, "%d/%d", accel_min[i], accel_max[i]);
				Serial_Print(GlobalStringBuffer);
				if (i < 2) Serial_Print("  ");
				else Serial_Println("");
			}
		} else if (CalibrationStep == 2 && !CalibrationPaused) {  // Magnetometer
			IMU_GetMagnetometer();

		    // Output MIN/MAX values
		    Serial_Print("magn x,y,z (min/max) = ");
		    for (int i = 0; i < 3; i++) {
		    	if (IMU_magnetom[i] < magnetom_min[i]) magnetom_min[i] = IMU_magnetom[i];
		    	if (IMU_magnetom[i] > magnetom_max[i]) magnetom_max[i] = IMU_magnetom[i];
		    	sprintf(GlobalStringBuffer, "%d/%d", magnetom_min[i], magnetom_max[i]);
		    	Serial_Print(GlobalStringBuffer);
		    	if (i < 2) Serial_Print("  ");
		    	else Serial_Println("");
		    }
		} else if (CalibrationStep == 3 && !CalibrationPaused) {  // Gyro
			IMU_GetGyro();

				// Average gyro values
				    for (int i = 0; i < 3; i++)
				      gyro_average[i] += IMU_gyro[i];
				    gyro_num_samples++;

				    // Output current and averaged gyroscope values
				    Serial_Print("gyro x,y,z (current/average) = ");
				    for (int i = 0; i < 3; i++) {
				    	sprintf(GlobalStringBuffer, "%.0f/%f", IMU_gyro[i], (gyro_average[i] / (float)gyro_num_samples));
				    	Serial_Print(GlobalStringBuffer);
				    	if (i < 2) Serial_Print("  ");
				    	else Serial_Println("");
				    }
		  }


		if (Serial_RxCounter > 0)
		{
			if (Serial_RxBuffer[0] == 'r') // Reset current
			{
				if (CalibrationStep == 1)
				{
					accel_min[0] = 0;
					accel_min[1] = 0;
					accel_min[2] = 0;
					accel_max[0] = 0;
					accel_max[1] = 0;
					accel_max[2] = 0;
				} else if (CalibrationStep == 2) {
					magnetom_min[0] = 0;
					magnetom_min[1] = 0;
					magnetom_min[2] = 0;
					magnetom_max[0] = 0;
					magnetom_max[1] = 0;
					magnetom_max[2] = 0;
				} else if (CalibrationStep == 3) {
					gyro_average[0] = 0;
					gyro_average[1] = 0;
					gyro_average[2] = 0;
					gyro_num_samples = 0;
				}
				Serial_Println("Data has been reset!");
				Serial_RxCounter = 0;
			} else if (Serial_RxBuffer[0] == 's') { // Start/Stop (pause)
				CalibrationPaused = !CalibrationPaused;

				Serial_RxCounter = 0;
			} else if (Serial_RxBuffer[0] == 'n') { // Next
				if (CalibrationStep < 3) CalibrationStep++;
				else CalibrationStep = 1;

				Serial_RxCounter = 0;
			} else if (Serial_RxBuffer[0] == 'f') { // Finish
				CalibrationStep = 0;
				Serial_RxCounter = 0;
			} else {
				Serial_RxCounter = 0;
			}
		}
		Delay_Ms(50);
	}

	Serial_RxCounter = 0;
}
