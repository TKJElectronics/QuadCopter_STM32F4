#ifndef __IMU_CALIBRATION_H__
#define __IMU_CALIBRATION_H__

#include "stm32f4xx.h"
#include "bool.h"

#include "Serial.h"

#include "IMU.h"
#include "IMU_Sensors.h"
#include "IMU_DCM.h"


// SENSOR CALIBRATION
/*****************************************************************/
// How to calibrate? Read the tutorial at http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs
// Put MIN/MAX and OFFSET readings for your board here!
// Accelerometer
// "accel x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define ACCEL_X_MIN ((float) -266)
#define ACCEL_X_MAX ((float) 268)
#define ACCEL_Y_MIN ((float) -262)
#define ACCEL_Y_MAX ((float) 268)
#define ACCEL_Z_MIN ((float) -328)
#define ACCEL_Z_MAX ((float) 229)

// Magnetometer
// "magn x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define MAGN_X_MIN ((float) -507) //-653
#define MAGN_X_MAX ((float) 456)  //514
#define MAGN_Y_MIN ((float) -476) //-567
#define MAGN_Y_MAX ((float) 463)  //461
#define MAGN_Z_MIN ((float) -487) //-401
#define MAGN_Z_MAX ((float) 417)  //763

// Gyroscope
// "gyro x,y,z (current/average) = .../OFFSET_X  .../OFFSET_Y  .../OFFSET_Z
#define GYRO_AVERAGE_OFFSET_X ((float) -30.997362)
#define GYRO_AVERAGE_OFFSET_Y ((float) 60.100266)
#define GYRO_AVERAGE_OFFSET_Z ((float) -3.649076)
/*#define GYRO_AVERAGE_OFFSET_X GyroOffsetX
#define GYRO_AVERAGE_OFFSET_Y GyroOffsetY
#define GYRO_AVERAGE_OFFSET_Z GyroOffsetZ*/


#define IMU_PREDEFINED_PITCH_OFFSET		0.0f
#define IMU_PREDEFINED_ROLL_OFFSET		0.0f
#define IMU_PREDEFINED_YAW_OFFSET		0.0f


extern s32 AccelerometerOffsetX, AccelerometerOffsetY, AccelerometerOffsetZ;
extern s32 GyroOffsetX, GyroOffsetY, GyroOffsetZ;
extern s32 MagnetometerOffsetX, MagnetometerOffsetY, MagnetometerOffsetZ;


void IMU_InitCalibrateAll(void);
void IMU_AfterInitCalibration(void);
void IMU_SetPredefinedCalibration(void);
void IMU_SerialMaxValuesCalibration(void);

#endif /* __IMU_CALIBRATION_H__ */
