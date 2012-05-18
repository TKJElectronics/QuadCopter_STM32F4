#ifndef __IMU_DCM_H__
#define __IMU_DCM_H__

#include "stm32f4xx.h"
#include "IMU_Vectors.h"
#include "IMU_Sensors.h"
#include "IMU_Calibration.h"
#include <Math.h>
#include <stdlib.h>



// Sensor calibration scale and offset values
#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_SCALE (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))
/*#define ACCEL_X_OFFSET AccelerometerOffsetX
#define ACCEL_Y_OFFSET AccelerometerOffsetY
#define ACCEL_Z_OFFSET AccelerometerOffsetZ
#define ACCEL_X_SCALE 1
#define ACCEL_Y_SCALE 1
#define ACCEL_Z_SCALE 1*/

#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))


// Gain for gyroscope (ITG-3200)
#define GYRO_GAIN 0.06957 // Same gain on all axes
#define GYRO_SCALED_RAD(x) (x * TO_RAD(GYRO_GAIN)) // Calculate the scaled gyro readings in radians per second

// DCM parameters
#define Kp_ROLLPITCH 0.05f		// 0.02
#define Ki_ROLLPITCH 0.00002f 	// 0.00002
#define Kp_YAW 1.2f 			// 1.2
#define Ki_YAW 0.00002f			// 0.00002

// Stuff
#define GRAVITY 256.0f // "1G reference" used for DCM filter and accelerometer calibration
#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi

extern const float PI;
#define RAD_TO_DEG					57.295779513f
#define DEG_TO_RAD					0.017453293f

/* Accelerometer definitions */
/* +-2g resolution, 10-bit values */
#define ACCELEROMETER_SENSITIVITY	256			// 256 LSB/g
#define ACCELEROMETER_SCALE			0.0039f		// 3.9 mg/LSB

/* Gyroscope definitions */
/* */
#define GYRO_SENSITIVITY			14.375f					// 14.375 LBS/(º/s)
#define GYRO_SCALE					0.0695652173913043f		// 0.695 (º/s)/LSB

/* Magnetometer definitions */
/* */
#define MAGNETOMETER_SENSITIVITY


extern float IMU_Yaw_Offset;
extern float IMU_Pitch_Offset;
extern float IMU_Roll_Offset;

// Euler angles
extern float IMU_Yaw, IMU_Yaw_Final;
extern float IMU_Pitch, IMU_Pitch_Final;
extern float IMU_Roll, IMU_Roll_Final;
extern float MAG_Heading;
extern float Accel_Vector[3];
extern float Omega_Vector[3]; // Corrected Gyro_Vector data - can be used in PID
extern float Gyro_Vector[3];

extern float G_Dt; // Integration time for DCM algorithm

extern float Omega_Vector[3]; // Corrected Gyro_Vector data




void IMU_Normalize(void);
void IMU_Drift_correction(void);
void IMU_Matrix_update(void);
void IMU_Euler_angles(void);
void IMU_Compass_Heading(void);


#endif /* __IMU_KALMAN_H__ */
