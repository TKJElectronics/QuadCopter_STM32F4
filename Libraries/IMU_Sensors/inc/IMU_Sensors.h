#ifndef __IMU_SENSORS_H__
#define __IMU_SENSORS_H__

#include "stm32f4xx.h"
#include "IMU_I2C.h"
#include "IMU_DCM.h"
#include "Delay.h"


// Sensor device addresses
#define ADXL345_ADDR 	(0x53 << 1) 	// Accelerometer I2C Address
#define ITG3200_ADDR 	(0x68 << 1) 	// Gyro I2C Address
#define HMC5883_ADDR 	(0x1E << 1) 	// Magnetometer I2C Address

// ITG3200 Register map (from datasheet)
#define WHO_AM_I        0x00
#define	SMPLRT_DIV		0x15
#define DLPF_FS         0x16
#define INT_CFG         0x17
#define INT_STATUS		0x1A
#define	GYROX_H	        0x1D
#define PWR_MGM	        0x3E

#define SIGN_ACCELEROMETER_X	1
#define SIGN_ACCELEROMETER_Y	1
#define SIGN_ACCELEROMETER_Z	1

#define SIGN_GYRO_X				-1
#define SIGN_GYRO_Y				-1
#define SIGN_GYRO_Z				-1

#define SIGN_MAGNETOMETER_X		1
#define SIGN_MAGNETOMETER_Y		-1
#define SIGN_MAGNETOMETER_Z		-1

#define UPSIDE_DOWN_CONFIGURATION		1 // z-axis inverted configuration - 1=enabled, 0=disabled

#define IMU_GYRO_SMOOTH_FACTOR	0.8f

extern float IMU_accel[3];
extern float IMU_magnetom[3];
extern float IMU_gyro[3];

extern int16_t IMU_I2C_ReceiveBuffer[3];

void IMU_GPIO_Config(void);
void IMU_Config(void);

void IMU_InitAll(void);
void IMU_Accelerometer_Init(void);
void IMU_Magnetometer_Init(void);
void IMU_Gyro_Init(void);

/* Two methods of grabbing sensor data
 * 1. IMU_SENSOR_X/Y/Z   - this takes time, because of it reading the sensor value individually
 * 2. IMU_GetSENSOR      - this is a lot faster because it grabs all 3 axis values for a sensor
 *
 * For grabbing all 9DOF sensor data it requires the following amount of time with the two methods
 * 1. 5-9 ms    (5000-9000 us)
 * 2. ~850 us
 */

s16 IMU_Accelerometer_X(void);
s16 IMU_Accelerometer_Y(void);
s16 IMU_Accelerometer_Z(void);
void IMU_GetAccelerometer(void);
void IMU_SampleAccelerometer(void);
void IMU_EvaluateAccelerometer(void);

s16 IMU_Magnetometer_X(void);
s16 IMU_Magnetometer_Y(void);
s16 IMU_Magnetometer_Z(void);
void IMU_GetMagnetometer(void);
void IMU_SampleMagnetometer(void);
void IMU_EvaluateMagnetometer(void);

s16 IMU_Gyro_X(void);
s16 IMU_Gyro_Y(void);
s16 IMU_Gyro_Z(void);
void IMU_GetGyro(void);
void IMU_MeasureFilteredGyro(void);
void IMU_SampleGyro(void);
void IMU_EvaluateGyro(void);

void IMU_SampleAll(void);

void IMU_ReadSensors(void);
void IMU_compensate_sensor_errors(void);
void IMU_reset_sensor_fusion(void);



#endif /* __IMU_SENSORS_H__ */
