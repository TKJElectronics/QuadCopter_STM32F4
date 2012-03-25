/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "IMU_Sensors.h"

int16_t IMU_I2C_ReceiveBuffer[3];

/**
  * @brief  Initializes the GPIO pins used by the I2C port to communicate with the IMU Sensors.
  * @param  None
  * @retval None
  */
void IMU_GPIO_Config(void)
{
	  GPIO_InitTypeDef GPIO_InitStructure;

	  /* Enable IMU_I2C and IMU_I2C_GPIO_PORT & Alternate Function clocks */
	  RCC_APB1PeriphClockCmd(IMU_I2C_CLK, ENABLE);
	  RCC_AHB1PeriphClockCmd(IMU_I2C_SCL_GPIO_CLK | IMU_I2C_SDA_GPIO_CLK, ENABLE);
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	  /* Reset IMU_I2C IP */
	  RCC_APB1PeriphResetCmd(IMU_I2C_CLK, ENABLE);

	  /* Release reset signal of IMU_I2C IP */
	  RCC_APB1PeriphResetCmd(IMU_I2C_CLK, DISABLE);

	  /* IMU_I2C SCL and SDA pins configuration */
	  GPIO_InitStructure.GPIO_Pin = IMU_I2C_SCL_PIN;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	  GPIO_Init(IMU_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

	  GPIO_InitStructure.GPIO_Pin = IMU_I2C_SDA_PIN;
	  GPIO_Init(IMU_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);

	  GPIO_PinAFConfig(IMU_I2C_SCL_GPIO_PORT, IMU_I2C_SCL_SOURCE, IMU_I2C_SCL_AF);
	  GPIO_PinAFConfig(IMU_I2C_SDA_GPIO_PORT, IMU_I2C_SDA_SOURCE, IMU_I2C_SDA_AF);
}


void IMU_Config(void)
{
	IMU_GPIO_Config();
	IMU_I2C_Config();
}

void IMU_InitAll(void)
{
	IMU_Accelerometer_Init();
	IMU_Gyro_Init();
	IMU_Magnetometer_Init();
	Delay_Ms(2000);
}

void IMU_Accelerometer_Init(void)
{
	I2C_WriteDeviceRegister(ADXL345_ADDR, 0x2D, 0x08);	// Power register -> measurement mode
	I2C_WriteDeviceRegister(ADXL345_ADDR, 0x31, 0x08);	// Data format register -> set to full resolution, +-2g resolution
	I2C_WriteDeviceRegister(ADXL345_ADDR, 0x2C, 0x0B);	// Rate -> set output data rate to 200Hz
}

void IMU_Magnetometer_Init(void)
{
	/* First two lines maybe not necessary */
	//I2C_WriteDeviceRegister(HMC5883_ADDR, 0x00, 0x70);	// Mode register -> 8 average, 15Hz, normal measurement
	//I2C_WriteDeviceRegister(HMC5883_ADDR, 0x01, 0xA0);	// Mode register -> gain = 5
	I2C_WriteDeviceRegister(HMC5883_ADDR, 0x02, 0x00);		// Mode register -> continuous measurement mode
	I2C_WriteDeviceRegister(HMC5883_ADDR, 0x00, 0b00011000);// Mode register -> 75Hz, normal measurement
}

void IMU_Gyro_Init(void)
{
	I2C_WriteDeviceRegister(ITG3200_ADDR, PWR_MGM, 0x80);		// Chip reset
	I2C_WriteDeviceRegister(ITG3200_ADDR, SMPLRT_DIV, 0x23);	//  SMPLRT_DIV = 10 (50Hz)
	I2C_WriteDeviceRegister(ITG3200_ADDR, DLPF_FS, 0x1D);	// 0x1A (2000º/s)+DLPF_CFG=2(98Hz)  //0x1B (2000º/s)+DLPF_CFG=3(42Hz)  //0x1D (2000º/s)+DLPF_CFG=5(10Hz)
	//I2C_WriteDeviceRegister(ADXL345_ADDR, INT_CFG, 0x11);	// Interrupt on raw data ready
	I2C_WriteDeviceRegister(ITG3200_ADDR, PWR_MGM, 0x01);	// Oscillator : PLL with X gyro reference
}

/* Accelerometer */
s16 IMU_Accelerometer_X(void)
{
	s16 TempX;
	TempX = I2C_ReadInteger_LSBFirst(ADXL345_ADDR, 0x34); // X axis (internal sensor y axis)
	return TempX*SIGN_ACCELEROMETER_X;
}
s16 IMU_Accelerometer_Y(void)
{
	s16 TempY;
	TempY = I2C_ReadInteger_LSBFirst(ADXL345_ADDR, 0x32);  // Y axis (internal sensor x axis)
	return TempY*SIGN_ACCELEROMETER_Y;
}

s16 IMU_Accelerometer_Z(void)
{
	s16 TempZ;
	TempZ = I2C_ReadInteger_LSBFirst(ADXL345_ADDR, 0x36); // Z axis
	return TempZ*SIGN_ACCELEROMETER_Z;
}

void IMU_GetAccelerometer(void)
{
	I2C_Read3Integer_LSBFirst(ADXL345_ADDR, 0x32, IMU_I2C_ReceiveBuffer);

	IMU_accel[0] = SIGN_ACCELEROMETER_X*IMU_I2C_ReceiveBuffer[1];
	IMU_accel[1] = SIGN_ACCELEROMETER_Y*IMU_I2C_ReceiveBuffer[0];
	IMU_accel[2] = SIGN_ACCELEROMETER_Z*IMU_I2C_ReceiveBuffer[2];
}

s32 IMU_accelerometerSamples[3];
u16 IMU_accelerometerSampleCount = 0;
// Samples 1 accelerometer value
void IMU_SampleAccelerometer(void)
{
	I2C_Read3Integer_LSBFirst(ADXL345_ADDR, 0x32, IMU_I2C_ReceiveBuffer);

	IMU_accelerometerSamples[0] += SIGN_ACCELEROMETER_X*IMU_I2C_ReceiveBuffer[1];
	IMU_accelerometerSamples[1] += SIGN_ACCELEROMETER_Y*IMU_I2C_ReceiveBuffer[0];
	IMU_accelerometerSamples[2] += SIGN_ACCELEROMETER_Z*IMU_I2C_ReceiveBuffer[2];
	IMU_accelerometerSampleCount++;
}

// Returns IMU_accel average value from the sampled values
void IMU_EvaluateAccelerometer(void)
{
	IMU_accel[0] = ((IMU_accelerometerSamples[0] / IMU_accelerometerSampleCount) - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
	IMU_accel[1] = ((IMU_accelerometerSamples[1] / IMU_accelerometerSampleCount) - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
	IMU_accel[2] = ((IMU_accelerometerSamples[2] / IMU_accelerometerSampleCount) - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;

	// Reset samples
	IMU_accelerometerSamples[0] = 0;
	IMU_accelerometerSamples[1] = 0;
	IMU_accelerometerSamples[2] = 0;
	IMU_accelerometerSampleCount = 0;
}


/* Magnetometer */
s16 IMU_Magnetometer_X(void)
{
	s16 TempX;
	TempX = I2C_ReadInteger_MSBFirst(HMC5883_ADDR, 0x03); // X axis
	return TempX*SIGN_MAGNETOMETER_X;
}
s16 IMU_Magnetometer_Y(void)
{
	s16 TempY;
	TempY = I2C_ReadInteger_MSBFirst(HMC5883_ADDR, 0x07); // Y axis
	return TempY*SIGN_MAGNETOMETER_Y;
}
s16 IMU_Magnetometer_Z(void)
{
	s16 TempZ;
	TempZ = I2C_ReadInteger_MSBFirst(HMC5883_ADDR, 0x05); // Z axis
	return TempZ*SIGN_MAGNETOMETER_Z;
}

void IMU_GetMagnetometer(void)
{
	I2C_Read3Integer_MSBFirst(HMC5883_ADDR, 0x03, IMU_I2C_ReceiveBuffer);

	IMU_magnetom[0] = SIGN_MAGNETOMETER_X*IMU_I2C_ReceiveBuffer[0];
	IMU_magnetom[1] = SIGN_MAGNETOMETER_Y*IMU_I2C_ReceiveBuffer[2];
	IMU_magnetom[2] = SIGN_MAGNETOMETER_Z*IMU_I2C_ReceiveBuffer[1];
}

s32 IMU_magnetometerSamples[3];
u16 IMU_magnetometerSampleCount = 0;
// Samples 1 accelerometer value
void IMU_SampleMagnetometer(void)
{
	I2C_Read3Integer_MSBFirst(HMC5883_ADDR, 0x03, IMU_I2C_ReceiveBuffer);

	IMU_magnetometerSamples[0] += SIGN_MAGNETOMETER_X*IMU_I2C_ReceiveBuffer[0];
	IMU_magnetometerSamples[1] += SIGN_MAGNETOMETER_Y*IMU_I2C_ReceiveBuffer[2];
	IMU_magnetometerSamples[2] += SIGN_MAGNETOMETER_Z*IMU_I2C_ReceiveBuffer[1];
	IMU_magnetometerSampleCount++;
}

// Returns IMU_accel average value from the sampled values
void IMU_EvaluateMagnetometer(void)
{
	IMU_magnetom[0] = ((IMU_magnetometerSamples[0] / IMU_magnetometerSampleCount) - MAGN_X_OFFSET) * MAGN_X_SCALE;
	IMU_magnetom[1] = ((IMU_magnetometerSamples[1] / IMU_magnetometerSampleCount) - MAGN_Y_OFFSET) * MAGN_Y_SCALE;
	IMU_magnetom[2] = ((IMU_magnetometerSamples[2] / IMU_magnetometerSampleCount) - MAGN_Z_OFFSET) * MAGN_Z_SCALE;

	// Reset samples
	IMU_magnetometerSamples[0] = 0;
	IMU_magnetometerSamples[1] = 0;
	IMU_magnetometerSamples[2] = 0;
	IMU_magnetometerSampleCount = 0;
}


/* Gyroscope */
s16 IMU_Gyro_X(void)
{
	s16 TempX;
	TempX = I2C_ReadInteger_MSBFirst(ITG3200_ADDR, 0x1F); // X axis (internal Y-axis)
	return TempX*SIGN_GYRO_X;
}
s16 IMU_Gyro_Y(void)
{
	s16 TempY;
	TempY = I2C_ReadInteger_MSBFirst(ITG3200_ADDR, 0x1D); // Y axis (internal X-axis)
	return TempY*SIGN_GYRO_Y;
}
s16 IMU_Gyro_Z(void)
{
	s16 TempZ;
	TempZ = I2C_ReadInteger_MSBFirst(ITG3200_ADDR, 0x21); // Z axis
	return TempZ*SIGN_GYRO_Z;
}

void IMU_GetGyro(void)
{
	I2C_Read3Integer_MSBFirst(ITG3200_ADDR, 0x1D, IMU_I2C_ReceiveBuffer);

	IMU_gyro[0] = SIGN_GYRO_X*IMU_I2C_ReceiveBuffer[1];
	IMU_gyro[1] = SIGN_GYRO_Y*IMU_I2C_ReceiveBuffer[0];
	IMU_gyro[2] = SIGN_GYRO_Z*IMU_I2C_ReceiveBuffer[2];
}


// Samples one Gyro value, filters it and returns it in IMU_gyro
void IMU_MeasureFilteredGyro(void)
{
  s16 gyroADC[3];
  I2C_Read3Integer_MSBFirst(ITG3200_ADDR, 0x1D, IMU_I2C_ReceiveBuffer);


  gyroADC[0] = (SIGN_GYRO_X*IMU_I2C_ReceiveBuffer[1]) - GYRO_AVERAGE_OFFSET_X;
  gyroADC[1] = (SIGN_GYRO_Y*IMU_I2C_ReceiveBuffer[0]) - GYRO_AVERAGE_OFFSET_X;
  gyroADC[2] = (SIGN_GYRO_Z*IMU_I2C_ReceiveBuffer[2]) - GYRO_AVERAGE_OFFSET_X;

  IMU_gyro[0] = filterSmooth(GYRO_SCALED_RAD(gyroADC[0]), IMU_gyro[0], IMU_GYRO_SMOOTH_FACTOR);
  IMU_gyro[1] = filterSmooth(GYRO_SCALED_RAD(gyroADC[1]), IMU_gyro[1], IMU_GYRO_SMOOTH_FACTOR);
  IMU_gyro[2] = filterSmooth(GYRO_SCALED_RAD(gyroADC[2]), IMU_gyro[2], IMU_GYRO_SMOOTH_FACTOR);
}

s32 IMU_gyroSamples[3];
u16 IMU_gyroSampleCount = 0;
// Samples 1 gyro value
void IMU_SampleGyro(void)
{
	I2C_Read3Integer_MSBFirst(ITG3200_ADDR, 0x1D, IMU_I2C_ReceiveBuffer);

	IMU_gyroSamples[0] += SIGN_GYRO_X*IMU_I2C_ReceiveBuffer[1];
	IMU_gyroSamples[1] += SIGN_GYRO_Y*IMU_I2C_ReceiveBuffer[0];
	IMU_gyroSamples[2] += SIGN_GYRO_Z*IMU_I2C_ReceiveBuffer[2];
	IMU_gyroSampleCount++;
}

// Returns IMU_gyro average value from the sampled values
void IMU_EvaluateGyro(void)
{
	s16 gyroADC[3];
	gyroADC[0] = (IMU_gyroSamples[0] / IMU_gyroSampleCount) - GYRO_AVERAGE_OFFSET_X;
	gyroADC[1] = (IMU_gyroSamples[1] / IMU_gyroSampleCount) - GYRO_AVERAGE_OFFSET_Y;
	gyroADC[2] = (IMU_gyroSamples[2] / IMU_gyroSampleCount) - GYRO_AVERAGE_OFFSET_Z;

	IMU_gyro[0] = filterSmooth(gyroADC[0], IMU_gyro[0], IMU_GYRO_SMOOTH_FACTOR);
	IMU_gyro[1] = filterSmooth(gyroADC[1], IMU_gyro[1], IMU_GYRO_SMOOTH_FACTOR);
	IMU_gyro[2] = filterSmooth(gyroADC[2], IMU_gyro[2], IMU_GYRO_SMOOTH_FACTOR);

	// Reset samples
	IMU_gyroSamples[0] = 0;
	IMU_gyroSamples[1] = 0;
	IMU_gyroSamples[2] = 0;
	IMU_gyroSampleCount = 0;
}

void IMU_SampleAll(void)
{
	I2C_Read3Integer_LSBFirst(ADXL345_ADDR, 0x32, IMU_I2C_ReceiveBuffer);
	IMU_accelerometerSamples[0] += SIGN_ACCELEROMETER_X*IMU_I2C_ReceiveBuffer[1];
	IMU_accelerometerSamples[1] += SIGN_ACCELEROMETER_Y*IMU_I2C_ReceiveBuffer[0];
	IMU_accelerometerSamples[2] += SIGN_ACCELEROMETER_Z*IMU_I2C_ReceiveBuffer[2];

	I2C_Read3Integer_MSBFirst(ITG3200_ADDR, 0x1D, IMU_I2C_ReceiveBuffer);
	IMU_gyroSamples[0] += SIGN_GYRO_X*IMU_I2C_ReceiveBuffer[1];
	IMU_gyroSamples[1] += SIGN_GYRO_Y*IMU_I2C_ReceiveBuffer[0];
	IMU_gyroSamples[2] += SIGN_GYRO_Z*IMU_I2C_ReceiveBuffer[2];

	I2C_Read3Integer_MSBFirst(HMC5883_ADDR, 0x03, IMU_I2C_ReceiveBuffer);
	IMU_magnetometerSamples[0] += SIGN_MAGNETOMETER_X*IMU_I2C_ReceiveBuffer[0];
	IMU_magnetometerSamples[1] += SIGN_MAGNETOMETER_Y*IMU_I2C_ReceiveBuffer[2];
	IMU_magnetometerSamples[2] += SIGN_MAGNETOMETER_Z*IMU_I2C_ReceiveBuffer[1];

	IMU_accelerometerSampleCount++;
	IMU_gyroSampleCount++;
	IMU_magnetometerSampleCount++;
}


float IMU_accel[3];
float IMU_magnetom[3];
float IMU_gyro[3];

void IMU_ReadSensors(void)
{
	/*IMU_gyro[0] = (IMU_Gyro_X() - GyroOffsetX) * GYRO_SCALE;
	IMU_gyro[1] = (IMU_Gyro_Y() - GyroOffsetY) * GYRO_SCALE;
	IMU_gyro[2] = (IMU_Gyro_Z() - GyroOffsetZ) * GYRO_SCALE;
	IMU_accel[0] = (IMU_Accelerometer_X() - AccelerometerOffsetX) * ACCELEROMETER_SCALE;
	IMU_accel[1] = (IMU_Accelerometer_Y() - AccelerometerOffsetY) * ACCELEROMETER_SCALE;
	IMU_accel[2] = (IMU_Accelerometer_Z() - AccelerometerOffsetZ) * ACCELEROMETER_SCALE;
	IMU_magnetom[0] = IMU_Magnetometer_X();
	IMU_magnetom[1] = IMU_Magnetometer_Y();
	IMU_magnetom[2] = IMU_Magnetometer_Z();*/
	/*IMU_gyro[0] = IMU_Gyro_X();
	IMU_gyro[1] = IMU_Gyro_Y();
	IMU_gyro[2] = IMU_Gyro_Z();
	IMU_accel[0] = IMU_Accelerometer_X();
	IMU_accel[1] = IMU_Accelerometer_Y();
	IMU_accel[2] = IMU_Accelerometer_Z();
	IMU_magnetom[0] = IMU_Magnetometer_X();
	IMU_magnetom[1] = IMU_Magnetometer_Y();
	IMU_magnetom[2] = IMU_Magnetometer_Z();*/

	IMU_GetGyro();
	IMU_GetAccelerometer();
	IMU_GetMagnetometer();
}

// Apply calibration to raw sensor readings
void IMU_compensate_sensor_errors(void) {
    // Compensate accelerometer error
	/*IMU_accel[0] = (IMU_accel[0] - AccelerometerOffsetX) * ACCEL_X_SCALE;
	IMU_accel[1] = (IMU_accel[1] - AccelerometerOffsetY) * ACCEL_Y_SCALE;
	IMU_accel[2] = (IMU_accel[2] - AccelerometerOffsetZ) * ACCEL_Z_SCALE;

	// Compensate magnetometer error
	IMU_magnetom[0] = (IMU_magnetom[0] - MAGN_X_OFFSET) * MAGN_X_SCALE;
	IMU_magnetom[1] = (IMU_magnetom[1] - MAGN_Y_OFFSET) * MAGN_Y_SCALE;
	IMU_magnetom[2] = (IMU_magnetom[2] - MAGN_Z_OFFSET) * MAGN_Z_SCALE;

	// Compensate gyroscope error
	IMU_gyro[0] -= GyroOffsetX;
	IMU_gyro[1] -= GyroOffsetX;
	IMU_gyro[2] -= GyroOffsetX;*/


    IMU_accel[0] = (IMU_accel[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
    IMU_accel[1] = (IMU_accel[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
    IMU_accel[2] = (IMU_accel[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;

    // Compensate magnetometer error
    IMU_magnetom[0] = (IMU_magnetom[0] - MAGN_X_OFFSET) * MAGN_X_SCALE;
    IMU_magnetom[1] = (IMU_magnetom[1] - MAGN_Y_OFFSET) * MAGN_Y_SCALE;
    IMU_magnetom[2] = (IMU_magnetom[2] - MAGN_Z_OFFSET) * MAGN_Z_SCALE;

    // Compensate gyroscope error
    IMU_gyro[0] -= GYRO_AVERAGE_OFFSET_X;
    IMU_gyro[1] -= GYRO_AVERAGE_OFFSET_Y;
    IMU_gyro[2] -= GYRO_AVERAGE_OFFSET_Z;
}
