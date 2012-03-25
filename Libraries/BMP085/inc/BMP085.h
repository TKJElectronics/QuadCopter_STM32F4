#ifndef __BMP085_H__
#define __BMP085_H__

#include "stm32f4xx.h"
#include "Math.h"
#include "IMU_I2C.h"
#include "Delay.h"
#include "Serial.h"

#define BMP085_DEBUG 0

#define BMP085_ADDR 	(0x77 << 1) 	// Barometric I2C Address

#define BMP085_ULTRALOWPOWER 0
#define BMP085_STANDARD      1
#define BMP085_HIGHRES       2
#define BMP085_ULTRAHIGHRES  3
#define BMP085_CAL_AC1           0xAA  // R   Calibration data (16 bits)
#define BMP085_CAL_AC2           0xAC  // R   Calibration data (16 bits)
#define BMP085_CAL_AC3           0xAE  // R   Calibration data (16 bits)
#define BMP085_CAL_AC4           0xB0  // R   Calibration data (16 bits)
#define BMP085_CAL_AC5           0xB2  // R   Calibration data (16 bits)
#define BMP085_CAL_AC6           0xB4  // R   Calibration data (16 bits)
#define BMP085_CAL_B1            0xB6  // R   Calibration data (16 bits)
#define BMP085_CAL_B2            0xB8  // R   Calibration data (16 bits)
#define BMP085_CAL_MB            0xBA  // R   Calibration data (16 bits)
#define BMP085_CAL_MC            0xBC  // R   Calibration data (16 bits)
#define BMP085_CAL_MD            0xBE  // R   Calibration data (16 bits)

#define BMP085_CONTROL           0xF4
#define BMP085_TEMPDATA          0xF6
#define BMP085_PRESSUREDATA      0xF6
#define BMP085_READTEMPCMD          0x2E
#define BMP085_READPRESSURECMD            0x34

#define BMP085_STD_SEALEVELPRESSURE 102460


void BMP085_Init(uint8_t mode);
uint16_t BMP085_readRawTemperature(void);
uint32_t BMP085_readRawPressure(void);
int32_t BMP085_readPressure(void);
float BMP085_readTemperature(void);
float BMP085_readAltitude(float sealevelPressure);


#endif /* __BMP085_H__ */
