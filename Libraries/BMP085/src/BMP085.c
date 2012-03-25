/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "BMP085.h"

  uint8_t oversampling;

  int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
  uint16_t ac4, ac5, ac6;

void BMP085_Init(uint8_t mode) {
  if (mode > BMP085_ULTRAHIGHRES)
    mode = BMP085_ULTRAHIGHRES;
  oversampling = mode;

  /* read calibration data */
  ac1 = I2C_ReadInteger_MSBFirst(BMP085_ADDR, BMP085_CAL_AC1);
  Delay_us(100);
  ac2 = I2C_ReadInteger_MSBFirst(BMP085_ADDR, BMP085_CAL_AC2);
  Delay_us(100);
  ac3 = I2C_ReadInteger_MSBFirst(BMP085_ADDR, BMP085_CAL_AC3);
  Delay_us(100);
  ac4 = I2C_ReadInteger_MSBFirst(BMP085_ADDR, BMP085_CAL_AC4);
  Delay_us(100);
  ac5 = I2C_ReadInteger_MSBFirst(BMP085_ADDR, BMP085_CAL_AC5);
  Delay_us(100);
  ac6 = I2C_ReadInteger_MSBFirst(BMP085_ADDR, BMP085_CAL_AC6);
  Delay_us(100);

  b1 = I2C_ReadInteger_MSBFirst(BMP085_ADDR, BMP085_CAL_B1);
  Delay_us(100);
  b2 = I2C_ReadInteger_MSBFirst(BMP085_ADDR, BMP085_CAL_B2);
  Delay_us(100);

  mb = I2C_ReadInteger_MSBFirst(BMP085_ADDR, BMP085_CAL_MB);
  Delay_us(100);
  mc = I2C_ReadInteger_MSBFirst(BMP085_ADDR, BMP085_CAL_MC);
  Delay_us(100);
  md = I2C_ReadInteger_MSBFirst(BMP085_ADDR, BMP085_CAL_MD);
  Delay_us(100);
#if (BMP085_DEBUG == 1)

  sprintf(GlobalStringBuffer, "ac1 = %d", ac1); Serial_Println(GlobalStringBuffer);
  sprintf(GlobalStringBuffer, "ac2 = %d", ac2); Serial_Println(GlobalStringBuffer);
  sprintf(GlobalStringBuffer, "ac3 = %d", ac3); Serial_Println(GlobalStringBuffer);
  sprintf(GlobalStringBuffer, "ac4 = %d", ac4); Serial_Println(GlobalStringBuffer);
  sprintf(GlobalStringBuffer, "ac5 = %d", ac5); Serial_Println(GlobalStringBuffer);
  sprintf(GlobalStringBuffer, "ac6 = %d", ac6); Serial_Println(GlobalStringBuffer);

  sprintf(GlobalStringBuffer, "b1 = %d", b1); Serial_Println(GlobalStringBuffer);
  sprintf(GlobalStringBuffer, "b2 = %d", b2); Serial_Println(GlobalStringBuffer);

  sprintf(GlobalStringBuffer, "mb = %d", mb); Serial_Println(GlobalStringBuffer);
  sprintf(GlobalStringBuffer, "mc = %d", mc); Serial_Println(GlobalStringBuffer);
  sprintf(GlobalStringBuffer, "md = %d", md); Serial_Println(GlobalStringBuffer);
#endif
}

uint16_t BMP085_readRawTemperature(void) {
  I2C_WriteDeviceRegister(BMP085_ADDR, BMP085_CONTROL, BMP085_READTEMPCMD);
  Delay_Ms(5);
  return (uint16_t)(I2C_ReadInteger_MSBFirst(BMP085_ADDR, BMP085_TEMPDATA));
}

uint32_t BMP085_readRawPressure(void) {
  uint8_t XLSB;
  uint32_t raw;

  I2C_WriteDeviceRegister(BMP085_ADDR, BMP085_CONTROL, BMP085_READPRESSURECMD + (oversampling << 6));

  if (oversampling == BMP085_ULTRALOWPOWER)
    Delay_Ms(5);
  else if (oversampling == BMP085_STANDARD)
    Delay_Ms(8);
  else if (oversampling == BMP085_HIGHRES)
    Delay_Ms(14);
  else
    Delay_Ms(26);

  raw = (uint16_t)(I2C_ReadInteger_MSBFirst(BMP085_ADDR, BMP085_PRESSUREDATA));
  XLSB = I2C_ReadDeviceRegister(BMP085_ADDR, BMP085_PRESSUREDATA+2);

  raw = ((raw << 8) | ((uint32_t)XLSB)) >> (8 - oversampling);

#if BMP085_DEBUG == 1
  sprintf(GlobalStringBuffer, "Raw pressure: %ld", raw); Serial_Println(GlobalStringBuffer);
#endif
  return raw;
}


int32_t BMP085_readPressure(void) {
  int32_t UT, UP, B3, B5, B6, X1, X2, X3, p;
  uint32_t B4, B7;

  UT = BMP085_readRawTemperature();
  UP = BMP085_readRawPressure();

#if BMP085_DEBUG == 1
  sprintf(GlobalStringBuffer, "UT = %ld", UT); Serial_Println(GlobalStringBuffer);
  sprintf(GlobalStringBuffer, "UP = %ld", UP); Serial_Println(GlobalStringBuffer);
#endif

#if BMP085_DEBUG == 2
  // use datasheet numbers!
  UT = 27898;
  UP = 23843;
  ac6 = 23153;
  ac5 = 32757;
  mc = -8711;
  md = 2868;
  b1 = 6190;
  b2 = 4;
  ac3 = -14383;
  ac2 = -72;
  ac1 = 408;
  ac4 = 32741;
  oversampling = 0;
#endif

  // do temperature calculations
  X1 = ((UT - (int32_t)ac6) * (int32_t)ac5) >> 15;
  X2 = ((int32_t)mc << 11) - (X1 + md)/2;     // round up
  X2 /= (X1 + md);
  B5 = X1 + X2;

#if BMP085_DEBUG == 1
  sprintf(GlobalStringBuffer, "X1 = %ld", X1); Serial_Println(GlobalStringBuffer);
  sprintf(GlobalStringBuffer, "X2 = %ld", X2); Serial_Println(GlobalStringBuffer);
  sprintf(GlobalStringBuffer, "B5 = %ld", B5); Serial_Println(GlobalStringBuffer);
#endif

  // do pressure calcs
  B6 = B5 - 4000;
  X1 = ((int32_t)b2 * ( (B6 * B6)>>12 )) >> 11;
  X2 = ((int32_t)ac2 * B6) >> 11;
  X3 = X1 + X2;
  B3 = ((((int32_t)ac1*4 + X3) << oversampling) + 2) / 4;

#if BMP085_DEBUG == 1
  sprintf(GlobalStringBuffer, "B6 = %ld", B6); Serial_Println(GlobalStringBuffer);
  sprintf(GlobalStringBuffer, "X1 = %ld", X1); Serial_Println(GlobalStringBuffer);
  sprintf(GlobalStringBuffer, "X2 = %ld", X2); Serial_Println(GlobalStringBuffer);
  sprintf(GlobalStringBuffer, "B3 = %ld", B3); Serial_Println(GlobalStringBuffer);
#endif

  X1 = ((int32_t)ac3 * B6) >> 13;
  X2 = ((int32_t)b1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  B4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
  B7 = ((uint32_t)UP - B3) * (uint32_t)( 50000UL >> oversampling );

#if BMP085_DEBUG == 1
  sprintf(GlobalStringBuffer, "X1 = %ld", X1); Serial_Println(GlobalStringBuffer);
  sprintf(GlobalStringBuffer, "X2 = %ld", X2); Serial_Println(GlobalStringBuffer);
  sprintf(GlobalStringBuffer, "B4 = %ld", B4); Serial_Println(GlobalStringBuffer);
  sprintf(GlobalStringBuffer, "B7 = %ld", B7); Serial_Println(GlobalStringBuffer);
#endif

  if (B7 < 0x80000000) {
    p = (B7 * 2) / B4;
  } else {
    p = (B7 / B4) * 2;
  }
  X1 = (p >> 8) * (p >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * p) >> 16;

#if BMP085_DEBUG == 1
  sprintf(GlobalStringBuffer, "p = %ld", p); Serial_Println(GlobalStringBuffer);
  sprintf(GlobalStringBuffer, "X1 = %ld", X1); Serial_Println(GlobalStringBuffer);
  sprintf(GlobalStringBuffer, "X2 = %ld", X2); Serial_Println(GlobalStringBuffer);
#endif

  p = p + ((X1 + X2 + (int32_t)3791)>>4);
#if BMP085_DEBUG == 1
  sprintf(GlobalStringBuffer, "p = %ld", p); Serial_Println(GlobalStringBuffer);
#endif
  return p;
}


float BMP085_readTemperature(void) {
  int32_t UT, X1, X2, B5;     // following ds convention
  float temp;

  UT = BMP085_readRawTemperature();

#if BMP085_DEBUG == 2
  // use datasheet numbers!
  UT = 27898;
  ac6 = 23153;
  ac5 = 32757;
  mc = -8711;
  md = 2868;
#endif

  // step 1
  X1 = ((UT - (int32_t)ac6) * (int32_t)ac5) >> 15;
  X2 = ((int32_t)mc << 11) / (X1 + (int32_t)md);
  B5 = X1 + X2;
  temp = (B5 + 8) >> 4;
  temp /= 10;

  return temp;
}

float BMP085_readAltitude(float sealevelPressure) {
  float altitude;

  float pressure = BMP085_readPressure();

  altitude = 44330 * (1.0 - pow(pressure /sealevelPressure,0.1903));

  return altitude;
}

