#ifndef __IMU_I2C_H__
#define __IMU_I2C_H__

#include "stm32f4xx.h"

/**
  * @brief  I2C port definitions
  */
#define IMU_I2C                          I2C1
#define IMU_I2C_CLK                      RCC_APB1Periph_I2C1
#define IMU_I2C_SCL_PIN                  GPIO_Pin_6
#define IMU_I2C_SCL_GPIO_PORT            GPIOB
#define IMU_I2C_SCL_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define IMU_I2C_SCL_SOURCE               GPIO_PinSource6
#define IMU_I2C_SCL_AF                   GPIO_AF_I2C1
#define IMU_I2C_SDA_PIN                  GPIO_Pin_9
#define IMU_I2C_SDA_GPIO_PORT            GPIOB
#define IMU_I2C_SDA_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define IMU_I2C_SDA_SOURCE               GPIO_PinSource9
#define IMU_I2C_SDA_AF                   GPIO_AF_I2C1
#define IMU_I2C_DR                       ((uint32_t)0x40005410) 	// DMA_PeripheralBaseAddr member

/* I2C clock speed configuration (in Hz)
  WARNING:
   Make sure that this define is not already declared in other files (ie.
  stm324xg_eval.h file). It can be used in parallel by other modules. */
#ifndef I2C_SPEED
 #define I2C_SPEED                        300000
#endif /* I2C_SPEED */

/**
  * @brief  IMU DMA definitions
  */
#define IMU_DMA_CLK                      RCC_AHB1Periph_DMA1
#define IMU_DMA_CHANNEL                  DMA_Channel_1
#define IMU_DMA_TX_STREAM                DMA1_Stream6
#define IMU_DMA_RX_STREAM                DMA1_Stream0
#define IMU_DMA_TX_TCFLAG                DMA_FLAG_TCIF6
#define IMU_DMA_RX_TCFLAG                DMA_FLAG_TCIF0

/**
 * @brief Uncomment the line below to enable verfying each written byte in write
 *        operation. The I2C_WriteDeviceRegister() function will then compare the
 *        written and read data and return error status if a mismatch occurs.
 */
/* #define VERIFY_WRITTENDATA */

/**
 * @brief Uncomment the line below if you want to use user timeout callback.
 *        Function prototypes is declared in this file but function body may be
 *        implemented into user application.
 */
/* #define USE_TIMEOUT_USER_CALLBACK */

/**
  * @brief  Timeout user callback function. This function is called when a timeout
  *         condition occurs during communication with IO Expander. Only protoype
  *         of this function is decalred in IO Expander driver. Its implementation
  *         may be done into user application. This function may typically stop
  *         current operations and reset the I2C peripheral and IO Expander.
  *         To enable this function use uncomment the define USE_TIMEOUT_USER_CALLBACK
  *         at the top of this file.
  */
#ifdef USE_TIMEOUT_USER_CALLBACK
 uint8_t IMU_I2C_TimeoutUserCallback(void);
#else
 #define IMU_I2C_TimeoutUserCallback()  0
#endif /* USE_TIMEOUT_USER_CALLBACK */


/**
  * @brief  IMU DMA Direction
  */
typedef enum
{
  IMU_DMA_TX = 0,
  IMU_DMA_RX = 1
} IMU_DMADirection_TypeDef;

#define TIMEOUT_MAX    0x3000 /*<! The value of the maximal timeout for I2C waiting loops */

void IMU_I2C_Config(void);
void IMU_I2C_DMA_Config(IMU_DMADirection_TypeDef Direction, uint8_t* buffer);
uint8_t I2C_WriteDeviceRegister(uint8_t DeviceAddr, uint8_t RegisterAddr, uint8_t RegisterValue);
uint8_t I2C_ReadDeviceRegister(uint8_t DeviceAddr, uint8_t RegisterAddr);
int16_t I2C_ReadInteger_MSBFirst(uint8_t DeviceAddr, uint32_t RegisterAddr);
int16_t I2C_ReadInteger_LSBFirst(uint8_t DeviceAddr, uint32_t RegisterAddr);
void I2C_Read3Integer_LSBFirst(uint8_t DeviceAddr, uint32_t RegisterAddr, int16_t* IntegerBuffer);
void I2C_Read3Integer_MSBFirst(uint8_t DeviceAddr, uint32_t RegisterAddr, int16_t* IntegerBuffer);

#endif /* __IMU_I2C_H__ */
