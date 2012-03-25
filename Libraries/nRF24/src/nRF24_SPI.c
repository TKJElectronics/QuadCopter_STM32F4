/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "nRF24_SPI.h"

void nRF24_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	  /* Enable the SPI periph */
	  RCC_APB2PeriphClockCmd(nRF24_SPI_CLK, ENABLE);

	  /* Enable SCK, MOSI and MISO GPIO clocks */
	  RCC_AHB1PeriphClockCmd(nRF24_SPI_SCK_GPIO_CLK | nRF24_SPI_MISO_GPIO_CLK | nRF24_SPI_MOSI_GPIO_CLK, ENABLE);

	  /* Enable CS  GPIO clock */
	  RCC_AHB1PeriphClockCmd(nRF24_SPI_CS_GPIO_CLK, ENABLE);

	  GPIO_PinAFConfig(nRF24_SPI_SCK_GPIO_PORT, nRF24_SPI_SCK_SOURCE, nRF24_SPI_SCK_AF);
	  GPIO_PinAFConfig(nRF24_SPI_MISO_GPIO_PORT, nRF24_SPI_MISO_SOURCE, nRF24_SPI_MISO_AF);
	  GPIO_PinAFConfig(nRF24_SPI_MOSI_GPIO_PORT, nRF24_SPI_MOSI_SOURCE, nRF24_SPI_MOSI_AF);

	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	  /* SPI SCK pin configuration */
	  GPIO_InitStructure.GPIO_Pin = nRF24_SPI_SCK_PIN;
	  GPIO_Init(nRF24_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

	  /* SPI  MOSI pin configuration */
	  GPIO_InitStructure.GPIO_Pin =  nRF24_SPI_MOSI_PIN;
	  GPIO_Init(nRF24_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

	  /* SPI MISO pin configuration */
	  GPIO_InitStructure.GPIO_Pin = nRF24_SPI_MISO_PIN;
	  GPIO_Init(nRF24_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);

	  /* Configure GPIO PIN for Chip select */
	  GPIO_InitStructure.GPIO_Pin = nRF24_SPI_CS_PIN;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(nRF24_SPI_CS_GPIO_PORT, &GPIO_InitStructure);

	  /* Deselect : Chip Select high */
	  GPIO_SetBits(nRF24_SPI_CS_GPIO_PORT, nRF24_SPI_CS_PIN);

	  /* Configure GPIO PIN for CE */
	  GPIO_InitStructure.GPIO_Pin = nRF24_CE_PIN;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(nRF24_CE_GPIO_PORT, &GPIO_InitStructure);
}

void nRF24_SPI_Init(void)
{
	  SPI_InitTypeDef  SPI_InitStructure;

	  /* SPI configuration -------------------------------------------------------*/
	  SPI_I2S_DeInit(nRF24_SPI);
	  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
	  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	  SPI_InitStructure.SPI_CRCPolynomial = 7;
	  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	  SPI_Init(nRF24_SPI, &SPI_InitStructure);

	  /* Enable SPI1  */
	  SPI_Cmd(nRF24_SPI, ENABLE);
}


uint8_t nRF24_SPI_SendByte(uint8_t byte)
{
  /*!< Loop while DR register in not empty */
  while (SPI_I2S_GetFlagStatus(nRF24_SPI, SPI_I2S_FLAG_TXE) == RESET);

  /*!< Send byte through the SPI1 peripheral */
  SPI_I2S_SendData(nRF24_SPI, byte);

  /*!< Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(nRF24_SPI, SPI_I2S_FLAG_RXNE) == RESET);

  /*!< Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(nRF24_SPI);
}

uint8_t nRF24_SPI_ReadByte(void)
{
  return (nRF24_SPI_SendByte(0xFF));
}
