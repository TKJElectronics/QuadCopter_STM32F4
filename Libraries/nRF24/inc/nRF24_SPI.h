#ifndef __NRF24_SPI_H__
#define __NRF24_SPI_H__

#include "stm32f4xx.h"

/* M25P FLASH SPI Interface pins  */
#define nRF24_SPI                       SPI1
#define nRF24_SPI_CLK                   RCC_APB2Periph_SPI1

#define nRF24_SPI_SCK_PIN               GPIO_Pin_5                  /* PA.05 */
#define nRF24_SPI_SCK_GPIO_PORT         GPIOA                       /* GPIOA */
#define nRF24_SPI_SCK_GPIO_CLK          RCC_AHB1Periph_GPIOA
#define nRF24_SPI_SCK_SOURCE            GPIO_PinSource5
#define nRF24_SPI_SCK_AF                GPIO_AF_SPI1

#define nRF24_SPI_MISO_PIN              GPIO_Pin_6                  /* PA.6 */
#define nRF24_SPI_MISO_GPIO_PORT        GPIOA                       /* GPIOA */
#define nRF24_SPI_MISO_GPIO_CLK         RCC_AHB1Periph_GPIOA
#define nRF24_SPI_MISO_SOURCE           GPIO_PinSource6
#define nRF24_SPI_MISO_AF               GPIO_AF_SPI1

#define nRF24_SPI_MOSI_PIN              GPIO_Pin_7                  /* PA.7 */
#define nRF24_SPI_MOSI_GPIO_PORT        GPIOA                       /* GPIOA */
#define nRF24_SPI_MOSI_GPIO_CLK         RCC_AHB1Periph_GPIOA
#define nRF24_SPI_MOSI_SOURCE           GPIO_PinSource7
#define nRF24_SPI_MOSI_AF               GPIO_AF_SPI1

#define nRF24_SPI_CS_PIN                GPIO_Pin_12                  /* PE.12 */
#define nRF24_SPI_CS_GPIO_PORT          GPIOB                       /* GPIOB */
#define nRF24_SPI_CS_GPIO_CLK           RCC_AHB1Periph_GPIOB

#define nRF24_CE_PIN                GPIO_Pin_13                  /* PE.13 */
#define nRF24_CE_GPIO_PORT          GPIOB                       /* GPIOB */
#define nRF24_CE_GPIO_CLK           RCC_AHB1Periph_GPIOB

/* Exported macro ------------------------------------------------------------*/
/* Select nRF24: Chip Select pin low */
#define nRF24_SPI_CS_LOW()       GPIO_ResetBits(nRF24_SPI_CS_GPIO_PORT, nRF24_SPI_CS_PIN)
/* Deselect nRF24: Chip Select pin high */
#define nRF24_SPI_CS_HIGH()      GPIO_SetBits(nRF24_SPI_CS_GPIO_PORT, nRF24_SPI_CS_PIN)

#define nRF24_CE_LOW()       GPIO_ResetBits(nRF24_CE_GPIO_PORT, nRF24_CE_PIN)
#define nRF24_CE_HIGH()      GPIO_SetBits(nRF24_CE_GPIO_PORT, nRF24_CE_PIN)


void nRF24_GPIO_Init(void);
void nRF24_SPI_Init(void);

uint8_t nRF24_SPI_SendByte(uint8_t byte);
uint8_t nRF24_SPI_ReadByte(void);


#endif /* __NRF24_SPI_H__ */
