/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "IO_Devices.h"

void IO_InitLEDs(void)
{
	  GPIO_InitTypeDef GPIO_InitStructure;

	  // Enable the GPIO_LED Clock
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	  // Configure PD12 in output pushpull mode
	  GPIO_InitStructure.GPIO_Pin = IO_LED_LEFT | IO_LED_RIGHT | IO_LED_FRONT | IO_LED_BACK;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOD, &GPIO_InitStructure);

	  GPIOD->BSRRH = IO_LED_LEFT | IO_LED_RIGHT | IO_LED_FRONT | IO_LED_BACK; // Turn off the LEDs
}

void IO_SetLED(uint16_t LED_IO_Pin, bool State)
{
	if (State == true)
	{
		GPIOD->BSRRL = LED_IO_Pin;
	}
	else
	{
		GPIOD->BSRRH = LED_IO_Pin ;
	}
}
