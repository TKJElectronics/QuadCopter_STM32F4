#ifndef __IO_DEVICES_H__
#define __IO_DEVICES_H__

#include "stm32f4xx.h"
#include "bool.h"

#define IO_LED_BACK GPIO_Pin_12
#define IO_LED_LEFT GPIO_Pin_13
#define IO_LED_FRONT GPIO_Pin_14
#define IO_LED_RIGHT GPIO_Pin_15

void IO_InitLEDs(void);
void IO_SetLED(uint16_t LED_IO_Pin, bool State);


#endif /* __IO_DEVICES_H__ */
