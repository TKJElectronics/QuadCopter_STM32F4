#ifndef __RC_RECEIVER_H__
#define __RC_RECEIVER_H__

#include "stm32f4xx.h"
#include "Delay.h"

int RC_Init(void);
void RC_TIM_Config(void);
void TIM4_IRQHandler(void);



#endif /* __RC_RECEIVER_H__ */
