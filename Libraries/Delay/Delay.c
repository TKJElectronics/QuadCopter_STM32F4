/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "Delay.h"

__IO uint32_t TimingDelayus1;
__IO uint32_t TimingDelayus2;
uint32_t MillisCounter = 0;
uint32_t MicrosCounter = 0;

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay10us_Decrement(void)
{
  if (TimingDelayus1 >= 10)
  {
    TimingDelayus1 -= 10;
  } else if (TimingDelayus1 < 10) {
	TimingDelayus1 = 0;
  }

  if (TimingDelayus2 >= 10)
  {
    TimingDelayus2 -= 10;
  } else if (TimingDelayus2 < 10) {
	TimingDelayus2 = 0;
  }

}

void Delay_us(__IO uint32_t nTime)
{
  TimingDelayus1 = nTime;

  while(TimingDelayus1 != 0);
}

void Delay_Ms(__IO uint32_t nTime)
{ 
  TimingDelayus2 = nTime*1000;

  while(TimingDelayus2 != 0);
}

uint32_t Millis(void)
{
    return MicrosCounter/1000;
}

void Increase10Micros(void)
{
    MicrosCounter += 10;
}

uint32_t Micros(void)
{
    return MicrosCounter;
}
