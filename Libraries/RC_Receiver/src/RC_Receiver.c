/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "RC_Receiver.h"

uint16_t Throttle, Aileron, Elevation, Rudder;
uint32_t IC3ReadValues[10];
uint8_t CaptureNumber1;

int RC_Init(void)
{
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	uint16_t PrescalerValue = 0;
	  /*!< At this stage the microcontroller clock setting is already configured,
	       this is done through SystemInit() function which is called from startup
	       file (startup_stm32f4xx.s) before to branch to application main.
	       To reconfigure the default setting of SystemInit() function, refer to
	       system_stm32f4xx.c file
	     */

	  /* TIM1 Configuration */
	RC_TIM_Config();

	  /* Compute the prescaler value */
	  PrescalerValue = (uint16_t) ((SystemCoreClock/2) / 2000000) - 1;
	  TIM_PrescalerConfig(TIM4, PrescalerValue, TIM_PSCReloadMode_Immediate);

	  /* TIM1 configuration: Input Capture mode ---------------------
	     The external signal is connected to TIM1 CH2 pin (PE.11)
	     The Rising edge is used as active edge,
	     The TIM1 CCR2 is used to compute the frequency value
	  ------------------------------------------------------------ */

	  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
	  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	  TIM_ICInitStructure.TIM_ICFilter = 0x0;

	  TIM_ICInit(TIM4, &TIM_ICInitStructure);

	  /* TIM enable counter */
	  TIM_Cmd(TIM4, ENABLE);

	  /* Enable the CC2 Interrupt Request */
	  TIM_ITConfig(TIM4, TIM_IT_CC1, ENABLE);

	  while (1);
}

/**
  * @brief  Configure the TIM1 Pins.
  * @param  None
  * @retval None
  */
void RC_TIM_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* TIM1 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  /* GPIOA clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  /* TIM4 channel 1 pin (PD.12) configuration */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* Connect TIM pins to AF2 */
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);

  /* Enable the TIM1 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles TIM1 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM4_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM4, TIM_IT_CC1) == SET)
  {
    /* Clear TIM1 Capture compare interrupt pending bit */
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
    if (TIM_GetCapture1(TIM4) > 20000) {
    	CaptureNumber1 = 0;
    	TIM_SetCounter(TIM4, 0);

    	// Check is package is valid
    	if (IC3ReadValues[1] < 4 && IC3ReadValues[3] < 4 && IC3ReadValues[5] < 4) {
    		Aileron = ((u32)IC3ReadValues[0]*1000000) / 2000000; // in microseconds
    		Throttle = ((u32)IC3ReadValues[2]*1000000) / 2000000; // in microseconds
    		Elevation = ((u32)IC3ReadValues[4]*1000000) / 2000000; // in microseconds
    		Rudder = ((u32)IC3ReadValues[6]*1000000) / 2000000; // in microseconds
    	}
    } else {
    	IC3ReadValues[CaptureNumber1] = TIM_GetCapture1(TIM4); // Output in micro seconds
    	TIM_SetCounter(TIM4, 0);
    	CaptureNumber1++;
    }
  }
}
