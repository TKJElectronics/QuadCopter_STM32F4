/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "Ultrasonic_Sensor.h"

uint32_t IC1ReadValues[8];
uint32_t Capture;
uint8_t StateBefore;
uint8_t CaptureNumber2;

int Ultrasonic_Init(void)
{
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	uint16_t PrescalerValue = 0;
	  /*!< At this stage the microcontroller clock setting is already configured,
	       this is done through SystemInit() function which is called from startup
	       file (startup_stm32f4xx.s) before to branch to application main.
	       To reconfigure the default setting of SystemInit() function, refer to
	       system_stm32f4xx.c file
	     */

	  Ultrasonic_InitOutput();

	  /* TIM1 Configuration */
	  Ultrasonic_TIM_Config();

	  /* Compute the prescaler value */
	  PrescalerValue = (uint16_t) (SystemCoreClock / 2000000) - 1;
	  TIM_PrescalerConfig(TIM1, PrescalerValue, TIM_PSCReloadMode_Immediate);

	  /* TIM1 configuration: Input Capture mode ---------------------
	     The external signal is connected to TIM1 CH2 pin (PE.11)
	     The Rising edge is used as active edge,
	     The TIM1 CCR2 is used to compute the frequency value
	  ------------------------------------------------------------ */

	  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
	  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	  TIM_ICInitStructure.TIM_ICFilter = 0x0;

	  TIM_ICInit(TIM1, &TIM_ICInitStructure);

	  /* TIM enable counter */
	  TIM_Cmd(TIM1, ENABLE);

	  /* Enable the CC2 Interrupt Request */
	  TIM_ITConfig(TIM1, TIM_IT_CC2, ENABLE);
	  TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

	  while (1);
}

/**
  * @brief  Configure the TIM1 Pins.
  * @param  None
  * @retval None
  */
void Ultrasonic_TIM_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* TIM1 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

  /* GPIOA clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

  /* TIM1 channel 2 pin (PE.11) configuration */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  /* Connect TIM pins to AF2 */
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);

  /* Enable the TIM1 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable the TIM1 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void Ultrasonic_InitOutput(void)
{
	  GPIO_InitTypeDef GPIO_InitStructure;

	  // Enable the GPIO_LED Clock
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	  // Configure PD12 in output pushpull mode
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);

	  GPIOC->BSRRH = GPIO_Pin_1; // Set output to 0
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
void TIM1_CC_IRQHandler(void)
{
	  if(TIM_GetITStatus(TIM1, TIM_IT_CC1) == SET)
	  {
	    /* Clear TIM1 Capture compare interrupt pending bit */
	    TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);

	    Capture = TIM_GetCapture2(TIM1);

	    if (StateBefore == (uint8_t)Bit_RESET)
	    {
	    	// Rising edge
	    	TIM_SetCounter(TIM1, 0);
	    	StateBefore = (uint8_t)Bit_SET; // State has now changed to SET
	    } else if (StateBefore == (uint8_t)Bit_SET) {
	    	// Falling edge
	    	Capture = TIM_GetCapture2(TIM1);
	    	TIM_SetCounter(TIM1, 0);
	    	StateBefore = (uint8_t)Bit_RESET; // State has now changed to RESET

	        if (Capture > 200) {
	        	IC1ReadValues[CaptureNumber2] = (Capture/4) * 0.034f; // Divide with 2 because of counter freq (2MHz), divide with 2 more because of double length (reflection) -> then from microseconds to cm
	        	CaptureNumber2++;
	        	if (CaptureNumber2 == 8) CaptureNumber2 = 0;
	        }
	    }
	  }
}

void TIM1_UP_TIM10_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

		  GPIOC->BSRRH = GPIO_Pin_1; // 0
		  for (u32 i=0; i < 25; i++);
		  GPIOC->BSRRL = GPIO_Pin_1; // 1
		  for (u32 i=0; i < 63; i++);
		  GPIOC->BSRRH = GPIO_Pin_1; // 0

		  StateBefore = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_11);
	}
}
