/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "ESC.h"

u16 ESC_Motor_Values[4] = {0, 0, 0, 0};

void ESC_InitAll(bool CalibrationEnabled)
{
	ESC_Outputs_Init();
	ESC_Timer_Init();

	if (CalibrationEnabled == ESC_WITH_CALIBRATION)
	{
		ESC_ThrottleProgramming();
	} else {
		ESC_SetSpeed(ESC_LEFT, ESC_THROTTLE_MIN);
		ESC_SetSpeed(ESC_RIGHT, ESC_THROTTLE_MIN);
		ESC_SetSpeed(ESC_FRONT, ESC_THROTTLE_MIN);
		ESC_SetSpeed(ESC_BACK, ESC_THROTTLE_MIN);
		Delay_Ms(4000);
	}
}

void ESC_Outputs_Init(void)
{
	  GPIO_InitTypeDef GPIO_InitStructure;

	  /* TIM3 clock enable */
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	  /* GPIOC clock enable */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	  /* GPIOC Configuration: TIM3 CH1 (PB4), TIM3 CH2 (PB5), TIM3 CH3 (PB0) and TIM3 CH4 (PB1) */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_0 | GPIO_Pin_1;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);

	  /* Connect TIM4 pins to AF2 */
	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);
	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);
}

u16 PrescalerValue, PeriodValue;
void ESC_Timer_Init(void)
{

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    // 400KHz
    PrescalerValue = (uint16_t) (SystemCoreClock / 800000) - 1;
    // 400KHz in -> We want to have 1000 in PWM resolution -> This gives ~150Hz PWM signal
    PeriodValue = 8000; //

	  /* Time base configuration */
	  TIM_TimeBaseStructure.TIM_Period = PeriodValue;
	  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	  /* PWM1 Mode configuration: Channel1 */
	  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	  TIM_OCInitStructure.TIM_Pulse = 0;
	  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	  TIM_OC1Init(TIM3, &TIM_OCInitStructure);

	  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

	  /* PWM1 Mode configuration: Channel2 */
	  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	  TIM_OCInitStructure.TIM_Pulse = 0;

	  TIM_OC2Init(TIM3, &TIM_OCInitStructure);

	  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

	  /* PWM1 Mode configuration: Channel3 */
	  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	  TIM_OCInitStructure.TIM_Pulse = 0;

	  TIM_OC3Init(TIM3, &TIM_OCInitStructure);

	  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

	  /* PWM1 Mode configuration: Channel4 */
	  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	  TIM_OCInitStructure.TIM_Pulse = 0;

	  TIM_OC4Init(TIM3, &TIM_OCInitStructure);

	  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

	  TIM_ARRPreloadConfig(TIM3, ENABLE);

	  /* TIM3 enable counter */
	  TIM_Cmd(TIM3, ENABLE);
}

/* Used to set Speed/PWM signal for ESC */
void ESC_SetSpeed(u8 ESC_Number, u16 Speed)
{
	// The speed value should be between ESC_THROTTLE_MIN and ESC_THROTTLE_MAX (0 to 1000)
	if (Speed < ESC_THROTTLE_MIN || Speed > ESC_THROTTLE_MAX) return;
	switch (ESC_Number)
	{
		case 0:
			TIM_SetCompare1(TIM3, Speed);
		break;
		case 1:
			TIM_SetCompare2(TIM3, Speed);
		break;
		case 2:
			TIM_SetCompare3(TIM3, Speed);
		break;
		case 3:
			TIM_SetCompare4(TIM3, Speed);
		break;
	}
	ESC_Motor_Values[ESC_Number] = Speed;
}

void ESC_UpdateAll(void)
{
	if (ESC_Motor_Values[0] < ESC_THROTTLE_MIN) ESC_Motor_Values[0] = ESC_THROTTLE_MIN;
	if (ESC_Motor_Values[0] > ESC_THROTTLE_MAX) ESC_Motor_Values[0] = ESC_THROTTLE_MAX;
	TIM_SetCompare1(TIM3, ESC_Motor_Values[0]);

	if (ESC_Motor_Values[1] < ESC_THROTTLE_MIN) ESC_Motor_Values[1] = ESC_THROTTLE_MIN;
	if (ESC_Motor_Values[1] > ESC_THROTTLE_MAX) ESC_Motor_Values[1] = ESC_THROTTLE_MAX;
	TIM_SetCompare2(TIM3, ESC_Motor_Values[1]);

	if (ESC_Motor_Values[2] < ESC_THROTTLE_MIN) ESC_Motor_Values[2] = ESC_THROTTLE_MIN;
	if (ESC_Motor_Values[2] > ESC_THROTTLE_MAX) ESC_Motor_Values[2] = ESC_THROTTLE_MAX;
	TIM_SetCompare3(TIM3, ESC_Motor_Values[2]);

	if (ESC_Motor_Values[3] < ESC_THROTTLE_MIN) ESC_Motor_Values[3] = ESC_THROTTLE_MIN;
	if (ESC_Motor_Values[3] > ESC_THROTTLE_MAX) ESC_Motor_Values[3] = ESC_THROTTLE_MAX;
	TIM_SetCompare4(TIM3, ESC_Motor_Values[3]);
}

void ESC_ThrottleProgramming(void)
{
	u16 i;
	// Battery back should be disconnected from the ESC's before this is initiated

	// Set full Throttle
	ESC_SetSpeed(ESC_LEFT, ESC_THROTTLE_MAX);
	ESC_SetSpeed(ESC_RIGHT, ESC_THROTTLE_MAX);
	ESC_SetSpeed(ESC_FRONT, ESC_THROTTLE_MAX);
	ESC_SetSpeed(ESC_BACK, ESC_THROTTLE_MAX);

	Delay_Ms(8000); // Wait for battery packs to be connected

	// Do throttle sweep down
	for (i = ESC_THROTTLE_MAX; i > ESC_THROTTLE_MIN; i--)
	{
		ESC_SetSpeed(ESC_LEFT, i);
		ESC_SetSpeed(ESC_RIGHT, i);
		ESC_SetSpeed(ESC_FRONT, i);
		ESC_SetSpeed(ESC_BACK, i);
		Delay_Ms(3); // Total sweep time = 2 x 1000ms = 2s
	}
	ESC_SetSpeed(ESC_LEFT, ESC_THROTTLE_MIN);
	ESC_SetSpeed(ESC_RIGHT, ESC_THROTTLE_MIN);
	ESC_SetSpeed(ESC_FRONT, ESC_THROTTLE_MIN);
	ESC_SetSpeed(ESC_BACK, ESC_THROTTLE_MIN);
	Delay_Ms(2000);
}




#include "IO_Devices.h"
void ESC_MotorOrientationTest(void)
{
	IO_SetLED(IO_LED_FRONT, 1);
	ESC_SetSpeed(ESC_FRONT, 500);
	Delay_Ms(2000);
	ESC_SetSpeed(ESC_FRONT, ESC_THROTTLE_MIN);
	IO_SetLED(IO_LED_FRONT, 0);

	IO_SetLED(IO_LED_RIGHT, 1);
	ESC_SetSpeed(ESC_RIGHT, 500);
	Delay_Ms(2000);
	ESC_SetSpeed(ESC_RIGHT, ESC_THROTTLE_MIN);
	IO_SetLED(IO_LED_RIGHT, 0);

	IO_SetLED(IO_LED_BACK, 1);
	ESC_SetSpeed(ESC_BACK, 500);
	Delay_Ms(2000);
	ESC_SetSpeed(ESC_BACK, ESC_THROTTLE_MIN);
	IO_SetLED(IO_LED_BACK, 0);

	IO_SetLED(IO_LED_LEFT, 1);
	ESC_SetSpeed(ESC_LEFT, 500);
	Delay_Ms(2000);
	ESC_SetSpeed(ESC_LEFT, ESC_THROTTLE_MIN);
	IO_SetLED(IO_LED_LEFT, 0);
}

void ESC_MotorSpeedTest(void)
{
	u16 i;
	for (i = ESC_THROTTLE_MIN; i < ESC_THROTTLE_MAX; i++)
	{
		ESC_SetSpeed(ESC_FRONT, i);
		ESC_SetSpeed(ESC_BACK, i);
		ESC_SetSpeed(ESC_LEFT, i);
		ESC_SetSpeed(ESC_RIGHT, i);
		Delay_Ms(50);
	}

	for (i = ESC_THROTTLE_MAX; i > ESC_THROTTLE_MIN; i--)
	{
		ESC_SetSpeed(ESC_FRONT, i);
		ESC_SetSpeed(ESC_BACK, i);
		ESC_SetSpeed(ESC_LEFT, i);
		ESC_SetSpeed(ESC_RIGHT, i);
		Delay_Ms(50);
	}
}

/**
 * processThrottleCorrection
 *
 * This function will add some throttle imput if the craft is angled
 * this prevent the craft to loose altitude when angled.
 * it also add the battery throttle correction in case
 * of we are in auto-descent.
 *
 * Special thank to Ziojo for this.
 */
/*
void processThrottleCorrection() {

  int throttleAsjust = throttle / (cos (radians (kinematicsAngle[XAXIS])) * cos (radians (kinematicsAngle[YAXIS])));
  throttleAsjust = constrain ((throttleAsjust - throttle), 0, 160); //compensate max  +/- 25 deg XAXIS or YAXIS or  +/- 18 ( 18(XAXIS) + 18(YAXIS))
  throttle = throttle + throttleAsjust + (int)batteyMonitorThrottleCorrection;

  throttle = constrain(throttle,MINCOMMAND,MAXCOMMAND-150);  // limmit throttle to leave some space for motor correction in max throttle manuever
}
*/
