/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "IMU.h"

u32 IMU_TimeStamp, IMU_TimeStamp_Old;
u16 IMU_TimeStepus;
bool IMU_Do_Calculations = false;
u8 TaskCounter = 0;
AverageMeasurements IMU_Average;

void IMU_InitTimer(u16 IMU_Freq)
{
	IMU_TimeStepus = 1000000 / IMU_Freq;
	IMU_Timer_Configuration(IMU_Freq);
	IMU_Timer_Interrupt_Configuration();
}

void IMU_Task(void)
{
     	IMU_TimeStamp_Old = IMU_TimeStamp;
		IMU_TimeStamp = Micros();

	    if (IMU_TimeStamp > IMU_TimeStamp_Old)
	      G_Dt = (float)(IMU_TimeStamp - IMU_TimeStamp_Old) / 1000000.0f; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
	    else G_Dt = 0;
		//G_Dt = (float)(IMU_TimeStepus) / 1000000.0f;

		//IMU_ReadSensors();
	    // Apply sensor calibration
	    //IMU_compensate_sensor_errors();

	    IMU_EvaluateGyro();
	    IMU_EvaluateAccelerometer();
	    IMU_EvaluateMagnetometer();

	    // Run DCM algorithm
	    IMU_Compass_Heading(); // Calculate magnetic heading
	    IMU_Matrix_update();
	    IMU_Normalize();
	    IMU_Drift_correction();
	    IMU_Euler_angles();

	    PID_Task(QuadCopter.Pitch, QuadCopter.Roll, QuadCopter.Yaw, QuadCopter.Throttle);

	    // Safety check
#ifdef TESTING_RIG_INSTALLED
	    if ((IMU_Roll_Final > (PI/3)) || (IMU_Roll_Final < -(PI/3)))
	    {
	    	ESC_SetSpeed(ESC_LEFT, ESC_THROTTLE_MIN);
	    	ESC_SetSpeed(ESC_RIGHT, ESC_THROTTLE_MIN);
	    	ESC_SetSpeed(ESC_FRONT, ESC_THROTTLE_MIN);
	    	ESC_SetSpeed(ESC_BACK, ESC_THROTTLE_MIN);
	    	while (1)
	    	{
	    		Serial_Println("Testing rig failure detected! Reset necessary.");
	    		Delay_Ms(1000);
	    	}
	    }
#endif
}


void IMU_Timer_Configuration(u16 IMU_Freq)
{
	u16 PrescalerValue, PeriodValue;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    // 10KHz
    PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 10000) - 1;
    // 10KHz in -> We want ?KHz out (IMU_Freq)
    PeriodValue = (10000 / IMU_Freq) - 1;

    /* Enable TIM2 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* TIM2 configuration */
    TIM_TimeBaseStructure.TIM_Period = PeriodValue; // 133
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0000;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_OCStructInit(&TIM_OCInitStructure);

    /* Output Compare Timing Mode configuration: Channel1 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
    TIM_OCInitStructure.TIM_Pulse = 0x0;
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);

    /* TIM2 enable counter */
    TIM_Cmd(TIM2, ENABLE);

    /* Immediate load of TIM2 Precaler value */
    //TIM_PrescalerConfig(TIM2, ((SystemCoreClock/1200) - 1), TIM_PSCReloadMode_Immediate);

    /* Clear TIM2 update pending flag */
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);

    /* Enable TIM2 Update interrupt */
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}

void IMU_Timer_Interrupt_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

    /* Configure two bits for preemption priority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* Enable the TIM2 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Reset the IMU time counter
    IMU_TimeStamp = Micros();
}

// TIM2 (IMU Timer) callback handler
void TIM2_IRQHandler(void)
{
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

	IMU_Do_Calculations = true;
}


