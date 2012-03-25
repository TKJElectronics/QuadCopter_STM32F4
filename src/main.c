/**
*****************************************************************************
**
**  File        : main.c
**
**  Abstract    : main function.
**
**  Functions   : main
**
**  Environment : Atollic TrueSTUDIO(R)
**                STMicroelectronics STM32F4xx Standard Peripherals Library
**
**  Distribution: The file is distributed “as is,” without any warranty
**                of any kind.
**
**  (c)Copyright Atollic AB.
**  You may use this file as-is or modify it according to the needs of your
**  project. Distribution of this file (unmodified or modified) is not
**  permitted. Atollic AB permit registered Atollic TrueSTUDIO(R) users the
**  rights to distribute the assembled, compiled & linked contents of this
**  file as part of an application binary file, provided that it is built
**  using the Atollic TrueSTUDIO(R) toolchain.
**
**
*****************************************************************************
*/

/* Includes */
#include "stm32f4xx.h"

#include "stdio.h"

#include "Delay.h"
#include "IMU.h"
#include "BMP085.h"
#include "ESC.h"
#include "PID.h"
#include "IO_Devices.h"

#include "Serial.h"
#include "IO_Devices.h"
#include "nRF24.h"

/* Private macro */
/* Private variables */
/* Private function prototypes */
/* Private functions */
void GPIO_Configuration(void);
void NVIC_Configuration(void);


void GetSensors(void);
void PrintSensors(void);

/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/

u32 TimeBefore_IMU_Task, TimeBefore_SendIMUData_Task;

float OutputArray[9];

unsigned long Number;
uint8_t SendBuffer[8];
uint8_t ReceiveBuffer[8];
bool State;

u16 mainI;
float tmpFloat;
u16 tmpInteger;
bool SendIMUData = false;

int main(void)
{
	/* NVIC configuration ------------------------------------------------------*/
	NVIC_Configuration();

	/* GPIO configuration ------------------------------------------------------*/
	GPIO_Configuration();

	RCC_ClocksTypeDef RCC_Clocks;

	/* SysTick end of count event each 10us */

	RCC_GetClocksFreq(&RCC_Clocks);
	SysTick_Config(RCC_Clocks.SYSCLK_Frequency / 100000);

	Serial_Init();
	Serial_Print("Initializing: ");

	IO_InitLEDs();

	/*nRF24_Init();
	nRF24_Open();
	nRF24_Task();*/

	Serial_Print("Motors, ");
    //ESC_InitAll(ESC_WITH_CALIBRATION);
	ESC_InitAll(ESC_WITHOUT_CALIBRATION);
	ESC_SetSpeed(ESC_LEFT, ESC_THROTTLE_MIN);
	ESC_SetSpeed(ESC_RIGHT, ESC_THROTTLE_MIN);
	ESC_SetSpeed(ESC_FRONT, ESC_THROTTLE_MIN);
	ESC_SetSpeed(ESC_BACK, ESC_THROTTLE_MIN);
	//ESC_MotorOrientationTest();

	QuadCopter.Throttle = 000; // Half throttle
	QuadCopter.Pitch = 0.0f;
	QuadCopter.Roll = 0.0f;
	QuadCopter.Yaw = 0.0f;
	PID_Init();

	IO_SetLED(IO_LED_BACK, 1);

	Serial_Print("IMU, ");
	IMU_Config();
	IMU_InitAll();
	//BMP085_Init(BMP085_ULTRAHIGHRES);
	//IMU_InitCalibrateAll(); // Taking long, not used
	IMU_reset_sensor_fusion();

	IO_SetLED(IO_LED_FRONT, 1);

	//IMU_AfterInitCalibration(); // Use this to get predefined pitch/roll/yaw angle offsets
	IMU_SetPredefinedCalibration();

	IO_SetLED(IO_LED_LEFT, 1);

	Serial_Print("PID, ");
	PID_ResetRegulator(&PID_PitchVar);
	PID_ResetRegulator(&PID_RollVar);
	PID_ResetRegulator(&PID_YawVar);
	PID_Enabled = true;

	IO_SetLED(IO_LED_RIGHT, 1);

	Serial_Println("Finished!");
	Serial_Println("");

	TimeBefore_IMU_Task = Micros();

// TOMORROW, MONDAY -> Try the new filtered data with stabilizing, works well?
//                  -> Try to play with gyroVector data into PID filter instead

	/* Infinite loop */
	while (1)
	{
		IMU_SampleAll();

		if ((Micros()-TimeBefore_IMU_Task) >= 20000)
		{
			IMU_Task();

			TimeBefore_IMU_Task = Micros();
		}

		if ((Micros()-TimeBefore_SendIMUData_Task) >= 100000 && SendIMUData)
		{
			OutputArray[0] = TO_DEG(IMU_Roll_Final); // Yaw
			OutputArray[1] = TO_DEG(IMU_Pitch_Final);
			OutputArray[2] = TO_DEG(IMU_Yaw_Final);
			OutputArray[3] = Omega_Vector[0];
			OutputArray[4] = Omega_Vector[1];
			OutputArray[5] = Omega_Vector[2];
			OutputArray[6] = Accel_Vector[0];
			OutputArray[7] = Accel_Vector[1];
			OutputArray[8] = Accel_Vector[2];
			Serial_WriteBytes((u8*)OutputArray, 36);
			Serial_WriteByte(0xCC); Serial_WriteByte(0xCC);

			TimeBefore_SendIMUData_Task = Micros();
		}

			if (Serial_RxCounter > 0)
			{
				if (Serial_RxBuffer[0] == 'R' || Serial_RxBuffer[0] == 'r')
				{
					PID_ResetRegulator(&PID_PitchVar);
					PID_ResetRegulator(&PID_RollVar);
					PID_ResetRegulator(&PID_YawVar);
					Serial_Println("PID Controls has been reset!");
					Serial_RxCounter = 0;
				} else if (Serial_RxBuffer[0] == 'C' || Serial_RxBuffer[0] == 'c') {
					IMU_SerialMaxValuesCalibration();
					Serial_RxCounter = 0;
				} else if (Serial_RxBuffer[0] == 'Z' || Serial_RxBuffer[0] == 'z') { // Get pos
					sprintf(GlobalStringBuffer, "Roll: %f", TO_DEG(IMU_Roll_Final));
					Serial_Println(GlobalStringBuffer);
					sprintf(GlobalStringBuffer, "Pitch: %f", TO_DEG(IMU_Pitch_Final));
					Serial_Println(GlobalStringBuffer);
					sprintf(GlobalStringBuffer, "Yaw: %f", TO_DEG(IMU_Yaw_Final));
					Serial_Println(GlobalStringBuffer);
					Serial_Println("");
					Serial_RxCounter = 0;
				} else if (Serial_RxBuffer[0] == 'S' || Serial_RxBuffer[0] == 's') { // Data stream
					if (SendIMUData)
					{
						Serial_Println("Disabling IMU data stream!");
					} else {
						Serial_RxCounter = 0;
						Serial_Println("Enabling IMU data stream!");
						Serial_WriteByte(0xAA); Serial_WriteByte(0xAA); // Sync stream
						while (Serial_RxBuffer[0] != 'A')
						{
							Serial_RxCounter = 0;
							Delay_Ms(10);
						}
						Serial_RxCounter = 0;
					    IMU_EvaluateGyro();
					    IMU_EvaluateAccelerometer();
					    IMU_EvaluateMagnetometer();
					    TimeBefore_SendIMUData_Task = Micros();
					}
					SendIMUData = !SendIMUData;

					Serial_RxCounter = 0;
				} else if (Serial_RxBuffer[0] == 'X' || Serial_RxBuffer[0] == 'x') { // Get PID values
					sprintf(GlobalStringBuffer, "P: %.4f", PID_RollVar.Kp);
					Serial_Println(GlobalStringBuffer);
					sprintf(GlobalStringBuffer, "I: %.4f", PID_RollVar.Ki);
					Serial_Println(GlobalStringBuffer);
					sprintf(GlobalStringBuffer, "D: %.4f", PID_RollVar.Kd);
					Serial_Println(GlobalStringBuffer);
					sprintf(GlobalStringBuffer, "Throttle: %d", QuadCopter.Throttle);
					Serial_Println(GlobalStringBuffer);
					Serial_Println("");
					Serial_RxCounter = 0;
				} else if (Serial_RxBuffer[0] == 'T' || Serial_RxBuffer[0] == 't') { // Get PID values
					if (Serial_RxCounter >= 4)
					{
						tmpInteger = 0;
						for (mainI = 1; mainI < 4; mainI++)
						{
							tmpInteger += (Serial_RxBuffer[mainI]-0x30) * (pow(10, (3-mainI)));
						}

						QuadCopter.Throttle = tmpInteger;
						if (tmpInteger == 0)
						{
							PID_Enabled = false;
							ESC_SetSpeed(ESC_LEFT, ESC_THROTTLE_MIN);
							ESC_SetSpeed(ESC_RIGHT, ESC_THROTTLE_MIN);
							ESC_SetSpeed(ESC_FRONT, ESC_THROTTLE_MIN);
							ESC_SetSpeed(ESC_BACK, ESC_THROTTLE_MIN);
						} else {
							if (PID_Enabled == false)
							{
								PID_ResetRegulator(&PID_PitchVar);
								PID_ResetRegulator(&PID_RollVar);
								PID_ResetRegulator(&PID_YawVar);
								PID_Enabled = true;
							}
						}

						if (!SendIMUData)
						{
							sprintf(GlobalStringBuffer, "Throttle: %d", tmpInteger);
							Serial_Println(GlobalStringBuffer);
							Serial_Println("");
						}

						Serial_RxCounter = 0;
					}
				} else if (Serial_RxBuffer[0] == 'A' || Serial_RxBuffer[0] == 'a') { // Get PID values
					if (Serial_RxCounter >= 4)
					{
						tmpInteger = 0;
						for (mainI = 2; mainI < 4; mainI++)
						{
							tmpInteger += (Serial_RxBuffer[mainI]-0x30) * (pow(10, (3-mainI)));
						}

						if (Serial_RxBuffer[1] == '-')
						{
							QuadCopter.Roll = (float)TO_RAD(-tmpInteger);
							sprintf(GlobalStringBuffer, "Angle: %d", (-tmpInteger));
						} else if (Serial_RxBuffer[1] == '+') {
							QuadCopter.Roll = (float)TO_RAD(tmpInteger);
							sprintf(GlobalStringBuffer, "Angle: %d", tmpInteger);
						} else {
							sprintf(GlobalStringBuffer, "Error in sign (+/-)");
						}

						Serial_Println(GlobalStringBuffer);
						Serial_Println("");

						Serial_RxCounter = 0;
					}
				} else if (Serial_RxBuffer[0] == 'P' || Serial_RxBuffer[0] == 'p' || Serial_RxBuffer[0] == 'I' || Serial_RxBuffer[0] == 'i' || Serial_RxBuffer[0] == 'D' || Serial_RxBuffer[0] == 'd') {
					if (Serial_RxCounter >= 8)
					{
						if (Serial_RxBuffer[3] == '.')
						{
							tmpFloat = 0.0f;
							for (mainI = 1; mainI < 3; mainI++)
							{
								tmpFloat += (float)(Serial_RxBuffer[mainI]-0x30) * (float)(pow(10, (2-mainI)));
							}
							for (mainI = 4; mainI < 8; mainI++)
							{
								tmpFloat += (float)(Serial_RxBuffer[mainI]-0x30) * (float)(pow(10, (3-(s8)mainI)));
							}

							if (Serial_RxBuffer[0] == 'P' || Serial_RxBuffer[0] == 'p')
							{
								PID_RollVar.Kp = tmpFloat;
								sprintf(GlobalStringBuffer, "P: %.4f", tmpFloat);
							} else if (Serial_RxBuffer[0] == 'I' || Serial_RxBuffer[0] == 'i') {
								PID_RollVar.Ki = tmpFloat;
								sprintf(GlobalStringBuffer, "I: %.4f", tmpFloat);
							} else if (Serial_RxBuffer[0] == 'D' || Serial_RxBuffer[0] == 'd') {
								PID_RollVar.Kd = tmpFloat;
								sprintf(GlobalStringBuffer, "D: %.4f", tmpFloat);
							}
							Serial_Println(GlobalStringBuffer);
							Serial_Println("");

							PID_ResetRegulator(&PID_PitchVar);
							PID_ResetRegulator(&PID_RollVar);
							PID_ResetRegulator(&PID_YawVar);
							Serial_RxCounter = 0;
						} else {
							Serial_Println("Error in float format!");
							Serial_Println("");
							Serial_RxCounter = 0;
						}
					}
				} else {
					Serial_RxCounter = 0;
				}
			}
	}
}


void PrintSensors(void)
{
	Serial_Println("-----------------");
	sprintf(GlobalStringBuffer, "#ACC=%.2f,%.2f,%.2f", IMU_accel[0], IMU_accel[1], IMU_accel[2]);
	Serial_Println(GlobalStringBuffer);
	sprintf(GlobalStringBuffer, "#MAG=%.2f,%.2f,%.2f", IMU_magnetom[0], IMU_magnetom[1], IMU_magnetom[2]);
	Serial_Println(GlobalStringBuffer);
	sprintf(GlobalStringBuffer, "#GYR=%.2f,%.2f,%.2f", IMU_gyro[0], IMU_gyro[1], IMU_gyro[2]);
	Serial_Println(GlobalStringBuffer);
	Serial_Println("-----------------");
	Serial_Println("");
}


/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Configuration(void)
{

}

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures interrupts etc
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{

}
