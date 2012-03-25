#ifndef __NRF24_PACKAGE_H__
#define __NRF24_PACKAGE_H__

#include "stm32f4xx.h"
#include "nRF24.h"
#include "bool.h"

#define nRF24_WRITING_PIPE_ADDR 0xF0F0F0F0E1LL
#define nRF24_READING_PIPE_ADDR 0xF0F0F0F0D2LL
#define nRF24_PACKAGE_SIZE	16	// bytes


void nRF24_Open(void);
void nRF24_Task(void);
bool nRF24_VerifyChecksum(uint8_t* PackageData);
u16 nRF24_CalculateChecksum(uint8_t* PackageData);
void nRF24_ParsePackage(uint8_t* PackageData);
void nRF24_SendChecksumErrorResponse(void);
void nRF24_SendResponsePackage(uint8_t* PackageData);





/* Test code */
/*
 		nRF24_stopListening();

		State = nRF24_write(SendBuffer, 8);
		if (State)
		{
			Serial_Println("Sent response");
		} else {
			Serial_Println("Error sending response!");
		}

		Number++;

		nRF24_startListening();

		while  ( !nRF24_available(0) );
		State = nRF24_read( ReceiveBuffer, 8 );

		sprintf(StringBuffer, "Received message '%s'", ReceiveBuffer);
		Serial_Println(StringBuffer);

		Delay_Ms(1000);
 */



#endif /* __NRF24_PACKAGE_H__ */
