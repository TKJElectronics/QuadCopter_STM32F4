/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "nRF24_Package.h"
#include "Serial.h"
#include "Delay.h"

uint8_t nRF24_Receive_Buffer[nRF24_PACKAGE_SIZE];
uint8_t nRF24_Transmit_Buffer[nRF24_PACKAGE_SIZE];


void nRF24_Open(void)
{
    // optionally, increase the delay between retries & # of retries
	nRF24_setRetries(15,15);

	// optionally, reduce the payload size.  seems to
	// improve reliability
	nRF24_setPayloadSize(nRF24_PACKAGE_SIZE);

	//
	// Open pipes to other nodes for communication
	//

	// This simple sketch opens two pipes for these two nodes to communicate
	// back and forth.
	// Open 'our' pipe for writing
	// Open the 'other' pipe for reading, in position #1 (we can have up to 5 pipes open for reading)

	nRF24_openWritingPipe(nRF24_WRITING_PIPE_ADDR);
	nRF24_openReadingPipe(1, nRF24_READING_PIPE_ADDR);

	nRF24_printDetails(); // Used for debugging

	//
	// Start listening
	//

	nRF24_startListening();
}

void nRF24_Task(void)
{
	bool ReceiveState;

	if (nRF24_available(0))
	{
		while (nRF24_available(0))
		{
			ReceiveState = nRF24_read(nRF24_Receive_Buffer, nRF24_PACKAGE_SIZE);
		}

		Delay_Ms(100);

		// Parse incoming package
		if (nRF24_VerifyChecksum(nRF24_Receive_Buffer)) // Verify if the Checksum is valid, and the package has been received correctly
			nRF24_ParsePackage(nRF24_Receive_Buffer);
		else
			nRF24_SendChecksumErrorResponse();
	}
}

bool nRF24_VerifyChecksum(uint8_t* PackageData)
{
	u16 PackageChecksum = ((u16)PackageData[14] << 8) | PackageData[15];
	u16 CalculatedChecksum = nRF24_CalculateChecksum(PackageData);

	return (PackageChecksum == CalculatedChecksum);
}

u16 nRF24_CalculateChecksum(uint8_t* PackageData)
{
	u8 i;
	u16 CalculatedChecksum = 0;

	for (i = 0; i < 14; i++)
	{
		CalculatedChecksum += PackageData[i];
	}
	CalculatedChecksum ^= 0xFFFF;
	CalculatedChecksum += 1;

	return CalculatedChecksum;
}



void nRF24_ParsePackage(uint8_t* PackageData)
{
	uint8_t CMD = PackageData[0];

	nRF24_SendResponsePackage(PackageData); // Just send received data back for testing purpose

	Serial_Println("Received package successfully!");
}

void nRF24_SendChecksumErrorResponse(void)
{
	u8 i;
	u16 PackageChecksum;
	for (i = 0; i < 16; i++)
	{
		nRF24_Transmit_Buffer[i] = 0;
	}
	nRF24_Transmit_Buffer[0] = 0xFF; // Error indicator!

	PackageChecksum = nRF24_CalculateChecksum(nRF24_Transmit_Buffer);
	nRF24_Transmit_Buffer[14] = (PackageChecksum & 0xFF00) >> 8;
	nRF24_Transmit_Buffer[15] = PackageChecksum & 0x00FF;

	nRF24_SendResponsePackage(nRF24_Transmit_Buffer);
}

void nRF24_SendResponsePackage(uint8_t* PackageData)
{
	bool TransmitState;
	nRF24_stopListening();

	do
	{
		TransmitState = nRF24_write(PackageData, 16);

		if (TransmitState)
		{
			Serial_Println("Sent response");
		} else {
			Serial_Println("Error sending response!");
		}
	} while (!TransmitState);

	nRF24_startListening();
}
