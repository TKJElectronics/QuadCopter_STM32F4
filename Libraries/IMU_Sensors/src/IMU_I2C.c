/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "IMU_I2C.h"

uint32_t IMU_TimeOut = TIMEOUT_MAX; /*<! Value of Timeout when I2C communication fails */

/**
  * @brief  Configure the I2C Peripheral used to communicate with the IMU Sensors.
  * @param  None
  * @retval None
  */
void IMU_I2C_Config(void)
{
	  I2C_InitTypeDef I2C_InitStructure;

	  /* IMU_I2C configuration */
	  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	  I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	  I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;

	  I2C_Init(IMU_I2C, &I2C_InitStructure);
}


/**
  * @brief  Configure the DMA Peripheral used to handle communication via I2C.
  * @param  None
  * @retval None
  */

void IMU_I2C_DMA_Config(IMU_DMADirection_TypeDef Direction, uint8_t* buffer)
{
  DMA_InitTypeDef DMA_InitStructure;

  RCC_AHB1PeriphClockCmd(IMU_DMA_CLK, ENABLE);

  /* Initialize the DMA_Channel member */
  DMA_InitStructure.DMA_Channel = IMU_DMA_CHANNEL;

  /* Initialize the DMA_PeripheralBaseAddr member */
  DMA_InitStructure.DMA_PeripheralBaseAddr = IMU_I2C_DR;

  /* Initialize the DMA_Memory0BaseAddr member */
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)buffer;

  /* Initialize the DMA_PeripheralInc member */
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;

  /* Initialize the DMA_MemoryInc member */
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;

  /* Initialize the DMA_PeripheralDataSize member */
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;

  /* Initialize the DMA_MemoryDataSize member */
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;

  /* Initialize the DMA_Mode member */
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;

  /* Initialize the DMA_Priority member */
  DMA_InitStructure.DMA_Priority = DMA_Priority_Low;

  /* Initialize the DMA_FIFOMode member */
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;

  /* Initialize the DMA_FIFOThreshold member */
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;

  /* Initialize the DMA_MemoryBurst member */
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;

  /* Initialize the DMA_PeripheralBurst member */
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

  /* If using DMA for Reception */
  if (Direction == IMU_DMA_RX)
  {
    /* Initialize the DMA_DIR member */
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;

    /* Initialize the DMA_BufferSize member */
    DMA_InitStructure.DMA_BufferSize = 6;

    DMA_DeInit(IMU_DMA_RX_STREAM);

    DMA_Init(IMU_DMA_RX_STREAM, &DMA_InitStructure);
  }
  /* If using DMA for Transmission */
  else if (Direction == IMU_DMA_TX)
  {
    /* Initialize the DMA_DIR member */
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;

    /* Initialize the DMA_BufferSize member */
    DMA_InitStructure.DMA_BufferSize = 1;

    DMA_DeInit(IMU_DMA_TX_STREAM);

    DMA_Init(IMU_DMA_TX_STREAM, &DMA_InitStructure);
  }
}

/**
  * @brief  Writes a value in a register of the device through I2C.
  * @param  DeviceAddr: The address of the IMUxpander, could be : IMU_1_ADDR
  *         or IMU_2_ADDR.
  * @param  RegisterAddr: The target register address
  * @param  RegisterValue: The target register value to be written
  * @retval IMU_OK: if all operations are OK. Other value if error.
  */
uint8_t I2C_WriteDeviceRegister(uint8_t DeviceAddr, uint8_t RegisterAddr, uint8_t RegisterValue)
{
  uint32_t read_verif = 0;
  uint8_t IMU_BufferTX = 0;

  /* Get Value to be written */
  IMU_BufferTX = RegisterValue;

  /* Configure DMA Peripheral */
  IMU_I2C_DMA_Config(IMU_DMA_TX, (uint8_t*)IMU_BufferTX);

  /* Enable the I2C peripheral */
  I2C_GenerateSTART(IMU_I2C, ENABLE);

  /* Test on SB Flag */
  IMU_TimeOut = TIMEOUT_MAX;
  while (I2C_GetFlagStatus(IMU_I2C,I2C_FLAG_SB) == RESET)
  {
    if (IMU_TimeOut-- == 0) return(IMU_I2C_TimeoutUserCallback());
  }

  /* Transmit the slave address and enable writing operation */
  I2C_Send7bitAddress(IMU_I2C, DeviceAddr, I2C_Direction_Transmitter);

  /* Test on ADDR Flag */
  IMU_TimeOut = TIMEOUT_MAX;
  while (!I2C_CheckEvent(IMU_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
    if (IMU_TimeOut-- == 0) return(IMU_I2C_TimeoutUserCallback());
  }

  /* Transmit the first address for r/w operations */
  I2C_SendData(IMU_I2C, RegisterAddr);

  /* Test on TXE FLag (data dent) */
  IMU_TimeOut = TIMEOUT_MAX;
  while ((!I2C_GetFlagStatus(IMU_I2C,I2C_FLAG_TXE)) && (!I2C_GetFlagStatus(IMU_I2C,I2C_FLAG_BTF)))
  {
    if (IMU_TimeOut-- == 0) return(IMU_I2C_TimeoutUserCallback());
  }


  /* DMA Doesn't seem to work properly when sending/writing, so instead it sends the byte manually!
  // Enable I2C DMA request
  //I2C_DMACmd(IMU_I2C,ENABLE);

  /* Enable DMA TX Channel */
  //DMA_Cmd(IMU_DMA_TX_STREAM, ENABLE);

  /* Wait until DMA Transfer Complete */
  /*IMU_TimeOut = TIMEOUT_MAX;
  while (!DMA_GetFlagStatus(IMU_DMA_TX_STREAM,IMU_DMA_TX_TCFLAG))
  {
    if (IMU_TimeOut-- == 0) return(IMU_I2C_TimeoutUserCallback());
  }*/

  /* Transmit the register value manually (not by using DMA) */
  I2C_SendData(IMU_I2C, RegisterValue);

  /* Test on TXE FLag (data dent) */
  IMU_TimeOut = TIMEOUT_MAX;
  while ((!I2C_GetFlagStatus(IMU_I2C,I2C_FLAG_TXE)) && (!I2C_GetFlagStatus(IMU_I2C,I2C_FLAG_BTF)))
  {
    if (IMU_TimeOut-- == 0) return(IMU_I2C_TimeoutUserCallback());
  }


  /* Wait until BTF Flag is set before generating STOP */
  IMU_TimeOut = 2 * TIMEOUT_MAX;
  while ((!I2C_GetFlagStatus(IMU_I2C,I2C_FLAG_BTF)))
  {
  }

  /* Send STOP Condition */
  I2C_GenerateSTOP(IMU_I2C, ENABLE);

  /* Disable DMA TX Channel */
  DMA_Cmd(IMU_DMA_TX_STREAM, DISABLE);

  /* Disable I2C DMA request */
  I2C_DMACmd(IMU_I2C,DISABLE);

  /* Clear DMA TX Transfer Complete Flag */
  DMA_ClearFlag(IMU_DMA_TX_STREAM,IMU_DMA_TX_TCFLAG);

#ifdef VERIFY_WRITTENDATA
  /* Verify (if needed) that the loaded data is correct  */

  /* Read the just written register*/
  read_verif = I2C_ReadDeviceRegister(DeviceAddr, RegisterAddr);
  /* Load the register and verify its value  */
  if (read_verif != RegisterValue)
  {
    /* Control data wrongly transferred */
    read_verif = IMU_FAILURE;
  }
  else
  {
    /* Control data correctly transferred */
    read_verif = 0;
  }
#endif

  /* Return the verifying value: 0 (Passed) or 1 (Failed) */
  return read_verif;
}

/**
  * @brief  Reads a register of the device through I2C.
  * @param  DeviceAddr: The address of the device, could be : IMU_1_ADDR
  *         or IMU_2_ADDR.
  * @param  RegisterAddr: The target register address (between 00x and 0x24)
  * @retval The value of the read register (0xAA if Timeout occurred)
  */
uint8_t I2C_ReadDeviceRegister(uint8_t DeviceAddr, uint8_t RegisterAddr)
{
  uint8_t IMU_BufferRX[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  /* Configure DMA Peripheral */
  IMU_I2C_DMA_Config(IMU_DMA_RX, (uint8_t*)IMU_BufferRX);

  /* Enable DMA NACK automatic generation */
  I2C_DMALastTransferCmd(IMU_I2C, ENABLE);

  /* Enable the I2C peripheral */
  I2C_GenerateSTART(IMU_I2C, ENABLE);

  /* Test on SB Flag */
  IMU_TimeOut = TIMEOUT_MAX;
  while (!I2C_GetFlagStatus(IMU_I2C,I2C_FLAG_SB))
  {
    if (IMU_TimeOut-- == 0) return(IMU_I2C_TimeoutUserCallback());
  }

  /* Send device address for write */
  I2C_Send7bitAddress(IMU_I2C, DeviceAddr, I2C_Direction_Transmitter);

  /* Test on ADDR Flag */
  IMU_TimeOut = TIMEOUT_MAX;
  while (!I2C_CheckEvent(IMU_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
    if (IMU_TimeOut-- == 0) return(IMU_I2C_TimeoutUserCallback());
  }

  /* Send the device's internal address to write to */
  I2C_SendData(IMU_I2C, RegisterAddr);

  /* Test on TXE FLag (data dent) */
  IMU_TimeOut = TIMEOUT_MAX;
  while ((!I2C_GetFlagStatus(IMU_I2C,I2C_FLAG_TXE)) && (!I2C_GetFlagStatus(IMU_I2C,I2C_FLAG_BTF)))
  {
    if (IMU_TimeOut-- == 0) return(IMU_I2C_TimeoutUserCallback());
  }

  /* Send START condition a second time */
  I2C_GenerateSTART(IMU_I2C, ENABLE);

  /* Test on SB Flag */
  IMU_TimeOut = TIMEOUT_MAX;
  while (!I2C_GetFlagStatus(IMU_I2C,I2C_FLAG_SB))
  {
    if (IMU_TimeOut-- == 0) return(IMU_I2C_TimeoutUserCallback());
  }

  /* Send address for read */
  I2C_Send7bitAddress(IMU_I2C, DeviceAddr, I2C_Direction_Receiver);

  /* Test on ADDR Flag */
  IMU_TimeOut = TIMEOUT_MAX;
  while (!I2C_CheckEvent(IMU_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
  {
    if (IMU_TimeOut-- == 0) return(IMU_I2C_TimeoutUserCallback());
  }

  /* Enable I2C DMA request */
  I2C_DMACmd(IMU_I2C,ENABLE);

  /* Enable DMA RX Channel */
  DMA_Cmd(IMU_DMA_RX_STREAM, ENABLE);

  /* Wait until DMA Transfer Complete */
  IMU_TimeOut = 2 * TIMEOUT_MAX;
  while (!DMA_GetFlagStatus(IMU_DMA_RX_STREAM,IMU_DMA_RX_TCFLAG))
  {
    if (IMU_TimeOut-- == 0) return(IMU_I2C_TimeoutUserCallback());
  }

  /* Send STOP Condition */
  I2C_GenerateSTOP(IMU_I2C, ENABLE);

  /* Disable DMA RX Channel */
  DMA_Cmd(IMU_DMA_RX_STREAM, DISABLE);

  /* Disable I2C DMA request */
  I2C_DMACmd(IMU_I2C,DISABLE);

  /* Clear DMA RX Transfer Complete Flag */
 DMA_ClearFlag(IMU_DMA_RX_STREAM,IMU_DMA_RX_TCFLAG);

  /* return a pointer to the IMU_Buffer */
  return (uint8_t)IMU_BufferRX[0];
}


/**
  * @brief  Reads a buffer of 2 bytes from the device registers.
  * @param  DeviceAddr: The address of the device, could be : IMU_1_ADDR
  *         or IMU_2_ADDR.
  * @param  RegisterAddr: The target register address (between 00x and 0x24)
  * @retval A pointer to the buffer containing the two returned bytes (in halfword).
  */
int16_t I2C_ReadInteger_MSBFirst(uint8_t DeviceAddr, uint32_t RegisterAddr)
{
  uint8_t tmp= 0;
  uint8_t IMU_BufferRX[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  /* Configure DMA Peripheral */
  IMU_I2C_DMA_Config(IMU_DMA_RX, (uint8_t*)IMU_BufferRX);

  /* Enable DMA NACK automatic generation */
  I2C_DMALastTransferCmd(IMU_I2C, ENABLE);

  /* Enable the I2C peripheral */
  I2C_GenerateSTART(IMU_I2C, ENABLE);

  /* Test on SB Flag */
  IMU_TimeOut = TIMEOUT_MAX;
  while (!I2C_GetFlagStatus(IMU_I2C,I2C_FLAG_SB))
  {
    if (IMU_TimeOut-- == 0) return(IMU_I2C_TimeoutUserCallback());
  }

  /* Send device address for write */
  I2C_Send7bitAddress(IMU_I2C, DeviceAddr, I2C_Direction_Transmitter);

  /* Test on ADDR Flag */
  IMU_TimeOut = TIMEOUT_MAX;
  while (!I2C_CheckEvent(IMU_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
    if (IMU_TimeOut-- == 0) return(IMU_I2C_TimeoutUserCallback());
  }

  /* Send the device's internal address to write to */
  I2C_SendData(IMU_I2C, RegisterAddr);

  /* Test on TXE FLag (data dent) */
  IMU_TimeOut = TIMEOUT_MAX;
  while ((!I2C_GetFlagStatus(IMU_I2C,I2C_FLAG_TXE)) && (!I2C_GetFlagStatus(IMU_I2C,I2C_FLAG_BTF)))
  {
    if (IMU_TimeOut-- == 0) return(IMU_I2C_TimeoutUserCallback());
  }

  /* Send START condition a second time */
  I2C_GenerateSTART(IMU_I2C, ENABLE);

  /* Test on SB Flag */
  IMU_TimeOut = TIMEOUT_MAX;
  while (!I2C_GetFlagStatus(IMU_I2C,I2C_FLAG_SB))
  {
    if (IMU_TimeOut-- == 0) return(IMU_I2C_TimeoutUserCallback());
  }

  /* Send IMUxpander address for read */
  I2C_Send7bitAddress(IMU_I2C, DeviceAddr, I2C_Direction_Receiver);

  /* Test on ADDR Flag */
  IMU_TimeOut = TIMEOUT_MAX;
  while (!I2C_CheckEvent(IMU_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
  {
    if (IMU_TimeOut-- == 0) return(IMU_I2C_TimeoutUserCallback());
  }

  /* Enable I2C DMA request */
  I2C_DMACmd(IMU_I2C,ENABLE);

  /* Enable DMA RX Channel */
  DMA_Cmd(IMU_DMA_RX_STREAM, ENABLE);

  /* Wait until DMA Transfer Complete */
  IMU_TimeOut = 2 * TIMEOUT_MAX;
  while (!DMA_GetFlagStatus(IMU_DMA_RX_STREAM, IMU_DMA_RX_TCFLAG))
  {
    if (IMU_TimeOut-- == 0) return(IMU_I2C_TimeoutUserCallback());
  }

  /* Send STOP Condition */
  I2C_GenerateSTOP(IMU_I2C, ENABLE);

  /* Disable DMA RX Channel */
  DMA_Cmd(IMU_DMA_RX_STREAM, DISABLE);

  /* Disable I2C DMA request */
  I2C_DMACmd(IMU_I2C,DISABLE);

  /* Clear DMA RX Transfer Complete Flag */
  DMA_ClearFlag(IMU_DMA_RX_STREAM,IMU_DMA_RX_TCFLAG);

  /* Reorganize received data */
  tmp = IMU_BufferRX[0];
  IMU_BufferRX[0] = IMU_BufferRX[1];
  IMU_BufferRX[1] = tmp;

  /* return a pointer to the IMU_Buffer */
  return *(int16_t *)IMU_BufferRX;
}

/**
  * @brief  Reads a buffer of 2 bytes from the device registers.
  * @param  DeviceAddr: The address of the device, could be : IMU_1_ADDR
  *         or IMU_2_ADDR.
  * @param  RegisterAddr: The target register address (between 00x and 0x24)
  * @retval A pointer to the buffer containing the two returned bytes (in halfword).
  */
int16_t I2C_ReadInteger_LSBFirst(uint8_t DeviceAddr, uint32_t RegisterAddr)
{
  uint8_t IMU_BufferRX[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  /* Configure DMA Peripheral */
  IMU_I2C_DMA_Config(IMU_DMA_RX, (uint8_t*)IMU_BufferRX);

  /* Enable DMA NACK automatic generation */
  I2C_DMALastTransferCmd(IMU_I2C, ENABLE);

  /* Enable the I2C peripheral */
  I2C_GenerateSTART(IMU_I2C, ENABLE);

  /* Test on SB Flag */
  IMU_TimeOut = TIMEOUT_MAX;
  while (!I2C_GetFlagStatus(IMU_I2C,I2C_FLAG_SB))
  {
    if (IMU_TimeOut-- == 0) return(IMU_I2C_TimeoutUserCallback());
  }

  /* Send device address for write */
  I2C_Send7bitAddress(IMU_I2C, DeviceAddr, I2C_Direction_Transmitter);

  /* Test on ADDR Flag */
  IMU_TimeOut = TIMEOUT_MAX;
  while (!I2C_CheckEvent(IMU_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
    if (IMU_TimeOut-- == 0) return(IMU_I2C_TimeoutUserCallback());
  }

  /* Send the device's internal address to write to */
  I2C_SendData(IMU_I2C, RegisterAddr);

  /* Test on TXE FLag (data dent) */
  IMU_TimeOut = TIMEOUT_MAX;
  while ((!I2C_GetFlagStatus(IMU_I2C,I2C_FLAG_TXE)) && (!I2C_GetFlagStatus(IMU_I2C,I2C_FLAG_BTF)))
  {
    if (IMU_TimeOut-- == 0) return(IMU_I2C_TimeoutUserCallback());
  }

  /* Send START condition a second time */
  I2C_GenerateSTART(IMU_I2C, ENABLE);

  /* Test on SB Flag */
  IMU_TimeOut = TIMEOUT_MAX;
  while (!I2C_GetFlagStatus(IMU_I2C,I2C_FLAG_SB))
  {
    if (IMU_TimeOut-- == 0) return(IMU_I2C_TimeoutUserCallback());
  }

  /* Send IMUxpander address for read */
  I2C_Send7bitAddress(IMU_I2C, DeviceAddr, I2C_Direction_Receiver);

  /* Test on ADDR Flag */
  IMU_TimeOut = TIMEOUT_MAX;
  while (!I2C_CheckEvent(IMU_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
  {
    if (IMU_TimeOut-- == 0) return(IMU_I2C_TimeoutUserCallback());
  }

  /* Enable I2C DMA request */
  I2C_DMACmd(IMU_I2C,ENABLE);

  /* Enable DMA RX Channel */
  DMA_Cmd(IMU_DMA_RX_STREAM, ENABLE);

  /* Wait until DMA Transfer Complete */
  IMU_TimeOut = 2 * TIMEOUT_MAX;
  while (!DMA_GetFlagStatus(IMU_DMA_RX_STREAM, IMU_DMA_RX_TCFLAG))
  {
    if (IMU_TimeOut-- == 0) return(IMU_I2C_TimeoutUserCallback());
  }

  /* Send STOP Condition */
  I2C_GenerateSTOP(IMU_I2C, ENABLE);

  /* Disable DMA RX Channel */
  DMA_Cmd(IMU_DMA_RX_STREAM, DISABLE);

  /* Disable I2C DMA request */
  I2C_DMACmd(IMU_I2C,DISABLE);

  /* Clear DMA RX Transfer Complete Flag */
  DMA_ClearFlag(IMU_DMA_RX_STREAM,IMU_DMA_RX_TCFLAG);

  /* return a pointer to the IMU_Buffer */
  return *(int16_t *)IMU_BufferRX;
}


/**
  * @brief  Reads a buffer of 6 bytes from the device registers.
  * @param  DeviceAddr: The address of the device, could be : IMU_1_ADDR
  *         or IMU_2_ADDR.
  * @param  RegisterAddr: The target register address (between 00x and 0x24)
  * @retval A pointer to the buffer containing the two returned bytes (in halfword).
  */
void I2C_Read3Integer_LSBFirst(uint8_t DeviceAddr, uint32_t RegisterAddr, int16_t* IntegerBuffer)
{
  uint8_t IMU_BufferRX[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  /* Configure DMA Peripheral */
  IMU_I2C_DMA_Config(IMU_DMA_RX, (uint8_t*)IMU_BufferRX);

  /* Enable DMA NACK automatic generation */
  I2C_DMALastTransferCmd(IMU_I2C, ENABLE);

  /* Enable the I2C peripheral */
  I2C_GenerateSTART(IMU_I2C, ENABLE);

  /* Test on SB Flag */
  IMU_TimeOut = TIMEOUT_MAX;
  while (!I2C_GetFlagStatus(IMU_I2C,I2C_FLAG_SB))
  {
    if (IMU_TimeOut-- == 0) return;
  }

  /* Send device address for write */
  I2C_Send7bitAddress(IMU_I2C, DeviceAddr, I2C_Direction_Transmitter);

  /* Test on ADDR Flag */
  IMU_TimeOut = TIMEOUT_MAX;
  while (!I2C_CheckEvent(IMU_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
    if (IMU_TimeOut-- == 0) return;
  }

  /* Send the device's internal address to write to */
  I2C_SendData(IMU_I2C, RegisterAddr);

  /* Test on TXE FLag (data dent) */
  IMU_TimeOut = TIMEOUT_MAX;
  while ((!I2C_GetFlagStatus(IMU_I2C,I2C_FLAG_TXE)) && (!I2C_GetFlagStatus(IMU_I2C,I2C_FLAG_BTF)))
  {
    if (IMU_TimeOut-- == 0) return;
  }

  /* Send START condition a second time */
  I2C_GenerateSTART(IMU_I2C, ENABLE);

  /* Test on SB Flag */
  IMU_TimeOut = TIMEOUT_MAX;
  while (!I2C_GetFlagStatus(IMU_I2C,I2C_FLAG_SB))
  {
    if (IMU_TimeOut-- == 0) return;
  }

  /* Send IMUxpander address for read */
  I2C_Send7bitAddress(IMU_I2C, DeviceAddr, I2C_Direction_Receiver);

  /* Test on ADDR Flag */
  IMU_TimeOut = TIMEOUT_MAX;
  while (!I2C_CheckEvent(IMU_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
  {
    if (IMU_TimeOut-- == 0) return;
  }

  /* Enable I2C DMA request */
  I2C_DMACmd(IMU_I2C,ENABLE);

  /* Enable DMA RX Channel */
  DMA_Cmd(IMU_DMA_RX_STREAM, ENABLE);

  /* Wait until DMA Transfer Complete */
  IMU_TimeOut = 2 * TIMEOUT_MAX;
  while (!DMA_GetFlagStatus(IMU_DMA_RX_STREAM, IMU_DMA_RX_TCFLAG))
  {
    if (IMU_TimeOut-- == 0) return;
  }

  /* Send STOP Condition */
  I2C_GenerateSTOP(IMU_I2C, ENABLE);

  /* Disable DMA RX Channel */
  DMA_Cmd(IMU_DMA_RX_STREAM, DISABLE);

  /* Disable I2C DMA request */
  I2C_DMACmd(IMU_I2C,DISABLE);

  /* Clear DMA RX Transfer Complete Flag */
  DMA_ClearFlag(IMU_DMA_RX_STREAM,IMU_DMA_RX_TCFLAG);

  /* Reorganize received data */
  IntegerBuffer[0] = (IMU_BufferRX[1] << 8) | IMU_BufferRX[0];
  IntegerBuffer[1] = (IMU_BufferRX[3] << 8) | IMU_BufferRX[2];
  IntegerBuffer[2] = (IMU_BufferRX[5] << 8) | IMU_BufferRX[4];
}

/**
  * @brief  Reads a buffer of 6 bytes from the device registers.
  * @param  DeviceAddr: The address of the device, could be : IMU_1_ADDR
  *         or IMU_2_ADDR.
  * @param  RegisterAddr: The target register address (between 00x and 0x24)
  * @retval A pointer to the buffer containing the two returned bytes (in halfword).
  */
void I2C_Read3Integer_MSBFirst(uint8_t DeviceAddr, uint32_t RegisterAddr, int16_t* IntegerBuffer)
{
  uint8_t IMU_BufferRX[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  /* Configure DMA Peripheral */
  IMU_I2C_DMA_Config(IMU_DMA_RX, (uint8_t*)IMU_BufferRX);

  /* Enable DMA NACK automatic generation */
  I2C_DMALastTransferCmd(IMU_I2C, ENABLE);

  /* Enable the I2C peripheral */
  I2C_GenerateSTART(IMU_I2C, ENABLE);

  /* Test on SB Flag */
  IMU_TimeOut = TIMEOUT_MAX;
  while (!I2C_GetFlagStatus(IMU_I2C,I2C_FLAG_SB))
  {
    if (IMU_TimeOut-- == 0) return;
  }

  /* Send device address for write */
  I2C_Send7bitAddress(IMU_I2C, DeviceAddr, I2C_Direction_Transmitter);

  /* Test on ADDR Flag */
  IMU_TimeOut = TIMEOUT_MAX;
  while (!I2C_CheckEvent(IMU_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
    if (IMU_TimeOut-- == 0) return;
  }

  /* Send the device's internal address to write to */
  I2C_SendData(IMU_I2C, RegisterAddr);

  /* Test on TXE FLag (data dent) */
  IMU_TimeOut = TIMEOUT_MAX;
  while ((!I2C_GetFlagStatus(IMU_I2C,I2C_FLAG_TXE)) && (!I2C_GetFlagStatus(IMU_I2C,I2C_FLAG_BTF)))
  {
    if (IMU_TimeOut-- == 0) return;
  }

  /* Send START condition a second time */
  I2C_GenerateSTART(IMU_I2C, ENABLE);

  /* Test on SB Flag */
  IMU_TimeOut = TIMEOUT_MAX;
  while (!I2C_GetFlagStatus(IMU_I2C,I2C_FLAG_SB))
  {
    if (IMU_TimeOut-- == 0) return;
  }

  /* Send IMUxpander address for read */
  I2C_Send7bitAddress(IMU_I2C, DeviceAddr, I2C_Direction_Receiver);

  /* Test on ADDR Flag */
  IMU_TimeOut = TIMEOUT_MAX;
  while (!I2C_CheckEvent(IMU_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
  {
    if (IMU_TimeOut-- == 0) return;
  }

  /* Enable I2C DMA request */
  I2C_DMACmd(IMU_I2C,ENABLE);

  /* Enable DMA RX Channel */
  DMA_Cmd(IMU_DMA_RX_STREAM, ENABLE);

  /* Wait until DMA Transfer Complete */
  IMU_TimeOut = 2 * TIMEOUT_MAX;
  while (!DMA_GetFlagStatus(IMU_DMA_RX_STREAM, IMU_DMA_RX_TCFLAG))
  {
    if (IMU_TimeOut-- == 0) return;
  }

  /* Send STOP Condition */
  I2C_GenerateSTOP(IMU_I2C, ENABLE);

  /* Disable DMA RX Channel */
  DMA_Cmd(IMU_DMA_RX_STREAM, DISABLE);

  /* Disable I2C DMA request */
  I2C_DMACmd(IMU_I2C,DISABLE);

  /* Clear DMA RX Transfer Complete Flag */
  DMA_ClearFlag(IMU_DMA_RX_STREAM,IMU_DMA_RX_TCFLAG);

  /* Reorganize received data */
  IntegerBuffer[0] = (IMU_BufferRX[0] << 8) | IMU_BufferRX[1];
  IntegerBuffer[1] = (IMU_BufferRX[2] << 8) | IMU_BufferRX[3];
  IntegerBuffer[2] = (IMU_BufferRX[4] << 8) | IMU_BufferRX[5];
}
