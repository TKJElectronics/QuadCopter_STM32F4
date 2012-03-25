/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "nRF24.h"
#include "Delay.h"

#include "stdio.h"
#include "Serial.h"
#include "String.h"

  bool wide_band; /* 2Mbs data rate in use? */
  bool p_variant; /* False for RF24L01 and true for RF24L01P */
  uint8_t payload_size; /**< Fixed size of payloads */
  bool ack_payload_available; /**< Whether there is an ack payload waiting */
  bool dynamic_payloads_enabled; /**< Whether dynamic payloads are enabled. */
  uint8_t ack_payload_length; /**< Dynamic size of pending ack payload. */
  uint64_t pipe0_reading_address; /**< Last address set on pipe 0 for reading. */

  char StringBuffer2[50];

void nRF24_Init(void)
{
	wide_band = true;
	p_variant = false;
	payload_size = 16;
	ack_payload_available = false;
	dynamic_payloads_enabled = false;
	pipe0_reading_address = 0;

	nRF24_DoInit();

	Delay_Ms(500); // Wait for RF signal to stabilize

	// Make sure the module is initialized properly
	while (nRF24_getPALevel() == 0)
		nRF24_DoInit();
}

void nRF24_DoInit(void)
{
	nRF24_GPIO_Init();
	nRF24_SPI_Init();

	  nRF24_CE_LOW();
	  nRF24_SPI_CS_HIGH();

	  // Must allow the radio time to settle else configuration bits will not necessarily stick.
	  // This is actually only required following power up but some settling time also appears to
	  // be required after resets too. For full coverage, we'll always assume the worst.
	  // Enabling 16b CRC is by far the most obvious case if the wrong timing is used - or skipped.
	  // Technically we require 4.5ms + 14us as a worst case. We'll just call it 5ms for good measure.
	  // WARNING: Delay is based on P-variant whereby non-P *may* require different timing.
	  Delay_Ms(5) ;

	  // Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
	  // WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
	  // sizes must never be used. See documentation for a more complete explanation.
	  nRF24_write_register(SETUP_RETR, (0b0100 << ARD) | (0b1111 << ARC));

	  // Restore our default PA level
	  nRF24_setPALevel( RF24_PA_MAX ) ;

	  // Determine if this is a p or non-p RF24 module and then
	  // reset our data rate back to default value. This works
	  // because a non-P variant won't allow the data rate to
	  // be set to 250Kbps.
	  if( nRF24_setDataRate( RF24_250KBPS ) )
	  {
	    p_variant = true ;
	  }

	  // Then set the data rate to the slowest (and most reliable) speed supported by all
	  // hardware.
	  nRF24_setDataRate( RF24_1MBPS ) ;

	  // Initialize CRC and request 2-byte (16bit) CRC
	  nRF24_setCRCLength( RF24_CRC_16 ) ;

	  // Disable dynamic payloads, to match dynamic_payloads_enabled setting
	  nRF24_write_register(DYNPD,0);

	  // Reset current status
	  // Notice reset and flush is the last thing we do
	  nRF24_write_register(STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

	  // Set up default configuration.  Callers can always change it later.
	  // This channel should be universally safe and not bleed over into adjacent
	  // spectrum.
	  nRF24_setChannel(76);

	  // Flush buffers
	  nRF24_flush_rx();
	  nRF24_flush_tx();
}

/****************************************************************************/

uint8_t nRF24_read_register_buf(uint8_t reg, uint8_t* buf, uint8_t len)
{
  uint8_t status;

  nRF24_SPI_CS_LOW();
  status = nRF24_SPI_SendByte( R_REGISTER | ( REGISTER_MASK & reg ) );
  while ( len-- )
    *buf++ = nRF24_SPI_SendByte(0xff);

  nRF24_SPI_CS_HIGH();

  return status;
}

/****************************************************************************/

uint8_t nRF24_read_register(uint8_t reg)
{
  nRF24_SPI_CS_LOW();
  nRF24_SPI_SendByte( R_REGISTER | ( REGISTER_MASK & reg ) );
  uint8_t result = nRF24_SPI_SendByte(0xff);

  nRF24_SPI_CS_HIGH();
  return result;
}

/****************************************************************************/

uint8_t nRF24_write_register_buf(uint8_t reg, uint8_t* buf, uint8_t len)
{
  uint8_t status;

  nRF24_SPI_CS_LOW();
  status = nRF24_SPI_SendByte( W_REGISTER | ( REGISTER_MASK & reg ) );
  while ( len-- )
	  nRF24_SPI_SendByte(*buf++);

  nRF24_SPI_CS_HIGH();

  return status;
}

/****************************************************************************/

uint8_t nRF24_write_register(uint8_t reg, uint8_t value)
{
  uint8_t status;

  nRF24_SPI_CS_LOW();
  status = nRF24_SPI_SendByte( W_REGISTER | ( REGISTER_MASK & reg ) );
  nRF24_SPI_SendByte(value);
  nRF24_SPI_CS_HIGH();

  return status;
}

/****************************************************************************/

uint8_t nRF24_write_payload(uint8_t* buf, uint8_t len)
{
  uint8_t status;

  uint8_t data_len;
  uint8_t blank_len;

  if (len <= payload_size)
	  data_len = len;
  else
	  data_len = payload_size;

  blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;

  //printf("[Writing %u bytes %u blanks]",data_len,blank_len);

  nRF24_SPI_CS_LOW();
  status = nRF24_SPI_SendByte( W_TX_PAYLOAD );
  while ( data_len-- )
	  nRF24_SPI_SendByte(*buf++);
  while ( blank_len-- )
	  nRF24_SPI_SendByte(0);
  nRF24_SPI_CS_HIGH();

  return status;
}

/****************************************************************************/

uint8_t nRF24_read_payload(uint8_t* buf, uint8_t len)
{
  uint8_t status;

  uint8_t data_len;
  uint8_t blank_len;

  if (len <= payload_size)
	  data_len = len;
  else
	  data_len = payload_size;

  blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;

  //printf("[Reading %u bytes %u blanks]",data_len,blank_len);

  nRF24_SPI_CS_LOW();
  status = nRF24_SPI_SendByte( R_RX_PAYLOAD );
  while ( data_len-- )
    *buf++ = nRF24_SPI_SendByte(0xff);
  while ( blank_len-- )
	  nRF24_SPI_SendByte(0xff);
  nRF24_SPI_CS_HIGH();

  return status;
}

/****************************************************************************/

uint8_t nRF24_flush_rx(void)
{
  uint8_t status;

  nRF24_SPI_CS_LOW();
  status = nRF24_SPI_SendByte( FLUSH_RX );
  nRF24_SPI_CS_HIGH();

  return status;
}

/****************************************************************************/

uint8_t nRF24_flush_tx(void)
{
  uint8_t status;

  nRF24_SPI_CS_LOW();
  status = nRF24_SPI_SendByte( FLUSH_TX );
  nRF24_SPI_CS_HIGH();

  return status;
}

/****************************************************************************/

uint8_t nRF24_get_status(void)
{
  uint8_t status;

  nRF24_SPI_CS_LOW();
  status = nRF24_SPI_SendByte( NOP );
  nRF24_SPI_CS_HIGH();

  return status;
}



/****************************************************************************/

void nRF24_setChannel(uint8_t channel)
{
  // TODO: This method could take advantage of the 'wide_band' calculation
  // done in setChannel() to require certain channel spacing.

  if (channel <= max_channel)
	  nRF24_write_register(RF_CH,channel);
  else
	  nRF24_write_register(RF_CH, max_channel);
}

/****************************************************************************/

void nRF24_setPayloadSize(uint8_t size)
{
	if (size <= max_payload_size)
		payload_size = size;
	else
		payload_size = max_payload_size;
}

/****************************************************************************/

uint8_t nRF24_getPayloadSize(void)
{
  return payload_size;
}

/****************************************************************************/

static const char rf24_datarate_e_str_0[] = "1MBPS";
static const char rf24_datarate_e_str_1[] = "2MBPS";
static const char rf24_datarate_e_str_2[] = "250KBPS";
static const char * const rf24_datarate_e_str_P[] = {
  rf24_datarate_e_str_0,
  rf24_datarate_e_str_1,
  rf24_datarate_e_str_2,
};
static const char rf24_model_e_str_0[] = "nRF24L01";
static const char rf24_model_e_str_1[] = "nRF24L01+";
static const char * const rf24_model_e_str_P[] = {
  rf24_model_e_str_0,
  rf24_model_e_str_1,
};
static const char rf24_crclength_e_str_0[] = "Disabled";
static const char rf24_crclength_e_str_1[] = "8 bits";
static const char rf24_crclength_e_str_2[] = "16 bits" ;
static const char * const rf24_crclength_e_str_P[] = {
  rf24_crclength_e_str_0,
  rf24_crclength_e_str_1,
  rf24_crclength_e_str_2,
};
static const char rf24_pa_dbm_e_str_0[] = "PA_MIN";
static const char rf24_pa_dbm_e_str_1[] = "PA_LOW";
static const char rf24_pa_dbm_e_str_2[] = "LA_MED";
static const char rf24_pa_dbm_e_str_3[] = "PA_HIGH";
static const char * const rf24_pa_dbm_e_str_P[] = {
  rf24_pa_dbm_e_str_0,
  rf24_pa_dbm_e_str_1,
  rf24_pa_dbm_e_str_2,
  rf24_pa_dbm_e_str_3,
};


/****************************************************************************/

void nRF24_startListening(void)
{
	nRF24_write_register(CONFIG, nRF24_read_register(CONFIG) | _BV(PWR_UP) | _BV(PRIM_RX));
	nRF24_write_register(STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

  // Restore the pipe0 adddress, if exists
  if (pipe0_reading_address)
	  nRF24_write_register_buf(RX_ADDR_P0, (uint8_t *)(&pipe0_reading_address), 5);

  // Flush buffers
  nRF24_flush_rx();
  nRF24_flush_tx();

  // Go!
  nRF24_CE_HIGH();

  // wait for the radio to come up (130us actually only needed)
  Delay_us(130);
}

/****************************************************************************/

void nRF24_stopListening(void)
{
  nRF24_CE_LOW();
  nRF24_flush_tx();
  nRF24_flush_rx();
}

/****************************************************************************/

void nRF24_powerDown(void)
{
	nRF24_write_register(CONFIG,nRF24_read_register(CONFIG) & ~_BV(PWR_UP));
}

/****************************************************************************/

void nRF24_powerUp(void)
{
	nRF24_write_register(CONFIG,nRF24_read_register(CONFIG) | _BV(PWR_UP));
}

/******************************************************************/

bool nRF24_write( uint8_t* buf, uint8_t len )
{
  uint8_t status;
  uint32_t sent_at;
  bool result = false;

  // Begin the write
  nRF24_startWrite(buf,len);

  // ------------
  // At this point we could return from a non-blocking write, and then call
  // the rest after an interrupt

  // Instead, we are going to block here until we get TX_DS (transmission completed and ack'd)
  // or MAX_RT (maximum retries, transmission failed).  Also, we'll timeout in case the radio
  // is flaky and we get neither.

  // IN the end, the send should be blocking.  It comes back in 60ms worst case, or much faster
  // if I tighted up the retry logic.  (Default settings will be 1500us.
  // Monitor the send
  uint8_t observe_tx;
  sent_at = Millis();
  const uint32_t timeout = 500; //ms to wait for timeout
  do
  {
    status = nRF24_read_register_buf(OBSERVE_TX,&observe_tx,1);
  }
  while( ! ( status & ( _BV(TX_DS) | _BV(MAX_RT) ) ) && ( Millis() - sent_at < timeout ) );

  // The part above is what you could recreate with your own interrupt handler,
  // and then call this when you got an interrupt
  // ------------

  // Call this when you get an interrupt
  // The status tells us three things
  // * The send was successful (TX_DS)
  // * The send failed, too many retries (MAX_RT)
  // * There is an ack packet waiting (RX_DR)
  bool tx_ok, tx_fail;
  nRF24_whatHappened(&tx_ok, &tx_fail, &ack_payload_available);

  /*sprintf(StringBuffer2, "%u%u%u\r\n",tx_ok,tx_fail,ack_payload_available);
  Serial_Println(StringBuffer2);*/


  result = tx_ok;

  // Handle the ack packet
  if ( ack_payload_available )
  {
    ack_payload_length = nRF24_getDynamicPayloadSize();
  }

  // Yay, we are done.

  // Power down
  nRF24_powerDown();

  // Flush buffers (Is this a relic of past experimentation, and not needed anymore??)
  nRF24_flush_tx();

  return result;
}
/****************************************************************************/

void nRF24_startWrite( uint8_t* buf, uint8_t len )
{
  // Transmitter power-up
  nRF24_write_register(CONFIG, ( nRF24_read_register(CONFIG) | _BV(PWR_UP) ) & ~_BV(PRIM_RX) );
  Delay_us(150);

  // Send the payload
  nRF24_write_payload( buf, len );

  // Allons!
  nRF24_CE_HIGH();
  Delay_us(15);
  nRF24_CE_LOW();
}

/****************************************************************************/

uint8_t nRF24_getDynamicPayloadSize(void)
{
  uint8_t result = 0;

  nRF24_SPI_CS_LOW();
  nRF24_SPI_SendByte( R_RX_PL_WID );
  result = nRF24_SPI_SendByte(0xff);
  nRF24_SPI_CS_HIGH();

  return result;
}

/****************************************************************************/

bool nRF24_available(uint8_t* pipe_num)
{
  uint8_t status = nRF24_get_status();

  // Too noisy, enable if you really want lots o data!!
  //IF_SERIAL_DEBUG(print_status(status));

  bool result = ( status & _BV(RX_DR) );

  if (result)
  {
    // If the caller wants the pipe number, include that
    if ( pipe_num )
      *pipe_num = ( status >> RX_P_NO ) & 0b111;

    // Clear the status bit

    // ??? Should this REALLY be cleared now?  Or wait until we
    // actually READ the payload?

    nRF24_write_register(STATUS,_BV(RX_DR) );

    // Handle ack payload receipt
    if ( status & _BV(TX_DS) )
    {
    	nRF24_write_register(STATUS,_BV(TX_DS));
    }
  }

  return result;
}

/****************************************************************************/

bool nRF24_read( uint8_t* buf, uint8_t len )
{
  // Fetch the payload
	nRF24_read_payload( buf, len );

  // was this the last of the data available?
  return nRF24_read_register(FIFO_STATUS) & _BV(RX_EMPTY);
}

/****************************************************************************/

void nRF24_whatHappened(bool* tx_ok, bool* tx_fail, bool* rx_ready)
{
  // Read the status & reset the status in one easy call
  // Or is that such a good idea?
  uint8_t status = nRF24_write_register(STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

  // Report to the user what happened
  *tx_ok = status & _BV(TX_DS);
  *tx_fail = status & _BV(MAX_RT);
  *rx_ready = status & _BV(RX_DR);
}

/****************************************************************************/

u8 currentPipes = 0;

void nRF24_openWritingPipe(uint64_t value)
{
  // Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
  // expects it LSB first too, so we're good.

  nRF24_write_register_buf(RX_ADDR_P0, (uint8_t*)(&value), 5);
  nRF24_write_register_buf(TX_ADDR, (uint8_t*)(&value), 5);

  if (payload_size <= max_payload_size)
	  nRF24_write_register(RX_PW_P0,payload_size);
  else
	  nRF24_write_register(RX_PW_P0,max_payload_size);

  currentPipes |= 1;
}

/****************************************************************************/

static const uint8_t child_pipe[] =
{
  RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5
};
static const uint8_t child_payload_size[] =
{
  RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5
};
static const uint8_t child_pipe_enable[] =
{
  ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5
};

void nRF24_openReadingPipe(uint8_t child, uint64_t address)
{
  // If this is pipe 0, cache the address.  This is needed because
  // openWritingPipe() will overwrite the pipe 0 address, so
  // startListening() will have to restore it.
  if (child == 0)
    pipe0_reading_address = address;

  if (child <= 6)
  {
    // For pipes 2-5, only write the LSB
    if ( child < 2 )
      nRF24_write_register_buf(child_pipe[child], (uint8_t*)(&address), 5);
    else
      nRF24_write_register_buf(child_pipe[child], (uint8_t*)(&address), 1);

    nRF24_write_register(child_payload_size[child], payload_size);

    // Note it would be more efficient to set all of the bits for all open
    // pipes at once.  However, I thought it would make the calling code
    // more simple to do it this way.
    nRF24_write_register(EN_RXADDR, /*nRF24_read_register(EN_RXADDR)*/ currentPipes | _BV(child_pipe_enable[child]));
    currentPipes |= _BV(child_pipe_enable[child]);
  }
}

/****************************************************************************/

void nRF24_toggle_features(void)
{
  nRF24_SPI_CS_LOW();
  nRF24_SPI_SendByte( ACTIVATE );
  nRF24_SPI_SendByte( 0x73 );
  nRF24_SPI_CS_HIGH();
}

/****************************************************************************/

void nRF24_enableDynamicPayloads(void)
{
  // Enable dynamic payload throughout the system
  nRF24_write_register(FEATURE, nRF24_read_register(FEATURE) | _BV(EN_DPL) );

  // If it didn't work, the features are not enabled
  if ( ! nRF24_read_register(FEATURE) )
  {
    // So enable them and try again
    nRF24_toggle_features();
    nRF24_write_register(FEATURE, nRF24_read_register(FEATURE) | _BV(EN_DPL) );
  }


  // Enable dynamic payload on all pipes
  //
  // Not sure the use case of only having dynamic payload on certain
  // pipes, so the library does not support it.
  nRF24_write_register(DYNPD, nRF24_read_register(DYNPD) | _BV(DPL_P5) | _BV(DPL_P4) | _BV(DPL_P3) | _BV(DPL_P2) | _BV(DPL_P1) | _BV(DPL_P0));

  dynamic_payloads_enabled = true;
}

/****************************************************************************/

void nRF24_enableAckPayload(void)
{
  //
  // enable ack payload and dynamic payload features
  //

  nRF24_write_register(FEATURE,nRF24_read_register(FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL) );

  // If it didn't work, the features are not enabled
  if ( ! nRF24_read_register(FEATURE) )
  {
    // So enable them and try again
	nRF24_toggle_features();
    nRF24_write_register(FEATURE,nRF24_read_register(FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL) );
  }

  //
  // Enable dynamic payload on pipes 0 & 1
  //

  nRF24_write_register(DYNPD,nRF24_read_register(DYNPD) | _BV(DPL_P1) | _BV(DPL_P0));
}

/****************************************************************************/

void nRF24_writeAckPayload(uint8_t pipe, uint8_t* buf, uint8_t len)
{
  nRF24_SPI_CS_LOW();
  nRF24_SPI_SendByte( W_ACK_PAYLOAD | ( pipe & 0b111 ) );
  uint8_t data_len;
  if (len <= max_payload_size)
	  data_len = len;
  else
	  data_len = max_payload_size;
  while ( data_len-- )
	  nRF24_SPI_SendByte(*buf++);

  nRF24_SPI_CS_HIGH();
}

/****************************************************************************/

bool nRF24_isAckPayloadAvailable(void)
{
  bool result = ack_payload_available;
  ack_payload_available = false;
  return result;
}

/****************************************************************************/

bool nRF24_isPVariant(void)
{
  return p_variant ;
}

/****************************************************************************/

void nRF24_setAutoAck(bool enable)
{
  if ( enable )
    nRF24_write_register(EN_AA, 0b111111);
  else
    nRF24_write_register(EN_AA, 0);
}

/****************************************************************************/

void nRF24_setAutoAck_pipe( uint8_t pipe, bool enable )
{
  if ( pipe <= 6 )
  {
    uint8_t en_aa = nRF24_read_register( EN_AA ) ;
    if( enable )
    {
      en_aa |= _BV(pipe) ;
    }
    else
    {
      en_aa &= ~_BV(pipe) ;
    }
    nRF24_write_register( EN_AA, en_aa ) ;
  }
}

/****************************************************************************/

bool nRF24_testCarrier(void)
{
  return ( nRF24_read_register(CD) & 1 );
}

/****************************************************************************/

bool nRF24_testRPD(void)
{
  return ( nRF24_read_register(RPD) & 1 ) ;
}

/****************************************************************************/

void nRF24_setPALevel(rf24_pa_dbm_e level)
{
  uint8_t setup = nRF24_read_register(RF_SETUP) ;
  setup &= ~(_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;

  // switch uses RAM (evil!)
  if ( level == RF24_PA_MAX )
  {
    setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;
  }
  else if ( level == RF24_PA_HIGH )
  {
    setup |= _BV(RF_PWR_HIGH) ;
  }
  else if ( level == RF24_PA_LOW )
  {
    setup |= _BV(RF_PWR_LOW);
  }
  else if ( level == RF24_PA_MIN )
  {
    // nothing
  }
  else if ( level == RF24_PA_ERROR )
  {
    // On error, go to maximum PA
    setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;
  }

  nRF24_write_register( RF_SETUP, setup ) ;
}

/****************************************************************************/

rf24_pa_dbm_e nRF24_getPALevel(void)
{
  rf24_pa_dbm_e result = RF24_PA_ERROR ;
  uint8_t power = nRF24_read_register(RF_SETUP) & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;

  // switch uses RAM (evil!)
  if ( power == (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) )
  {
    result = RF24_PA_MAX ;
  }
  else if ( power == _BV(RF_PWR_HIGH) )
  {
    result = RF24_PA_HIGH ;
  }
  else if ( power == _BV(RF_PWR_LOW) )
  {
    result = RF24_PA_LOW ;
  }
  else
  {
    result = RF24_PA_MIN ;
  }

  return result ;
}

/****************************************************************************/

bool nRF24_setDataRate(rf24_datarate_e speed)
{
  bool result = false;
  uint8_t setup = nRF24_read_register(RF_SETUP) ;

  // HIGH and LOW '00' is 1Mbs - our default
  wide_band = false ;
  setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH)) ;
  if( speed == RF24_250KBPS )
  {
    // Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
    // Making it '10'.
    wide_band = false ;
    setup |= _BV( RF_DR_LOW ) ;
  }
  else
  {
    // Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
    // Making it '01'
    if ( speed == RF24_2MBPS )
    {
      wide_band = true ;
      setup |= _BV(RF_DR_HIGH);
    }
    else
    {
      // 1Mbs
      wide_band = false ;
    }
  }
  nRF24_write_register(RF_SETUP,setup);

  // Verify our result
  if ( nRF24_read_register(RF_SETUP) == setup )
  {
    result = true;
  }
  else
  {
    wide_band = false;
  }

  return result;
}

/****************************************************************************/

rf24_datarate_e nRF24_getDataRate( void )
{
  rf24_datarate_e result ;
  uint8_t dr = nRF24_read_register(RF_SETUP) & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

  // switch uses RAM (evil!)
  // Order matters in our case below
  if ( dr == _BV(RF_DR_LOW) )
  {
    // '10' = 250KBPS
    result = RF24_250KBPS ;
  }
  else if ( dr == _BV(RF_DR_HIGH) )
  {
    // '01' = 2MBPS
    result = RF24_2MBPS ;
  }
  else
  {
    // '00' = 1MBPS
    result = RF24_1MBPS ;
  }
  return result ;
}

/****************************************************************************/

void nRF24_setCRCLength(rf24_crclength_e length)
{
  uint8_t config = nRF24_read_register(CONFIG) & ~( _BV(CRCO) | _BV(EN_CRC)) ;

  // switch uses RAM (evil!)
  if ( length == RF24_CRC_DISABLED )
  {
    // Do nothing, we turned it off above.
  }
  else if ( length == RF24_CRC_8 )
  {
    config |= _BV(EN_CRC);
  }
  else
  {
    config |= _BV(EN_CRC);
    config |= _BV( CRCO );
  }
  nRF24_write_register( CONFIG, config ) ;
}

/****************************************************************************/

rf24_crclength_e nRF24_getCRCLength(void)
{
  rf24_crclength_e result = RF24_CRC_DISABLED;
  uint8_t config = nRF24_read_register(CONFIG) & ( _BV(CRCO) | _BV(EN_CRC)) ;

  if ( config & _BV(EN_CRC ) )
  {
    if ( config & _BV(CRCO) )
      result = RF24_CRC_16;
    else
      result = RF24_CRC_8;
  }

  return result;
}

/****************************************************************************/

void nRF24_disableCRC( void )
{
  uint8_t disable = nRF24_read_register(CONFIG) & ~_BV(EN_CRC) ;
  nRF24_write_register( CONFIG, disable ) ;
}

/****************************************************************************/
void nRF24_setRetries(uint8_t delay, uint8_t count)
{
 nRF24_write_register(SETUP_RETR,(delay&0xf)<<ARD | (count&0xf)<<ARC);
}


void nRF24_printDetails(void)
{
  nRF24_print_status(nRF24_get_status());

  nRF24_print_address_register("RX_ADDR_P0-1",RX_ADDR_P0,2);
  nRF24_print_byte_register("RX_ADDR_P2-5",RX_ADDR_P2,4);
  nRF24_print_address_register("TX_ADDR",TX_ADDR, 1);

  nRF24_print_byte_register("RX_PW_P0-6",RX_PW_P0,6);
  nRF24_print_byte_register("EN_AA",EN_AA, 1);
  nRF24_print_byte_register("EN_RXADDR",EN_RXADDR, 1);
  nRF24_print_byte_register("RF_CH",RF_CH, 1);
  nRF24_print_byte_register("RF_SETUP",RF_SETUP, 1);
  nRF24_print_byte_register("CONFIG",CONFIG, 1);
  nRF24_print_byte_register("DYNPD/FEATURE",DYNPD,2);

  sprintf(StringBuffer2, "Data Rate\t = %s\r\n",rf24_datarate_e_str_P[nRF24_getDataRate()]);
  Serial_Println(StringBuffer2);
  sprintf(StringBuffer2, "Model\t\t = %s\r\n",rf24_model_e_str_P[nRF24_isPVariant()]);
  Serial_Println(StringBuffer2);
  sprintf(StringBuffer2, "CRC Length\t = %s\r\n",rf24_crclength_e_str_P[nRF24_getCRCLength()]);
  Serial_Println(StringBuffer2);
  sprintf(StringBuffer2, "PA Power\t = %s\r\n",rf24_pa_dbm_e_str_P[nRF24_getPALevel()]);
  Serial_Println(StringBuffer2);
}


void nRF24_print_status(uint8_t status)
{
  sprintf(StringBuffer2, "STATUS\t\t = 0x%02x RX_DR=%x TX_DS=%x MAX_RT=%x RX_P_NO=%x TX_FULL=%x\r\n",
           status,
           (status & _BV(RX_DR))?1:0,
           (status & _BV(TX_DS))?1:0,
           (status & _BV(MAX_RT))?1:0,
           ((status >> RX_P_NO) & 0b111),
           (status & _BV(TX_FULL))?1:0
          );
  Serial_Println(StringBuffer2);
}

/****************************************************************************/

void nRF24_print_observe_tx(uint8_t value)
{
  sprintf(StringBuffer2, "OBSERVE_TX=%02x: POLS_CNT=%x ARC_CNT=%x\r\n",
           value,
           (value >> PLOS_CNT) & 0b1111,
           (value >> ARC_CNT) & 0b1111
          );
  Serial_Println(StringBuffer2);
}

/****************************************************************************/

void nRF24_print_byte_register(const char* name, uint8_t reg, uint8_t qty)
{
  char extra_tab = strlen(name) < 8 ? '\t' : 0;
  sprintf(StringBuffer2, "\t%s =  ",name);
  Serial_Print(StringBuffer2);
  while (qty--)
  {
	  sprintf(StringBuffer2, " 0x%02x",nRF24_read_register(reg++));
	  Serial_Print(StringBuffer2);
  }
  Serial_Println("");
}

/****************************************************************************/

void nRF24_print_address_register(const char* name, uint8_t reg, uint8_t qty)
{
  char extra_tab = strlen(name) < 8 ? '\t' : 0;
  sprintf(StringBuffer2, "\t%s =  ",name);
  Serial_Print(StringBuffer2);

  while (qty--)
  {
    uint8_t buffer[5];
    nRF24_read_register_buf(reg++,buffer,sizeof buffer);

    Serial_Print(" 0x");
    uint8_t* bufptr = buffer + sizeof buffer;
    while( --bufptr >= buffer )
    {
    	sprintf(StringBuffer2, "%02x",*bufptr);
    	Serial_Print(StringBuffer2);
    }
  }

  Serial_Println("");
}

// vim:ai:cin:sts=2 sw=2 ft=cpp
