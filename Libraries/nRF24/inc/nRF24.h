#ifndef __NRF24_H__
#define __NRF24_H__

#include "stm32f4xx.h"
#include "bool.h"

#include "nRF24L01.h"
#include "nRF24_SPI.h"
#include "nRF24_Package.h"

/**
 * Power Amplifier level.
 *
 * For use with setPALevel()
 */
typedef enum { RF24_PA_MIN = 0,RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX, RF24_PA_ERROR } rf24_pa_dbm_e ;

/**
 * Data rate.  How fast data moves through the air.
 *
 * For use with setDataRate()
 */
typedef enum { RF24_1MBPS = 0, RF24_2MBPS, RF24_250KBPS } rf24_datarate_e;

/**
 * CRC Length.  How big (if any) of a CRC is included.
 *
 * For use with setCRCLength()
 */
typedef enum { RF24_CRC_DISABLED = 0, RF24_CRC_8, RF24_CRC_16 } rf24_crclength_e;

#define _BV(bit) (1 << (bit))

void nRF24_Init(void);
void nRF24_DoInit(void);

#define max_channel 127
#define max_payload_size 32



uint8_t nRF24_read_register_buf(uint8_t reg, uint8_t* buf, uint8_t len);
uint8_t nRF24_read_register(uint8_t reg);
uint8_t nRF24_write_register_buf(uint8_t reg, uint8_t* buf, uint8_t len);
uint8_t nRF24_write_register(uint8_t reg, uint8_t value);
uint8_t nRF24_write_payload(uint8_t* buf, uint8_t len);
uint8_t nRF24_read_payload(uint8_t* buf, uint8_t len);
uint8_t nRF24_flush_rx(void);
uint8_t nRF24_flush_tx(void);
uint8_t nRF24_get_status(void);
void nRF24_setChannel(uint8_t channel);
void nRF24_setPayloadSize(uint8_t size);
uint8_t nRF24_getPayloadSize(void);
void nRF24_startListening(void);
void nRF24_stopListening(void);
void nRF24_powerDown(void);
void nRF24_powerUp(void);
bool nRF24_write( uint8_t* buf, uint8_t len );
void nRF24_startWrite( uint8_t* buf, uint8_t len );
uint8_t nRF24_getDynamicPayloadSize(void);
bool nRF24_available(uint8_t* pipe_num);
bool nRF24_read( uint8_t* buf, uint8_t len );
void nRF24_whatHappened(bool* tx_ok, bool* tx_fail, bool* rx_ready);
void nRF24_openWritingPipe(uint64_t value);
void nRF24_openReadingPipe(uint8_t child, uint64_t address);
void nRF24_toggle_features(void);
void nRF24_enableDynamicPayloads(void);
void nRF24_enableAckPayload(void);
void nRF24_writeAckPayload(uint8_t pipe, uint8_t* buf, uint8_t len);
bool nRF24_isAckPayloadAvailable(void);
bool nRF24_isPVariant(void);
void nRF24_setAutoAck(bool enable);
void nRF24_setAutoAck_pipe( uint8_t pipe, bool enable );
bool nRF24_testCarrier(void);
bool nRF24_testRPD(void);
void nRF24_setPALevel(rf24_pa_dbm_e level);
rf24_pa_dbm_e nRF24_getPALevel(void);
bool nRF24_setDataRate(rf24_datarate_e speed);
rf24_datarate_e nRF24_getDataRate( void );
void nRF24_setCRCLength(rf24_crclength_e length);
rf24_crclength_e nRF24_getCRCLength(void);
void nRF24_disableCRC( void );
void nRF24_setRetries(uint8_t delay, uint8_t count);














uint8_t nRF24_getDynamicPayloadSize(void);



#endif /* __NRF_H__ */
