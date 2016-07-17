#ifndef __NRF24L01_PLUS_H__
#define __NRF24L01_PLUS_H__


#include "NRF24L01P_MemMap.h"

#include <stddef.h>
#include <time.h>
#include <stdbool.h>

#include "fsl_gpio.h"
#include "fsl_port.h"

/*****************************************************************************/


#define NRF_CS_PIN							9U
#define NRF_CS_GPIO_PORT					GPIOB
#define NRF_CS_GPIO_Init()					GPIO_PinInit(NRF_CS_GPIO_PORT, NRF_CS_PIN, &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)})
#define NRF_CS_HIGH()						GPIO_SetPinsOutput(NRF_CS_GPIO_PORT, 1U << NRF_CS_PIN)
#define NRF_CS_LOW()						GPIO_ClearPinsOutput(NRF_CS_GPIO_PORT, 1U << NRF_CS_PIN)

#define NRF_CE_PIN							0U
#define NRF_CE_GPIO_PORT					GPIOC
#define NRF_CE_GPIO_Init()					GPIO_PinInit(NRF_CE_GPIO_PORT, NRF_CE_PIN, &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)})
#define NRF_CE_HIGH()						GPIO_SetPinsOutput(NRF_CE_GPIO_PORT, 1U << NRF_CE_PIN)
#define NRF_CE_LOW()						GPIO_ClearPinsOutput(NRF_CE_GPIO_PORT, 1U << NRF_CE_PIN)

#define NRF_IRQ_PIN							23U
#define NRF_IRQ_GPIO_PORT					GPIOB
#define NRF_IRQ_GPIO_Init()					GPIO_PinInit(NRF_IRQ_PIN, NRF_IRQ_PIN, &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)})
#define NRF_IRQ_HIGH()						GPIO_SetPinsOutput(NRF_IRQ_PIN, 1U << NRF_IRQ_PIN)
#define NRF_IRQ_LOW()						GPIO_ClearPinsOutput(NRF_IRQ_PIN, 1U << NRF_IRQ_PIN)


#define _BV(x)		 (1 << x)

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

#define rf24_max(a,b) (a>b?a:b)
#define rf24_min(a,b) (a<b?a:b)

/**********************API Functions *************************************/

unsigned char NRF24P_ReadRegister(unsigned char reg);
unsigned char NRF24P_ReadRegisterArray(unsigned char reg, unsigned char* buf, unsigned char len);
unsigned char NRF24P_WriteRegister(unsigned char reg, unsigned char value);
unsigned char NRF24P_WriteRegisterArray(unsigned char reg, unsigned char* buf, unsigned char len);
unsigned char NRF24P_SendCMD(unsigned char cmd);
unsigned char NRF24P_GetStatus(void);


uint8_t       NRF24P_write_payload(const void* buf, uint8_t data_len, const uint8_t writeType);
uint8_t       NRF24P_read_payload(void* buf, uint8_t data_len);

unsigned char NRF24P_flush_rx(void);
unsigned char NRF24P_flush_tx(void);

void          NRF24P_Init(void);
void          NRF24P_startListening(void);
void          NRF24P_stopListening(void);

void          NRF24P_PowerDown(void);
void          NRF24P_PowerUp(void);

void          NRF24P_openWritingPipe(const unsigned char *address);
void          NRF24P_closeReadingPipe(unsigned char pipe );

void          NRF24P_setChannel(unsigned char channel);
unsigned char NRF24P_getChannel();
void          NRF24P_setPayloadSize(unsigned char size);
unsigned char NRF24P_getPayloadSize(void);


bool          NRF24P_writeM( const void* buf, uint8_t len, const bool multicast );
bool          NRF24P_write( const void* buf, uint8_t len );
bool          NRF24P_writeBlocking( const void* buf, uint8_t len, uint32_t timeout );

void          NRF24P_reUseTX(void);
bool          NRF24P_writeFastMulticast( const void* buf, uint8_t len, const bool multicast );
bool          NRF24P_writeFast( const void* buf, uint8_t len );
void          NRF24P_startFastWrite( const void* buf, uint8_t len, const bool multicast, bool startTx);
void          NRF24P_startWrite( const void* buf, uint8_t len, const bool multicast );

bool          NRF24P_rxFifoFull(void);
bool          NRF24P_txStandBy(void);
bool          NRF24P_txStandByT(uint32_t timeout, bool startTx);

void          NRF24P_maskIRQ(bool tx, bool fail, bool rx);
uint8_t       NRF24P_getDynamicPayloadSize(void);

bool          NRF24P_available(uint8_t* pipe_num);
void          NRF24P_read( void* buf, uint8_t len );
void          NRF24P_whatHappened(bool *tx_ok,bool *tx_fail,bool *rx_ready);
void          NRF24P_openWritingPipe(const unsigned char *address);


void          NRF24P_setAddressWidth(unsigned char a_width);

void          NRF24P_openReadingPipe(uint8_t child, const unsigned char *address);
void          NRF24P_closeReadingPipe( uint8_t pipe );
void          NRF24P_toggle_features(void);

void          NRF24P_enableDynamicPayloads(void);
void          NRF24P_enableDynamicAck(void);

void          NRF24P_writeAckPayload(uint8_t pipe, const void* buf, uint8_t len);
bool          NRF24P_isAckPayloadAvailable(void);
bool          NRF24P_isPVariant(void);

void          NRF24P_setAutoAck(bool enable);
void          NRF24P_setAutoAckPipe( uint8_t pipe, bool enable );

bool          NRF24P_testCarrier(void);
bool          NRF24P_testRPD(void);

void             NRF24P_setPALevel(uint8_t level);
uint8_t          NRF24P_getPALevel(void);

bool             NRF24P_setDataRate(rf24_datarate_e speed);
rf24_datarate_e  NRF24P_getDataRate( void );

void             NRF24P_setCRCLength(rf24_crclength_e length);
rf24_crclength_e NRF24P_getCRCLength(void);

void          NRF24P_disableCRC( void );
void          NRF24P_setRetries(unsigned char delay, unsigned char count);
void          NRF24P_ClearINT(void);

void 		  NRF24P_enableAckPayload(void);
void          NRF24P_print_status(unsigned char status);
void          NRF24P_print_observe_tx(unsigned char value);
void          NRF24P_print_byte_register(const char* name, unsigned char reg, unsigned char qty);
void 		  NRF24P_print_address_register(const char* name, uint8_t reg, uint8_t qty);

void 		  NRF24P_maskIRQ(bool tx, bool fail, bool rx);

void          NRF24P_printDetails(void);


void NRF24P_SetupPins(void);

#endif
