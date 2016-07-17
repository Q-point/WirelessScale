#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_dspi.h"

#include "board.h"

#include "pin_mux.h"
#include <stdbool.h>
#include <string.h>

#include <stdbool.h>


#include "NRF24L01.h"
#include "SPI.h"

#include "Utilities.h"


unsigned char payload_size = 0;
unsigned int txRxDelay;                 /**< Var for adjusting delays depending on datarate */
unsigned char addr_width;               /**< The address width to use - 3,4 or 5 bytes. */
unsigned char dynamic_payloads_enabled; /**< Whether dynamic payloads are enabled. */
unsigned char pipe0_reading_address[5]; /**< Last address set on pipe 0 for reading. */
bool p_variant; 						/* False for RF24L01 and true for RF24L01P */

static const unsigned char child_pipe_enable[] = { ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5 };

static const uint8_t child_pipe[] = { RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5 };

static const uint8_t child_payload_size[] = { RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5 };



/**
  * @brief  Configures LED GPIO.
  * @param  Led: Specifies the Led to be configured.
  *   This parameter can be one of following parameters:
  *     @arg LED2
  */
void NRF24P_SetupPins(void)
{


  NRF_CS_HIGH();
  NRF_CE_LOW();

}

/**
  * @brief  DeInit LEDs.
  * @param  Led: LED to be de-init
  * @note Led DeInit does not disable the GPIO clock nor disable the Mfx
  */
void NRF24P_DeInit(void)
{

}

/*
 * @brief Initialize the radio pins and status register
 * @param none
 */
void NRF24P_Init(void)
{
	delay_ms(5);

    NRF24P_WriteRegister( CONFIG, 0b00001100 ) ; // Reset CONFIG and enable 16-bit CRC.
	NRF24P_setRetries(5,15);

	NRF24P_setDataRate( RF24_1MBPS ) ;

    NRF24P_toggle_features();				//Disable dynamic payloads, to match dynamic_payloads_enabled setting - Reset value is 0
    NRF24P_WriteRegister(FEATURE,0 );
    NRF24P_WriteRegister(DYNPD,0);
    delay_ms(50);

	NRF24P_WriteRegister(STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) ); 	//Disable IRQ interrupt when data received,sent, max retries

	NRF24P_setChannel(76);

    NRF24P_SendCMD(FLUSH_TX);
    NRF24P_SendCMD(FLUSH_RX);
    NRF24P_ClearINT();
}



/*
 * @brief Update register with new value.
 * @param reg   name of register
 * @param value value to be written
 * @return none
 */
unsigned char NRF24P_WriteRegister(unsigned char reg, unsigned char value)
{
    unsigned char status = 0;
    // Start SPI transaction
    NRF_CS_LOW();
    status = SPI_Write(W_REGISTER | ( REGISTER_MASK & reg ) );
    SPI_Write(value);
    NRF_CS_HIGH();

    return status;
}

/*
 * @brief Write a multibyte array to a register address
 * @param reg name of register
 * @param buf Values to write
 * @param len Length of buffer
 * @return none
 */
unsigned char NRF24P_WriteRegisterArray(unsigned char reg, unsigned char* buf, unsigned char len)
{
  unsigned char status;

  NRF_CS_LOW();
  status = SPI_Write(W_REGISTER | ( REGISTER_MASK & reg ) );
  while ( len-- )
  {
    SPI_Write(*buf++);
  }
  NRF_CS_HIGH();

  return status;
}

/*
 * @brief Read register value
 * @param reg  Register address
 * @return regsiter value
 */
unsigned char NRF24P_ReadRegister(unsigned char reg)
{
    unsigned char result;
    NRF_CS_LOW();
    SPI_Write(R_REGISTER | ( REGISTER_MASK & reg ));
	result = SPI_Read();
	NRF_CS_HIGH();
	return result;
}


/*
 * @brief Read register array into buffer
 * @param reg
 * @param buf
 * @param len   Length of buffer 0-32
 */
unsigned char NRF24P_ReadRegisterArray(unsigned char reg, unsigned char* buff, unsigned char len)
{

//  NRF_CS_LOW();
//  status =  SPI_Write(R_REGISTER | ( REGISTER_MASK & reg ));
//  while ( len-- )
//  {
//    *buf++ = SPI_Read();
//  }
//  NRF_CS_HIGH();

  unsigned char i = 0;

  for(i=0;i<len;i++)
  {
	  *buff++ = NRF24P_ReadRegister(reg+i);
  }


  return 0;
}

/*
 * @brief Send command to the radio
 * @param cmd Name of command
 */
unsigned char NRF24P_SendCMD(unsigned char cmd)
{
	unsigned char status = 0;
	NRF_CS_LOW();
    status = SPI_Write(cmd);
    NRF_CS_HIGH();
	return status;
}

/*
 * @brief
 * @param
 * @param
 * @param
 */
uint8_t NRF24P_write_payload(const void* buf, uint8_t data_len, const uint8_t writeType)
{
  uint8_t status;
  uint8_t* current = (uint8_t*)(buf);

  data_len = rf24_min(data_len, payload_size);
  uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;

  NRF_CS_LOW();
  status = SPI_Write(writeType );
  while ( data_len-- )
  {
    SPI_Write(*current++);
  }
  while ( blank_len-- )
  {
    SPI_Write(0);
  }
  NRF_CS_HIGH();

  return status;
}

/*
 * @brief
 * @param
 * @param
 * @param
 */
uint8_t NRF24P_read_payload(void* buf, uint8_t data_len)
{
  uint8_t status;
  uint8_t* current = (buf);

  if(data_len > payload_size) data_len = payload_size;
  uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;

  NRF_CS_LOW();
  status = SPI_Write(R_RX_PAYLOAD );
  while ( data_len-- )
  {
    *current++ = SPI_Read();
  }
  while ( blank_len-- )
  {
    SPI_Write(0xff);
  }
  NRF_CS_HIGH();

  return status;
}

/*
 * @brief
 * @param
 * @param
 * @param
 */
unsigned char NRF24P_flush_rx(void)
{
  return NRF24P_SendCMD(FLUSH_RX );
}

/*
 * @brief
 * @param
 * @param
 * @param
 */
unsigned char NRF24P_flush_tx(void)
{
  return NRF24P_SendCMD(FLUSH_TX );
}

/*
 * @brief Get value of status register
 * @param none
 * @return Status register value
 */
unsigned char NRF24P_GetStatus(void)
{
    unsigned char status;

    NRF_CS_LOW();
    status = SPI_Read();
    NRF_CS_HIGH();                                  //Deselect chip

    return status;
}


/*
 * @brief Set operating channel frequency, remeber to be within legal boundaries
 * @param channel value of channel , does not do any incorrect value checking
 */
void NRF24P_setChannel(unsigned char channel)
{
	const unsigned char max_channel = 125;
    NRF24P_WriteRegister(RF_CH,rf24_min(channel,max_channel));
}


/*
 * @brief
 */
unsigned char NRF24P_getChannel(void)
{
  return NRF24P_ReadRegister(RF_CH);
}

void NRF24P_setPayloadSize(unsigned char size)
{
	payload_size = rf24_min(size,32);
}


unsigned char NRF24P_getPayloadSize(void)
{
  return payload_size;
}


void NRF24P_startListening(void)
{
  NRF24P_WriteRegister(CONFIG, NRF24P_ReadRegister(CONFIG) | _BV(PRIM_RX));
  NRF24P_WriteRegister(STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );
  NRF_CE_HIGH();

  if (pipe0_reading_address[0] > 0)		//Restore the pipe0 adddress, if exists
  {
    NRF24P_WriteRegisterArray(RX_ADDR_P0, pipe0_reading_address, addr_width);
  }else
  {
	NRF24P_closeReadingPipe(0);
  }

  if(NRF24P_ReadRegister(FEATURE) & _BV(EN_ACK_PAY)){
	NRF24P_flush_tx();
  }
}

void NRF24P_stopListening(void)
{
  NRF_CE_LOW();

  //_delay_us(txRxDelay);

  if(NRF24P_ReadRegister(FEATURE) & _BV(EN_ACK_PAY)){
    //_delay_us(txRxDelay); //200
	NRF24P_flush_tx();
  }
  NRF24P_WriteRegister(CONFIG, ( NRF24P_ReadRegister(CONFIG) ) & ~_BV(PRIM_RX) );
  NRF24P_WriteRegister(EN_RXADDR,NRF24P_ReadRegister(EN_RXADDR) | _BV(child_pipe_enable[0])); // Enable RX on pipe0

}

/*
 * @brief
 * @param cmd
 */
void NRF24P_PowerDown(void)
{
	NRF_CE_LOW();
    NRF24P_WriteRegister(CONFIG,NRF24P_ReadRegister(CONFIG) & ~_BV(PWR_UP));
}

/*
 * @brief
 * @param cmd
 */
void NRF24P_PowerUp(void)
{
  unsigned char temp = NRF24P_ReadRegister(CONFIG);

	if (!(temp & _BV(PWR_UP)))
	{
		NRF24P_WriteRegister(CONFIG,NRF24P_ReadRegister(CONFIG) | _BV(PWR_UP));
		delay_ms(5);
	}
}


//Similar to the previous write, clears the interrupt flags
bool NRF24P_writeM( const void* buf, uint8_t len, const bool multicast )
{
	//Start Writing
	NRF24P_startWrite(buf,len,multicast);
	while( ! ( NRF24P_GetStatus()  & ( _BV(TX_DS) | _BV(MAX_RT) ))) ;

	NRF_CE_LOW();
	uint8_t status = NRF24P_WriteRegister(STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );
	  //Max retries exceeded
	  if( status & _BV(MAX_RT))
	  {
		NRF24P_flush_tx(); //Only going to be 1 packet int the FIFO at a time using this method, so just flush
		return 0;
	  }
	//TX OK 1 or 0
  return 1;
}

bool NRF24P_write( const void* buf, uint8_t len )
{
	return NRF24P_writeM(buf,len,0);
}
/****************************************************************************/

//For general use, the interrupt flags are not important to clear
bool NRF24P_writeBlocking( const void* buf, uint8_t len, uint32_t timeout )
{
	//Block until the FIFO is NOT full.
	//Keep track of the MAX retries and set auto-retry if seeing failures
	//This way the FIFO will fill up and allow blocking until packets go through
	//The radio will auto-clear everything in the FIFO as long as CE remains high

	uint32_t timer = millis();							       //Get the time that the payload transmission started
	while( ( NRF24P_GetStatus()  & ( _BV(TX_FULL) ))) 		       //Blocking only if FIFO is full. This will loop and block until TX is successful or timeout
	{

		if( NRF24P_GetStatus() & _BV(MAX_RT)){					  //If MAX Retries have been reached
			NRF24P_reUseTX();								  //Set re-transmit and clear the MAX_RT interrupt flag
			if(millis() - timer > timeout){ return 0; }		  //If this payload has exceeded the user-defined timeout, exit and return 0
		}
  	} 	//Start Writing
	NRF24P_startWrite(buf,len,0);							  //Write the payload if a buffer is clear
	return 1;												  //Return 1 to indicate successful transmission
}

void NRF24P_reUseTX(void)
{
		NRF24P_WriteRegister(STATUS,_BV(MAX_RT) );			  //Clear max retry flag
		NRF24P_SendCMD( REUSE_TX_PL );
		NRF_CE_LOW();										  //Re-Transfer packet
		NRF_CE_HIGH();
}


bool NRF24P_writeFastMulticast( const void* buf, uint8_t len, const bool multicast )
{
	//Block until the FIFO is NOT full.
	//Keep track of the MAX retries and set auto-retry if seeing failures
	//Return 0 so the user can control the retrys and set a timer or failure counter if required
	//The radio will auto-clear everything in the FIFO as long as CE remains high
	while( ( NRF24P_GetStatus()  & ( _BV(TX_FULL) ))) {			  //Blocking only if FIFO is full. This will loop and block until TX is successful or fail

		if( NRF24P_GetStatus() & _BV(MAX_RT)){
			//reUseTX();										  //Set re-transmit
			NRF24P_WriteRegister(STATUS,_BV(MAX_RT) );			  //Clear max retry flag
			return 0;										  //Return 0. The previous payload has been retransmitted
															  //From the user perspective, if you get a 0, just keep trying to send the same payload
		}
  	}

	//Start Writing
	NRF24P_startWrite(buf,len,multicast);

	return 1;
}

bool NRF24P_writeFast( const void* buf, uint8_t len )
{
	return NRF24P_writeFastMulticast(buf,len,0);
}

/****************************************************************************/

//Per the documentation, we want to set PTX Mode when not listening. Then all we do is write data and set CE high
//In this mode, if we can keep the FIFO buffers loaded, packets will transmit immediately (no 130us delay)
//Otherwise we enter Standby-II mode, which is still faster than standby mode
//Also, we remove the need to keep writing the config register over and over and delaying for 150 us each time if sending a stream of data

void NRF24P_startFastWrite( const void* buf, uint8_t len, const bool multicast, bool startTx)
{
	NRF24P_write_payload( buf, len,multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD ) ;
	if(startTx)
	{
		NRF_CE_HIGH();
	}
}

/****************************************************************************/

//Added the original startWrite back in so users can still use interrupts, ack payloads, etc
//Allows the library to pass all tests
void NRF24P_startWrite( const void* buf, uint8_t len, const bool multicast )
{
  NRF24P_write_payload( buf, len,multicast? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD ) ;		  // Send the payload
  NRF_CE_HIGH();
  delay_ms(1);
  NRF_CE_LOW();
}


bool NRF24P_rxFifoFull(void)
{
	return NRF24P_ReadRegister(FIFO_STATUS) & _BV(RX_FULL);
}


bool NRF24P_txStandBy(void)
{
	while( ! (NRF24P_ReadRegister(FIFO_STATUS) & _BV(TX_EMPTY)) )
	{
		if( NRF24P_GetStatus() & _BV(MAX_RT))
		{
			NRF24P_WriteRegister(STATUS,_BV(MAX_RT) );
			NRF_CE_LOW();
			NRF24P_flush_tx();    //Non blocking, flush the data
			return 0;
		}
	}

	NRF_CE_LOW();			   //Set STANDBY-I mode
	return 1;
}


bool NRF24P_txStandByT(uint32_t timeout, bool startTx)
{
    if(startTx)
	{
	  NRF24P_stopListening();
	  NRF_CE_HIGH();
	}
	uint32_t start = millis();

	while( ! (NRF24P_ReadRegister(FIFO_STATUS) & _BV(TX_EMPTY)) )
	{
		if( NRF24P_GetStatus() & _BV(MAX_RT))
		{
			NRF24P_WriteRegister(STATUS,_BV(MAX_RT) );
			NRF_CE_LOW();										  //Set re-transmit
				NRF_CE_HIGH();
				if(millis() - start >= timeout){
					NRF_CE_LOW();
					NRF24P_flush_tx();
					return 0;
				}
		}
	}

	NRF_CE_LOW();				   //Set STANDBY-I mode
	return 1;
}

/****************************************************************************/

void NRF24P_maskIRQ(bool tx, bool fail, bool rx)
{
	uint8_t config = NRF24P_ReadRegister(CONFIG);
	/* clear the interrupt flags */
	config &= ~(1 << MASK_MAX_RT | 1 << MASK_TX_DS | 1 << MASK_RX_DR);
	/* set the specified interrupt flags */
	config |= fail << MASK_MAX_RT | tx << MASK_TX_DS | rx << MASK_RX_DR;
	NRF24P_WriteRegister(CONFIG, config);

}

uint8_t NRF24P_getDynamicPayloadSize(void)
{
  uint8_t result = 0;

  SPI_Write(R_RX_PL_WID );
  result = SPI_Write(0xff);

  if(result > 32)
  {
	NRF24P_flush_rx();
	delay_ms(2);
	return 0;
  }
  return result;
}

/****************************************************************************/


bool NRF24P_available(uint8_t* pipe_num)
{
  if (!( NRF24P_ReadRegister(FIFO_STATUS) & _BV(RX_EMPTY) )){

    // If the caller wants the pipe number, include that
    if ( pipe_num ){
	  uint8_t status = NRF24P_GetStatus();
      *pipe_num = ( status >> RX_P_NO ) & 0b111;
  	}
  	return 1;
  }

  return 0;


}

/****************************************************************************/

void NRF24P_read( void* buf, uint8_t len )
{
  // Fetch the payload
  NRF24P_read_payload( buf, len );
  //Clear the two possible interrupt flags with one command
  NRF24P_WriteRegister(STATUS,_BV(RX_DR) | _BV(MAX_RT) | _BV(TX_DS) );

}

/****************************************************************************/

void NRF24P_whatHappened(bool *tx_ok,bool *tx_fail,bool *rx_ready)
{
  // Read the status & reset the status in one easy call
  // Or is that such a good idea?
  uint8_t status = NRF24P_WriteRegister(STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

  *tx_ok = status & _BV(TX_DS);		  // Report to the user what happened
  *tx_fail = status & _BV(MAX_RT);
  *rx_ready = status & _BV(RX_DR);
}

/****************************************************************************/

void NRF24P_openWritingPipe(const unsigned char *address)
{
  NRF24P_WriteRegisterArray(RX_ADDR_P0, (unsigned char *)address, addr_width);
  NRF24P_WriteRegisterArray(TX_ADDR, (unsigned char *)address, addr_width);
  NRF24P_WriteRegister(RX_PW_P0,payload_size);
}

/****************************************************************************/


void NRF24P_setAddressWidth(uint8_t a_width)
{
	if(a_width -= 2)
	{
		NRF24P_WriteRegister(SETUP_AW,a_width%4);
		addr_width = (a_width%4) + 2;
	}

}

void NRF24P_openReadingPipe(uint8_t child, const unsigned char *address)
{
  // If this is pipe 0, cache the address.  This is needed because
  // openWritingPipe() will overwrite the pipe 0 address, so
  // startListening() will have to restore it.
  if (child == 0){
    memcpy(pipe0_reading_address,address,addr_width);
  }
  if (child <= 6)
  {
    // For pipes 2-5, only write the LSB
    if ( child < 2 ){
      NRF24P_WriteRegisterArray(child_pipe[child], (unsigned char*) address, addr_width);
    }else{
      NRF24P_WriteRegisterArray(child_pipe[child], (unsigned char*) address, 1);
	}
    NRF24P_WriteRegister(child_payload_size[child],payload_size);

    // Note it would be more efficient to set all of the bits for all open
    // pipes at once.  However, I thought it would make the calling code
    // more simple to do it this way.
    NRF24P_WriteRegister(EN_RXADDR,NRF24P_ReadRegister(EN_RXADDR) | _BV(child_pipe_enable[child]));
  }
}

/****************************************************************************/

void NRF24P_closeReadingPipe( uint8_t pipe )
{
  NRF24P_WriteRegister(EN_RXADDR,NRF24P_ReadRegister(EN_RXADDR) & ~_BV(child_pipe_enable[pipe]));
}

void NRF24P_toggle_features(void)
{
    // Start SPI transaction
    SPI_Write(ACTIVATE );
    SPI_Write(0x73 );
    NRF_CS_HIGH();

}

void NRF24P_enableDynamicPayloads(void)
{
  // Enable dynamic payload throughout the system

  //toggle_features();
  NRF24P_WriteRegister(FEATURE,NRF24P_ReadRegister(FEATURE) | _BV(EN_DPL) );
  // Enable dynamic payload on all pipes

  // Not sure the use case of only having dynamic payload on certain
  // pipes, so the library does not support it.
  NRF24P_WriteRegister(DYNPD,NRF24P_ReadRegister(DYNPD) | _BV(DPL_P5) | _BV(DPL_P4) | _BV(DPL_P3) | _BV(DPL_P2) | _BV(DPL_P1) | _BV(DPL_P0));
  dynamic_payloads_enabled = true;
}

void NRF24P_enableAckPayload(void)
{
  //toggle_features();
  NRF24P_WriteRegister(FEATURE,NRF24P_ReadRegister(FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL) );		  // enable ack payload and dynamic payload features

  NRF24P_WriteRegister(DYNPD,NRF24P_ReadRegister(DYNPD) | _BV(DPL_P1) | _BV(DPL_P0));	  // Enable dynamic payload on pipes 0 & 1
  dynamic_payloads_enabled = true;
}


void NRF24P_enableDynamicAck(void)
{
    NRF24P_WriteRegister(FEATURE,NRF24P_ReadRegister(FEATURE) | _BV(EN_DYN_ACK) );   // enable dynamic ack features
}

void NRF24P_writeAckPayload(uint8_t pipe, const void* buf, uint8_t len)
{
  uint8_t* current = (uint8_t*)buf;

  uint8_t data_len = rf24_min(len,32);

  // Start SPI transaction
  SPI_Write(W_ACK_PAYLOAD | ( pipe & 0b111 ) );

  while ( data_len-- )
  {
    SPI_Write(*current++);
  }

}


bool NRF24P_isAckPayloadAvailable(void)
{
  return ! (NRF24P_ReadRegister(FIFO_STATUS) & _BV(RX_EMPTY));
}

bool NRF24P_isPVariant(void)
{
  return p_variant ;
}

void NRF24P_setAutoAck(bool enable)
{
  if ( enable ){
    NRF24P_WriteRegister(EN_AA, 0b111111);
  }else{
    NRF24P_WriteRegister(EN_AA, 0);
  }
}

void NRF24P_setAutoAckPipe( uint8_t pipe, bool enable )
{
  if ( pipe <= 6 )
  {
    uint8_t en_aa = NRF24P_ReadRegister( EN_AA ) ;
    if( enable )
    {
      en_aa |= _BV(pipe) ;
    }
    else
    {
      en_aa &= ~_BV(pipe) ;
    }
    NRF24P_WriteRegister( EN_AA, en_aa ) ;
  }
}

/****************************************************************************/

bool NRF24P_testCarrier(void)
{
  return ( NRF24P_ReadRegister(CD) & 1 );
}

bool NRF24P_testRPD(void)
{
  return ( NRF24P_ReadRegister(RPD) & 1 ) ;
}

void NRF24P_setPALevel(uint8_t level)
{
  uint8_t setup = NRF24P_ReadRegister(RF_SETUP) & 0b11111000;

  if(level > 3){  						// If invalid level, go to max PA
	  level = (RF24_PA_MAX << 1) + 1;		// +1 to support the SI24R1 chip extra bit
  }else{
	  level = (level << 1) + 1;	 		// Else set level as requested
  }

  NRF24P_WriteRegister( RF_SETUP, setup |= level ) ;	// Write it to the chip
}

uint8_t NRF24P_getPALevel(void)
{

  return (NRF24P_ReadRegister(RF_SETUP) & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH))) >> 1 ;
}


bool NRF24P_setDataRate(rf24_datarate_e speed)
{
  bool result = false;
  uint8_t setup = NRF24P_ReadRegister(RF_SETUP) ;

  // HIGH and LOW '00' is 1Mbs - our default
  setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH)) ;

    txRxDelay=250;
  if( speed == RF24_250KBPS )
  {
    // Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
    // Making it '10'.
    setup |= _BV( RF_DR_LOW ) ;
    txRxDelay=450;
  }
  else
  {
    // Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
    // Making it '01'
    if ( speed == RF24_2MBPS )
    {
      setup |= _BV(RF_DR_HIGH);
      txRxDelay=190;
    }
  }
  NRF24P_WriteRegister(RF_SETUP,setup);

  // Verify our result
  if ( NRF24P_ReadRegister(RF_SETUP) == setup )
  {
    result = true;
  }
  return result;
}


rf24_datarate_e NRF24P_getDataRate( void )
{
  rf24_datarate_e result ;
  uint8_t dr = NRF24P_ReadRegister(RF_SETUP) & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

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

void NRF24P_setCRCLength(rf24_crclength_e length)
{
  uint8_t config = NRF24P_ReadRegister(CONFIG) & ~( _BV(CRCO) | _BV(EN_CRC)) ;

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
  NRF24P_WriteRegister( CONFIG, config ) ;
}

rf24_crclength_e NRF24P_getCRCLength(void)
{
  rf24_crclength_e result = RF24_CRC_DISABLED;

  uint8_t config = NRF24P_ReadRegister(CONFIG) & ( _BV(CRCO) | _BV(EN_CRC)) ;
  uint8_t AA = NRF24P_ReadRegister(EN_AA);

  if ((config & _BV(EN_CRC )) || AA)
  {
    if ( config & _BV(CRCO) )
      result = RF24_CRC_16;
    else
      result = RF24_CRC_8;
  }

  return result;
}

void NRF24P_disableCRC( void )
{
  uint8_t disable = NRF24P_ReadRegister(CONFIG) & ~_BV(EN_CRC) ;
  NRF24P_WriteRegister( CONFIG, disable );
}

void NRF24P_setRetries(unsigned char delay, unsigned char count)
{
    NRF24P_WriteRegister(SETUP_RETR,(delay&0xf)<<ARD | (count&0xf)<<ARC);
}


/*
 * @brief Clear interrupts in status register
 * @param cmd
 */
void NRF24P_ClearINT(void)
{
    unsigned char status;
    status = NRF24P_ReadRegister(STATUS);
    NRF24P_WriteRegister(STATUS, (status | 0x70));  //Clears pending interrupts
}

/***************************************************************************/

#if defined (MINIMAL)

static const char rf24_datarate_e_str_0[]  = "1MBPS";
static const char rf24_datarate_e_str_1[]  = "2MBPS";
static const char rf24_datarate_e_str_2[]  = "250KBPS";
static const char * const rf24_datarate_e_str_P[]  = {
  rf24_datarate_e_str_0,
  rf24_datarate_e_str_1,
  rf24_datarate_e_str_2,
};
static const char rf24_model_e_str_0[]  = "nRF24L01";
static const char rf24_model_e_str_1[]  = "nRF24L01+";
static const char * const rf24_model_e_str_P[]  = {
  rf24_model_e_str_0,
  rf24_model_e_str_1,
};
static const char rf24_crclength_e_str_0[]  = "Disabled";
static const char rf24_crclength_e_str_1[]  = "8 bits";
static const char rf24_crclength_e_str_2[]  = "16 bits" ;
static const char * const rf24_crclength_e_str_P[]  = {
  rf24_crclength_e_str_0,
  rf24_crclength_e_str_1,
  rf24_crclength_e_str_2,
};
static const char rf24_pa_dbm_e_str_0[]  = "PA_MIN";
static const char rf24_pa_dbm_e_str_1[]  = "PA_LOW";
static const char rf24_pa_dbm_e_str_2[]  = "PA_HIGH";
static const char rf24_pa_dbm_e_str_3[]  = "PA_MAX";
static const char * const rf24_pa_dbm_e_str_P[]  = {
  rf24_pa_dbm_e_str_0,
  rf24_pa_dbm_e_str_1,
  rf24_pa_dbm_e_str_2,
  rf24_pa_dbm_e_str_3,
};


#endif

void NRF24P_print_status(unsigned char status)
{
  printf("STATUS\t\t = 0x%02x RX_DR=%x TX_DS=%x MAX_RT=%x RX_P_NO=%x TX_FULL=%x\r\n",
           status,
           (status & _BV(RX_DR))?1:0,
           (status & _BV(TX_DS))?1:0,
           (status & _BV(MAX_RT))?1:0,
           ((status >> RX_P_NO) & 0b111),
           (status & _BV(TX_FULL))?1:0
          );
}

void NRF24P_print_observe_tx(unsigned char value)
{
  printf("OBSERVE_TX=%02x: POLS_CNT=%x ARC_CNT=%x\r\n",
           value,
           (value >> PLOS_CNT) & 0b1111,
           (value >> ARC_CNT) & 0b1111
          );
}

void NRF24P_print_byte_register(const char* name, unsigned char reg, unsigned char qty)
{
  printf("%s\t =", name);
  while (qty--)
  {
	  printf(" 0x%02x",NRF24P_ReadRegister(reg++));
  }
  printf("\r\n");
}

void NRF24P_print_address_register(const char* name, uint8_t reg, uint8_t qty)
{

    printf("%s\t =",name);

  while (qty--)
  {
    uint8_t buffer[addr_width];
    NRF24P_ReadRegisterArray(reg++,buffer,sizeof buffer);

    printf(" 0x");
    uint8_t* bufptr = buffer + sizeof buffer;
    while( --bufptr >= buffer )
      printf("%02x",*bufptr);
  }

  printf("\r\n");
}

void NRF24P_printDetails(void)
{
  NRF24P_print_status(NRF24P_GetStatus());

  NRF24P_print_address_register(("RX_ADDR_P0-1"),RX_ADDR_P0,2);
  NRF24P_print_byte_register("RX_ADDR_P2-5",RX_ADDR_P2,4);
  NRF24P_print_address_register(("TX_ADDR\t"),TX_ADDR,5);

  NRF24P_print_byte_register("RX_PW_P0-6",RX_PW_P0,6);
  NRF24P_print_byte_register("EN_AA\t",EN_AA,1);
  NRF24P_print_byte_register("EN_RXADDR",EN_RXADDR,1);
  NRF24P_print_byte_register("RF_CH\t",RF_CH,1);
  NRF24P_print_byte_register("RF_SETUP",RF_SETUP,1);
  NRF24P_print_byte_register("CONFIG\t",CONFIG,1);
  NRF24P_print_byte_register("DYNPD/FEATURE",DYNPD,2);

}



