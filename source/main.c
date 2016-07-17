/*
 * Copyright (c) 2013 - 2016, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_adc16.h"


#include "NRF24L01.h"
#include "NRF24L01P_MemMap.h"
#include "SPI.h"
#include "Utilities.h"

uint8_t buffer = 0;

// Assign a unique identifier for this node, 0 or 1.
bool radioNumber = 1;
// Radio pipe addresses for the 2 nodes to communicate.
const uint8_t addresses[][6] = {"1Node","2Node"};

bool role_ping_out = 1, role_pong_back = 0, role = 0;
uint8_t weightScalevalue = 0;

static char g_StrNewline[] = "\r\n";

/**************************************************************************/
#define DEMO_ADC16_BASE ADC0
#define DEMO_ADC16_CHANNEL_GROUP 0U
#define DEMO_ADC16_USER_CHANNEL 23U


adc16_config_t adc16ConfigStruct;
adc16_channel_config_t adc16ChannelConfigStruct;

/**************************************************************************/

void ADCConfiguration(void);

/*!
 * @brief Application entry point.
 */
int main(void) {
  /* Init board hardware. */
  BOARD_InitPins();
  BOARD_BootClockRUN();
  BOARD_InitDebugConsole();

  ADCConfiguration();
  ADC16_SetChannelConfig(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);


  SPI_Initialize();				//Initialize SPI peripheral
  NRF24P_SetupPins();
  NRF24P_Init();

//  delay_ms(2000);
//  PRINTF("Status: 0x%x. \r\n", NRF24P_ReadRegister(STATUS));
//  delay_ms(2000);
//  NRF24P_WriteRegister(STATUS, 0x01);
//  PRINTF("Status: 0x%x. \r\n", NRF24P_ReadRegister(STATUS));
//
//  delay_ms(2000);
//  PRINTF("CONFIG: 0x%x. \r\n", NRF24P_ReadRegister(CONFIG));
//  PRINTF("EN_AA: 0x%x. \r\n", NRF24P_ReadRegister(EN_AA));
//  PRINTF("SETUP_AW: 0x%x. \r\n", NRF24P_ReadRegister(SETUP_AW));
//  PRINTF("SETUP_RETR: 0x%x. \r\n", NRF24P_ReadRegister(SETUP_RETR));
//  PRINTF("RF_CH: 0x%x. \r\n", NRF24P_ReadRegister(RF_CH));
//  delay_ms(2000);
//  PRINTF("RF_SETUP: 0x%x. \r\n", NRF24P_ReadRegister(RF_SETUP));
//  PRINTF("OBSERVE_TX: 0x%x. \r\n", NRF24P_ReadRegister(OBSERVE_TX));
//  PRINTF("CD: 0x%x. \r\n", NRF24P_ReadRegister(CD));
//  PRINTF("FIFO_STATUS: 0x%x. \r\n", NRF24P_ReadRegister(FIFO_STATUS));
//  PRINTF("DYNPD: 0x%x. \r\n", NRF24P_ReadRegister(DYNPD));


  PRINTF("\n ************ Role Setup ***********\r\n");

  PRINTF("Choose a role: Enter 0 for pong_back, 1 for ping_out (CTRL+C to exit)\r\n>");

  buffer = GETCHAR();
  PUTCHAR(buffer);
  PRINTF(g_StrNewline);

  if(buffer == '0'){
	  PRINTF("Role: Pong Back, awaiting transmission.\r\n");
	  role = role_pong_back;
  }else{
	  PRINTF("Role: Ping Out, starting transmission.\r\n");
	  role = role_ping_out;
  }


  // This opens two pipes for these two nodes to communicate back and forth.
	if ( !radioNumber )    {
	  NRF24P_openWritingPipe(addresses[0]);
	  NRF24P_openReadingPipe(1,addresses[1]);
	}else{
	  NRF24P_openWritingPipe(addresses[1]);
	  NRF24P_openReadingPipe(1,addresses[0]);
	}
	NRF24P_startListening();
	NRF24P_writeAckPayload(1,&weightScalevalue,1);

	delay_ms(100);


	NRF24P_printDetails();

      while(1)
      {
  	/****************** Ping Out Role ***************************/

  	  if (role == role_ping_out){                               // Radio is in ping mode

  	  uint8_t gotByte;                                        // Initialize a variable for the incoming response

  		NRF24P_stopListening();                                  // First, stop listening so we can talk.
  		PRINTF("Weight value %d as payload. ",weightScalevalue);          // Use a simple byte counter as payload
  		unsigned long time = millis();                          // Record the current microsecond count

  		if ( NRF24P_write(&weightScalevalue,1) ){                         // Send the counter variable to the other radio
  			if(!NRF24P_available(NULL)){                             // If nothing in the buffer, we got an ack but it is blank
  				PRINTF("Got blank response. round-trip delay: %lu ms\n\r",millis()-time);
  			}else{
  				while(NRF24P_available(NULL) ){                      // If an ack with payload was received
  					NRF24P_read( &gotByte, 1 );                  // Read it, and display the response time
  					PRINTF("Got response %d, round-trip delay: %lu ms\n\r",gotByte,millis()-time);
  				}


			while (0U == (kADC16_ChannelConversionDoneFlag &  ADC16_GetChannelStatusFlags(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP)))
			{
			}
			weightScalevalue = ADC16_GetChannelConversionValue(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP);
			PRINTF("ADC Value: %d\r\n", weightScalevalue );

  			}

  		}else
  		{
  			PRINTF("Sending failed.\n\r"); }          // If no ack response, sending failed

  		delay_ms(1000);  // Try again later
  	  }

  	/****************** Pong Back Role ***************************/

  	  if ( role == role_pong_back ) {
  		uint8_t pipeNo, gotByte;           		        // Declare variables for the pipe and the byte received
  		if( NRF24P_available(&pipeNo)){               	// Read all available payloads
  		  NRF24P_read( &gotByte, 1 );
  														// Since this is a call-response. Respond directly with an ack payload.
  		  gotByte += 1;  								// Ack payloads are much more efficient than switching to transmit mode to respond to a call
  		  NRF24P_writeAckPayload(pipeNo,&gotByte, 1 );   // This can be commented out to send empty payloads.
  		  PRINTF("Loaded next response %d \n\r", gotByte);
  		  delay_ms(900);  							  //Expects a payload every second
  	   }
  	 }


     }

  	SPI_Close();

      return 0;
  }





void SwitchMode(void)
{
	buffer = GETCHAR();

  if ( buffer == 'T' && role == role_pong_back ){
    PRINTF("*** CHANGING TO TRANSMIT ROLE -- PRESS 'R' TO SWITCH BACK");
    role = role_ping_out;  // Become the primary transmitter (ping out)
    weightScalevalue = 1;
  }

  if ( buffer == 'R' && role == role_ping_out ){
     PRINTF("*** CHANGING TO RECEIVE ROLE -- PRESS 'T' TO SWITCH BACK");
     role = role_pong_back; // Become the primary receiver (pong back)
     NRF24P_startListening();
     counter = 1;
     NRF24P_writeAckPayload(1,&weightScalevalue,1);
  }
}


void ADCConfiguration(void)
{
	ADC16_GetDefaultConfig(&adc16ConfigStruct);
	ADC16_Init(DEMO_ADC16_BASE, &adc16ConfigStruct);
	ADC16_EnableHardwareTrigger(DEMO_ADC16_BASE, false); /* Make sure the software trigger is used. */
#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
	if (kStatus_Success == ADC16_DoAutoCalibration(DEMO_ADC16_BASE))
	{
		PRINTF("ADC16_DoAutoCalibration() Done.\r\n");
	}
	else
	{
		PRINTF("ADC16_DoAutoCalibration() Failed.\r\n");
	}
#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */
	PRINTF("Press any key to get user channel's ADC value ...\r\n");

	adc16ChannelConfigStruct.channelNumber = DEMO_ADC16_USER_CHANNEL;
	adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false;
#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
	adc16ChannelConfigStruct.enableDifferentialConversion = false;
#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */
}
