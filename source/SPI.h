/**
 * @file SPI.h
 * @author D.Qendri
 * @date 30 May 2016
 * @brief SPI library header.   
 */
 
#ifndef __SPI_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_dspi.h"
#include "fsl_gpio.h"

#include "board.h"

#include "pin_mux.h"
#include <stdbool.h>
#include <stdint.h>


#define DSPI_MASTER_BASEADDR 		 		 SPI1
#define DSPI_MASTER_CLK_SRC 				 DSPI1_CLK_SRC
#define DSPI_MASTER_PCS_FOR_INIT 	 		 kDSPI_Pcs1
#define DSPI_MASTER_PCS_FOR_TRANSFER 		 kDSPI_MasterPcs1

#define TRANSFER_SIZE 						 256U        /*! Transfer dataSize */
#define TRANSFER_BAUDRATE 					 100000		//500000U 	 /*! Transfer baudrate - 500k */



void            SPI_Initialize(void);
unsigned char   SPI_Write(unsigned char data);
void 			SPI_Write_Array(char* DATA, unsigned int length);
unsigned char 	SPI_Read(void);
void 			SPI_Read_Array(char* sArray, char* rArray, char length);
void 			SPI_Close(void);

void 			SPIx_Error (void);

#endif
