/**
 * @file 
 * @author D.Qendri
 * @date 30 May 2016
 * @brief SPI library.   
 */
 
#include "SPI.h"
#include <stdio.h>
#include <stdint.h>


/**
 *@brief Initializes SPI HAL.
 *@return none
 */
void SPI_Initialize(void)
{	
    uint32_t srcClock_Hz;
	dspi_master_config_t masterConfig;

	/* Master config */
	masterConfig.whichCtar = kDSPI_Ctar0;
	masterConfig.ctarConfig.baudRate = TRANSFER_BAUDRATE;
	masterConfig.ctarConfig.bitsPerFrame = 8U;
	masterConfig.ctarConfig.cpol = kDSPI_ClockPolarityActiveHigh;
	masterConfig.ctarConfig.cpha = kDSPI_ClockPhaseFirstEdge;
	masterConfig.ctarConfig.direction = kDSPI_MsbFirst;
	masterConfig.ctarConfig.pcsToSckDelayInNanoSec = 1000000000/TRANSFER_BAUDRATE;
	masterConfig.ctarConfig.lastSckToPcsDelayInNanoSec = 1000000000/TRANSFER_BAUDRATE;
	masterConfig.ctarConfig.betweenTransferDelayInNanoSec = 1000000000/TRANSFER_BAUDRATE;

	masterConfig.whichPcs = DSPI_MASTER_PCS_FOR_TRANSFER;
	masterConfig.pcsActiveHighOrLow = kDSPI_PcsActiveLow;

	masterConfig.enableContinuousSCK = false;
	masterConfig.enableRxFifoOverWrite = false;
	masterConfig.enableModifiedTimingFormat = false;
	masterConfig.samplePoint = kDSPI_SckToSin0Clock;

	srcClock_Hz = CLOCK_GetFreq(DSPI_MASTER_CLK_SRC);
	DSPI_MasterInit(DSPI_MASTER_BASEADDR, &masterConfig, srcClock_Hz);
}

/**
 *@brief Writes a byte of data to the SPI bus
 *@param data Byte data to write on bus
 *@return result Result of data written on SPI bus.
 */
unsigned char SPI_Write(unsigned char data)
{


}

/**
 *@brief Reads a byte of data from the SPI bus
 *@return result Result of data written on SPI bus.
 */
unsigned char SPI_Read(void)
{



}

/**
 *@brief Writes an array of bytes to the SPI bus
 *@param buff Pointer to array of char data.
 *@param length Length of data array.
 *@return none
 */
void SPI_Write_Array(char* buff, unsigned int length)
{

}

/**
 *@brief Reads an array of data from the SPI bus
 *@param sArray Buffer of data to send  
 *@param rArray Buffer of data to send  
 *@param length Length of data buffer
 *@return none
 */
void SPI_Read_Array(char* sArray, char* rArray, char length)
{

	
}

/**
  * @brief  SPI error treatment function.
  */
void SPIx_Error (void)
{
  /* De-initialize the SPI communication BUS */

  /* Re-Initiaize the SPI communication BUS */
  SPI_Initialize();
}

/**
 *@brief Close SPI bus, should not be called if another SPI device is being used
 *@return none
 */
void SPI_Close(void)
{
	/* De-initialize the SPI communication BUS */

}
