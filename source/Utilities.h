

/**
 * @file
 * @author D.Qendri
 * @date 30 May 2016
 * @brief Utilities library header.   
 */
 
#ifndef __UTILITIES_H__
#define __UTILITIES_H__

#include <stdio.h>
#include <stdint.h>

#include "board.h"
#include "clock_config.h"


void delay_ms(unsigned int ms);

uint32_t millis(void);
void 	 millisDelay(uint32_t DelayTime);

#endif
