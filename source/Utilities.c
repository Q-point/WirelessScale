/**
 * @file Display/Example1/Utilities.c
 * @author D.Qendri
 * @date 30 May 2015
 * @brief Utilities library.   
 */
 
 
#include "Utilities.h"

/// \defgroup utilities Utilities
/// These are common helper functions that are used to read and write GPIO pins and for timing delays.
/// @{

/**
 * @brief Delay in ms.
 * @param ms Delay in milliseconds
 * @return none
 */	
void delay_ms(unsigned int ms)
{

    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;                      /* use core clock */
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;                       /* disable interrupt */
    SysTick->LOAD = ms * ((CLOCK_GetFreq(kCLOCK_CoreSysClk)) / 1000U); /* n ms */
    SysTick->VAL = 0U;                                                /* clear COUNTFLAG */

    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

    /* wait for timeout */
    while (0U == (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk))
    {
    }

    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}


/**
  * @brief	Returns the current value of the milliscounter
  * @param	None
  * @retval	The milliscounter
  */
uint32_t millis(void)
{
	return 0;
}


/**
  * @brief	Delay for the specified them
  * @param	DelayTime: Time to delay
  * @retval	none
  */
void millisDelay(uint32_t DelayTime)
{
	static uint32_t delayTimer = 0;
	delayTimer = millis();
	while (millis() - delayTimer < DelayTime);
}







/// @}
