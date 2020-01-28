/*0
  *******************************************************************************
  * @file      	delay.c
  * @author     Hardware Team
  * @version    v2.1.0
  * @date       12/23/2015
  * @brief
  ******************************************************************************
  * @attention
  *
  * COPYRIGHT(c) 2015 AgTech Labs, Inc.
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of AgTech Labs, Inc. nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */


/* Includes =================================================================*/
#include <limits.h>
#include <stdint.h>
	
#include "delay.h"
#include "stm32f7xx.h"
#include "agBoardInit.h"
/* Global Declarations=======================================================*/
uint32_t multiplier;
//uint32_t multiplier_micros;
uint32_t microsecs;
uint32_t mil;

/* Functions ================================================================*/


//#pragma optimize=none
/* ============================================================================
 * @brief	initializes the system timer
 * currently increments the uwtick variable every 1 milliseconds
 * ==========================================================================*/
void delayInit(void)
{
  	multiplier = 0;
	uint32_t usec_res = 1000000;
	
	float ticks_perwhile = 1;
	uint32_t divisor = ticks_perwhile * usec_res;
    multiplier = (uint32_t)SystemCoreClock / (uint32_t)divisor; //original = 4000
}

//#pragma optimize=none
/* ============================================================================
 * @brief	halt the program for a time period in microseconds
 * @param	micros: amount of time in microsecods
 * ==========================================================================*/
void delayMicros(uint32_t micros)
{
  	microsecs =  micros * multiplier;
    while (--microsecs);
}

//#pragma optimize=none
/* ============================================================================
 * @brief	halt the program for a time period in microseconds
 * @param	millis: amount of time in milliseconds
 * ==========================================================================*/
void delayMillis(uint32_t millis)
{
  	mil = millis * multiplier * 1000;
    while (--mil);
}

/* ============================================================================
 * @brief 	computes the time between the 2 parameters
 * @param	t_start: number of ticks when the timing started
 * @param	t_end:	 number of ticks when the timing ended
 * @retval	ellapsed time in milliseconds
 * ==========================================================================*/
uint32_t delayTimeDiff(uint32_t t_start, uint32_t t_end)
{
	uint32_t time_diff;
	if(t_start > t_end)
		time_diff = t_end + (ULONG_MAX - t_start) + 1;
	else
		time_diff = t_end - t_start;
	return time_diff;
}


/******************************************************************************
  * Revision History
  *	@file      	delay.c
  *****************************************************************************
  * @version    v2.1.0
  * @date       12/23/2015
  * @author     D.Lasdoce
  * @changes
  ********************************************************************************
  * @version
  * @date
  * @author
  * @changes	- created file
  *****************************************************************************
  */
