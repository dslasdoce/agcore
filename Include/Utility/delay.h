/********************************************************************************
  * @file      	delay.h
  * @author		Hardware Team
  * @version	v2.0.1
  * @date		09/14/2015
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


#ifndef __DELAY_H_
#define __DELAY_H_

/* Includes =================================================================*/
#include <stdint.h>

/* Defines ==================================================================*/
#define TIMER_FREQUENCY_HZ (1000u)
//extern uint32_t multiplier;
/* Function Prototypes ======================================================*/
//void timerTick (void);
void delayInit(void);
void delayMicros(uint32_t micros);
void delayMillis(uint32_t millis);
uint32_t delayTimeDiff(uint32_t t_start, uint32_t t_end);
void delayMicrosTest(volatile uint32_t Delay);

extern uint32_t multiplier;
extern uint32_t multiplier_micros;
extern uint32_t microsecs;
extern uint32_t mil;

#endif

/******************************************************************************
  * Revision History
  *	@file      	delay.h
  *****************************************************************************
  * @version
  * @date
  * @author
  * @changes	- created file
  *****************************************************************************
  */
