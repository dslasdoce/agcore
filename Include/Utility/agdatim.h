/**
  *****************************************************************************
  * @file    	agdatim.h
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

#ifndef AGDATIM_H_
#define AGDATIM_H_

/* Includes =================================================================*/

#include "agstring.h"
#include "rtc.h"

/* Typedefs =================================================================*/
typedef enum
{
	ADT_EQUAL,
	ADT_NEGATIVE,
	ADT_POSITIVE
}AgdatimDiff;

/* Function Prototypes ======================================================*/
AgdatimDiff agdatimCompare(RTCDateTimeStruct datimeA, RTCDateTimeStruct datimeB);
RTCDateTimeStruct agdatimMinAdjuster(RTCDateTimeStruct datime, int16_t operand);
void agdatimAPIParser(char * datim_ptr, RTCDateTimeStruct * conv_datim);
char * agdatimEncoder(RTCDateTimeStruct dt, DTEncType type);
void agdatimGETRspParser(char * datim_ptr, RTCDateTimeStruct * conv_datim);
void agdatimGpsParser(char * date_ptr, char * time_ptr, RTCDateTimeStruct * conv_datim);
#endif /*agdatim.h*/

/******************************************************************************
 * Revision History
 *	@file	agdatim.h
 ******************************************************************************
  * @version    v2.1.0
  * @date       12/23/2015
  * @author     J.Fajardo
  * @changes
  *****************************************************************************
 * @version	v2.0.1
 * @date	11/13/2015
 * @author	JWF
 * @changes	- initial release
 * 			- function:
 * 				+ agdatimCompare
 * 				+ agdatimMinuteAdder
 * 				+ agdatimAPIParser
 * 				+ agdatimAPIEncoder
 * 				+ agdatimGETRspParser
 ******************************************************************************
 */

