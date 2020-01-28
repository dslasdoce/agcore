/********************************************************************************
  * @file      	utilCalc.h
  * @author		Hardware Team
  * @version	v2.2.0
  * @date		04/08/16
  * @brief		
  ******************************************************************************
  * @attention
  *
  * COPYRIGHT(c) 2016 AgTech Labs, Inc.
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

#ifndef __UTILCALC_H_
#define __UTILCALC_H_

/* Includes ==================================================================*/
#include <math.h>

/* Defines ===================================================================*/
#define	THREE_DEC_PLACES			(uint32_t)1000
#define FLOAT_TO_INT(val, ptr)    	((uint32_t)(fabs(modff(val, ptr)\
										* THREE_DEC_PLACES)))

/* Function Prototypes =======================================================*/

float utilCalcPercentDiff(float value_1, float value_2);
float utilCalcMeanValue(int count, ...);
float utilCalcStandardDev(float value_1, float value_2, float value_3);
float utilCalcRoundDec(float value, uint8_t dec_points);

#endif /* __UTILCALC_H_ */

/******************************************************************************
  * Revision History
  *	@file      	utilCalc.h
  *****************************************************************************
  * @version	v2.2.0
  * @date		04/08/16
  * @author		MI Lorica
  * @changes	created file
  *****************************************************************************
  */