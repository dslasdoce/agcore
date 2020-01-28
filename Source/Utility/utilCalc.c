/**
  ******************************************************************************
  * @file      	utilCalc.c
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

/* Includes ==================================================================*/
#include <math.h>
#include <stdarg.h>
#include <stdint.h>

/* Defines ===================================================================*/


/* Functions =================================================================*/

/*==============================================================================
 * @brief
 * @param
 * @retval
==============================================================================*/
float utilCalcPercentDiff(float value_1, float value_2)
{
	float percent_diff = 0;
	percent_diff = fabs( (value_1 - value_2)/(value_1 + 0.000001) )*100;
	return percent_diff;
}

/*==============================================================================
 * @brief
 * @param
 * @retval
==============================================================================*/
float utilCalcMeanValue(int count, ...)
{
	float sum = 0;
	uint8_t i;
	va_list valist;
	va_start(valist, count);
	for(i = 0; i<count; i++)
	{
		sum += (float)va_arg(valist, double);
	}
	va_end(valist);
	return sum/count;
}

/*==============================================================================
 * @brief
 * @param
 * @retval
==============================================================================*/
float utilCalcStandardDev(float value_1, float value_2, float value_3)
{
	//uint8_t spike_code;
	float mean_value = utilCalcMeanValue(3, value_1, value_2, value_3);
	float variance = (pow(mean_value - value_1, 2) +
					  pow(mean_value - value_2, 2) +
					  pow(mean_value - value_3, 2))/3;
	return pow(variance, 0.5);
}

float utilCalcRoundDec(float value, uint8_t dec_points)
{
	return roundf(value*pow(10, dec_points))/pow(10, dec_points);
}
/******************************************************************************
  * Revision History
  *	@file      	utilCalc.c
  ******************************************************************************
  * @version    v2.2.
  * @date       04/08/16
  * @author     MI Lorica
  * @changes	created file
  ******************************************************************************
  */