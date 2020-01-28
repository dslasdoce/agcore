
#include	<deployAssert.h>
#include "agBoardInit.h"
#include "delay.h"
/********************************************************************************
 * @file      	:	utilPwrMonitor.c
 * @author		:	Hardware Team
 * @version		:	v2.2.0
 * @date		:	04/08/16
 * @brief		:	Module of Post/Get to cloud server
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

#define		PWRMNTR_CMD_WR			(0x00)
#define		PWRMNTR_CMD_RD			(0x01)

#define		PWRMNTR_ADD_OPCTL		(0xF0)
#define		PWRMNTR_ADD_V			(0xA0)
#define		PWRMNTR_ADD_T			(0xA2)

#define			PWRMNTR_MODE_CONT_ON	(0x08)
   #define		PWRMNTR_MODE_SHDN		(0x01)
#define			PWRMNTR_MODE_CONT_OFF	(0x00)

#define		PWRMNTR_VRES			((float)0.002)
#define		PWRMNTR_TEMPCALC(temp)	((float)0.204*(float)temp + (float)5.5)

void utilPwrModeCont(AgTechIOStruct *cs, bool mode)
{
	uint8_t pwrmntr_status = 0;
	uint8_t target_register = PWRMNTR_ADD_OPCTL;
	uint8_t operation_type = PWRMNTR_CMD_WR;
	uint8_t pwrmntr_new_stat = 0;
	delayMillis(100);
	
	if(mode)
		pwrmntr_status = PWRMNTR_MODE_CONT_ON;
	else
	  	pwrmntr_status = PWRMNTR_MODE_SHDN;
	
	gpioDigitalWrite(cs, LOW);
	delayMillis(200);
	if(HAL_SPI_Transmit(&SPIPwrMonitorHandle, (uint8_t *)&operation_type,
						1, 1000) != HAL_OK)
		assert(0);
	if(HAL_SPI_Transmit(&SPIPwrMonitorHandle, (uint8_t *)&target_register,
						1, 1000) != HAL_OK)
		assert(0);
	if(HAL_SPI_Transmit(&SPIPwrMonitorHandle, (uint8_t *)&pwrmntr_status,
						1, 1000) != HAL_OK)
		assert(0);
	gpioDigitalWrite(cs, HIGH);
	
	delayMillis(100);
	if(mode)
	{
		gpioDigitalWrite(cs, LOW);
		operation_type = PWRMNTR_CMD_RD;
		if(HAL_SPI_Transmit(&SPIPwrMonitorHandle, (uint8_t *)&operation_type,
								1, 1000) != HAL_OK)
				assert(0);
		if(HAL_SPI_Transmit(&SPIPwrMonitorHandle, (uint8_t *)&target_register,
							1, 1000) != HAL_OK)
			assert(0);
		if(HAL_SPI_Receive(&SPIPwrMonitorHandle, (uint8_t *)(&pwrmntr_new_stat),
						2, 100) != HAL_OK)
			assert(0);
		
		if(pwrmntr_new_stat != pwrmntr_status)
			assert(0);
		gpioDigitalWrite(cs, HIGH);
		delayMillis(200);
	}
}

float utilPwrGetVoltage(AgTechIOStruct *cs)
{
	uint8_t vdata[2];
	uint8_t target_register = PWRMNTR_ADD_V;
	uint8_t operation_type = PWRMNTR_CMD_RD;
	int16_t voltage_byte = 0;
	gpioDigitalWrite(cs, LOW);
	delayMillis(10);
	if(HAL_SPI_Transmit(&SPIPwrMonitorHandle, (uint8_t *)&operation_type,
							1, 1000) != HAL_OK)
			assert(0);
	if(HAL_SPI_Transmit(&SPIPwrMonitorHandle, (uint8_t *)&target_register,
						1, 1000) != HAL_OK)
		assert(0);
	if(HAL_SPI_Receive(&SPIPwrMonitorHandle, (uint8_t *)(vdata),
					2, 100) != HAL_OK)
		assert(0);
	gpioDigitalWrite(cs, HIGH);
	
	voltage_byte = (vdata[0]<<8) | vdata[1];
	return PWRMNTR_VRES*voltage_byte;
}

float utilPwrGetTemp(AgTechIOStruct *cs)
{
	uint8_t tdata[2];
	uint8_t target_register = PWRMNTR_ADD_T;
	uint8_t operation_type = PWRMNTR_CMD_RD;
	int16_t tempval = 0;
	gpioDigitalWrite(cs, LOW);
	delayMillis(10);
	if(HAL_SPI_Transmit(&SPIPwrMonitorHandle, (uint8_t *)&operation_type,
							1, 1000) != HAL_OK)
			assert(0);
	if(HAL_SPI_Transmit(&SPIPwrMonitorHandle, (uint8_t *)&target_register,
						1, 1000) != HAL_OK)
		assert(0);
	if(HAL_SPI_Receive(&SPIPwrMonitorHandle, (uint8_t *)(tdata),
					2, 100) != HAL_OK)
		assert(0);
	gpioDigitalWrite(cs, HIGH);
	
	tempval = (tdata[0]<<8 | tdata[1]);
	return PWRMNTR_TEMPCALC(tempval);
}
