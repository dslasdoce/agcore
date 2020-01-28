/********************************************************************************
  * @file      	daqSensInterfaces.c
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
#include <string.h>
#include <stdlib.h>
#include <deployAssert.h>


#include "agBoardInit.h"
#include "gpio.h"
#include "delay.h"
#include "pexpander.h"
#include "port.h"
#include "sdi12.h"
#include "equations.h"
#include "sigpathSetup.h"
#include "isrMain.h"
#include "daqSensInterface.h"
#include "daqAdcSpi.h"
#include "mspSpi.h"

/* Defines ===================================================================*/
#define TIMEOUT_DAVIS	((uint32_t)60)

/* Global Declarations =======================================================*/

/* External Declarations =====================================================*/
//uint8_t rxbuff_rs485[8];

/* Functions =================================================================*/

/*==============================================================================
 * @brief
 * @param
 * @retval
==============================================================================*/
void daqCounterEn(uint8_t port, bool state)
{
//	if(state)
//	{
//		sigpathCtrEn(port, P_COUNTER);
//		pexpanderWrite(RS485_RE_ENB, LOW, I2CADD_PEXPANDER1);
//		pexpanderWrite(RS485_DE_EN, HIGH, I2CADD_PEXPANDER1);
//		pexpanderWrite(RS485_RES_EN, LOW, I2CADD_PEXPANDER1);
//		pexpanderWrite(RS232_EN, LOW, I2CADD_PEXPANDER1);
//
//		switch(port)
//		{
//			case 1:
//				gpioDigitalWrite(&CTR1_EN, HIGH);
//				break;
//			case 2:
//				gpioDigitalWrite(&CTR2_EN, HIGH);
//				break;
//			case 3:
//				gpioDigitalWrite(&CTR3_EN, HIGH);
//				break;
//			case 4:
//				gpioDigitalWrite(&CTR4_EN, HIGH);
//				break;
//			default:
//				break;
//		}
//	}
//	else
//	{
//		gpioDigitalWrite(&CTR1_EN, LOW);
//		gpioDigitalWrite(&CTR2_EN, LOW);
//		gpioDigitalWrite(&CTR3_EN, LOW);
//		gpioDigitalWrite(&CTR4_EN, LOW);
//
//		pexpanderWrite(RS485_RE_ENB, HIGH, I2CADD_PEXPANDER1);
//		pexpanderWrite(RS485_DE_EN, LOW, I2CADD_PEXPANDER1);
//		pexpanderWrite(RS485_RES_EN, LOW, I2CADD_PEXPANDER1);
//		pexpanderWrite(RS232_EN, LOW, I2CADD_PEXPANDER1);
//	}
//	extiCfgGPIO(port, state);
}

/*==============================================================================
 * @brief
 * @param
 * @retval
==============================================================================*/
void daqRS485EN(uint8_t port, bool state)
{
	if(state)
	{
		gpioDigitalWrite(&RSx_A_EN, HIGH);	
		gpioDigitalWrite(&RSx_B_EN, HIGH);

		
		gpioDigitalWrite(&RS485_RES_EN, LOW);
		gpioDigitalWrite(&RS232_ENB, HIGH);
		gpioDigitalWrite(&RS485_DXEN, LOW);
		gpioDigitalWrite(&RS485_RXENB, LOW);
		gpioDigitalWrite(&RS485_FEN, HIGH);
		
		portRSXSet(port);
	}

	else
	{
		gpioDigitalWrite(&RSx_A_EN, LOW);	
		gpioDigitalWrite(&RSx_B_EN, LOW);
		
		gpioDigitalWrite(&RS485_RES_EN, LOW);
		gpioDigitalWrite(&RS232_ENB, LOW);
		gpioDigitalWrite(&RS485_DXEN, LOW);
		gpioDigitalWrite(&RS485_RXENB, HIGH);
		gpioDigitalWrite(&RS485_FEN, LOW);
	}
}


float daqSensAnalogI(uint8_t port)
{
  	gpioDigitalWrite(&VCON15_EN, HIGH);
  	delayMillis(250);
	mspSetPowerRails(MSPCMD2_PR_420, true);
	delayMillis(250);
	
	AgTechAdcStruct *adc[] = {NULL, &SIG1_0_5_ADC, &SIG2_0_5_ADC, 
							&SIG3_0_5_ADC, &SIG4_0_5_ADC};
	gpioDigitalWrite(&VCON15_EN, LOW);
	mspSetPowerRails(MSPCMD2_PR_420, false);
	delayMillis(250);
	return gpioAnalogRead(adc[port]);
}

float daqSensTherm(uint8_t port)
{
  
  	AgTechAdcStruct *adc[] = {NULL, &THERM1_ADC, &THERM2_ADC, &THERM3_ADC, &THERM4_ADC};
	AgTechIOStruct *sigx_sel_ptr[] = {&SIG1_SEL, &SIG2_SEL, &SIG3_SEL ,&SIG4_SEL};
	
	gpioDigitalWrite(sigx_sel_ptr[port - 1], false);
	delayMillis(500);
	adc_therm_ref = gpioAnalogRead(adc[port]);
	
	gpioDigitalWrite(sigx_sel_ptr[port - 1], true);
	delayMillis(500);
	return gpioAnalogRead(adc[port]);
}
/* ============================================================================
 * @brief	acquires data from analog sensors
 * @param	gpio:	pointer to AgTechIOStruct that indicates the details of
 * 					pin to be used in reading
 * @param	sensordetails_ptr:	pointer to SensorRegistrationStruct that
 * 							    contains driver details of the sensor
 * @param	raw_values:	pointer to the arraw where raw_values are saved
 * @retval	number of acquired data(always 1 for analog)
 * ==========================================================================*/
uint8_t daqSensorAnalogV(uint8_t port, float *raw_values)
{
//	assert_param(IS_INSTANCE_ANALOG(sensordetails_ptr->gpio.group,
//									sensordetails_ptr->gpio.pin));
	float adc_val;
	
	if (adcSpiRead(&adc_val, &AD_CS, port - 1) != SUCCESSFUL)
	  	return 0;
	
	raw_values[0] = adc_val;
	return 1;
}

/* ============================================================================
 * @brief	acquires data from Counter sensors
 * @param	gpio:	pointer to AgTechIOStruct that indicates the details of
 * 					pin to be used in reading
 * @param	sensordetails_ptr:	pointer to SensorRegistrationStruct that
 * 								contains driver details of the sensor
 * @param	raw_values:	pointer to the arraw where raw_values are saved
 * @retval	number of acquired data
 * ==========================================================================*/
uint8_t daqSensorCounter(SensorRegistrationStruct *sensordetails_ptr,
						 float *raw_values)
{
/*Read From MSP430*/
}

/* ============================================================================
 * @brief	acquires data from SDI12 sensors
 * @param	gpio:	pointer to AgTechIOStruct that indicates the details of
 * 					pin to be used in reading
 * @param	sensordetails_ptr:	pointer to SensorRegistrationStruct that
 * 								contains driver details of the sensor
 * @param	raw_values:	pointer to the arraw where raw_values are saved
 * @retval	number of acquired data
 * ==========================================================================*/
uint8_t daqSensorSDI(SensorRegistrationStruct *sensordetails_ptr,
					 float *raw_values)
{
	assert_param(IS_INSTANCE_DIGITAL(sensordetails_ptr->gpio.group,
									sensordetails_ptr->gpio.pin));
	char *cmd_str = strdup(sensordetails_ptr->command);
	char *commands[10];
	float tmp_values[20];
	uint8_t cmd_count = eqStringSplit(commands,cmd_str,",");
	uint8_t i,tmp_i;
	uint8_t datacount = 0;
	uint8_t tmp_count = 0;
	uint8_t ptr_increment = 0;
	for(i=0;i<cmd_count;i++)
	{
		tmp_count = sdiMain(&sensordetails_ptr->gpio, commands[i],tmp_values);
		//free(commands[i]);
		for(tmp_i=0; tmp_i<tmp_count; tmp_i++)
		{
			*(raw_values + ptr_increment++) = tmp_values[tmp_i];
		}
		datacount += tmp_count;
	}
	eqStringSplitFree(commands, cmd_count);
	free(cmd_str);
	gpioMode(&sensordetails_ptr->gpio, MODE_INPUT);
	return datacount;
}

uint8_t daqSensorDavisDTH(SensorRegistrationStruct *sensordetails_ptr,
					 float *raw_values)
{
	assert_param(IS_INSTANCE_DIGITAL(sensordetails_ptr->gpio.group,
									sensordetails_ptr->gpio.pin));
	uint8_t datacount = 0;
	io_dth_sck = &sensordetails_ptr->gpio;
	AgTechIOStruct *io_data[5] = {NULL, &SIG1_ID, &SIG2_ID, &SIG3_ID, &SIG4_ID};
	
	/*temporarily disable BLE IRQ because it is on the same line as DIG2(sck for dig2)*/
	gpioDettachInt(&BLE_IRQ);
	delayMillis(2000);
	//gpioDigitalWrite(&DLS_VS, HIGH);
	gpioDigitalWrite(&DLS_OE, LOW);
	gpioDigitalWrite(&DLS_DIR, HIGH);
	gpioMode(io_dth_sck, MODE_OUTPUT);
	
//	HAL_NVIC_EnableIRQ(TIM4_IRQn);
//	while(1);
	if (davisDTHReadTemp(raw_values++, io_data[sensordetails_ptr->port]
						 , io_dth_sck)  == SUCCESSFUL)
		datacount++;
	
	if (davisDTHReadRelHum(raw_values, io_data[sensordetails_ptr->port], 
						   io_dth_sck)  == SUCCESSFUL)
		datacount++;

	gpioDigitalWrite(&DLS_OE, HIGH);
	gpioDigitalWrite(&DLS_DIR, LOW);
	gpioDigitalWrite(&DLS_VS, LOW);
	gpioMode(io_dth_sck, MODE_INPUT);
	gpioMode(io_data[sensordetails_ptr->port], MODE_INPUT);
	delayMillis(100);
	gpioDettachInt(io_dth_sck);
	gpioAttachInt(&BLE_IRQ, IT_MODE_FALLING);
	return datacount;
}


/******************************************************************************
  * Revision History
  *	@file      	daqSensInterface.c
  *****************************************************************************
  * @version	v2.2.0
  * @date		04/08/16
  * @author		D. Lasdoce
  * @changes	- created file
  *****************************************************************************
  */
