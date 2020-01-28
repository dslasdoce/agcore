/********************************************************************************
  * @file      	daq.h
  * @author		Hardware Team
  * @version	v2.2.0
  * @date		04/08/16
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

#ifndef __DAQSENSINTERFACE_H_
#define __DAQSENSINTERFACE_H_

#include "gpio.h"
#include "agBoardInit.h"
#include "equations.h"

#define IS_INSTANCE_ANALOG(GPIOx, PIN) 		(((GPIOx == GPIOF)\
												|| (GPIOx == GPIOA) \
												|| (GPIOx == GPIOB)) \
												&& ((PIN <= 15) ))
#define IS_INSTANCE_DIGITAL(GPIOx, PIN) 	((GPIOx == GPIOF) \
												&& ((PIN == 11) || (PIN == 12) \
												|| (PIN == 13) || (PIN == 14)) )

/* External Declarations =====================================================*/
extern uint8_t rxbuff_rs485[8];

/* Function Prototypes =======================================================*/
void daqCounterEn(uint8_t port, bool state);
void daqRS485EN(uint8_t port, bool state);
//uint8_t daqSensorAnalog(SensorRegistrationStruct *sensordetails_ptr,
//						float *raw_values, bool read_subgpio);
uint8_t daqSensorCounter(SensorRegistrationStruct *sensordetails_ptr,
						 float *raw_values);
uint8_t daqSensorSDI(SensorRegistrationStruct *sensordetails_ptr,
					 float *raw_values);
float daqSensAnalogI(uint8_t port);
uint8_t daqSensorAnalogV(uint8_t port, float *raw_values);
uint8_t daqSensorDavisDTH(SensorRegistrationStruct *sensordetails_ptr,
					 float *raw_values);
float daqSensTherm(uint8_t port);


#endif

/*******************************************************************************
  * Revision History
  *	@file      	daqSensInterface.h
  ******************************************************************************
  * @version	v2.2.0
  * @date		04/08/16
  * @author		D. Lasdoce
  * @changes	- created file
  ******************************************************************************
  */
