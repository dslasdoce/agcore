/**
  ******************************************************************************
  * @file      	sigpathSetup.c
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
#include <deployAssert.h>
#include "sigpathSetup.h"
#include "isrMain.h"
#include "gpio.h"
#include "i2c.h"
#include "agBoardInit.h"
#include "pexpander.h"

/* Typedefs ==================================================================*/
typedef struct
{
	uint8_t port;
	AgTechIOStruct *pin_an_sel_s0;
	AgTechIOStruct *pin_an_sel_s1;
	AgTechIOStruct *pin_sigsel;
}PathSelIOStruct;

/* Functions =================================================================*/

/*==============================================================================
 * @brief
 * @param
 * @retval
==============================================================================*/
void sigpathAnalogEn(uint8_t port, bool enable)
{
//	gpioDigitalWrite(&SIG1_AD_EN, LOW);
//	gpioDigitalWrite(&SIG2_AD_EN, LOW);
//	gpioDigitalWrite(&SIG3_AD_EN, LOW);
//	gpioDigitalWrite(&SIG4_AD_EN, LOW);
	if(enable)
	{
		switch(port)
		{
			case 1:
				//gpioDigitalWrite(&SIG1_AD_EN, HIGH);
				break;
			case 2:
				//gpioDigitalWrite(&SIG2_AD_EN, HIGH);
				break;
			case 3:
				//gpioDigitalWrite(&SIG3_AD_EN, HIGH);
				break;
			case 4:
				//gpioDigitalWrite(&SIG4_AD_EN, HIGH);
				break;
			default:
				assert(0);
		}
	}
}


/* ============================================================================
 * @brief	select between data or counter line for each port
 * @param	port:	port to be configured
 * @param	code_interface:	sensor interface type
 * ==========================================================================*/
void sigpathCtrEn(uint8_t port,  InterfaceCodeEnum code_interface)
{
}

/* ============================================================================
 * @brief 	set the path of signal from sensor to MCU
 * @param	port: port number to be configured
 * @param	rescode: code of fixed resistor to be used based on analog mux
 * @param	intcode: code of interface
 * @param	dpot_res: potentiometer resistance
 * ==========================================================================*/
InterfaceStatusEnum sigpathSet(uint8_t port, int code_analogmux,
							   InterfaceCodeEnum code_interface,
							   int resistance_dpot)
{
	assert_param(IS_PORT(port));
	assert_param(IS_CODE_AMUX(code_analogmux));
	//assert_param(IS_INTERFACE(code_interface));
	assert_param(IS_DPOT(resistance_dpot));
	InterfaceStatusEnum status = INTERFACE_OK;

	/*set interface*/
	sigpathSetInterface(port, code_interface);
	return INTERFACE_OK;
}

/*******************************************************************************
  * Revision History
  *	@file      	sigpathSetup.c
  ******************************************************************************
  * @version    v2.2.0
  * @date       04/08/16
  * @author     D.Lasdoce
  * @changes	- created file
  ******************************************************************************
  */