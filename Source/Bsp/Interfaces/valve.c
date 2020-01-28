/********************************************************************************
  * @file      	valve.c
  * @author		Hardware Team
  * @version	v2.0.2
  * @date		09/21/2015
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


/* Includes ------------------------------------------------------------------*/
#include <agBoardInit.h>
#include <stdio.h>
#include <gpio.h>
#include <valve.h>
#include <deployAssert.h>
#include "delay.h"

ValveTrigPins pins_t[] = {
		{&VALVE_TRIG1_A, &VALVE_TRIG1_B},
		{&VALVE_TRIG2_A, &VALVE_TRIG2_B}
		};
void valveCtrlTypeRP(ValveTrigPins *pins, ValveStateEnum state);
/*
 * @brief 	put all the valve output to high impedance state
 */
static void valveReset(void)
{
	gpioDigitalWrite(&VALVE_TRIG1_A, LOW);
	gpioDigitalWrite(&VALVE_TRIG1_B, LOW);
	gpioDigitalWrite(&VALVE_TRIG2_A, LOW);
	gpioDigitalWrite(&VALVE_TRIG2_B, LOW);
}

/*
 * @brief	control scheme function for reverse polarity control
 * @param	pins: pointer to ValveTrigPins table
 * @para	state: defines wether to close or open the valve
 */
void valveCtrlTypeRP(ValveTrigPins *pins, ValveStateEnum state)
{
	if(state == VALVE_OPEN)
	{
		gpioDigitalWrite(pins->trigger_a, HIGH);
		gpioDigitalWrite(pins->trigger_b, LOW);
	}
	else
	{
		gpioDigitalWrite(pins->trigger_a, LOW);
		gpioDigitalWrite(pins->trigger_b, HIGH);
	}
}

/*
 * @brief	control scheme function for positive excitation control
 * @param	pins: pointer to ValveTrigPins table
 * @para	state: defines wether to close or open the valve
 */
static void valveCtrlTypePE(ValveTrigPins *pins, ValveStateEnum state)
{
	if(state == VALVE_OPEN)
	{
		gpioDigitalWrite(pins->trigger_a, HIGH);
		gpioDigitalWrite(pins->trigger_b, LOW);
	}
}

/*
 * @brief	generic function call in controling valves
 * @param	port: valve port to be controlled
 * @para	state: defines wether to close or open the valve
 */
InterfaceStatusEnum valveSetState(uint8_t port, ValveStateEnum state, ValveCtrlModeEnum ctrltype)
{
  	gpioDigitalWrite(&VCON15_EN, HIGH);
  	delayMillis(100);
	valveReset();
	ValveTrigPins *pins = pins_t;
	if (port == 2)
		++pins;

	if(ctrltype == REVERSE_POLARITY)
		valveCtrlTypeRP(pins, state);
	else if(ctrltype == POSITIVE_EXCITATION)
		valveCtrlTypePE(pins, state);
	else
		assert(0);
	delayMillis(100);
	valveReset();
	gpioDigitalWrite(&VCON15_EN, LOW);
	return INTERFACE_OK;
}
