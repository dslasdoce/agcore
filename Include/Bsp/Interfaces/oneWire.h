/**
  *****************************************************************************
  * @file      	oneWire.h
  * @author     Hardware Team
  * @version    v2.2.0
  * @date       04/08/16
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
  *****************************************************************************
  */

#ifndef ONE_WIRE_H_
#define	ONE_WIRE_H_

/* Includes =================================================================*/
#include "gpio.h"

/* Defines ==================================================================*/
#define OWIRE_PARASITIC_POWEROFF		0
#define OWIRE_PARASITIC_POWERON			1

/* Function Prototypes ======================================================*/
uint8_t oneWireSearch(AgTechIOStruct *gpio_ptr, uint8_t *newAddr);
uint8_t crc8(const uint8_t *addr, uint8_t len);
void oneWireSelect(AgTechIOStruct *gpio_ptr, const uint8_t rom[8]);
uint8_t oneWireReset(AgTechIOStruct *gpio_ptr);
uint8_t oneWireRead(AgTechIOStruct *gpio_ptr);
void oneWireReadBytes(AgTechIOStruct *gpio_ptr, uint8_t *buf, uint16_t count);
void oneWireWrite(AgTechIOStruct *gpio_ptr, uint8_t out_byte, bool power);
void oneWireWriteBytes(AgTechIOStruct *gpio_ptr, const uint8_t *buf,
						uint16_t count, bool power);

#endif
/*********************************************************************************
  * Revision History
  * @file       oneWire.h	
  ********************************************************************************
  * @version    v2.2.0
  * @date       04/08/16
  * @author     D.Lasdoce
  * @changes
  ********************************************************************************
  * @version    v2.1.0
  * @date       12/23/2015
  * @author     D.Lasdoce
  * @changes     - created file
  ********************************************************************************
  */
