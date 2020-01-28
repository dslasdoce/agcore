/**
  ******************************************************************************
  * @file      	i2c.h
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

#ifndef I2C_H_
#define I2C_H_

/* Includes ==================================================================*/
#include "agBoardInit.h"

/* Defines ==================================================================*/
#define I2CADD_PWRMONITOR 0x40
#define I2C_TIMEOUT 	((uint16_t)500)
#define I2C_BUFFSIZE 	((uint16_t)58)

#define I2CADD_ACCE 	(0x1D)
#define I2CADD_EEPROM 	(0x54)

#define i2c2Write(i2c_add,p_data,size)				(i2cWrite(&I2C2HandleDef, \
													(uint16_t)(i2c_add<<1),\
													(uint8_t *)p_data, \
													(uint16_t)size, I2C_TIMEOUT ))
#define i2c2Read(i2c_add,i2c_reg,p_data,size)		(i2cRead(&I2C2HandleDef, \
													(uint16_t)(i2c_add<<1), \
													(uint8_t)i2c_reg,\
													(uint8_t *)p_data,\
													(uint16_t)size, I2C_TIMEOUT))
#define i2c1Write(i2c_add,p_data,size)				(i2cWrite(&I2C1HandleDef, (uint16_t)(i2c_add<<1), (uint8_t *)p_data, (uint16_t)size, I2C_TIMEOUT ))
#define i2c1Read(i2c_add,i2c_reg,p_data,size)		(i2cRead(&I2C1HandleDef, (uint16_t)(i2c_add<<1), (uint8_t)i2c_reg,(uint8_t *)p_data, (uint16_t)size, I2C_TIMEOUT))

														
#define i2c1Readx(i2c_add,i2c_reg,p_data,size)		(i2cReadx(&I2C1HandleDef, (uint16_t)(i2c_add<<1), (uint8_t)i2c_reg,(uint8_t *)p_data, (uint16_t)size, I2C_TIMEOUT))
/* Function Prototypes ======================================================*/
InterfaceStatusEnum i2cWrite(I2C_HandleTypeDef *i2c_handle, uint16_t i2c_add,
								uint8_t *p_data, uint16_t size, uint32_t timeout);
InterfaceStatusEnum i2cRead(I2C_HandleTypeDef *i2c_handle, uint16_t i2c_add,
							   uint8_t i2c_reg, uint8_t *p_data, uint16_t size,
							   uint32_t timeout);

//InterfaceStatusEnum i2cPwrMntrRead(float *pwr_stat);
#endif

/*********************************************************************************
  * Revision History
  * @file       i2c.h
  ********************************************************************************
  * @version    v2.1.0
  * @date       12/23/2015
  * @author     D.Lasdoce
  * @changes     - created file
  ********************************************************************************
  */
