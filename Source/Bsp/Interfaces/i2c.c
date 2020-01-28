/********************************************************************************
  * @file      	i2c.c
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


/* Includes =================================================================*/
#include <agBoardInit.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include "stm32f7xx_hal.h"
#include "i2c.h"
#include "delay.h"

#define SIZE_I2C_CMD_BUFF 			((uint32_t)2)
#define I2CADD_DPOT0 				((uint32_t)0x2C)
#define I2CADD_DPOT1 				((uint32_t)0x2D)
#define I2CADD_DPOT2 				((uint32_t)0x2E)
#define I2CADD_I2CMUX 				((uint32_t)0x70)
#define DPOT_MAXIMUM_RESISTANCE		((uint32_t)50000)
#define I2CREG_DPOTNVW0 			((uint8_t)0x00)
#define I2CREG_DPOTNVW1 			((uint8_t)0x01)


/* ============================================================================
 * @brief	write data to i2c
 * @param	i2c_handle: ptr to I2C_HandleTypeDef
 * @param 	i2c_add: target i2c address
 * @param	p_data: start address of data to be transmitted
 * @param	size: number of data to be transmitted
 * @param	timeout: i2c bus response timeout
 * ==========================================================================*/
InterfaceStatusEnum i2cWrite(I2C_HandleTypeDef *i2c_handle, uint16_t i2c_add,
							 uint8_t *p_data, uint16_t size, uint32_t timeout)
{
  	HAL_StatusTypeDef i2c_stat = 0;
	i2c_stat = HAL_I2C_Master_Transmit(i2c_handle, (uint16_t)(i2c_add),
								   (uint8_t *)p_data, (uint16_t)size,
								   (uint32_t)timeout); 
	if ( i2c_stat != HAL_OK )
		return INTERFACE_ERROR;

	return INTERFACE_OK;
}

/* ============================================================================
 * @brief	write data to i2c
 * @param	i2c_handle: ptr to I2C_HandleTypeDef
 * @param 	i2c_add: target i2c address
 * @param	i2c_reg: target i2c register
 * @param	p_data: starting address of data storage
 * @param	size: number of data to be transmitted
 * @param	timeout: i2c bus response timeout
 * ==========================================================================*/
InterfaceStatusEnum i2cRead(I2C_HandleTypeDef *i2c_handle,uint16_t i2c_add,
							   uint8_t i2c_reg, uint8_t *p_data, uint16_t size,
							   uint32_t timeout)
{
	uint8_t i2c_pointer_reset[] = {i2c_reg};
	if ( !(HAL_I2C_Master_Transmit(i2c_handle, (uint16_t)(i2c_add),
								   (uint8_t *)i2c_pointer_reset,
								   (uint16_t)sizeof(i2c_pointer_reset),
								   (uint32_t)I2C_TIMEOUT) == HAL_OK) )
		return INTERFACE_ERROR;
	if ( !(HAL_I2C_Master_Receive (i2c_handle, (uint16_t)(i2c_add),
								   (uint8_t *)p_data, (uint16_t)size,
								   (uint32_t)timeout) == HAL_OK) )
		return INTERFACE_ERROR;

	return INTERFACE_OK;
}

InterfaceStatusEnum i2cReadx(I2C_HandleTypeDef *i2c_handle,uint16_t i2c_add,
							   uint8_t i2c_reg, uint8_t *p_data, uint16_t size,
							   uint32_t timeout)
{
	uint8_t i2c_pointer_reset[] = {0,0};
	if ( !(HAL_I2C_Master_Transmit(i2c_handle, (uint16_t)(i2c_add),
								   (uint8_t *)i2c_pointer_reset,
								   2,
								   (uint32_t)I2C_TIMEOUT) == HAL_OK) )
		return INTERFACE_ERROR;
	if ( !(HAL_I2C_Master_Receive (i2c_handle, (uint16_t)(i2c_add),
								   (uint8_t *)p_data, (uint16_t)size,
								   (uint32_t)timeout) == HAL_OK) )
		return INTERFACE_ERROR;

	return INTERFACE_OK;
}

/* ============================================================================
 * @brief	change dpot resistance
 * @param	port: agtechboard port
 * @param	res: resistance value
 * ==========================================================================*/
InterfaceStatusEnum i2cDpotWrite(uint8_t port, int resistance)
{
	uint16_t i2cadd_dpot;
	uint8_t i2creg_dpotwiper = 0 ;
	uint8_t nbits = 7;
	uint8_t wiperstep;
	//uint32_t resmax = 100000;
	uint8_t cmd[SIZE_I2C_CMD_BUFF];

	wiperstep = (uint8_t)lround(resistance *
				((double)(1<<nbits)/DPOT_MAXIMUM_RESISTANCE));
	/*
	 * Get i2c address based on port
	 */
	if ((port-1)/2)
			i2cadd_dpot = I2CADD_DPOT1;
	else
		i2cadd_dpot = I2CADD_DPOT0;
	/*
	 * get resistor register address
	 */
	if (port & 0x01)
		i2creg_dpotwiper = I2CREG_DPOTNVW0;
	else
		i2creg_dpotwiper = I2CREG_DPOTNVW1;

	/*
	 * set command to be sent to dpot
	 * write command (4 bit device reg address,2 bit command type(r/w),1 bit don't care,
	 * 1 bit representing msb(bit 8) of data byte)
	*/
	cmd[0] = (uint8_t)(i2creg_dpotwiper<<4) |
			 (uint8_t)( (wiperstep&(1<<nbits)) >>nbits);

	/*data byte(8 bit value representing step) *note that whole value of step is 9 bits,
	 * but the msb will be moved to the lsb of the write command
	 * since i2c only supports 1 byte per data frame
	 */
	cmd[1] = (uint8_t)(wiperstep & (~(1<<nbits)));

	return i2c2Write(i2cadd_dpot, cmd, sizeof(cmd));
}







/*********************************************************************************
  * Revision History
  * @file       i2c.c
  ********************************************************************************
  * @version    v2.1.0
  * @date       12/23/2015
  * @author     D.Lasdoce
  * @changes     - created file
  ********************************************************************************
  */
