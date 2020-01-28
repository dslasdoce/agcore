/**
  ******************************************************************************
  * @file    	modbus.h
  * @author  	Hardware Team
  * @version 	v2.0.1
  * @date    	14-September-2015
  * @brief   	Header file of modbus module.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MODBUS_H
#define __MODBUS_H

/* Includes =================================================================*/
#include <stdbool.h>
#include "delay.h"
#include <deployAssert.h>
#include "stm32f7xx_hal.h"

/* Object-like Macros =======================================================*/
#define MB_MAX_REG_COUNT	((uint8_t)40)
#define MB_CRC_SZ			sizeof(ModbusCrcStruct)

//supported modbus function codes for Banner radio
#define MODBUS_READ             ((uint8_t)0x03)
#define MODBUS_WRITE_SINGLE     ((uint8_t)0x06)
#define MODBUS_WRITE_MULTIPLE   ((uint8_t)0x10)
        
/* Function-like Macros =====================================================*/
#define MB_CLEAR(data)		memset(&data, ((uint8_t)0), sizeof(data))
#define MB_ENDIAN_16B_INV(data)		data = ((data >> 8) | (data << 8))

/* Typedefs =================================================================*/
typedef enum
{
	MODBUS_BANNER_RADIO,
	MODBUS_XBEE_RADIO
} ModbusPort;

typedef enum
{
	MODBUS_STATNULL,
    MODBUS_OK,
    MODBUS_CRC_ERROR,
	MODBUS_ERROR	
} ModbusStatEnum;

/*typedef enum 		//Modbus Standard Function Code
{
	MODBUS_FUNCNULL,
	MODBUS_READCOILSTAT,
	MODBUS_READINPUTSTAT,
	MODBUS_READHOLDINGREG,
	MODBUS_READINPUTREG,
	MODBUS_FORCESINGLECOIL,
	MODBUS_FORCESINGLEREG,
	MODBUS_FORCEMULTICOIL = 0x0F,
	MODBUS_FORCEMULTIREG,
	MODBUS_REPORTSLAVEID
} ModbusFuncEnum;*/

typedef struct
{
	uint8_t		crc_hi;
	uint8_t		crc_lo;
} ModbusCrcStruct;

typedef struct
{
	uint8_t modbus_id;
	uint8_t function;
} ModbusCommonHeadStruct;

typedef struct
{
	ModbusCommonHeadStruct;
	uint16_t start_addr;
	uint16_t reg_cnt;
} ModbusHeadStruct;

#pragma pack(1)
typedef struct
{
	uint8_t modbus_id;
    uint16_t rad_dev_addr;
	uint8_t function;
	uint16_t start_addr;
	uint16_t reg_cnt;
} ModbusExtendedHeadStruct;

typedef struct
{
	ModbusHeadStruct head;
	ModbusCrcStruct crc;
} ModbusFrameStruct;

typedef struct
{
	ModbusExtendedHeadStruct head;
	ModbusCrcStruct crc;
} ModbusExtendedFrameStruct;

typedef struct
{
	uint8_t 	status;
	uint8_t		count;
	uint16_t 	payload[MB_MAX_REG_COUNT];
} ModbusRxDataStruct;

/* Function Prototypes ======================================================*/
ModbusStatEnum modbusInverse16bitPayload(uint16_t *a_data, uint8_t a_data_count);
ModbusCrcStruct modbusCrcCalc(uint8_t *puchMsg, uint16_t  usDataLen);
ModbusStatEnum modbusCrcChk(uint8_t* a_frameptr, uint8_t a_sz);
ModbusRxDataStruct modbusRead(UART_HandleTypeDef *a_uart_handler, ModbusHeadStruct *a_head);
ModbusStatEnum modbusWriteMultiple(UART_HandleTypeDef *a_uart_handler, ModbusHeadStruct a_head, uint16_t *a_payload);
ModbusRxDataStruct modbusExtendedRead(UART_HandleTypeDef *a_uart_handler, ModbusExtendedHeadStruct *a_head);
ModbusStatEnum modbusExtendedWriteMultiple(UART_HandleTypeDef *a_uart_handler, ModbusExtendedHeadStruct a_head, uint16_t *a_payload);

#endif
/*********************************************************************************
  * Revision History
  * @file         
  ********************************************************************************
  * @version
  * @date
  * @author         
  * @changes     - created file
  ********************************************************************************
  */

