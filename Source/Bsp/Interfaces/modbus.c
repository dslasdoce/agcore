/**
  ******************************************************************************
  * @file		modbus.c
  * @author		Hardware Team
  * @version	v2.0.1
  * @date		14-September-2015
  * @brief		radio module
  * 			This file provides firmware functions to manage the following
  * 			functionalities of ModBus protocol:
  * 				+ Query Command
  * 				+ Update Command
  * 				+ Response to Command
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

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "utilMem.h"
#include "stm32f7xx_hal.h"
#include "agBoardInit.h"
#include "modbus.h"

/* Object-like Macros =======================================================*/
#define MB_RXDLY 			    ((uint32_t)2500)
#define MB_UARTRX_TIMEOUT       ((uint32_t)5000)
#define MB_EXTENDED_ID          ((uint8_t)0xFA)
#define MB_FRAME_SZ				sizeof(ModbusFrameStruct)
#define MB_EXT_FRAME_SZ			sizeof(ModbusExtendedFrameStruct)
#define MB_HEAD_SZ				sizeof(ModbusHeadStruct)
#define MB_EXT_HEAD_SZ			sizeof(ModbusExtendedHeadStruct)     

/* Global Declarations =====================================================*/
/* Table of CRC values for high–order byte */
char auchCRCHi[] =
	{0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
					0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
					0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
					0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
					0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
					0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
					0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
					0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
					0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
					0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
					0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
					0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
					0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
					0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
					0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
					0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
					0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
					0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
					0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
					0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
					0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
					0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
					0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
					0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
					0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
					0x00, 0xC1, 0x81, 0x40};

/* Table of CRC values for low–order byte */
char auchCRCLo[] =
	{0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
					0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF,
					0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
					0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE,
					0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15,
					0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1,
					0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
					0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC,
					0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB,
					0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B,
					0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C,
					0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2,
					0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
					0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65,
					0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE,
					0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8,
					0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF,
					0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7,
					0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
					0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56,
					0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D,
					0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59,
					0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A,
					0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 0x44, 0x84,
					0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
					0x41, 0x81, 0x80, 0x40};


/* Static Functions ========================================================*/
#pragma optimize=none
/*==============================================================================
* @brief    
* @param	
* @retval	
==============================================================================*/
static ModbusStatEnum modbusRx(UART_HandleTypeDef *a_uart_handler
                               , uint8_t *a_frame_ptr, uint8_t a_sz)
{
	HAL_StatusTypeDef hal_stat;
	uint32_t t_start;					/*reference of receiving delay*/

	MB_CLEAR(hal_stat);
	t_start = HAL_GetTick();
	while(delayTimeDiff(t_start, HAL_GetTick()) < MB_UARTRX_TIMEOUT)
	{
        hal_stat = HAL_UART_Receive(a_uart_handler, a_frame_ptr, a_sz, MB_RXDLY);

		if(hal_stat == HAL_OK)
		{
			return modbusCrcChk(a_frame_ptr, a_sz);
		}
        else
        {
            HAL_UART_DeInit(a_uart_handler);
            HAL_UART_Init(a_uart_handler);
        }
        
        if(a_sz < 40)
            break;
	}

	return MODBUS_ERROR;
}

/*==============================================================================
* @brief    
* @param	
* @retval	
==============================================================================*/
static ModbusStatEnum modbusTx(UART_HandleTypeDef *a_uart_handler
                               , uint8_t *a_frame_ptr, uint8_t a_sz)
{
	if(modbusCrcChk(a_frame_ptr, a_sz) != MODBUS_OK)
	{
		return MODBUS_CRC_ERROR;
	}

	delayMillis(RD_TRANSMIT_DELAY);
    HAL_UART_Transmit(a_uart_handler, a_frame_ptr, a_sz, (uint32_t)2000);
    
	return MODBUS_OK;
}

/* External Functions ======================================================*/
/*==============================================================================
* @brief    
* @param	
* @retval	
==============================================================================*/
ModbusStatEnum modbusInverse16bitPayload(uint16_t *a_payload, uint8_t a_data_count)
{
	uint8_t idx;

	for(idx = 0; idx < a_data_count; idx++)
	{
		MB_ENDIAN_16B_INV(a_payload[idx]);
	}
	return MODBUS_OK;
}

/*============================================================================
* @brief    Modbus CRC Generator
* @param
* @retval
============================================================================*/
ModbusCrcStruct modbusCrcCalc(uint8_t *puchMsg, uint16_t usDataLen)
{
	ModbusCrcStruct result;
	uint8_t uchCRCHi = 0xFF;	/* high byte of CRC initialized */
	uint8_t uchCRCLo = 0xFF; 	/* low byte of CRC initialized */
	uint16_t uIndex; 			/* will index into CRC lookup table */

	while (usDataLen--) /* pass through message buffer */
	{
		uIndex = uchCRCHi ^ *puchMsg++; /* calculate the CRC */
		uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
		uchCRCLo = auchCRCLo[uIndex];
	}

	result.crc_hi = uchCRCHi;
	result.crc_lo = uchCRCLo;
	return result;
}

/*==============================================================================
* @brief    
* @param	
* @retval	
==============================================================================*/
ModbusStatEnum modbusCrcChk(uint8_t* a_frameptr, uint8_t a_sz)
{
	ModbusCrcStruct que_crc;	/*CRC verifier*/
	ModbusCrcStruct frame_crc;

	que_crc = modbusCrcCalc(a_frameptr, a_sz - MB_CRC_SZ);
	memcpy(&frame_crc, a_frameptr + a_sz - MB_CRC_SZ , MB_CRC_SZ);

	if(que_crc.crc_hi == frame_crc.crc_hi && que_crc.crc_lo == frame_crc.crc_lo)
	{
		return MODBUS_OK;
	}
	else
	{
		return MODBUS_ERROR;
	}
}

#pragma optimize=none
/*==============================================================================
* @brief    
* @param	
* @retval	
==============================================================================*/
ModbusRxDataStruct modbusRead(UART_HandleTypeDef *a_uart_handler
                                        , ModbusHeadStruct *a_head)
{
	ModbusStatEnum retval;
	ModbusRxDataStruct rxdata;
	ModbusFrameStruct txframe;
	uint8_t *rxframe_ptr;
	uint8_t rxframe_sz;
	uint8_t  byte_count;
    enum rxhead {SLAVE_ADDR, FUNC, BYTE_CNT, RXHEAD_SZ};

	byte_count = a_head->reg_cnt * sizeof(uint16_t);
    rxframe_sz = sizeof(ModbusCommonHeadStruct) + sizeof(byte_count)
                + byte_count + MB_CRC_SZ;
	rxframe_ptr = (uint8_t*) memAlloc(rxframe_sz);
    
    a_head->function = MODBUS_READ;
	txframe.head = *a_head;
	MB_ENDIAN_16B_INV(txframe.head.start_addr);
	MB_ENDIAN_16B_INV(txframe.head.reg_cnt);
	txframe.crc = modbusCrcCalc((uint8_t*)&txframe.head, MB_HEAD_SZ);
	
    modbusTx(a_uart_handler, (uint8_t*)&txframe, sizeof(txframe));
	retval = modbusRx(a_uart_handler, rxframe_ptr, rxframe_sz);

	MB_CLEAR(rxdata);
	rxdata.status = retval;
    if(rxdata.status == MODBUS_OK)
    {
        rxdata.count = rxframe_ptr[BYTE_CNT];
        memcpy(rxdata.payload, rxframe_ptr + RXHEAD_SZ, rxdata.count);
        modbusInverse16bitPayload(rxdata.payload, rxdata.count);
    }

    if(rxframe_ptr) free(rxframe_ptr);
	return rxdata;
}

#pragma optimize=none
/*==============================================================================
* @brief    
* @param	
* @retval	
==============================================================================*/
ModbusStatEnum modbusWriteMultiple(UART_HandleTypeDef *a_uart_handler
                                 , ModbusHeadStruct a_head, uint16_t *a_payload)
{
	ModbusStatEnum retval;
	ModbusFrameStruct rxframe;
	ModbusCrcStruct crc;
	uint8_t *txdata_ptr;
	uint8_t malloc_sz;
	uint8_t data_byte_sz;

	data_byte_sz = a_head.reg_cnt * sizeof(uint16_t);
	malloc_sz = MB_HEAD_SZ + sizeof(data_byte_sz) + data_byte_sz + MB_CRC_SZ;
	txdata_ptr = memAlloc(malloc_sz);
    
    memcpy(txdata_ptr + MB_HEAD_SZ + sizeof(data_byte_sz)
           , a_payload, data_byte_sz);
    modbusInverse16bitPayload((uint16_t*)&txdata_ptr[MB_HEAD_SZ + sizeof(data_byte_sz)], a_head.reg_cnt);

	a_head.function = MODBUS_WRITE_MULTIPLE;
	MB_ENDIAN_16B_INV(a_head.start_addr);
	MB_ENDIAN_16B_INV(a_head.reg_cnt);
	memcpy(txdata_ptr, &a_head, MB_HEAD_SZ);
	memcpy(txdata_ptr + MB_HEAD_SZ, &data_byte_sz, sizeof(data_byte_sz));
	
	crc = modbusCrcCalc(txdata_ptr, malloc_sz - MB_CRC_SZ);
	memcpy(txdata_ptr + malloc_sz - MB_CRC_SZ, &crc, MB_CRC_SZ);
    MB_CLEAR(rxframe);

	modbusTx(a_uart_handler, txdata_ptr, malloc_sz);
    retval = modbusRx(a_uart_handler, (uint8_t*)&rxframe, MB_FRAME_SZ);
	
    if(retval != MODBUS_OK || rxframe.head.reg_cnt != a_head.reg_cnt)
	{
        retval = MODBUS_ERROR;
	}

    if(txdata_ptr) free(txdata_ptr);
	return retval;
}

#pragma optimize=none
/*==============================================================================
* @brief    
* @param	
* @retval	
==============================================================================*/
ModbusRxDataStruct modbusExtendedRead(UART_HandleTypeDef *a_uart_handler
                                      , ModbusExtendedHeadStruct *a_head)
{
	ModbusStatEnum retval;
	ModbusRxDataStruct rxdata;
	ModbusExtendedFrameStruct txframe;
	uint8_t *rxframe_ptr;
	uint8_t rxframe_sz;
	uint8_t  byte_count;
    enum rxhead {EXT_MBID, RADADDR2, RADADDR1, FUNC, BYTE_CNT, PAYLOAD};
    
    byte_count = a_head->reg_cnt * sizeof(uint16_t);
    rxframe_sz = sizeof(ModbusCommonHeadStruct) + sizeof(a_head->rad_dev_addr)
                + sizeof(byte_count) + byte_count + MB_CRC_SZ;
	rxframe_ptr = (uint8_t*) memAlloc(rxframe_sz);

    a_head->modbus_id = MB_EXTENDED_ID;
    a_head->function = MODBUS_READ;
	txframe.head = *a_head;
    MB_ENDIAN_16B_INV(txframe.head.rad_dev_addr);
	MB_ENDIAN_16B_INV(txframe.head.start_addr);
	MB_ENDIAN_16B_INV(txframe.head.reg_cnt);
	txframe.crc = modbusCrcCalc((uint8_t*)&txframe.head, MB_EXT_HEAD_SZ);
	
    modbusTx(a_uart_handler, (uint8_t*)&txframe, sizeof(txframe));
	retval = modbusRx(a_uart_handler, rxframe_ptr, rxframe_sz);

	MB_CLEAR(rxdata);
	rxdata.status = retval;
    if(rxdata.status == MODBUS_OK)
    {
        rxdata.count = rxframe_ptr[BYTE_CNT];
        memcpy(rxdata.payload, rxframe_ptr + PAYLOAD, rxdata.count);
    }
    
    if(rxframe_ptr) free(rxframe_ptr);
	return rxdata;
}

#pragma optimize=none
/*==============================================================================
* @brief    
* @param	
* @retval	
==============================================================================*/
ModbusStatEnum modbusExtendedWriteMultiple(UART_HandleTypeDef *a_uart_handler
                                           , ModbusExtendedHeadStruct a_head
                                           , uint16_t *a_payload)
{
	ModbusStatEnum retval;
	ModbusExtendedFrameStruct rxframe;
	ModbusCrcStruct crc;
	uint8_t *txdata_ptr;
	uint8_t malloc_sz;
	uint8_t data_byte_sz;

	data_byte_sz = a_head.reg_cnt * sizeof(uint16_t);
	malloc_sz = MB_EXT_HEAD_SZ + sizeof(data_byte_sz) + data_byte_sz + MB_CRC_SZ;
	txdata_ptr = memAlloc(malloc_sz);

    a_head.modbus_id = MB_EXTENDED_ID;
	a_head.function = MODBUS_WRITE_MULTIPLE;
	MB_ENDIAN_16B_INV(a_head.start_addr);
	MB_ENDIAN_16B_INV(a_head.reg_cnt);
    modbusInverse16bitPayload(a_payload, data_byte_sz);

	memcpy(txdata_ptr, &a_head, MB_EXT_HEAD_SZ);
	memcpy(txdata_ptr + MB_EXT_HEAD_SZ, &data_byte_sz, sizeof(data_byte_sz));
	memcpy(txdata_ptr + MB_EXT_HEAD_SZ + sizeof(data_byte_sz)
           , a_payload, data_byte_sz);
	crc = modbusCrcCalc(txdata_ptr, malloc_sz - MB_CRC_SZ);
	memcpy(txdata_ptr + malloc_sz - MB_CRC_SZ, &crc, MB_CRC_SZ);
    MB_CLEAR(rxframe);

	modbusTx(a_uart_handler, txdata_ptr, malloc_sz);
    retval = modbusRx(a_uart_handler, (uint8_t*)&rxframe, MB_EXT_FRAME_SZ);
	
    if(retval != MODBUS_OK || rxframe.head.reg_cnt != a_head.reg_cnt)
	{
        retval = MODBUS_ERROR;
	}

    if(txdata_ptr) free(txdata_ptr);
	return retval;
}

/*
 * @brief ModBus update command
 * @param uart: UART pin number of radio's Rx/Tx pin
 * @param mb_id: modbus id
 * @param start_addr: start register address to be query
 * @param reg_size: number of register address (in bytes) to be query
 * @param data: data to be updated (npt yet included)
 * @retval: return status
 */
/*
ReturnStatusEnum modbusWrite(uint8_t uart, uint8_t mb_id, uint8_t start_addr, uint8_t reg_size)
{
	//uint8_t mbdata[] = {mb_id,0x10,start_addr,start_addr>>8,reg_size>>8,reg_size-1,reg_size*2//,addthe data here//};
	uint16_t crc = generateCRC((uint8_t*)mbdata,sizeof(mbdata));

	uint8_t crc8[2];
	crc8[0] = (uint8_t)(crc & 0xff);
	crc8[1] = (uint8_t)(crc >> 8);

	mbdata[sizeof(mbdata)] = crc8[0];
	mbdata[sizeof(mbdata)+1] = crc8[1];

	switch(uart){
		case 5:
			uartTx5(mbdata, sizeof(mbdata)+2);
	}
	return SUCCESSFUL;
}
*/

/******************************************************************************
  * Revision History
  *	@file      	
  *****************************************************************************
  * @version	
  * @date		
  * @author		JWF
  * @changes
  *****************************************************************************
  */


//=================================== END ====================================//
