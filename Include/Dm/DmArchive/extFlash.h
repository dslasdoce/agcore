/**
  *******************************************************************************
  * @file      	extFlash.h
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

#ifndef __EXTFLASH_H_
#define __EXTFLASH_H_

/* Includes ==================================================================*/
#include <stdint.h>

#include "dmMain.h"

/* Constant ==================================================================*/
/*External Flash sector Number*/
typedef enum
{
	EXT_FL_SEC_0 = 0,
	EXT_FL_SEC_1 = 1,
	EXT_FL_SEC_2 = 2,
	EXT_FL_SEC_3 = 3,
	EXT_FL_SEC_4 = 4,
	EXT_FL_SEC_5 = 5,
	EXT_FL_SEC_6 = 6,
	EXT_FL_SEC_7 = 7,
	EXT_FL_SEC_8 = 8,
	EXT_FL_SEC_9 = 9,
	EXT_FL_SEC_10 = 10,
	EXT_FL_SEC_11 = 11,
	EXT_FL_SEC_12 = 12,
	EXT_FL_SEC_13 = 13,
	EXT_FL_SEC_14 = 14,
	EXT_FL_SEC_15 = 15,
	EXT_FL_SEC_16 = 16,
	EXT_FL_SEC_17 = 17,
	EXT_FL_SEC_18 = 18,
	EXT_FL_SEC_19 = 19,
	EXT_FL_SEC_20 = 20,
	EXT_FL_SEC_21 = 21,
	EXT_FL_SEC_22 = 22,
	EXT_FL_SEC_23 = 23,
	EXT_FL_SEC_24 = 24,
	EXT_FL_SEC_25 = 25,
	EXT_FL_SEC_26 = 26,
	EXT_FL_SEC_27 = 27,
	EXT_FL_SEC_28 = 28,
	EXT_FL_SEC_29 = 29,
	EXT_FL_SEC_30 = 30,
	EXT_FL_SEC_31 = 31
} ExtFlSecNumEnum;

typedef struct ReadStatRegStruct
{
    uint8_t Wip    :1;    // [bit 0]
    uint8_t Wel    :1;    // [bit 1]
    uint8_t Bp0    :1;    // [bit 2]
    uint8_t Bp1    :1;    // [bit 3]
    uint8_t Bp2    :1;    // [bit 4]
    uint8_t Rsvd   :2;    // [bits 5, 6]
    uint8_t Srwd   :1;    // [bit 7]
} READ_STAT_REG_STRUCT;

/* Defines ===================================================================*/
/*ext flash characteristics*/
#define		EXT_FL_PAGE_SZ			0x100	
#define		EXT_FL_SUBSEC_SZ		0x1000
#define		EXT_FL_SEC_SZ			0x10000
#define  	EXT_FL_TOT_SEC_CNT		0x20
#define  	EXT_FL_PAGES_PER_SEC    0x100


/*Flash SPI Allocation: by Sector*/
#define 	EXT_FL_SEC_0_ADDR	((uint32_t)0x000000)//0	64 KB	4 bytes Parameter Data				
/* Area for Old Firmware(Total Allocation Size: 512 KB) */
#define 	EXT_FL_SEC_1_ADDR	((uint32_t)0x010000)//1	64 KB	
#define 	EXT_FL_SEC_2_ADDR	((uint32_t)0x020000)//2	64 KB					
#define 	EXT_FL_SEC_3_ADDR	((uint32_t)0x030000)//3	64 KB					
#define 	EXT_FL_SEC_4_ADDR	((uint32_t)0x040000)//4	64 KB					
#define 	EXT_FL_SEC_5_ADDR	((uint32_t)0x050000)//5	64 KB					
#define 	EXT_FL_SEC_6_ADDR	((uint32_t)0x060000)//6	64 KB					
#define 	EXT_FL_SEC_7_ADDR	((uint32_t)0x070000)//7	64 KB					
#define 	EXT_FL_SEC_8_ADDR	((uint32_t)0x080000)//8	64 KB					
/*Area for New Firmware (Total Allocation Size: 512 KB)*/
#define 	EXT_FL_SEC_9_ADDR	((uint32_t)0x090000)//9  64 KB	
#define 	EXT_FL_SEC_10_ADDR	((uint32_t)0x0A0000)//10 64 KB
#define 	EXT_FL_SEC_11_ADDR	((uint32_t)0x0B0000)//11 64 KB					
#define 	EXT_FL_SEC_12_ADDR	((uint32_t)0x0C0000)//12 64 KB					
#define 	EXT_FL_SEC_13_ADDR	((uint32_t)0x0D0000)//13 64 KB					
#define 	EXT_FL_SEC_14_ADDR	((uint32_t)0x0E0000)//14 64 KB					
#define 	EXT_FL_SEC_15_ADDR	((uint32_t)0x0F0000)//15 64 KB					
#define 	EXT_FL_SEC_16_ADDR	((uint32_t)0x100000)//16 64 KB	
/* Unallocated/Free Memory Area */
#define 	EXT_FL_SEC_17_ADDR	((uint32_t)0x110000)//17 64KB
#define 	EXT_FL_SEC_18_ADDR	((uint32_t)0x120000)//18 64KB
#define 	EXT_FL_SEC_19_ADDR	((uint32_t)0x130000)//19 64KB
#define 	EXT_FL_SEC_20_ADDR	((uint32_t)0x140000)//20 64KB
#define 	EXT_FL_SEC_21_ADDR	((uint32_t)0x150000)//21 64KB
#define 	EXT_FL_SEC_22_ADDR	((uint32_t)0x160000)//22 64KB
#define 	EXT_FL_SEC_23_ADDR	((uint32_t)0x170000)//23 64KB
#define 	EXT_FL_SEC_24_ADDR	((uint32_t)0x180000)//24 64KB
#define 	EXT_FL_SEC_25_ADDR	((uint32_t)0x190000)//25 64KB
#define 	EXT_FL_SEC_26_ADDR	((uint32_t)0x1A0000)//26 64KB
#define 	EXT_FL_SEC_27_ADDR	((uint32_t)0x1B0000)//27 64KB
#define 	EXT_FL_SEC_28_ADDR	((uint32_t)0x1C0000)//28 64KB
#define 	EXT_FL_SEC_29_ADDR	((uint32_t)0x1D0000)//29 64KB
#define 	EXT_FL_SEC_30_ADDR	((uint32_t)0x1E0000)//30 64KB
#define 	EXT_FL_SEC_31_ADDR	((uint32_t)0x1F0000)//31 64KB
/*Sector Specific Allocation*/
#define  PARAM_DATA_START_SEC_ADDR		EXT_FL_SEC_0_ADDR
#define  OLD_FW_START_SEC_ADDR			EXT_FL_SEC_1_ADDR
#define  NEW_FW_START_SEC_ADDR			EXT_FL_SEC_9_ADDR
#define  END_SEC_ADDR                   EXT_FL_SEC_31_ADDR

/* Function Prototypes =======================================================*/
ReturnStatusEnum extFlashInit(void);
ReturnStatusEnum extFlBulkErase(ExtFlSecNumEnum sec_start, uint8_t sec_cnt);
ReturnStatusEnum extFlSectorErase(uint8_t sec_cnt, uint32_t sec_addr);
ReturnStatusEnum extFlPageErase(uint8_t sec_start, uint8_t sec_cnt);
ReturnStatusEnum extFlEraseWrite(int8_t *buff_ptr, uint32_t dest_addr, 
								 uint16_t buff_size);
ReturnStatusEnum extFlWrite(int8_t *buff_ptr, uint32_t dest_addr, 
							uint16_t buff_size);
ReturnStatusEnum extFlRead(int8_t *buff_ptr, uint32_t src_addr, 
						   uint16_t buff_size);
ReturnStatusEnum extFlReadId(uint8_t *buff_ptr, uint16_t buff_size);
ReturnStatusEnum extFlReadStatReg (uint8_t *buff_ptr, uint16_t buff_size);

#endif		/*EXTFLASH_H_*/

/******************************************************************************
  * Revision History
  *	@file      	extFlash.h
  *****************************************************************************
  * @version	v2.2.0
  * @date		04/08/16
  * @author		M.I. Lorica
  * @changes	- created file
  *****************************************************************************
  */
