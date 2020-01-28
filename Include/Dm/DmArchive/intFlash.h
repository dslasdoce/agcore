/**
  ******************************************************************************
  * @file      	intFlash.h
  * @author     Hardware Team
  * @version    v2.2.0
  * @date       04/08/16
  * @brief
  ******************************************************************************
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

#ifndef __INTFLASH_H_
#define __INTFLASH_H_

/* Includes =================================================================*/
#include "stm32f7xx_hal.h"
#include "dmMain.h"
#include "loggerMain.h"


/* Typedefs =================================================================*/

typedef enum
{
	TEMP_BUFF_TRACKER,
	TEMP_UNSENT_BUFF_TRACKER,
	SENT_BUFF_TRACKER
}IntFlTrackerEnum;

	  
typedef struct
{
	/* Tracker for records in Temporary and Sent Flash Buffer (TEMP_FL_BUFF)*/
	uint16_t	last_rec_idx;
	uint16_t	unsent_rec_count;
	uint16_t	unsent_rec_cnt_in_sr;
	uint16_t	sent_last_rec_idx;
	uint16_t	rsvd_16b;
	uint16_t	sent_rec_count;
	uint32_t	last_rec_addr;
	uint32_t	unsent_tracker_addr;
	uint32_t	sent_last_rec_addr;
} FlBuffTrackerStruct;

/* External Declarations ====================================================*/
extern FlBuffTrackerStruct fl_handle[REC_TYP_CNT_TO_LOG];

// Single Bank Sector Mapping (Base Address)
#define ADDR_FLASH_SECTOR_0        ((uint32_t)0x08000000)    // 32 KB
#define ADDR_FLASH_SECTOR_1        ((uint32_t)0x08008000)    // 32 KB
#define ADDR_FLASH_SECTOR_2        ((uint32_t)0x08010000)    // 32 KB
#define ADDR_FLASH_SECTOR_3        ((uint32_t)0x08018000)    // 32 KB
#define ADDR_FLASH_SECTOR_4        ((uint32_t)0x08020000)    // 128 KB
#define ADDR_FLASH_SECTOR_5        ((uint32_t)0x08040000)    // 256 KB
#define ADDR_FLASH_SECTOR_6        ((uint32_t)0x08080000)    // 256 KB
#define ADDR_FLASH_SECTOR_7        ((uint32_t)0x080C0000)    // 256 KB
#define ADDR_FLASH_SECTOR_8        ((uint32_t)0x08100000)    // 256 KB
#define ADDR_FLASH_SECTOR_9        ((uint32_t)0x08140000)    // 256 KB
#define ADDR_FLASH_SECTOR_10       ((uint32_t)0x08180000)    // 256 KB
#define ADDR_FLASH_SECTOR_11       ((uint32_t)0x081C0000)    // 256 KB

// Single Bank Sector Sizes
#define SIZE_FLASH_SECTOR_0        0x8000     // 32 KB
#define SIZE_FLASH_SECTOR_1        0x8000     // 32 KB
#define SIZE_FLASH_SECTOR_2        0x8000     // 32 KB
#define SIZE_FLASH_SECTOR_3        0x8000     // 32 KB
#define SIZE_FLASH_SECTOR_4        0x20000    // 128 KB
#define SIZE_FLASH_SECTOR_5        0x40000    // 256 KB
#define SIZE_FLASH_SECTOR_6        0x40000    // 256 KB
#define SIZE_FLASH_SECTOR_7        0x40000    // 256 KB
#define SIZE_FLASH_SECTOR_8        0x40000    // 256 KB
#define SIZE_FLASH_SECTOR_9        0x40000    // 256 KB
#define SIZE_FLASH_SECTOR_10       0x40000    // 256 KB
#define SIZE_FLASH_SECTOR_11       0x40000    // 256 KB

/*Used for SysStat Memory Check*/
#define START_ADDR_OF_FIRST_SECTOR	 ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define END_ADDR_OF_LAST_SECTOR	     ((uint32_t)0x08100000) /* Base @ of Sector 12, 16 Kbytes */
#define CODE_FLASH_MEM_SIZE		 	 ((uint32_t)(ADDR_FLASH_SECTOR_5 \
										- START_ADDR_OF_FIRST_SECTOR))
#define ORIG_FLASH_MEM_SIZE		 	 ((uint32_t)(END_ADDR_OF_LAST_SECTOR \
										- START_ADDR_OF_FIRST_SECTOR))

/*Settings/Trackers address*/
#define FL_SETTINGS_START_ADDR			ADDR_FLASH_SECTOR_11
#define TELEM_FL_TRACKERS_ADDR			ADDR_FLASH_SECTOR_11
#define SYSSTAT_FL_TRACKERS_ADDR		TELEM_FL_TRACKERS_ADDR + \
  											REC_FL_SETTINGS_SZ_BYTES

#define REC_FL_SETTINGS_SZ_BYTES		(sizeof(FlBuffTrackerStruct))												
												
/*temp buff addr*/
#define	TELEM_TEMP_BUFF_START_ADDR		ADDR_FLASH_SECTOR_6
#define	SYSSTAT_TEMP_BUFF_START_ADDR	ADDR_FLASH_SECTOR_9
#define	TELEM_TEMP_BUFF_END_ADDR		ADDR_FLASH_SECTOR_7
#define	SYSSTAT_TEMP_BUFF_END_ADDR		ADDR_FLASH_SECTOR_10

												
/*sent buff addr*/
#define	TELEM_FL_SENT_START_ADDR		ADDR_FLASH_SECTOR_10
#define	TELEM_FL_SENT_END_ADDR			(uint32_t)(0x80D0000)
#define	SYSSTAT_FL_SENT_START_ADDR		TELEM_FL_SENT_END_ADDR
#define	SYSSTAT_FL_SENT_END_ADDR		ADDR_FLASH_SECTOR_11

#define	FL_SETTINGS_ADDR(rec_typ)		((rec_typ == TELEMETRY) ?	\
	  									    TELEM_FL_TRACKERS_ADDR:\
											SYSSTAT_FL_TRACKERS_ADDR)


#define	FL_START_TEMP_ADDR(rec_typ)		((rec_typ == TELEMETRY) ?	\
	  									     TELEM_TEMP_BUFF_START_ADDR:\
											 SYSSTAT_TEMP_BUFF_START_ADDR)
	  
#define	FL_END_TEMP_ADDR(rec_typ)		((rec_typ == TELEMETRY) ?	\
		  									 TELEM_FL_SENT_END_ADDR:\
											 SYSSTAT_TEMP_BUFF_END_ADDR)

#define	FL_START_SENT_ADDR(rec_typ)		((rec_typ == TELEMETRY) ?	\
	  									     TELEM_FL_SENT_START_ADDR:\
											 SYSSTAT_FL_SENT_START_ADDR)
	  
/*Sector Number*/
#define	TELEM_BU_BUFF_SECNUM			FLASH_SECTOR_6
#define	SYSSTAT_BU_BUFF_SECNUM			FLASH_SECTOR_9
#define	FL_SENTBUFF_SECNUM				FLASH_SECTOR_10
#define	FL_SETTINGS_SECNUM				FLASH_SECTOR_11

/*Sector Count per record*/
#define TELEM_BU_SEC_COUNT				3
#define SYSTAT_BU_SEC_COUNT				1
#define SENT_SEC_COUNT					1 

#define	BU_BUFF_SECNUM(rec_typ)			((rec_typ == TELEMETRY) ?	\
	  									     TELEM_BU_BUFF_SECNUM:\
											 SYSSTAT_BU_BUFF_SECNUM)

#define	BU_SEC_CNT(rec_typ)				((rec_typ == TELEMETRY) ?	\
	  									     TELEM_BU_SEC_COUNT:\
											 SYSTAT_BU_SEC_COUNT)

/*DMA Interrupt Macros*/
#define INTFL_DMA_STREAM_IRQ	       DMA2_Stream4_IRQn
#define INTFL_DMA_STREAM_IRQHANDLER    DMA2_Stream4_IRQHandler
#define	INTFL_DMA_STREAM				DMA2_Stream4

/* Function Prototypes========================================================*/

ReturnStatusEnum intFlashInit(void);

ReturnStatusEnum intFlDmaExec(uint32_t src_addr, uint32_t dest_addr, 
							  uint32_t dma_size);

ReturnStatusEnum intFlErase(uint8_t first_sector, uint8_t num_sec);
uint32_t intFlReadWord(uint32_t src_addr);
ReturnStatusEnum intFlRecEraseTempBuff(RecordTypEnum rec_typ,
									   uint8_t num_sec);
ReturnStatusEnum intFlEraseSentBuff(void);
ReturnStatusEnum intFlSaveSettings(void);

void intFlUpdateTracker(IntFlTrackerEnum tracker,
							   		uint16_t record_num, 
							   		RecordTypEnum rec_typ);
uint32_t intFlRetLastAddr(RecordTypEnum rec_typ);
#endif /* INTFLASH_H_ */

/*********************************************************************************
  * Revision History
  * @file         
  ********************************************************************************
  * @version    v2.2.0
  * @date       04/08/16
  * @author     M.I.Lorica
  * @changes     renamed flash.h to intFlash.h. Added fxn protypes.
  ********************************************************************************
  * @version    v2.1.0
  * @date       12/23/2015
  * @author     M.I.Lorica
  * @changes     - created file
  ********************************************************************************
  */


