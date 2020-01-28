/**
  *******************************************************************************
  * @file     	loggerMain.h
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
  ******************************************************************************
  */
#ifndef __LOGGERMAIN_H
#define __LOGGERMAIN_H

/* Includes ==================================================================*/
#include "dmMain.h"
#include "intFlash.h"
#include "productCfg.h"

/* Typedefs ==================================================================*/
 typedef struct
{
	uint16_t latest_idx;
	uint16_t oldest_idx;
	uint16_t rec_count;
	uint32_t seq_num;
} RingBuffTrackerStruct;
	  
/* Defines ===================================================================*/

extern TelemetryStruct telem_ptr[TELEM_MAX_REC_CNT_BUFF];
extern SystemStatusStruct sysstat_ptr[SYSSTAT_MAX_REC_CNT_BUFF];
extern RingBuffTrackerStruct rb_handle[REC_TYP_CNT_TO_LOG];
/* Function Prototypes =======================================================*/
ReturnStatusEnum loggerInit(void);
uint16_t loggerUpdateIdx(uint16_t buff_idx, uint16_t inc_count,
						 uint16_t dec_count, uint16_t max_buff_size);
void loggerFreeRecordTrackers(void);
/*LOGGER functions*/
uint32_t* loggerWrRecToRingBuff(uint8_t *stat_ptr, uint16_t rec_cnt, 
							RecordTypEnum rec_typ);
ReturnStatusEnum loggerGetRecFromRingBuff(uint32_t *buff_ptr, 
										  RecordTypEnum rec_typ,
										  uint16_t num_record,
										  uint16_t *ret_rec_cnt);
ReturnStatusEnum loggerSaveRec(uint32_t *src_addr_ptr,
							   uint16_t record_num,
							   RecordTypEnum rec_typ);
ReturnStatusEnum loggerBackUpRecords(void);

ReturnStatusEnum loggerFetchRecFromRingBuff(uint8_t **buff_ptr, 
										  RecordTypEnum rec_typ,
										  uint16_t num_record,
										  uint16_t *ret_rec_cnt);
#endif /* __LOGGERMAIN_H */

/*******************************************************************************
  * Revision History
  * @file       loggerMain.h
  ******************************************************************************
  * @version	v2.2.0
  * @date		04/08/16
  * @author		M.I. Lorica
  * @changes	renamed file to loggerMain.h and renamed and added fxn prototypes
  ******************************************************************************
  * @version	v2.1.0
  * @date		12/23/15
  * @author		M.I. Lorica
  * @changes	changed file from buff.c to dataMgmt.c
  * 			added functions to call specific record functions (Telemetry and
  * 			SysStat only)
  ******************************************************************************
  * @version
  * @date
  * @author         
  * @changes     - created file
  ******************************************************************************
 **/
