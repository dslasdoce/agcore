 /*
  ******************************************************************************
  * @file      	loggerMain.c
  * @author     Hardware Team
  * @version	v2.2.0
  * @date		04/08/16		
  * @brief		
  ******************************************************************************
  * @attention
  *
  * COPYRIGHT(c) 2016 AgTech Labs, Inc

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
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <deployAssert.h>

#include "productCfg.h"
#include "loggerMain.h"
#include "utilMem.h"
#include "intFlash.h"
#include "dmMain.h"

/* Typedefs ==================================================================*/


/* Global Declarations =======================================================*/
TelemetryStruct telem_ptr[TELEM_MAX_REC_CNT_BUFF];
SystemStatusStruct sysstat_ptr[SYSSTAT_MAX_REC_CNT_BUFF];

/* External Declarations======================================================*/
/*sram trackers*/
RingBuffTrackerStruct rb_handle[REC_TYP_CNT_TO_LOG];

/* Defines ===================================================================*/
#define TELEM_REC_CNT_TO_DMA				50
#define SYSSTAT_REC_CNT_TO_DMA				25

/* Get max rec allocated in the sram ring buff based on rec type*/	  	  
#define MAX_REC_CNT_RING_BUFF(rec_typ)		((rec_typ == TELEMETRY) ? \
												TELEM_MAX_REC_CNT_BUFF: \
												SYSSTAT_MAX_REC_CNT_BUFF)	  

/* Get total rec count to be DMAed */	  	  
#define REC_CNT_DMA(rec_typ)				((rec_typ == TELEMETRY) ? \
												TELEM_REC_CNT_TO_DMA: \
												SYSSTAT_REC_CNT_TO_DMA)	  


/* Get struct type to use based on record type for the sram ring buff*/
#define REC_RING_BUFF_HANDLE(rec_typ)		((rec_typ == TELEMETRY) ? \
												(uint32_t *)telem_ptr:\
												(uint32_t *)sysstat_ptr)	  

/* Functions =================================================================*/
	  
/*==============================================================================
 * @brief	initialize buffer tracker of each record module
 * @param	none
 * @retval  status
==============================================================================*/
ReturnStatusEnum loggerInit(void)
{
	uint8_t i;
	
	/*Initialize sram ring buffer (rb) tracker*/
	for(i = 0; i < REC_TYP_CNT_TO_LOG; i++)
	{	
		rb_handle[i].latest_idx = 0;
		rb_handle[i].oldest_idx = 0;
		rb_handle[i].rec_count = 0;
		rb_handle[i].seq_num = 0;
	}
	
	//telem_ptr = (TelemetryStruct *)memAlloc(TELEM_MAX_REC_CNT_BUFF * TELEM_REC_SIZE_BYTES);
	//sysstat_ptr = (SystemStatusStruct *)memAlloc(SYSSTAT_MAX_REC_CNT_BUFF 
	//					   						* SYSSTAT_REC_SIZE_BYTES);
	
	return SUCCESSFUL;
}

/*============================================================================
 * @brief	updates index of a ring buffer
 * @param	buff_idx: index to update
 * @param	inc_count:
 * @param	dec_count:
 * @param	max_buff_size: maximum buffer size
 * @retval  idx: updated index number
============================================================================*/
uint16_t loggerUpdateIdx(uint16_t buff_idx, uint16_t inc_count,
						 uint16_t dec_count, uint16_t max_buff_size)
{
	uint16_t idx;

	if((inc_count > 0) && (dec_count == 0))
		idx	= (buff_idx + inc_count) % max_buff_size;
	else	//dec_count > 0
	{
		idx  = (buff_idx + max_buff_size) - dec_count;
				//circular buffer idx mgmt
		if(idx >= max_buff_size)
			idx = buff_idx - dec_count;
	}

	return idx;
}

/*============================================================================
 * @brief
 * @param
 * @retval
============================================================================*/
void loggerFreeRingBuff(void)
{
	free(telem_ptr);
	free(sysstat_ptr);
	
}

/*============================================================================
 * @brief	Executes DMA read to get requested number of unsent/nonposted
 * 			records from flash
 * @param	record_num:
 * @retval  return status:
============================================================================*/
static ReturnStatusEnum loggerReadUnSentFlBuff(uint16_t record_num,
											   RecordTypEnum rec_typ)
{
	uint32_t *dest_buff_ptr = NULL;
	TelemetryStruct *telem_buff_ptr;
	SystemStatusStruct *sysstat_buff_ptr;
	uint32_t fl_addr;
	uint16_t data_size;
	uint16_t unsent_rec_count, avail_rec, rec_delta;	
	
	/*check for records availability*/
	unsent_rec_count = fl_handle[rec_typ].unsent_rec_count;

	if(unsent_rec_count == 0)
		return NOT_AVAIL;
	else if(unsent_rec_count >= record_num)
		unsent_rec_count = record_num;

	if(fl_handle[rec_typ].unsent_rec_cnt_in_sr == unsent_rec_count)
	{
		return SUCCESSFUL;
	}

	fl_addr = fl_handle[rec_typ].unsent_tracker_addr;

	data_size = 0;
	avail_rec = unsent_rec_count;

	data_size = REC_SZ(rec_typ) * avail_rec;
	dest_buff_ptr = (uint32_t *)memAlloc(data_size);
	
	switch(rec_typ)
	{
	case TELEMETRY:
		telem_buff_ptr = (TelemetryStruct *)dest_buff_ptr;
		break;
	case SYSTEM_STAT:
		sysstat_buff_ptr = (SystemStatusStruct *)dest_buff_ptr;
		break;
	default: 
		break;
	}
	
	while(1)
	{
		fl_addr -= data_size;
		if(intFlDmaExec(fl_addr, (uint32_t)dest_buff_ptr,
					    (data_size / sizeof(uint32_t))) != SUCCESSFUL)
		{
			free(dest_buff_ptr);
			return FAILED;
		}

		rec_delta = 0;
		for(uint16_t i = 0, j = avail_rec - 1; i < avail_rec; i++, j--)
		{
			
			switch(rec_typ)
			{
			case TELEMETRY:
			if(telem_buff_ptr[j].unsent_flag == true)
			{
				rec_delta++;
				/*overwrite seq num with flash address
				 * to be used in tracking sent records*/
				telem_buff_ptr[j].rad_modbus_id
						= fl_addr + (TELEM_REC_SIZE_BYTES * j);
				rb_handle[TELEMETRY].oldest_idx
						= loggerUpdateIdx(rb_handle[TELEMETRY].oldest_idx,
										  0, 1,
										  TELEM_MAX_REC_CNT_BUFF);

				memcpy(&telem_ptr[rb_handle[TELEMETRY].oldest_idx],
					   &telem_buff_ptr[j],
					   TELEM_REC_SIZE_BYTES);
				telem_ptr[rb_handle[TELEMETRY].oldest_idx].backup_flag = true;		
			}	
			break;
			case SYSTEM_STAT:
			if(sysstat_buff_ptr[j].unsent_flag == true)
			{
				rec_delta++;
				/*overwrite seq num with flash address
				 * to be used in tracking sent records*/
				sysstat_buff_ptr[j].rad_modbus_id
						= fl_addr + (SYSSTAT_REC_SIZE_BYTES * j);
				rb_handle[SYSTEM_STAT].oldest_idx
					= loggerUpdateIdx(rb_handle[SYSTEM_STAT].oldest_idx,
									  0, 1,
									  SYSSTAT_MAX_REC_CNT_BUFF);

				memcpy(&sysstat_ptr[rb_handle[SYSTEM_STAT].oldest_idx],
					   &sysstat_buff_ptr[j],
					   SYSSTAT_REC_SIZE_BYTES);
				sysstat_ptr[rb_handle[SYSTEM_STAT].oldest_idx].backup_flag = true;		
			}	
			break;
			default: 
				break;
			}	
			
		}
		
	}
	free(dest_buff_ptr);

	fl_handle[rec_typ].unsent_rec_cnt_in_sr += unsent_rec_count;
	fl_handle[rec_typ].unsent_tracker_addr
			-= (unsent_rec_count * REC_SZ(rec_typ));
	rb_handle[rec_typ].rec_count += unsent_rec_count;
	return SUCCESSFUL;
}

/*============================================================================
 * @brief	program/write unsent/nonposted records to flash
 * 			from sram buff during the following processes:
 * 			a. DAQ: POD [WR_UNSENT]
 * 			b. RADIO Rx: GATE [WR_UNSENT]
 * @param	record_num:
 * @retval  return status
============================================================================*/
static ReturnStatusEnum loggerFlushUnSent(uint16_t record_num, 
										  RecordTypEnum rec_typ)
{	
	TelemetryStruct *telem_dataptr;
	SystemStatusStruct *sysstat_dataptr;
	uint32_t *data_ptr;
	uint32_t *temp_data_ptr;
	
	uint32_t data_size;
	uint16_t idx;
	uint16_t bu_rec_cnt;	
	
	data_size = REC_SZ(rec_typ) * record_num;
	data_ptr = (uint32_t *)memAlloc(data_size);
	telem_dataptr = (TelemetryStruct *)data_ptr;
	sysstat_dataptr = (SystemStatusStruct*)data_ptr;
	
	/* check flash bu buff is already full */
	if((fl_handle[rec_typ].last_rec_addr + data_size) > FL_END_TEMP_ADDR(rec_typ))
	{
		/* if yes, erase SECTORS allocated to corresponding record type*/
		if(intFlRecEraseTempBuff(rec_typ, BU_SEC_CNT(rec_typ)) != FAILED)
			assert(0);
	}

	/* - Check if the records to be flushed is not yet
	 *   saved on bu buff of flash.
	 * - Use data_ptr to contain the records
	 *   */
	idx = loggerUpdateIdx(rb_handle[rec_typ].oldest_idx,
						record_num - 1, 0, MAX_REC_CNT_RING_BUFF(rec_typ));
	bu_rec_cnt = record_num;
	
	/* Only flush records that are not yet in the FL_TEMP_BUFF
	 * Sort these by checking backup_flag */
	switch(rec_typ)
	{
	case TELEMETRY:
		while(	 (telem_ptr[idx].backup_flag == false) 
			  && (bu_rec_cnt > 0))
		{
			bu_rec_cnt--;
			temp_data_ptr = (uint32_t *)&telem_dataptr[bu_rec_cnt];
			memcpy(temp_data_ptr,
				   &telem_ptr[idx],
				   TELEM_REC_SIZE_BYTES);
			idx = loggerUpdateIdx(idx, 0, 1, TELEM_MAX_REC_CNT_BUFF);
		}				
		break;
	case SYSTEM_STAT:
		while(	 (sysstat_dataptr[idx].backup_flag == false) 
			  && (bu_rec_cnt > 0))
		{
			bu_rec_cnt--;
			temp_data_ptr = (uint32_t *)&sysstat_dataptr[bu_rec_cnt];
			memcpy(temp_data_ptr,
				   &sysstat_ptr[idx],
				   SYSSTAT_REC_SIZE_BYTES);
			idx = loggerUpdateIdx(idx, 0, 1, SYSSTAT_MAX_REC_CNT_BUFF);
		}
		break;
	default:
		break;
	}
	
	/* update oldest_idx by record_num, regardless of backup_flag */
	rb_handle[rec_typ].oldest_idx
				= loggerUpdateIdx(rb_handle[rec_typ].oldest_idx,
								record_num, 0,
								MAX_REC_CNT_RING_BUFF(rec_typ));
	/* if at least one of the records subject for DMA has already been
	 *  backed-up in FL_TEMP_BUFF, recompute data_size needed for dma*/
	if(bu_rec_cnt > 0)
	{
		data_size = REC_SZ(rec_typ) * (record_num - bu_rec_cnt);
		fl_handle[rec_typ].unsent_rec_cnt_in_sr -= (record_num - bu_rec_cnt);
	}

	/* Execute dmaExecute ONLY IF at least one of the records subject for DMA
	 * is not yet backed-up in FL_TEMP_BUFF */
	if(bu_rec_cnt != record_num)
	{
		/* check if flash is full */
		if((	fl_handle[rec_typ].last_rec_addr + data_size) 
		     >= FL_END_TEMP_ADDR(rec_typ))
		{
			/* 1.TODO: to follow algo for transferring UNSENT records to sdCard*/
			/* 2. IfFlase Erase all records
			 * */
			free(data_ptr);
			return NO_MEM_AVAIL;
		}
		switch(rec_typ)
		{
		case TELEMETRY:
			break;
		case SYSTEM_STAT:
			break;
		default:
			break;
		}		
		if(intFlDmaExec((uint32_t)temp_data_ptr,
						fl_handle[rec_typ].last_rec_addr,
						(data_size / sizeof(uint32_t))) != SUCCESSFUL)
		{
			free(data_ptr);
			return FAILED;
		}

	}
	/* Update trackers of flash and sram buffers*/
	intFlUpdateTracker(TEMP_BUFF_TRACKER, record_num - bu_rec_cnt, rec_typ);
	rb_handle[rec_typ].rec_count -= record_num;

	free(data_ptr);
	return SUCCESSFUL;
}

ReturnStatusEnum loggerFetchRecFromRingBuff(uint8_t **buff_ptr, 
										  RecordTypEnum rec_typ,
										  uint16_t num_record,
										  uint16_t *ret_rec_cnt)
{
	//TelemetryStruct *telem_dataptr;
	//SystemStatusStruct *sysstat_dataptr;
	uint16_t rec_count = 0;
	ReturnStatusEnum status;
	uint16_t idx;
	uint16_t rec_cnt_to_dma;
	uint16_t i;
	
	rec_count = num_record;

//	if(rb_handle[rec_typ].rec_count < num_record)
//	{
//		rec_cnt_to_dma = REC_CNT_DMA(rec_typ);
//
//		if(	   rb_handle[rec_typ].latest_idx 
//		    == loggerUpdateIdx(rb_handle[rec_typ].oldest_idx,
//							   0, REC_CNT_DMA(rec_typ),
//							   MAX_REC_CNT_RING_BUFF(rec_typ)))
//		{
//			rec_cnt_to_dma = REC_CNT_DMA(rec_typ) - 1;
//		}
//
//		/* get records from flash */
//		status = loggerReadUnSentFlBuff(rec_cnt_to_dma, rec_typ);
//		/*If rec_count is still not enough after getting*/
//		if((	status == NOT_AVAIL)
//			|| (rb_handle[rec_typ].rec_count < num_record))	//to add implementation
//		{
//			rec_count = rb_handle[rec_typ].rec_count;
//		}
//		else if(status != SUCCESSFUL)
//			return FAILED;
//
//	}

	rec_count = (rb_handle[rec_typ].rec_count < num_record)
					? rb_handle[rec_typ].rec_count : num_record;

	idx = loggerUpdateIdx(rb_handle[rec_typ].latest_idx,
						  0, rec_count,
						  MAX_REC_CNT_RING_BUFF(rec_typ));

//	switch(rec_typ)
//	{
//	case TELEMETRY:
//		*buff_ptr = (uint8_t*)(telem_ptr + idx);
//		//buff_ptr += (idx*32);
////		telem_dataptr = (TelemetryStruct *)buff_ptr;
////		for(i = 0; i < rec_count; i++)
////		{
////			memcpy(&telem_dataptr[i],
////				   &telem_ptr[idx],
////				   TELEM_REC_SIZE_BYTES);
////			idx = loggerUpdateIdx(idx, 1, 0, TELEM_MAX_REC_CNT_BUFF);
////		}
//			
//		break;
//	case SYSTEM_STAT:
//		*buff_ptr = (uint8_t *)(sysstat_ptr + idx);
////		sysstat_dataptr = (SystemStatusStruct *)buff_ptr;
////		for(i = 0; i < rec_count; i++)
////		{
////			memcpy(&sysstat_dataptr[i],
////				   &sysstat_ptr[idx],
////				   SYSSTAT_REC_SIZE_BYTES);
////			idx = loggerUpdateIdx(idx, 1, 0, SYSSTAT_MAX_REC_CNT_BUFF);
////		}
//		break;
//			
//	default:
//		break;
//	}
	/*available record count*/
	*ret_rec_cnt = rec_count;
	return SUCCESSFUL;
}

/*============================================================================
* @brief	Returns the total number of logged records from the buffer
* 			to be sent either via:
* 			1. radio interface
* 			2. cloud interface
* @param	radio_buff_ptr:
* @param	record_type:
* @param	num_record:
* @param	radio_rec_count:
* @retval  	return status
============================================================================*/
ReturnStatusEnum loggerGetRecFromRingBuff(uint32_t *buff_ptr, 
										  RecordTypEnum rec_typ,
										  uint16_t num_record,
										  uint16_t *ret_rec_cnt)
{
	TelemetryStruct *telem_dataptr;
	SystemStatusStruct *sysstat_dataptr;
	uint16_t rec_count = 0;
	ReturnStatusEnum status;
	uint16_t idx;
	uint16_t rec_cnt_to_dma;
	uint16_t i;
	
	rec_count = num_record;

//	if(rb_handle[rec_typ].rec_count < num_record)
//	{
//		rec_cnt_to_dma = REC_CNT_DMA(rec_typ);
//
//		if(	   rb_handle[rec_typ].latest_idx 
//		    == loggerUpdateIdx(rb_handle[rec_typ].oldest_idx,
//							   0, REC_CNT_DMA(rec_typ),
//							   MAX_REC_CNT_RING_BUFF(rec_typ)))
//		{
//			rec_cnt_to_dma = REC_CNT_DMA(rec_typ) - 1;
//		}
//
//		/* get records from flash */
//		status = loggerReadUnSentFlBuff(rec_cnt_to_dma, rec_typ);
//		/*If rec_count is still not enough after getting*/
//		if((	status == NOT_AVAIL)
//			|| (rb_handle[rec_typ].rec_count < num_record))	//to add implementation
//		{
//			rec_count = rb_handle[rec_typ].rec_count;
//		}
//		else if(status != SUCCESSFUL)
//			return FAILED;
//
//	}

	rec_count = (rb_handle[rec_typ].rec_count < num_record)
					? rb_handle[rec_typ].rec_count : num_record;

	idx = loggerUpdateIdx(rb_handle[rec_typ].latest_idx,
						  0, rec_count,
						  MAX_REC_CNT_RING_BUFF(rec_typ));

	switch(rec_typ)
	{
	case TELEMETRY:
		telem_dataptr = (TelemetryStruct *)buff_ptr;
		for(i = 0; i < rec_count; i++)
		{
			memcpy(&telem_dataptr[i],
				   &telem_ptr[idx],
				   TELEM_REC_SIZE_BYTES);
			idx = loggerUpdateIdx(idx, 1, 0, TELEM_MAX_REC_CNT_BUFF);
		}
			
		break;
	case SYSTEM_STAT:
		sysstat_dataptr = (SystemStatusStruct *)buff_ptr;
		for(i = 0; i < rec_count; i++)
		{
			memcpy(&sysstat_dataptr[i],
				   &sysstat_ptr[idx],
				   SYSSTAT_REC_SIZE_BYTES);
			idx = loggerUpdateIdx(idx, 1, 0, SYSSTAT_MAX_REC_CNT_BUFF);
		}
		break;
			
	default:
		break;
	}
	/*available record count*/
	*ret_rec_cnt = rec_count;
	return SUCCESSFUL;
}




/*============================================================================
 * @brief	gets/prepares corresponding record buffer requested during DAQ
 * 			both in POD and GATE
 * @param	
 * @retval  
============================================================================*/
uint32_t* loggerWrRecToRingBuff(uint8_t *stat_ptr, uint16_t rec_cnt, 
							RecordTypEnum rec_typ)
{
	uint32_t *buff_ptr;
	
	*stat_ptr = SUCCESSFUL;
	
	switch(rec_typ)
	{
	case TELEMETRY:
		telem_ptr[rb_handle[rec_typ].latest_idx].backup_flag = 0;
		telem_ptr[rb_handle[rec_typ].latest_idx].unsent_flag = 1;
	
		buff_ptr = (uint32_t *)&telem_ptr[rb_handle[rec_typ].latest_idx];
		break;
	case SYSTEM_STAT:
		sysstat_ptr[rb_handle[rec_typ].latest_idx].backup_flag = 0;
		sysstat_ptr[rb_handle[rec_typ].latest_idx].unsent_flag = 1;
	
		buff_ptr = (uint32_t *)&sysstat_ptr[rb_handle[rec_typ].latest_idx];	
		break;
	default:
		break;
	
	}
		
	/*Update latest idx*/
	rb_handle[rec_typ].latest_idx
			= loggerUpdateIdx(rb_handle[rec_typ].latest_idx, rec_cnt, 0,
							  MAX_REC_CNT_RING_BUFF(rec_typ));
	
	if(rb_handle[rec_typ].latest_idx == rb_handle[rec_typ].oldest_idx)
	{
		/* if there is no enough space in the buffer for another record,
		 * then move TOTAL_TELEM_REC_TO_DMA records to flash*/
		//*stat_ptr = loggerFlushUnSent(REC_CNT_DMA(rec_typ), rec_typ);		
		for(uint32_t i = 0; i < REC_TYP_CNT_TO_LOG; i++)
		{	
			rb_handle[rec_typ].latest_idx = 0;
			rb_handle[rec_typ].oldest_idx = 0;
			rb_handle[rec_typ].rec_count = 0;
			rb_handle[rec_typ].seq_num = 0;
		}
		
		switch(rec_typ){ 
		case TELEMETRY:
			buff_ptr = (uint32_t *)&telem_ptr[rb_handle[rec_typ].latest_idx];
			break;
		case SYSTEM_STAT:
			buff_ptr = (uint32_t *)&sysstat_ptr[rb_handle[rec_typ].latest_idx];	
			break;
		default:
			break;
		}
		
		/*Update latest idx*/
		rb_handle[rec_typ].latest_idx
			= loggerUpdateIdx(rb_handle[rec_typ].latest_idx, rec_cnt, 0,
							  MAX_REC_CNT_RING_BUFF(rec_typ));
	}
	/*Update record count*/
	rb_handle[rec_typ].rec_count++;
	/* TODO: to add implementation when flash buff is full*/
	if(rb_handle[rec_typ].rec_count > MAX_REC_CNT_RING_BUFF(rec_typ))
	{
		*stat_ptr = FAILED;
	}

	return buff_ptr;
}


/*============================================================================
 * @brief	move records to flash from sram ring buff during the following
 * 			processes:
 * 			a. Successful RADIO Tx: POD
 * 			b. Successful CLOUD POST: GATE
 * @param	src_addr:
 * @param	record_num:
 * @retval  return status
============================================================================*/
ReturnStatusEnum loggerSaveRec(uint32_t *src_addr_ptr,
									   uint16_t record_num,
									   RecordTypEnum rec_typ)
{
	//TelemetryStruct telem_sent_rec_buff = {0};
	//SystemStatusStruct sysstat_sent_rec_buff = {0};
	//TelemetryStruct *telem_src_ptr, *telem_buff_ptr;
	//SystemStatusStruct *sysstat_src_ptr, *sysstat_buff_ptr;
	//uint32_t data_size;
	//uint16_t bu_rec_count;
	
	//bu_rec_count = 0;
	//data_size = REC_SZ(rec_typ) * record_num;
	
	/* a. check first if there is enough space for total num of records to be
	 * to be flushed to sent-buff*/
//	if(	  (fl_handle[rec_typ].sent_last_rec_addr + data_size) 
//	    > FL_END_TEMP_ADDR(rec_typ))
//	{
//		/*TODO: this is incomplete. still need to transfer records to sdcard
//		 		before erasing the sent buff sector*/
//		if(intFlEraseSentBuff() != FAILED)
//			return NO_MEM_AVAIL;
//	}
	
//	switch(rec_typ)
//	{
//	case TELEMETRY:
//		telem_src_ptr = (TelemetryStruct*)src_addr_ptr;
//		telem_buff_ptr = (TelemetryStruct*)memAlloc(data_size);
//	
//		/* b. then copy all the records that has backup_flag == true*/
//		for(uint16_t i = 0; i < record_num; i++)
//		{
//			if(telem_src_ptr[i].backup_flag == true)
//			{
//				memcpy(&telem_buff_ptr[bu_rec_count],
//					   &telem_sent_rec_buff,
//					   TELEM_REC_SIZE_BYTES);
//				bu_rec_count++;
//
//			}
//		}
//		/* c. update sent records in FL Temp buff with
//		 * 	  unsent_flag = false, by writing 0 values to record contents*/
//		if(bu_rec_count > 0)
//		{
//			/*use addr overwritten to sequence_num to get fl_addr of sent records*/
//			if(intFlDmaExec((uint32_t)telem_buff_ptr,
//							telem_src_ptr[0].device_num, /*oldest rec */
//							((bu_rec_count * TELEM_REC_SIZE_BYTES) / sizeof(uint32_t)))
//								!= SUCCESSFUL)
//				return FAILED;
//			
//			/* update non-sent flash tracker*/
//			intFlUpdateTracker(TEMP_UNSENT_BUFF_TRACKER, bu_rec_count, rec_typ);
//		}
//		free(telem_buff_ptr);
//			
//		break;
//	case SYSTEM_STAT:
//		sysstat_src_ptr = (SystemStatusStruct*)src_addr_ptr;
//		sysstat_buff_ptr = (SystemStatusStruct*)memAlloc(data_size);
//	
//		/* b. then copy all the records that has backup_flag == true*/
//		for(uint16_t i = 0; i < record_num; i++)
//		{
//			if(sysstat_src_ptr[i].backup_flag == true)
//			{
//				memcpy(&sysstat_buff_ptr[bu_rec_count],
//					   &sysstat_sent_rec_buff,
//					   SYSSTAT_REC_SIZE_BYTES);
//				bu_rec_count++;
//
//			}
//		}
//		/* c. update sent records in FL Temp buff with
//		 * 	  unsent_flag = false, by writing 0 values to record contents*/
//		if(bu_rec_count > 0)
//		{
//			/*use addr overwritten to sequence_num to get fl_addr of sent records*/
//			if(intFlDmaExec((uint32_t)sysstat_buff_ptr,
//							sysstat_src_ptr[0].device_num, /*oldest rec */
//						  ((bu_rec_count * SYSSTAT_REC_SIZE_BYTES) / sizeof(uint32_t)))
//								!= SUCCESSFUL)
//				return FAILED;
//			
//			/* update non-sent flash tracker*/
//			intFlUpdateTracker(TEMP_UNSENT_BUFF_TRACKER, bu_rec_count, rec_typ);
//		}
//		free(sysstat_buff_ptr);		
//		break;
//			
//	default:
//		break;
//	}
//	
//	/*d: flush sent records in sent fl buff*/
//	if(intFlDmaExec((uint32_t)src_addr_ptr,
//					fl_handle[rec_typ].sent_last_rec_addr,
//					((REC_SZ(rec_typ) * record_num)/ sizeof(uint32_t)))
//						!= SUCCESSFUL)
//	intFlUpdateTracker(SENT_BUFF_TRACKER, record_num, rec_typ);
	
	/* update sram buff tracker */
  	if(record_num > rb_handle[rec_typ].rec_count)
	{
		rb_handle[rec_typ].rec_count  = 0;
		rb_handle[rec_typ].latest_idx = 0;
		rb_handle[rec_typ].oldest_idx = 0;
		rb_handle[rec_typ].seq_num    = 0;
	}
	else
	{
		rb_handle[rec_typ].rec_count -= record_num;
		rb_handle[rec_typ].latest_idx =
		loggerUpdateIdx(rb_handle[rec_typ].latest_idx, 0, 
					record_num, MAX_REC_CNT_RING_BUFF(rec_typ));
	}
	return SUCCESSFUL;
}

/*============================================================================
 * @brief	called before every unexpected reset
 * @param
 * @retval  status
============================================================================*/
ReturnStatusEnum loggerBackUpRecords(void)
{	
	uint8_t i;
	for(i = 0; i < REC_TYP_CNT_TO_LOG; i++)
	{
		if(loggerFlushUnSent(rb_handle[i].rec_count, (RecordTypEnum)i))
			return FAILED;
	}
	
	return SUCCESSFUL;
}


/*******************************************************************************
  * Revision History
  *	@file      	loggerMain.c
  ******************************************************************************
  * @version	v2.2.0
  * @date		4/08/16
  * @author		MI Lorica
  * @changes	renamed file to loggerMain.h. merged telemetry and systat. added fxns
  ******************************************************************************
  * @version	v2.1.0
  * @date		12/23/15
  * @author		M.I. Lorica
  * @changes	changed file from buff.c to dataMgmt.c
  * 			added functions to call specific record functions (Telemetry and
  * 			SysStat only)
  ******************************************************************************
  * @version	v2.0.4
  * @date
  * @author		Ma. Irene Lorica
  * @changes	- removed buffRelocateRecords
  ******************************************************************************
  * @version	v2.0.3
  * @date
  * @author		Ma. Irene Lorica
  * @changes	- created file
  ******************************************************************************
  */
