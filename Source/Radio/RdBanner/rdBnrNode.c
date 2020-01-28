/******************************************************************************
  * @file		rdBnrNode.c
  * @author		Hardware Team
  * @version	v2.2.0
  * @date		04/08/16
  * @brief		General function module for Banner Radio
  *****************************************************************************
  * @attention
  *
  * COPYRIGHT(c) 2016 AgTech Labs, Inc.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright
  *   	 notice, this list of conditions and the following disclaimer in the
  *   	 documentation and/or other materials provided with the distribution.
  *   3. Neither the name of AgTech Labs, Inc. nor the names of its
  *   	 contributors may be used to endorse or promote products derived from
  *   	 this software without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
  * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  * POSSIBILITY OF SUCH DAMAGE.
  *****************************************************************************
  */

/* Includes =================================================================*/
#include <stdlib.h>
#include <string.h>

#include "productCfg.h"
#include "loggerMain.h"
#include "radio.h"
#include "rdBnrMain.h"
#include "rdBnrNode.h"
#include "extFlash.h"
#include "agBoardInit.h"
#include "utilMem.h"
#include "port.h"
#include "daqPrintUtil.h"
#include "cell.h"

#if DEVICE_TYPE == DEVICETYPE_NODE

#define MAX_EXT_FL_FWBUFF   (1280)

//extern TelemetryStruct *telem_ptr;
static void rdBnrNodeDataTx(RdHeadStruct *buff_head, uint8_t *data_ptr, uint32_t databuff_size, uint32_t rec_count);

/* Functions ================================================================*/
/*============================================================================
* @brief    Checking in Between Loop of Data Transmission if STOP Command Raise
* @param	none
* @retval	Function status
============================================================================*/
static RdStatEnum rdBnrNodeInBtwnLoopTx(void) {
	RdStatEnum retval;			        /*return status*/
	RdHeadStruct buff_header;		    /*header of received data*/
	uint32_t t_start;					/*reference of receiving delay*/

	retval = RDSTAT_NULL;
	rdClear(buff_header);
    
	t_start = HAL_GetTick();
	while (delayTimeDiff(t_start, HAL_GetTick()) < BNRRADIO_RXDLY) {
		if(rdBnrMainHeadRx(&buff_header) == RDSTAT_OK) {
			if(buff_header.function == RDRESP_STOP) {
				retval = RDSTAT_ERROR;
			}
			break;
		}
	}

	if(retval != RDSTAT_ERROR) {
		retval = RDSTAT_OK;
	}
    
	return retval;
}

/*============================================================================
* @brief    Waiting for Gate's Acknowledgment
* @param	arg_count:	count of data to acknowledge
* @retval	Function status
============================================================================*/
static RdStatEnum rdBnrNodeAckWait(uint8_t arg_count) {
	RdStatEnum retval;			        /*return status*/
	uint32_t t_start;					/*reference of receiving delay*/
	RdFrameStruct ackframe;	/*gate's acknowledgment frame*/

    retval = RDSTAT_ERROR;
	t_start = HAL_GetTick();
	while (delayTimeDiff(t_start, HAL_GetTick()) < BNRRADIO_RXDLY) {
		if(uartRxBanner((uint8_t*)&ackframe, RDFRAME_SZ, UART_TIMEOUT_BANNER) == HAL_OK
           && modbusCrcChk((uint8_t*)&ackframe, RDFRAME_SZ) == MODBUS_OK) {
			retval = RDSTAT_OK;
			break;
		}
		else
		{	
			HAL_UART_DeInit(&UART2HandlerDef);
			HAL_UART_Init(&UART2HandlerDef);
		}
	}

	if(retval == RDSTAT_OK && ackframe.header.value1 != arg_count) {
		retval = RDSTAT_ERROR;
	}

	return retval;
}

/*============================================================================
* @brief    Sending records as Gate's query
* @param	que_head:           
            get_rec_cnt:
            data_ptr:
            sent_rec_cnt_ptr:
* @retval	Function status
============================================================================*/
static RdStatEnum rdBnrNodeBulkDataTx(RdHeadStruct que_head
                                      , uint8_t get_rec_cnt
                                      , uint32_t *data_ptr
                                      , uint8_t *sent_rec_cnt_ptr) {
	RdStatEnum retval;			        /*return status*/
	RdHeadStruct rdiobuff_header;	    /*header of response frame*/
	ModbusCrcStruct rdiobuff_crc;		/*crc of response frame*/
	uint8_t *rdiobuff_ptr;				/*buffer of frame to send*/
	uint32_t frame_rec_idx;				/*index of data record per frame*/
	uint8_t loop_cnt;					/*number of loop to complete data tx*/
	uint8_t sending_rec_cnt;			/*number of record to send*/
	uint8_t max_frame_rec;				/*number of record into a frame*/
	uint8_t data_tx_sz;					/*byte size of data to send*/

	/*parameters for chunk data Tx process*/
	rdClear(frame_rec_idx);
	rdClear(*sent_rec_cnt_ptr);
	max_frame_rec = MAX_FRAME_REC[que_head.type];
	sending_rec_cnt = (get_rec_cnt > que_head.value1)
			? que_head.value1: get_rec_cnt;
	loop_cnt = sending_rec_cnt / max_frame_rec;
	if((sending_rec_cnt % max_frame_rec)!= 0) {
		loop_cnt++;
	}

	/*initial memory allocation for data Tx (max)*/
	rdiobuff_ptr = memAlloc(RDFRAME_SZ
			+ (BYTESIZE[que_head.type] * POLL_REC_COUNT[que_head.type]));
	assert(rdiobuff_ptr);

	while(loop_cnt != 0) {
		rdClear(rdiobuff_header);
		rdClear(rdiobuff_crc);
		rdiobuff_header.mb_id = DEVICE_INFO->rad_modbus_id;
		rdiobuff_header.function = RDRESP_GET;
		rdiobuff_header.type = que_head.type;
		if(loop_cnt == 1 && (sending_rec_cnt % max_frame_rec)!= 0) {
			rdiobuff_header.value1 = sending_rec_cnt % max_frame_rec;
		}
		else {
			rdiobuff_header.value1 = (get_rec_cnt >= max_frame_rec)
					? max_frame_rec: get_rec_cnt;
		}
 		data_tx_sz = rdiobuff_header.value1 * BYTESIZE[rdiobuff_header.type];
		frame_rec_idx += data_tx_sz;

		/*data to radio buffer*/
		//rdiobuff_ptr = memReAlloc(rdiobuff_ptr, RDFRAME_SZ + data_tx_sz);
		//assert(rdiobuff_ptr);

		memcpy(rdiobuff_ptr, &rdiobuff_header, RDHEAD_SZ);
		memcpy(rdiobuff_ptr + RDHEAD_SZ, data_ptr + (get_rec_cnt * BYTESIZE[que_head.type])
                   - frame_rec_idx
               , data_tx_sz);

		/*insert crc*/
		rdiobuff_crc = modbusCrcCalc(rdiobuff_ptr, RDHEAD_SZ + data_tx_sz);
		memcpy(rdiobuff_ptr + (RDFRAME_SZ + data_tx_sz - RDCRC_SZ)
               , &rdiobuff_crc
               , RDCRC_SZ);

		/* final stage on loop*/
		uartTxBanner((uint8_t*)rdiobuff_ptr, RDFRAME_SZ + data_tx_sz);
		if(rdBnrNodeInBtwnLoopTx() != RDSTAT_OK) {
			retval = RDSTAT_ERROR;
			break;
		}

		loop_cnt--;
		*sent_rec_cnt_ptr += rdiobuff_header.value1;
		delayMillis(BNRRADIO_DLY);
	}

	free(rdiobuff_ptr);
	return retval;
}

/*============================================================================
* @brief    Sending sensor registration as Gate's query
* @param	none
* @retval	Function status
============================================================================*/
static RdStatEnum rdBnrNodeSenRegTx(void) {
	RdStatEnum retval;                          /*return status*/
	RdHeadStruct header;                        /*transmitting frame header*/
	RdHeadStruct resp_head;                     /*response header*/
	SensorRegistrationStruct *sen_port_addr;    /*pointer of sen-reg*/
	uint8_t loop_idx;                           /*transmitting loop*/
	uint8_t sen_port;                           /*sensor port number*/
	uint8_t *data_addr;                         /*pointer of data to transmit*/
	uint8_t *data_ptr;                          /*data buffer*/

	const uint8_t LAST_BYTESZ = SENREG_REC_SIZE_BYTES % RDBNR_MAX_DATASZ;
	const uint8_t LOOP = SENREG_REC_SIZE_BYTES / RDBNR_MAX_DATASZ
        + ((LAST_BYTESZ > 0) ? 1: 0);

	data_ptr = memAlloc(SENREG_REC_SIZE_BYTES);
	assert(data_ptr);

	rdClear(sen_port);
	rdClear(header);
	retval = RDSTAT_OK;
	sen_port_addr = (SensorRegistrationStruct*)&PortConfig;
	header.mb_id = DEVICE_INFO->rad_modbus_id;
	header.function = RDFUNC_GET;
	header.type = SENSOR_REG;

	while(sen_port < 4) {
		memcpy(data_ptr, sen_port_addr + sen_port, SENREG_REC_SIZE_BYTES);
		sen_port++;
		loop_idx = LOOP;
		data_addr = data_ptr;

		while(loop_idx > 0) {
			header.value1 = sen_port;
			header.value2 = loop_idx;
			if(loop_idx == 1) {
				header.byt_sz = LAST_BYTESZ;
			}
			else {
				header.byt_sz = RDBNR_MAX_DATASZ;
			}

			rdBnrMainDataTx(header, data_addr, header.byt_sz);
			data_addr += header.byt_sz;
			loop_idx--;
		}
		
        rdBnrMainHeadRx(&resp_head);
		if(resp_head.function != RDRESP_ACKNOWLEDGE
           || resp_head.type != RDAPI_SENREG
           || resp_head.value1 != sen_port) {
			retval = RDSTAT_ERROR;
			break;
		}
	}

	free(data_ptr);
	return retval;
}

static RdStatEnum RdAckWait(RdHeadStruct *arg_head, uint8_t get_rec_cnt)
{
	ModbusCrcStruct rdiobuff_crc;
	ModbusCrcStruct rdcalc_crc;
	RdHeadStruct rx_header;
	RdStatEnum retval;
	uint8_t rxdata_cnt = 0;
	if (rdBnrMainHeadRx(&rx_header) == RDSTAT_OK)
	{
		if(HAL_UART_Receive(&UART2HandlerDef, (uint8_t*)&rdiobuff_crc, 2, 1000) == HAL_OK) 
		{
			rdcalc_crc = modbusCrcCalc((uint8_t*)&rx_header, 6);
			if( (rdiobuff_crc.crc_hi<<8 | rdiobuff_crc.crc_lo) 
								== (rdcalc_crc.crc_hi<<8 | rdcalc_crc.crc_lo))
			{
				if(rx_header.mb_id == DEVICE_INFO->rad_modbus_id)
				{
				  	rxdata_cnt += rx_header.value1;
					if(rx_header.value1 > get_rec_cnt)
					{
						//delayMillis(1000);
						loggerSaveRec((uint32_t*)NULL, get_rec_cnt, rx_header.type);		
					}
					else
						loggerSaveRec((uint32_t*)NULL, rx_header.value1, rx_header.type);
					retval = RDSTAT_OK;
				}
			}
			else
				retval = RDSTAT_ERROR;		
		}
		else
		  retval = RDSTAT_ERROR;	
	}
	return retval;
}

static void rdBnrNodeDataTxBulk(RdHeadStruct *buff_head, uint32_t databuff_size, uint32_t rec_count)
{
	uint32_t radioframe_size = 0;
	uint8_t *rdiobuff_ptr = NULL;
	ModbusCrcStruct rdiobuff_crc;
	uint8_t *data_ptr_buff = NULL;
	uint8_t rxdata_cnt = 0;
	uint8_t max_rec_per_frame = ((buff_head->type == TELEMETRY) 
							? MAX_REC_PER_FRAME_TELEM 
							: MAX_REC_PER_FRAME_SYSSTAT);
	uint8_t loopcount = rec_count/max_rec_per_frame;
	TelemetryStruct *telem_data_ptr;
	if(rec_count%max_rec_per_frame != 0)
	  loopcount++;
	uint8_t tx_rec_cnt = 0;
	uint8_t databulk_size = databuff_size * max_rec_per_frame;
	
	/*prepare buffer for frame + data to be transmitted*/
	radioframe_size = RDFRAME_SZ + databulk_size;
	rdiobuff_ptr = memAlloc(radioframe_size);
	
	
	
	for(uint32_t i = 0; i < loopcount; i++)
	{
	  	loggerFetchRecFromRingBuff(&data_ptr_buff
                                     , (RecordTypEnum)buff_head->type
                                     , max_rec_per_frame
                                     ,(uint16_t *)&tx_rec_cnt);
		
		databulk_size = databuff_size*tx_rec_cnt; //struct size of data to be sent * count of data to be sent
		radioframe_size = RDFRAME_SZ + databulk_size; //include the header size
		buff_head->value1 = tx_rec_cnt;
		buff_head->byt_sz = databulk_size;
		
		if(buff_head->type == TELEMETRY)
			data_ptr_buff = (uint8_t *)(telem_ptr + (rb_handle[TELEMETRY].latest_idx - tx_rec_cnt));
		else
			data_ptr_buff = (uint8_t *)(sysstat_ptr + (rb_handle[SYSTEM_STAT].latest_idx - tx_rec_cnt));
		
		memcpy(rdiobuff_ptr, (uint8_t *)buff_head, RDHEAD_SZ);
		memcpy(rdiobuff_ptr + RDHEAD_SZ, data_ptr_buff, databulk_size);
		
		/*insert crc*/
		rdiobuff_crc = modbusCrcCalc(rdiobuff_ptr, radioframe_size - 2);
		rdiobuff_ptr[databulk_size + RDHEAD_SZ] = rdiobuff_crc.crc_hi;
		rdiobuff_ptr[databulk_size + RDHEAD_SZ + 1] = rdiobuff_crc.crc_lo;
		
		/*Transmit header + data + CRC*/
		delayMillis(RD_TRANSMIT_DELAY);
		if(uartTxBanner((uint8_t*)rdiobuff_ptr, radioframe_size) != HAL_OK)
		{
			HAL_UART_DeInit(&UART2HandlerDef);
			HAL_UART_Init(&UART2HandlerDef);			
		}
		/*Wait for Gate ACK that data is received*/
		if( RdAckWait(buff_head, tx_rec_cnt) == RDSTAT_OK)
		{
			rxdata_cnt += tx_rec_cnt;
		}
		else
		  	break;
				
		/*move the data_ptr_buff to the next address to be sent*/
		data_ptr_buff -= (databulk_size);
	}
	#if PRINT_PROCESSES_IN_UART
		logWrite("*Sent: %d \nLoop:%d\n", rxdata_cnt, loopcount);
	#endif
	free(rdiobuff_ptr);
}

static void rdBnrNodeDataTx(RdHeadStruct *buff_head, uint8_t *data_ptr, uint32_t databuff_size, uint32_t rec_count)
{
	uint32_t radioframe_size = 0;
	uint8_t *rdiobuff_ptr = NULL;
	ModbusCrcStruct rdiobuff_crc;
	uint8_t *data_ptr_buff = data_ptr;
	uint8_t rxdata_cnt = 0;
	/*prepare buffer for frame + data to be transmitted*/
	radioframe_size = RDFRAME_SZ + databuff_size;
	rdiobuff_ptr = memAlloc(radioframe_size);
	buff_head->value1 = 1;
	buff_head->byt_sz = (buff_head->type == TELEMETRY ? sizeof(TelemetryStruct) : sizeof(SystemStatusStruct));
	for(uint32_t i = rec_count; i > 0; i--)
	{
		
		memcpy(rdiobuff_ptr, (uint8_t *)buff_head, RDHEAD_SZ);
		memcpy(rdiobuff_ptr + RDHEAD_SZ, data_ptr, databuff_size);
		
		/*insert crc*/
		rdiobuff_crc = modbusCrcCalc(rdiobuff_ptr, radioframe_size - 2);
		rdiobuff_ptr[databuff_size + RDHEAD_SZ] = rdiobuff_crc.crc_hi;
		rdiobuff_ptr[databuff_size + RDHEAD_SZ + 1] = rdiobuff_crc.crc_lo;
		
		/*Transmit header + data + CRC*/
		uartTxBanner((uint8_t*)rdiobuff_ptr, radioframe_size);
//		#if PRINT_RDPROC_IN_UART
//		if(buff_head->type == TELEMETRY)
//			daqTelemPrint((TelemetryStruct*)data_ptr);
//		else
//			sysShow((SystemStatusStruct*)data_ptr);
//		#endif
		delayMillis(RD_TRANSMIT_DELAY);
		if( RdAckWait(buff_head, 1) == RDSTAT_OK)
			rxdata_cnt++;
		else
		  	break;
		data_ptr -= databuff_size;
	}
	#if PRINT_PROCESSES_IN_UART
		logWrite("*Sent : %d Type:%d\n", rxdata_cnt, buff_head->type);
	#endif
	free(rdiobuff_ptr);
}


/*============================================================================
* @brief    Response on Gate's Query
* @param	arg_head:	header from Gate
* @retval	Function status
============================================================================*/
static RdStatEnum rdBnrNodeGetResp(RdHeadStruct arg_head) {
	RdStatEnum retval;			        /*return status*/
    RdHeadStruct buff_head;             /*header of transmitting frame*/
	uint8_t *data_ptr;					/*pointed address of data to send*/
	uint8_t get_rec_cnt;				/*returned count of querying record*/
	uint8_t sent_rec_cnt;				/*number of record sent*/
	retval = RDSTAT_NULL;
	uint32_t databuff_size = 0;
	uint32_t max_rec_per_poll;
	ModbusCrcStruct rdiobuff_crc;

	switch(arg_head.type) {
        case TELEMETRY: case SYSTEM_STAT:
			databuff_size = ((arg_head.type == TELEMETRY) 
							? sizeof(TelemetryStruct) 
							: sizeof(SystemStatusStruct));

            loggerFetchRecFromRingBuff(&data_ptr
                                     , (RecordTypEnum)arg_head.type
                                     , arg_head.value1
                                     ,(uint16_t *)&get_rec_cnt);
                        
            if(get_rec_cnt > arg_head.value1) {
                get_rec_cnt = 0;
            }
			
			if(arg_head.type == TELEMETRY)
			{
				data_ptr = (uint8_t *)(telem_ptr + (rb_handle[TELEMETRY].latest_idx - 1));
				max_rec_per_poll = MAX_REC_PER_FRAME_TELEM;
			}
			else
			{
				data_ptr = (uint8_t *)(sysstat_ptr + (rb_handle[SYSTEM_STAT].latest_idx - 1));
				max_rec_per_poll = MAX_REC_PER_FRAME_SYSSTAT;
			}
			
            rdClear(buff_head);
            buff_head.mb_id = DEVICE_INFO->rad_modbus_id;
            buff_head.type = arg_head.type;
            buff_head.value1 = get_rec_cnt;
			buff_head.value2 = 0;//sizeof(TelemetryStruct)*get_rec_cnt + RDFRAME_SZ;
            buff_head.function = RDRESP_REC_REPORT;
            rdBnrMainHeadTx(buff_head); 	//! no handler if failed
			#if PRINT_PROCESSES_IN_UART
				logWrite("\n*Header Transmitted\n");
				logWrite("*Rec: %d Type: %d\n", get_rec_cnt, arg_head.type);	
			#endif
			//delayMillis(200);
            if(get_rec_cnt == 0) {
                //free(data_ptr);
                return RDSTAT_OK;
            }
			
			/*NIC TEST*/
			delayMillis(RD_DATATX_DELAY);
			//if(get_rec_cnt >= max_rec_per_poll)
				rdBnrNodeDataTxBulk(&buff_head, databuff_size, get_rec_cnt);
			//else
			//	rdBnrNodeDataTx(&buff_head, (uint8_t*)data_ptr, databuff_size, get_rec_cnt);
////////////////////////////////////////////////////////////////////////////////			
//			radiobuff_size = RDFRAME_SZ + databuff_size;
//			memcpy(rdiobuff_ptr, &buff_head, RDHEAD_SZ);
//			memcpy(rdiobuff_ptr + RDHEAD_SZ, data_ptr, databuff_size);
//			/*insert crc*/
//			rdiobuff_crc = modbusCrcCalc(rdiobuff_ptr, radiobuff_size - 2);
//			rdiobuff_ptr[databuff_size + RDHEAD_SZ] = rdiobuff_crc.crc_hi;
//			rdiobuff_ptr[databuff_size + RDHEAD_SZ + 1] = rdiobuff_crc.crc_lo;	
//			uartTxBanner((uint8_t*)rdiobuff_ptr, radiobuff_size);
			
////////////////////////////////////////////////////////////////////////////////			
			/*NIC TEST*/
			/* final stage on loop*/

//            retval = rdBnrNodeBulkDataTx(arg_head
//                                         , get_rec_cnt
//                                         , data_ptr
//                                         , &sent_rec_cnt);

//            if(rdBnrNodeAckWait(sent_rec_cnt) == RDSTAT_OK 
//				&& loggerSaveRec((uint32_t*)data_ptr, sent_rec_cnt, 
//				(RecordTypEnum)arg_head.type) != SUCCESSFUL) 
//			{
//	      		retval = RDSTAT_ERROR;
//			}
//            //free(data_ptr);
            break;
        case SENSOR_REG:
            retval = rdBnrNodeSenRegTx();
            break;
        default:
            retval = RDSTAT_ERROR;
            break;
	}
	return retval;
}

static RdStatEnum rdBnrNodeFwRxData(RdHeadStruct arg_headptr, uint8_t *arg_dataptr) {
	RdStatEnum retval = RDSTAT_ERROR;			    /*return status*/
	uint8_t *frame_ptr;             /*buffer of data to receive*/
	uint8_t frame_sz;               /*bytesize of buffer of data to receive*/

    retval = RDSTAT_NULL;
	frame_ptr = NULL;
    ModbusCrcStruct rdiobuff_crc;
	
//	frame_sz = //arg_headptr.value1 
//	  	((arg_headptr.type == TELEMETRY) ? sizeof(TelemetryStruct) 
//		: sizeof(SystemStatusStruct))*arg_headptr.value1 + RDFRAME_SZ;
	frame_sz = arg_headptr.byt_sz +  RDFRAME_SZ;
	frame_ptr = (uint8_t*)memAlloc(frame_sz);
	assert(frame_ptr);
	memcpy(frame_ptr, &arg_headptr, RDHEAD_SZ);

	if(HAL_UART_Receive(&UART2HandlerDef, 
	(uint8_t*)(frame_ptr + RDHEAD_SZ), frame_sz - RDHEAD_SZ, 3000) == HAL_OK)
	//if(uartRxBanner(frame_ptr + RDHEAD_SZ, frame_sz - RDHEAD_SZ) == HAL_OK)
	{
        rdiobuff_crc = modbusCrcCalc(frame_ptr, frame_sz - 2);
        if( (rdiobuff_crc.crc_hi<<8 | rdiobuff_crc.crc_lo) 
							== (frame_ptr[frame_sz - 2]<<8 | frame_ptr[frame_sz - 1]))
        {
           retval = RDSTAT_OK;
           memcpy(arg_dataptr, frame_ptr + RDHEAD_SZ, arg_headptr.byt_sz);
        }

		//assert(*arg_dataptr);

		
	}
	else 
	{
		HAL_UART_DeInit(&UART2HandlerDef);
		HAL_UART_Init(&UART2HandlerDef);
		retval = RDSTAT_ERROR;
	}
    
	free(frame_ptr);
	return retval;
}

static ReturnStatusEnum RadWriteSPI(uint8_t *buffer, uint32_t addr, uint32_t loop_count)
{
	for(uint32_t i = 0; i < loop_count; i++)
	{
		if(extFlWrite((int8_t*)buffer, addr, SPI_WRMAX) != SUCCESSFUL)
			return FAILED;
		addr += SPI_WRMAX;
		buffer += SPI_WRMAX;
        delayMillis(1);
	}
    return SUCCESSFUL;
}

static void rdBnrNodeTxFWVer(void)
{
	RdHeadStruct tx_head;
	rdClear(tx_head);
	tx_head.mb_id = DEVICE_INFO->rad_modbus_id;
	tx_head.function = RDFUNC_FWVER_CHK;
	tx_head.type = RDAPI_FW;
	tx_head.value1 = FW_VERSION.fwver_main>>8;
	tx_head.value2 = (uint8_t)(FW_VERSION.fwver_submain & 0xFF);
	delayMillis(50);
	rdBnrMainHeadTx(tx_head);
}

/*============================================================================
* @brief    Receiving firmware from gate for update
* @param	header: header from Gate
* @retval	Function status
============================================================================*/
static RdStatEnum rdBnrNodeFwRx(RdHeadStruct arg_head)
{
	RdStatEnum retval;                  /*return status*/
	RdHeadStruct rx_head;               /*header of receiving frame*/
	RdHeadStruct resp_head;             /*header of receiving response*/
	uint8_t rxdata[164];                    /*pointer of receiving data*/
	uint32_t loop;                       /*number of loop to receive data*/
	uint8_t flash_addr;                 /*pointed address of saving firmware*/
    uint32_t fw_next_free_address = NEW_FW_START_SEC_ADDR;
    uint8_t ext_fl_buff[MAX_EXT_FL_FWBUFF];
    uint8_t ext_fl_buff_next_addr = 0;
    uint32_t rxdata_cnt_first_b = 0;
	uint16_t fw_ver = 0;
    retval = RDSTAT_NULL;
	FWStatEnum fw_stat;
	rdClear(rx_head);
	flash_addr = 0;
	extFlSectorErase(6, 0); // !temporary variable arguments
    
	rdClear(resp_head);
	resp_head.mb_id = DEVICE_INFO->rad_modbus_id;
	resp_head.function = RDRESP_UPDATE;
	resp_head.type = RDAPI_FW;
	resp_head.value2 = arg_head.value2;
    
	/*Transmit header to gate to ack start of fw transfer*/
	rdBnrMainHeadTx(resp_head);
	
    //delayMillis(200);
	
	/*calculate number of expected packets*/
    loop = arg_head.value1<<8 | arg_head.value2;
	for(uint32_t i = loop; i > 0; i--)
    {
	  	/*Receive Header from gate*/
		rdBnrMainHeadRx(&rx_head);
		retval = rdBnrNodeFwRxData(rx_head, rxdata);
        if(retval == RDSTAT_OK)
        {
		  	/*Check for wrong characters*/
            for(uint32_t ix = 0; ix <rx_head.byt_sz; ix ++)
            {
                if(rxdata[ix] != 0x3A
                   && rxdata[ix] != 0x0A
                   && rxdata[ix] != 0x0D
                   && (isxdigit(rxdata[ix]) == 0) )
                  assert(0);
            }
			
			/*Transfer data to external flash (maximum is 128 bytes per packet)*/
            if(extFlWrite((int8_t*)rxdata
                          , fw_next_free_address
                         , rx_head.byt_sz) != SUCCESSFUL) 
            {
                retval = RDSTAT_ERROR;
                break;
            }
			
            fw_next_free_address += rx_head.byt_sz;
            retval = RDSTAT_OK;
            flash_addr++;
            resp_head.value1 = arg_head.value1;
            delayMillis(50);
            
            /*send datarx ack*/
            rdBnrMainHeadTx(resp_head);
            
			/*End of Transmission*/
            if(i == 1)
            {
                uartSHOW(rxdata, rx_head.byt_sz);
				/*Get FW Version*/
				rdBnrMainBlankFrRx(&rx_head, 1500);
				fw_ver = rx_head.value1<<8 | rx_head.value2;
				
				/*Verify Firmware*/
                ota_fwsize = fw_next_free_address - NEW_FW_START_SEC_ADDR;
                fw_stat = fileCheck(ota_fwsize);
                if(fw_stat == FWSTAT_OK)
				{
				  	//TODO::
				  	/*Flag FW Update and Reset Board*/
				  	delayMillis(2000);
				}
            }
        }
        else
          break;
	}

	return retval;
}

/*============================================================================
* @brief    Update response
* @param	header: header from Gate
* @retval	Function status
============================================================================*/
static RdStatEnum rdBnrNodeUpdateResp(RdHeadStruct arg_rxhead) {
	RdStatEnum retval;              /*return status*/

	retval = RDSTAT_NULL;
	switch(arg_rxhead.type) {
		case RDAPI_VLVSCHED:
		case RDAPI_VLVDRV:
		case RDAPI_SENDRV:
		case RDAPI_RTC:
		case RDAPI_RSSI:
			break;
		case RDAPI_FW:
			retval = rdBnrNodeFwRx(arg_rxhead);
			break;
		default:
			retval = RDSTAT_ERROR;
	}
	return retval;
}

#pragma optimize=none
static RdStatEnum rdBnrNodeNetDevNumSet(RdHeadStruct arg_head)
{
  	RdStatEnum retval = RDSTAT_ERROR;			    /*return status*/

	#if PRINT_PROCESSES_IN_UART
		logWrite("*Set Modbus ID: 0x%02X!\n", arg_head.mb_id);
	#endif
		
	is_devnum_set = true;
	DEVICE_INFO->rad_modbus_id = (uint8_t)arg_head.mb_id;		
	sdUpdateDeviceRadioModbusID(DEVICE_INFO->rad_modbus_id);
	arg_head.mb_id = DEVICE_INFO->rad_modbus_id;
	#if PRINT_PROCESSES_IN_UART
		logWrite("*Network Modbus ID Set SUCCESS!\n");
	#endif
		
	retval = RDSTAT_OK;
	
	/*update all data acquired from devreg and daq*/
	for(uint32_t i = 0; i < SYSSTAT_MAX_REC_CNT_BUFF; i++)
	{
		sysstat_ptr[i].rad_modbus_id = DEVICE_INFO->rad_modbus_id;
	}
	
	for(uint32_t i = 0; i < TELEM_MAX_REC_CNT_BUFF; i++)
	{
		telem_ptr[i].rad_modbus_id = DEVICE_INFO->rad_modbus_id;
	}	

	rdBnrMainHeadTx(arg_head);
	
	return retval;
}

#pragma optimize=none
static RdStatEnum rdBnrNodeSetNetId(RdHeadStruct arg_head)
{
  	RdStatEnum retval = RDSTAT_ERROR;
    RdHeadStruct header;
    uint32_t net_id;
    
    rdClear(header);
    header.mb_id = DEVICE_INFO->rad_modbus_id;
	header.function = RDFUNC_NETID;
    
    memcpy(&net_id, (uint8_t*)&arg_head+2, sizeof(net_id));
    
    //Update Network Id to SD card
    if(sdUpdateNetworkId(net_id) == SUCCESSFUL)
    {
        header.value1 = RDSTAT_OK;
    }
    else
    {
        header.value1 = RDSTAT_ERROR;
    }
    
    //response to Gate
    rdBnrMainHeadTx(header);
    
    //set Network Id to Radio
    retval = rdBnrMainSetNetId(net_id);
    
    return retval;
}

#pragma optimize=none
static RdStatEnum rdBnrNodeStatCheckAnswer(RdHeadStruct arg_head)
{
  	RdStatEnum retval = RDSTAT_ERROR;
    RdHeadStruct header;
    
    rdClear(header);
    header.mb_id = DEVICE_INFO->rad_modbus_id;
	header.function = RDFUNC_STAT_CHK;
    
    //response to Gate
    retval = rdBnrMainHeadTx(header);
    
    return retval;
}
                                       
#pragma optimize=none
static RdStatEnum rdBnrNodeSleepAnswer(void)
{
  	RdStatEnum retval = RDSTAT_ERROR;
    RdHeadStruct header;
    
    rdClear(header);
    header.mb_id = DEVICE_INFO->rad_modbus_id;
	header.function = RDFUNC_SLEEP;
    
    //response to Gate
    retval = rdBnrMainHeadTx(header);
    
    return retval;
}

#pragma optimize=none
static void rdBnrNodeEnableMaintenanceModeToGate(void)
{
    RdHeadStruct header;
        
    rdClear(header);
    header.mb_id = DEVICE_INFO->rad_modbus_id;
	header.function = RDFUNC_MAINTENANCE;
    header.type = RDNETMAINTENANCEMODETYP_STARTCMD;
    header.value1 = RADNETMAINTENANCEMODE_ENABLE;
    header.value2 = sleep_attr.nmm_timer_min;
    
    //send Command to Gate
    rdBnrMainHeadTx(header);
}

#pragma optimize=none
static void rdBnrNodeDisableMaintenanceModeToGate(void)
{
    RdHeadStruct header;
        
    rdClear(header);
    header.mb_id = DEVICE_INFO->rad_modbus_id;
	header.function = RDFUNC_MAINTENANCE;
    header.type = RDNETMAINTENANCEMODETYP_STOPCMD;
    header.value1 = RADNETMAINTENANCEMODE_DISABLE;
    header.value2 = sleep_attr.nmm_timer_min;
    
    //send Command to Gate
    rdBnrMainHeadTx(header);
}
                                  
#pragma optimize=none
static void rdBnrNodeEditMaintenanceModeTimerToGate(uint8_t a_timer)
{
    RdHeadStruct header;
        
    rdClear(header);
    header.mb_id = DEVICE_INFO->rad_modbus_id;
	header.function = RDFUNC_MAINTENANCE;
    header.type = RDNETMAINTENANCEMODETYP_EDITCMD;
    header.value1 = RADNETMAINTENANCEMODE_ENABLE;
    header.value2 = a_timer;
    
    //send Command to Gate
    rdBnrMainHeadTx(header);
}

#pragma optimize=none
static void rdBnrNodeMaintenanceModeAckToGate(uint8_t a_type)
{
    RdHeadStruct header;
        
    rdClear(header);
    header.mb_id = DEVICE_INFO->rad_modbus_id;
	header.function = RDFUNC_MAINTENANCE;
    header.type = a_type;
    header.value1 = sleep_attr.nmm_status;
    header.value2 = sleep_attr.nmm_timer_min;

    //send Command to Gate
    rdBnrMainHeadTx(header);
}

#pragma optimize=none
static void rdBnrNodeMaintenanceModeReply(RdHeadStruct a_header)
{
    if(a_header.function == RDFUNC_MAINTENANCE)
    {
        switch(a_header.type) {
        case RDNETMAINTENANCEMODETYP_STOPCMD:
            sleep_attr.nmm_timer_min = 0;
            sleep_attr.nmm_status = RADNETMAINTENANCEMODE_DISABLE;
            rdBnrNodeMaintenanceModeAckToGate(a_header.type);
			break;
        case RDNETMAINTENANCEMODETYP_STARTCMD:
            sleep_attr.nmm_status = RADNETMAINTENANCEMODE_ENABLE;
        case RDNETMAINTENANCEMODETYP_EDITCMD:
            sleep_attr.nmm_timer_min = a_header.value2;
        case RDNETMAINTENANCEMODETYP_CHECKCMD:
            rdBnrNodeMaintenanceModeAckToGate(a_header.type);
            break;
        case RDNETMAINTENANCEMODETYP_RSSIDATA:
            //update rssi
            m_radio_signal_strength = a_header.value2;
            rdBnrNodeMaintenanceModeAckToGate(a_header.type);
        }        
    }
}

/*============================================================================
* @brief    Receiver function called by UART interrupt
* @param	header: received header from Gate via interrupt
* @retval	Function status
============================================================================*/
void rdBnrNodeReceiver(RdHeadStruct arg_head) {    
    switch(sleep_attr.nmm_status) {
    case RADNETMAINTENANCEMODE_DISABLE:
        switch(arg_head.function) {
		case RDFUNC_GET:
			rdBnrNodeGetResp(arg_head);
			break;
		case RDFUNC_UPDATE:
			rdBnrNodeUpdateResp(arg_head);
			break;
		case RDFUNC_SLEEP:
            sleep_attr.enter_sleepmode = true;
			RTC_SLEEPTIME = arg_head.value1<<8 | arg_head.value2;
            rdBnrNodeSleepAnswer();
			break;
		case RDFUNC_FWVER_CHK:
		  	rdBnrNodeTxFWVer();
		  	break;
		case RDFUNC_NETDEVNUMSET:
		  	rdBnrNodeNetDevNumSet(arg_head);
			break;
        case RDFUNC_NETID:
            rdBnrNodeSetNetId(arg_head);
            break;
        case RDFUNC_STAT_CHK:
            rdBnrNodeStatCheckAnswer(arg_head);
            break;
        case RDFUNC_MAINTENANCE:
            rdBnrNodeMaintenanceModeReply(arg_head); 
            break;
		default:
            break;
        }
        break;
    case RADNETMAINTENANCEMODE_ENABLE:
        rdBnrNodeMaintenanceModeReply(arg_head);        
        break;
    case RADNETMAINTENANCEMODE_TOBEENABLED:
        if(arg_head.function != RDFUNC_MAINTENANCE)
            rdBnrNodeEnableMaintenanceModeToGate();
        else
            rdBnrNodeMaintenanceModeReply(arg_head);
        break;
    case RADNETMAINTENANCEMODE_TOBEDISABLED:
        if(arg_head.function != RDFUNC_MAINTENANCE)
            rdBnrNodeDisableMaintenanceModeToGate();
        else
            rdBnrNodeMaintenanceModeReply(arg_head);
        break;
    default:
        break;
    }
}

////temporary function for the sake of mobile team's testing using BLE
//uint16_t rdBnrNodeRssiRx(void)
//{
//	BnrRadioDefFrameStruct rx_rssi;
//	uint32_t t_start;
//
//	CLEAR(rx_rssi);
//
//	t_start = HAL_GetTick();
//	while(delayTimeDiff(t_start, HAL_GetTick()) < BNRRADIO_RXDLY)
//	{
//		if(uartRxBanner((uint8_t*)&rx_rssi, RDFRAME_SZ) == HAL_OK)
//		{
//			return rx_rssi.header.value;
//		}
//	}
//	return 0;
//}

#endif //DEVICE_TYPE
/******************************************************************************
  * Revision History
  *	@file      	rdBnrNode.c
  *****************************************************************************
  * @version	v2.2.0
  * @date		04/08/16
  * @author		J. Fajardo
  * @changes
  *****************************************************************************
  * @version	v2.1
  * @date		01/12/2015
  * @author		J. Fajardo
  * @changes	initial release for V2.1 board
  *****************************************************************************
  */
