/******************************************************************************
  * @file		rdBnrGate.c
  * @author		Hardware Team, Agtech Labs Inc.
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
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>

#include "radio.h"
#include "rdBnrMain.h"
#include "rdBnrGate.h"
#include "productCfg.h"
#include "agBoardInit.h"
#include "extFlash.h"
#include "loggerMain.h"
#include "utilMem.h"
#include "daqPrintUtil.h"
#include "cell.h"
#include "modbus.h"

#if DEVICE_TYPE == DEVICETYPE_GATE

RdHeadStruct rx_header_test;
RdHeadStruct rx_header;
#define HDRX_TIMEOUT	(2000)

typedef struct 
{
    uint16_t rad_dev_addr[RDBNRGATE_MAX_NODE_COUNT];
    char rad_dev_sn[RDBNRGATE_MAX_NODE_COUNT][RADIONODE_MAX_DEVSN_CHARSZ];
} AttachedNodeStruct;
    

/* Functions ================================================================*/
/*============================================================================
* @brief    Receiving number of data to archive from specific Node
* @param	arg_head:   header of receiving frame
* 			retctr:     number of data to archive
* @retval	Function status
============================================================================*/
static RdStatEnum rdBnrGateGetRecSz(RdHeadStruct arg_head, uint32_t *retctr) {
	RdStatEnum retval;				    /*return status*/
				    /*header of received frame*/
	
	memset(&rx_header, 0, sizeof(RdHeadStruct));
	rdClear(*retctr);
	if(HAL_UART_Receive(&UART2HandlerDef, (uint8_t*)&rx_header, RDFRAME_SZ, HDRX_TIMEOUT) == HAL_OK) 
	{
		*retctr = rx_header.value1;
		retval = RDSTAT_OK;
		//delayMillis(10);
	}
	
//	if(rdBnrMainHeadRx(&rx_header) == RDSTAT_OK
//       && rx_header.mb_id == arg_head.mb_id
//       //&& rx_header.function == RDRESP_REC_REPORT
//       //&& rx_header.type == rx_header.type
//	)
//	{
//		*retctr = rx_header.value1;
//		retval = RDSTAT_OK;
//	}
//	else {
//		retval = RDSTAT_ERROR;
//	}

	return retval;
}

/*============================================================================
* @brief    Receiving records from specific node
* @param	arg_id:         unit id
            arg_apitype:    api type
* @retval	Function status
============================================================================*/
static RdStatEnum rdBnrGateGetRec(uint8_t arg_id, RdApiEnum arg_apitype) {
	RdStatEnum retval;			        /*return status*/
    RdHeadStruct tx_header;             /*header of transmitting frame*/
	RdHeadStruct resp_head;             /*header of transmitting response*/
	//RdHeadStruct rx_header;		        /*header of received frame*/
	RdHeadStruct rx_header;
	uint32_t get_rec_cnt;				/*returned number of record in Node*/
	//uint8_t *data_buff_ptr;				/*buffer of data received*/
	uint8_t *radio_databuff;			/*buffer of receiving frame*/
	uint32_t databuff_size = 0;
	uint8_t loopcount;			/*number of frame receiving loop*/
	uint8_t max_frame_rec;				/*max record to query per node*/
	uint8_t databuff_idxaddr;			/*pointed address of data on buffer*/
	uint8_t total_rec_count;			/*total count of record received*/
  	uint8_t rx_rec_count = 0;
	uint8_t *rx_dataptr;
	uint8_t testdata[8];
	uint8_t max_rec_per_frame = (arg_apitype == TELEMETRY 
								 ? MAX_REC_PER_FRAME_TELEM
								 : MAX_REC_PER_FRAME_SYSSTAT);

	#if PRINT_PROCESSES_IN_UART
		char printout[40];
	#endif
	
	retval = RDSTAT_NULL;
	rdClear(tx_header);
	tx_header.mb_id = arg_id;
	tx_header.function = RDFUNC_GET;
	tx_header.type = arg_apitype;
	tx_header.value1 = POLL_REC_COUNT[arg_apitype];

	#if PRINT_PROCESSES_IN_UART
    snprintf(printout,40,"*Start Polling!\n");
    uartSHOW((uint8_t*)printout, strlen(printout));
	#endif
    
	/* polling query and get record */

	retval = rdBnrMainHeadTx(tx_header);
	if (retval != RDSTAT_OK)
		return RDSTAT_ERROR;
	
	

	//retval = rdBnrGateGetRecSz(tx_header, &get_rec_cnt);
	if(rdBnrMainBlankFrRx(&rx_header, HEADERRX_TIMEOUT) == RDSTAT_OK)
    {
		get_rec_cnt = rx_header.value1;
    }
	else
		return RDSTAT_ERROR;
	
	#if PRINT_PROCESSES_IN_UART
				const char *msg_sleep = "*Header Received\n";
				snprintf(printout, 20, "Rec : %d\n", get_rec_cnt);
				uartSHOW((uint8_t*)msg_sleep, strlen(msg_sleep));
				uartSHOW((uint8_t*)printout, strlen(printout));
	#endif
				
	if(get_rec_cnt == 0) {
		return RDSTAT_OK;
	}
	
	/*CALCULATE LOOP COUNT*/
	loopcount = get_rec_cnt/max_rec_per_frame;
	if(get_rec_cnt%max_rec_per_frame !=0)
		loopcount++;
	
    if(retval == RDSTAT_OK) {
		/*INITIALIZE VARIABLES*/
        rdClear(databuff_idxaddr);
        rdClear(total_rec_count);
        max_frame_rec = tx_header.value1;

        if(arg_apitype == TELEMETRY)
			databuff_size = sizeof(TelemetryStruct);
		else
			databuff_size = sizeof(SystemStatusStruct);
		
		 radio_databuff = memAlloc(databuff_size);
		
        /*RECEIVING DATA LOOP*/
        for(uint32_t ixx = 0; ixx < loopcount; ixx++)//while(datarx_loop_count > 0) 
		{
            rdBnrMainHeadRx(&rx_header);
            retval = rdBnrMainDataRx(rx_header, (uint8_t*)radio_databuff);
            if(retval == RDSTAT_OK) 
			{
				rx_rec_count += rx_header.value1;
				resp_head.mb_id = arg_id;
				resp_head.value1 = rx_header.value1;
				resp_head.function = RDRESP_ACKNOWLEDGE;
				resp_head.type = arg_apitype;
				delayMillis(RD_TRANSMIT_DELAY);
				rdBnrMainHeadTx(resp_head);
            }
			else
				break;
		}
				
			#if PRINT_PROCESSES_IN_UART
				snprintf(printout, 20, "RxCnt : %d\n", rx_rec_count);
				uartSHOW((uint8_t*)printout, strlen(printout));
			#endif				

        free(radio_databuff);
	}

	return retval;
}

/*============================================================================
* @brief    Receiving sensor registration from specific node
* @param	arg_id:     unit id
* @retval	Function status
============================================================================*/
static RdStatEnum rdBnrGateGetSenReg(uint8_t arg_id) {
	RdStatEnum retval;                      /*return status*/
	RdHeadStruct que_head;                  /*header of transmitting frame*/
	RdHeadStruct rx_head;                   /*header of received frame*/
	PortConfigStruct rx_senreg;             /*received sensor registration*/
	uint8_t *portbuff;                      /*buffer of receiving port data*/
	uint8_t *rx_data;                       /*buffer of receiving data*/
	uint8_t sen_port;                       /*port number of receiving data*/
	uint8_t rxloop_ckr;                     /*checker flag of receiving loop*/
	uint32_t portbuff_addr;                 /*pointer of buffer of port data*/
	const uint8_t SENPORT_CNT = 4;          /*receiving number of port*/

	retval = RDSTAT_NULL;
	rdClear(que_head);
	que_head.mb_id = arg_id;
	que_head.function = RDFUNC_GET;
	que_head.type = RDAPI_SENREG;
	que_head.value1 = POLL_REC_COUNT[RDAPI_SENREG];

	portbuff = memAlloc(sizeof(SensorRegistrationStruct));
	rdClear(rxloop_ckr);
	rdClear(portbuff_addr);
	rdClear(sen_port);
	rdClear(rx_senreg);

	retval = rdBnrMainHeadTx(que_head);
    if(retval == RDSTAT_OK) {
        rx_data = NULL;
        while(sen_port < SENPORT_CNT) {
            rdClear(rx_head);
            rdBnrMainHeadRx(&rx_head);
            rdBnrMainDataRx(rx_head, rx_data);
            memcpy(portbuff + portbuff_addr, rx_data, rx_head.byt_sz);
            portbuff_addr += rx_head.byt_sz;
            free(rx_data);

            if(rx_head.function != RDRESP_GET
               || rx_head.type != RDAPI_SENREG
               || rx_head.value1 != sen_port+1) {
                que_head.function = RDRESP_ACKNOWLEDGE;
                que_head.value1 = RDRESP_STOP;
                rdBnrMainHeadTx(que_head);
                retval = RDSTAT_ERROR;
                break;
            }

            if(rxloop_ckr == 0) {
                rxloop_ckr = rx_head.value2;
            }

            if(rxloop_ckr - rx_head.value2 > 1) {
                que_head.function = RDRESP_ACKNOWLEDGE;
                que_head.value1 = RDRESP_STOP;
                rdBnrMainHeadTx(que_head);
                retval = RDSTAT_ERROR;
                break;
            }

            if(rx_head.value2 == 1) {
                memcpy((SensorRegistrationStruct*)&rx_senreg + sen_port
                       , portbuff
                       , sizeof(SensorRegistrationStruct));
                sen_port++;
                que_head.function = RDRESP_ACKNOWLEDGE;
                que_head.value1 = sen_port;
                que_head.value2 = RDSTAT_OK;
                rdBnrMainHeadTx(que_head);
                rdClear(portbuff_addr);
                retval = RDSTAT_OK;
            }

            rxloop_ckr--;
        }
    }

	if(retval == RDSTAT_OK)	{
		//sen-reg post here!!!
	}

	free(portbuff);
	return retval;
}

RdStatEnum rdBnrGateGetFWVer (uint8_t mb_id)
{
  	RdHeadStruct rx_head;
	RdHeadStruct tx_head;
	rdClear(tx_head);
	tx_head.mb_id = mb_id;
	tx_head.function = RDFUNC_FWVER_CHK;
	tx_head.type = RDAPI_FW;
	rdBnrMainHeadTx(tx_head);
	rdBnrMainBlankFrRx(&rx_head, 6000);
}

/*============================================================================
* @brief    Transmit firmware for update
* @param	arg_id:     unit id
            arg_fwsize: fimware byte size 
* @retval	Function status
============================================================================*/
static RdStatEnum rdBnrGateUpdFw(uint8_t arg_id, uint32_t arg_fwsize) {
    RdStatEnum retval;                      /*return status*/
	RdHeadStruct frame_head;                /*header of transmitting frame*/
	RdHeadStruct rx_head;                   /*header of received frame*/
	ModbusCrcStruct frame_crc;              /*crc og transmitting frame*/
	uint8_t frameptr[136];                      /*pointer of transmitting frame*/
	uint32_t flash_addr;                    /*pointed address of firmware*/
	uint32_t loop_cnt;                      /*number of transmitting loop*/
    uint32_t fw_next_free_address = NEW_FW_START_SEC_ADDR;
    retval = RDSTAT_NULL;
    uint32_t temp_bytesize = 0;
    uint8_t trythis[128];
	
	
    if(arg_fwsize == 0) 
	{
        retval = RDSTAT_ERROR;
    }
    
	if(retval != RDSTAT_ERROR) 
	{
	  	/*calculate number of packets to be sent (128 bytes per packet)*/
        loop_cnt = arg_fwsize / RDBNR_OTA_PACKETSZ;
        if(arg_fwsize % RDBNR_OTA_PACKETSZ > 0) 
		{
            loop_cnt++;
        }

		/*Send header/notify node that fw is coming*/
        rdClear(frame_head);
        frame_head.mb_id = arg_id;
        frame_head.function = RDFUNC_UPDATE;
        frame_head.type = RDAPI_FW;
        frame_head.value1 = loop_cnt>>8;
        frame_head.value2 = (uint8_t)loop_cnt;

        if(rdBnrMainHeadTx(frame_head) != RDSTAT_OK
           || rdBnrMainBlankFrRx(&rx_head, 6000) != RDSTAT_OK
           || rx_head.function != RDFUNC_UPDATE
           || rx_head.type != RDAPI_FW) 
        {
            retval = RDSTAT_ERROR;
        }
        delayMillis(200);
    }

    if(retval != RDSTAT_ERROR) 
	{  
        flash_addr = 0;
        retval = RDSTAT_OK;
        
		/*Transmit all packets*/
        while(loop_cnt > 0) 
        {
		  	/*check if last packet is the one to be sent*/
            if(loop_cnt == 1) 
            {
                frame_head.byt_sz = arg_fwsize % RDBNR_OTA_PACKETSZ;
                temp_bytesize = arg_fwsize % RDBNR_OTA_PACKETSZ;
            }
            else 
            {
                frame_head.byt_sz = RDBNR_OTA_PACKETSZ;
                temp_bytesize = RDBNR_OTA_PACKETSZ;
            }


            memcpy(frameptr, &frame_head, RDFRAME_SZ);
			
			/*Read FW from Ext Flash*/
            extFlRead((int8_t *)frameptr + RDHEAD_SZ
                      , fw_next_free_address
                      , RDBNR_OTA_PACKETSZ); // !temporary variable arguments
            frame_crc = modbusCrcCalc(frameptr, RDHEAD_SZ + frame_head.byt_sz);
            memcpy(frameptr + RDHEAD_SZ + frame_head.byt_sz
                   , &frame_crc, RDCRC_SZ);
            delayMillis(50);
            uartTxBanner((uint8_t*)frameptr, RDFRAME_SZ + frame_head.byt_sz);
            
            if(rdBnrMainBlankFrRx(&rx_head, HEADERRX_TIMEOUT) != RDSTAT_OK
               || rx_head.function != RDFUNC_UPDATE
                 || rx_head.type != RDAPI_FW) 
			{
                retval = RDSTAT_ERROR;
                break;
            }
           
            if(loop_cnt == 1)
            {
			  	/*send FW version to Node*/
			  	delayMillis(1000);
				rdClear(frame_head);
				frame_head.mb_id = arg_id;
				frame_head.function = RDFUNC_UPDATE;
				frame_head.type = RDAPI_FW;
				frame_head.value1 = FW_VERSION.fwver_main &0xF0 << 4 
				  					| FW_VERSION.fwver_submain;
				frame_head.value2 = FW_VERSION.fwver_minor;
      	 		rdBnrMainHeadTx(frame_head);
//              extFlRead((int8_t *)trythis
//                      , fw_next_free_address
//                      , RDBNR_OTA_PACKETSZ);
//              uartSHOW(trythis, RDFRAME_SZ + frame_head.byt_sz);
              if (fileCheck(ota_fwsize) == true)
                delayMillis(3000);
              delayMillis(50);
            }
            fw_next_free_address += 128;
            flash_addr++;
            loop_cnt--;
        }
        //free(frameptr);
	}

    return retval;
}

/*============================================================================
* @brief    Query >>> Get Command
* @param	arg_id:     unit id
            vlist:      variable arguments/parameter
                >> va_arg1 = api type
* @retval	Function status
============================================================================*/
static RdStatEnum rdBnrGateGet(uint8_t arg_id, va_list valist) {
    RdStatEnum retval;                      /*return status*/
	RdApiEnum apitype;                      /*api type*/

    retval = RDSTAT_NULL;
    apitype = (RdApiEnum)va_arg(valist,int);
	
    char printout[40];
    //snprintf(printout,40,"*Get Record %d\n",apitype);
    //uartSHOW((uint8_t*)printout, strlen(printout));
    
    switch(apitype) {
        case RDAPI_TELEM:
        case RDAPI_DEVREG:
        case RDAPI_VLVCMD:
            retval = rdBnrGateGetRec(arg_id, apitype);
            break;
        case RDAPI_SENREG:
            retval = rdBnrGateGetSenReg(arg_id);
            break;
        default:
            retval = RDSTAT_ERROR;
    }

	return retval;
}

/*============================================================================
* @brief    Query >>> Update Command
* @param	arg_id:     unit id
            vlist:      variable arguments/parameter
                >> va_arg1 = api type
                >> va_arg2 = firmware byte size  
* @retval	Function status
============================================================================*/
static RdStatEnum rdBnrGateUpdate(uint8_t arg_id, uint32_t fw_size) {
	RdStatEnum retval;                      /*return status*/
	RdApiEnum apitype;                      /*api type*/
    uint32_t fwsize = fw_size;                        /*firmware byte size */

    retval = RDSTAT_NULL;
	apitype = RDAPI_FW;//(RdApiEnum)va_arg(valist, int);

	switch(apitype) {
        case RDAPI_FW:
            //fwsize = va_arg(valist, int);
            retval = rdBnrGateUpdFw(arg_id, ota_fwsize);
            break;
        default:
            retval = RDSTAT_ERROR;
	}

	return retval;
}

/*============================================================================
* @brief    Query >>> Sleep Command
* @param	recordtype: Type of data to query
* @retval	Function status
============================================================================*/
static RdStatEnum rdBnrGateSleep(uint8_t arg_id) {
    RdStatEnum retval;                      /*return status*/
	RdHeadStruct buffer;                    /*frame buffer*/

    retval = RDSTAT_NULL;
	rdClear(buffer);
    
	buffer.mb_id = arg_id;
	buffer.value1 = RTC_SLEEPTIME >> 8;
	buffer.value2 = (uint8_t)RTC_SLEEPTIME;
	buffer.function = RDFUNC_SLEEP;
    
	#if PRINT_PROCESSES_IN_UART
		char printout[20];
		snprintf(printout, 20, "*SleepCmd : 0x%02X\n", arg_id);
		uartSHOW((uint8_t*)printout, strlen(printout));
		
	#endif
	retval = rdBnrMainHeadTx(buffer);
    
    rdClear(buffer);
    if(rdBnrMainBlankFrRx(&buffer, HEADERRX_TIMEOUT) == RDSTAT_OK)
    {
        if(buffer.mb_id == arg_id && buffer.function == RDFUNC_STAT_CHK)
        {
            retval = RDSTAT_OK;
        }
    }

	return retval;
}

#pragma optimize=none
static RdStatEnum rdBnrGateStatChkRequest(uint8_t arg_id)
{
    RdStatEnum retval = RDSTAT_ERROR;       /*return status*/
	RdHeadStruct buffer;                    /*frame buffer*/

	rdClear(buffer);
    buffer.mb_id = arg_id;
	buffer.function = RDFUNC_STAT_CHK;
    
	rdBnrMainHeadTx(buffer);
    
    //response handler
    rdClear(buffer);
    if(rdBnrMainBlankFrRx(&buffer, HEADERRX_TIMEOUT) == RDSTAT_OK)
    {
        if(buffer.mb_id == arg_id)
        {
            retval = RDSTAT_OK;
        }
    }
    
	return retval;
}

#pragma optimize=none
static RdStatEnum rdBnrGateSendNetId(uint8_t arg_id, uint32_t arg_netid) 
{
    RdStatEnum retval = RDSTAT_ERROR;
	RdHeadStruct header;
    char printout[100];
    
    rdClear(header);
	header.mb_id = arg_id;
	header.function = RDFUNC_NETID;
    memcpy((uint8_t*)&header+2, &arg_netid, sizeof(arg_netid));
    
	sprintf(printout, "rdBnrGateSendNetId[%d] to Node[0x%x]\n", arg_netid, arg_id);
    uartSHOW((uint8_t*)printout, strlen(printout));
	
    //send network id
    rdBnrMainHeadTx(header);
    
    //response handler
    rdClear(header);
    if(rdBnrMainRxResp(&header, DATARX_TIMEOUT) == RDSTAT_OK)
    {
        if(header.mb_id == arg_id && header.value1 == RDSTAT_OK)
        {
            retval == RDSTAT_OK;
        }
    }
    
	return retval;
}

#pragma optimize=none
RdStatEnum rdBnrGateNetworkAutoConfig(uint32_t arg_netid)
{
    RdStatEnum retval = RDSTAT_ERROR;
    
    //send new network id to all nodes in nodelist
    for(uint8_t i = 1; i <= NODE_COUNT; i++)
	{
        rdBnrGateSendNetId(DEVICE_INFO_LIST[i].rad_modbus_id ,arg_netid);
    }
    
    //self config of network id
    rdBnrMainSetNetId(arg_netid);
    
    //verify all nodes - life stat check
    for(uint8_t i = 1; i <= NODE_COUNT; i++)
	{
        rdBnrGateStatChkRequest(arg_netid);
    }
    
    return retval;
}

#pragma optimize=none
RdStatEnum rdBnrGateSetModbusOfNode(uint8_t rad_mb_id) 
{
	RdStatEnum retval = RDSTAT_ERROR;	        /*return status*/
    RdHeadStruct tx_header; 
	RdHeadStruct rx_header;
    
	/*Prepare Header*/
	retval = RDSTAT_NULL;
	rdClear(tx_header);
	tx_header.mb_id = rad_mb_id;
	tx_header.function = RDFUNC_NETDEVNUMSET;
	tx_header.type = RDAPI_NETDEVNUM;
	
	/*Transmit Header*/
	retval = rdBnrMainHeadTx(tx_header);
	
	/*Read response from node*/
	if (retval == RDSTAT_OK)
	{
		/*receive header*/
        retval = rdBnrMainBlankFrRx(&rx_header, DATARX_TIMEOUT);
		if (retval == RDSTAT_OK)
		{
			if(rx_header.mb_id != rad_mb_id)
			{
				logWrite("Incorrect Modbus response! ");
                retval = RDSTAT_ERROR;
			}
		}
		else
		{
            logWrite("Node no reply! ");
			retval = RDSTAT_ERROR;
		}
	}

	return retval;
}

/*============================================================================
* @brief    Gate's query to a specific Node via Banner radio
* @param	arg_id:     unit id
            arg_func:   type of radio function (i.e. get, update, sleep, check)
            vlist:      variable arguments/parameter
                >> va_arg1 = api type (for get & update only)
                >> va_arg2 = firmware size (for update-firmware only)   
* @retval	Function status
============================================================================*/
RdStatEnum rdBnrGateQuery(uint8_t arg_id, RdFuncEnum arg_func, va_list vlist) {
    RdStatEnum retval;                      /*return status*/
	
    retval = RDSTAT_NULL;
   
    switch(arg_func) {
        case RDFUNC_GET:
            retval = rdBnrGateGet(arg_id, vlist);
            break;
        case RDFUNC_UPDATE:
            retval = rdBnrGateUpdate(arg_id, ota_fwsize);
            break;
        case RDFUNC_SLEEP:
            retval = rdBnrGateSleep(arg_id);
            break;
        case RDFUNC_STAT_CHK:
            retval = rdBnrGateStatChkRequest(arg_id);
            break;
        default:
            retval = RDSTAT_ERROR;
	}

	return retval;
}

#pragma optimize=none
//Insertion Sort Algorithm
static void rdBnrGateSortRegisteredDevAddrList(uint16_t* a_devaddrlist, uint16_t* a_mbidlist, uint16_t a_list_sz)
{
    int index;
    int holePosition;
    uint8_t mb_id_to_insert;
    uint16_t dev_addr_to_insert;

    //sorting
    for (index=1; index<=a_list_sz; index++)
    {
        /* select value to be inserted */
        mb_id_to_insert = a_mbidlist[index];
        dev_addr_to_insert = a_devaddrlist[index];
        holePosition = index;

        /*locate hole position for the element to be inserted */

        while (holePosition > 0 && a_mbidlist[holePosition-1] > mb_id_to_insert)
        {
            a_devaddrlist[holePosition] = a_devaddrlist[holePosition-1];
            a_mbidlist[holePosition] = a_mbidlist[holePosition-1];
            holePosition = holePosition -1;
        }

        /* insert the number at hole position */
        a_devaddrlist[holePosition] = dev_addr_to_insert;
        a_mbidlist[holePosition] = mb_id_to_insert;    
    }
}

#pragma optimize=none
static void reverseOrder(char *dev_sn, const char size)
{
	char *temp_char = memAlloc(size);
	memcpy(temp_char, dev_sn, size);
	uint8_t max_arr_idx = size - 1;
	for(uint8_t i = 0; i < size; i++)
	{
		dev_sn[max_arr_idx - i] = temp_char[i];
	}
	
	for(uint8_t i = size-1; i >= 0; i--)
	{
		if(dev_sn[i] == 0x20)
            dev_sn[i] = 0;
        else
            break;
	}
}

#pragma optimize=none
static void rdBnrGateSetHighPowerMode(bool a_state)
{
    uint8_t modbus_rxdata[9];
    uint8_t modbus_powermode_header[] 
        = {0xFB, 0x00, 0x10, 0x18, 0xB8, 0x00, 0x01, 0x02, 0x00, 0x00, 0x26, 0x68};
    
    logWrite("rdBnrGateSetHighPowerMode[%d]...", a_state);
    if(a_state == true)
    {
        modbus_powermode_header[9] = 0x01;
        modbus_powermode_header[10] = 0xE7;
        modbus_powermode_header[11] = 0xA8;
    }
    
    memset(modbus_rxdata, 0, sizeof(modbus_rxdata));
    for(uint8_t counter=1; counter<=3; counter++ )
    {
        if((uartTxBanner((uint8_t*)&modbus_powermode_header, sizeof(modbus_powermode_header)) == HAL_OK)
           && (HAL_UART_Receive(&RD_BANNER_HANDLER, (uint8_t*)&modbus_rxdata, sizeof(modbus_rxdata), 1000) == HAL_OK)
           && modbus_rxdata[6] == 1)
        {
            logWrite("SUCCESS\n");
            return;
        }
        HAL_UART_DeInit(&RD_BANNER_HANDLER);
        HAL_UART_Init(&RD_BANNER_HANDLER);
        delayMillis(RD_TRANSMIT_DELAY);
        if(counter == 3) {logWrite("FAILED!\n");}
    }
    logWrite("FAILED!\n");
}

#pragma optimize=none
static uint16_t rdBnrGateGetNumberOfAttachedRadio(void)
{
    ModbusHeadStruct modbus_header;
    ModbusRxDataStruct modbus_rxdata;
    uint16_t attached_node_count = 0;
    static uint8_t counter;
    
    logWrite("Getting number of attached Node from Banner radio...");
    modbus_header.modbus_id = RDBNR_MULTIHOPRADIO_ID;
    modbus_header.start_addr = RDBNRGATE_REGADDR_NUM_OF_DEV;
    modbus_header.reg_cnt = 1;
    for( counter=1; counter<=3; counter++ )
    {
        modbus_rxdata = modbusRead(&RD_BANNER_HANDLER, &modbus_header);
        if(modbus_rxdata.status == MODBUS_OK) 
        {
            attached_node_count = modbus_rxdata.payload[0] - 1; //exclude Master
            attached_node_count = (attached_node_count > RDBNRGATE_MAX_NODE_COUNT) 
                                    ? RDBNRGATE_MAX_NODE_COUNT : attached_node_count;
            logWrite("SUCCESS\n");
            break;
        }
        delayMillis(RD_TRANSMIT_DELAY);
    }
    if(modbus_rxdata.status != MODBUS_OK) 
        logWrite("FAILED!\n");

    logWrite("Attached Node found = [%d]\n", attached_node_count);
    return attached_node_count;
}

#pragma optimize=none
static RdStatEnum rdBnrGateListAllAttachedNodes(uint16_t* a_list, uint16_t a_node_count)
{
    RdStatEnum retval = RDSTAT_ERROR;
    ModbusHeadStruct modbus_header;
    ModbusRxDataStruct modbus_rxdata;
    static uint8_t counter;
    uint16_t rad_dev_addr_templist[RDBNRGATE_MAX_NODE_COUNT];
    
    logWrite("Getting list of radio device address from Banner radio...\n");
    MB_CLEAR(modbus_header);
    modbus_header.modbus_id = RDBNR_MULTIHOPRADIO_ID;
    
    // fetching 1 of 3 sets
    if(a_node_count > 0)
    {   
        logWrite("Set 1...");
        modbus_header.start_addr = RDBNRGATE_REGADDR_DEV_ADDR_LIST;
        modbus_header.reg_cnt = (a_node_count < MB_MAX_REG_COUNT) 
                                ? a_node_count : MB_MAX_REG_COUNT;
        for( counter=1; counter<=3; counter++ )
        {
            modbus_rxdata = modbusRead(&RD_BANNER_HANDLER, &modbus_header);
            if(modbus_rxdata.status == MODBUS_OK) 
            {
                memcpy(rad_dev_addr_templist, &modbus_rxdata.payload[0], modbus_rxdata.count);
                logWrite("SUCCESS\n");
                retval = RDSTAT_OK;
                delayMillis(RD_TRANSMIT_DELAY);
                break;
            }
        }
        if(modbus_rxdata.status != MODBUS_OK)
        {
            logWrite("FAILED!\n");
        }
    }
    else
    {
        logWrite("Nothing to get\n");
    }
    
    // fetching 2 of 3 sets
    if((retval == RDSTAT_OK) && (a_node_count > MB_MAX_REG_COUNT))
    {
        logWrite("Set 2...");
        modbus_header.start_addr = RDBNRGATE_REGADDR_DEV_ADDR_LIST + MB_MAX_REG_COUNT;
        modbus_header.reg_cnt = (a_node_count-MB_MAX_REG_COUNT < MB_MAX_REG_COUNT) 
                                ? a_node_count-MB_MAX_REG_COUNT : MB_MAX_REG_COUNT;
        for( counter=1; counter<=3; counter++ )
        {
            modbus_rxdata = modbusRead(&RD_BANNER_HANDLER, &modbus_header);
            if(modbus_rxdata.status == MODBUS_OK) 
            {
                memcpy(rad_dev_addr_templist+MB_MAX_REG_COUNT, &modbus_rxdata.payload[0], modbus_rxdata.count);
                logWrite("SUCCESS\n");
                delayMillis(RD_TRANSMIT_DELAY);
                break;
            }
        }
        if(modbus_rxdata.status != MODBUS_OK)
        {
            logWrite("FAILED!\n");
            retval = RDSTAT_ERROR;
        }
    }    
    
    // fetching 3 of 3 sets
    if((retval == RDSTAT_OK) && (a_node_count-MB_MAX_REG_COUNT > MB_MAX_REG_COUNT))
    {
        logWrite("Set 3...");
        modbus_header.start_addr = RDBNRGATE_REGADDR_DEV_ADDR_LIST + MB_MAX_REG_COUNT*2;
        modbus_header.reg_cnt = (a_node_count-MB_MAX_REG_COUNT*2 < MB_MAX_REG_COUNT) 
                            ? a_node_count-MB_MAX_REG_COUNT*2 : MB_MAX_REG_COUNT;
        for( counter=1; counter<=3; counter++ )
        {
            modbus_rxdata = modbusRead(&RD_BANNER_HANDLER, &modbus_header);
            if(modbus_rxdata.status == MODBUS_OK) 
            {
                memcpy(rad_dev_addr_templist+(MB_MAX_REG_COUNT*2), &modbus_rxdata.payload[0], modbus_rxdata.count);
                logWrite("SUCCESS\n");
                delayMillis(RD_TRANSMIT_DELAY);
                break;
            }
        }
        if(modbus_rxdata.status != MODBUS_OK)
        {
            logWrite("FAILED!\n");
            retval = RDSTAT_ERROR;
        }
    }
    
    if (retval == RDSTAT_OK)
    {
        memcpy(a_list, rad_dev_addr_templist, modbus_rxdata.count);
    }
      
    return retval;
}

#pragma optimize=none
static void rdBnrGateUpdateModbusToDeviceTable(uint16_t* a_list, uint16_t a_node_count)
{
    ModbusHeadStruct modbus_header;
    ModbusRxDataStruct modbus_rxdata;
    static uint8_t counter;
    
    logWrite("Writing the sorted list of attached Nodes to 'Modbus to Device' table of Banner radio:\n");
    modbus_header.modbus_id = RDBNR_MULTIHOPRADIO_ID;
    
    // update 1 of 3 sets
    if(a_node_count > 0)
    {
        logWrite("Set 1...");
        modbus_header.start_addr = RDBNRGATE_REGADDR_MODBUS_TODEVICE_LIST;
        modbus_header.reg_cnt = (a_node_count < MB_MAX_REG_COUNT) 
                                ? a_node_count : MB_MAX_REG_COUNT;
        for( counter=1; counter<=3; counter++ )
        {
            if(modbusWriteMultiple(&RD_BANNER_HANDLER, modbus_header, a_list) == MODBUS_OK)
            {
                logWrite("SUCCESS\n");
                delayMillis(RD_TRANSMIT_DELAY);
                break;
            }
            else
            {
                if(counter == 3)
                {
                    logWrite("FAILED!\n");
                }
            }
        }
    }
    else
    {
        logWrite("Nothing to write\n");
    }
    
    // update 2 of 3 sets
    if(a_node_count > MB_MAX_REG_COUNT)
    {
        logWrite("Set 2...");
        modbus_header.start_addr = RDBNRGATE_REGADDR_MODBUS_TODEVICE_LIST + MB_MAX_REG_COUNT;
        modbus_header.reg_cnt = (a_node_count-MB_MAX_REG_COUNT < MB_MAX_REG_COUNT) 
                            ? a_node_count-MB_MAX_REG_COUNT : MB_MAX_REG_COUNT;
        for( counter=1; counter<=3; counter++ )
        {
            if(modbusWriteMultiple(&RD_BANNER_HANDLER, modbus_header, a_list+MB_MAX_REG_COUNT) == MODBUS_OK)
            {
                logWrite("SUCCESS\n");
                delayMillis(RD_TRANSMIT_DELAY);
                break;
            }
            else
            {
                if(counter == 3)
                {
                    logWrite("FAILED!\n");
                }
            }
        }
    }    
    
    // update 3 of 3 sets
    if(a_node_count-MB_MAX_REG_COUNT > MB_MAX_REG_COUNT)
    {
        logWrite("Set 3...");
        modbus_header.start_addr = RDBNRGATE_REGADDR_MODBUS_TODEVICE_LIST + MB_MAX_REG_COUNT*2;
        modbus_header.reg_cnt = a_node_count - MB_MAX_REG_COUNT;
        modbus_header.reg_cnt = (a_node_count-MB_MAX_REG_COUNT*2 < MB_MAX_REG_COUNT) 
                            ? a_node_count-MB_MAX_REG_COUNT*2 : MB_MAX_REG_COUNT;
        for( counter=1; counter<=3; counter++ )
        {
            if(modbusWriteMultiple(&RD_BANNER_HANDLER, modbus_header, a_list+(MB_MAX_REG_COUNT*2)) == MODBUS_OK)
            {
                logWrite("SUCCESS\n");
                delayMillis(RD_TRANSMIT_DELAY);
                break;
            }
            else
            {
                if(counter == 3)
                {
                    logWrite("FAILED!\n");
                }
            }
        }
    }
}

#pragma optimize=none
static ModbusRxDataStruct rdBnrGateGetNodeDeviceSn(uint16_t a_addr)
{
    ModbusExtendedHeadStruct modbus_extheader;
    ModbusRxDataStruct modbus_rxdata;
    uint8_t attempt_counter;
    
    modbus_rxdata.status = MODBUS_ERROR;
    MB_CLEAR(modbus_extheader);
    modbus_extheader.rad_dev_addr = a_addr;
    modbus_extheader.start_addr = RDBNRGATE_REGADDR_AGDEVICESN;
    modbus_extheader.reg_cnt = RADIONODE_MAX_DEVSN_CHARSZ/2;
    for( attempt_counter=1; attempt_counter<=3; attempt_counter++ )
    {
        modbus_rxdata = modbusExtendedRead(&RD_BANNER_HANDLER, &modbus_extheader);
        delayMillis(RD_TRANSMIT_DELAY);
        if(modbus_rxdata.status == MODBUS_OK) break;
    }
    
    if(modbus_rxdata.status == MODBUS_OK)
        reverseOrder((char*)modbus_rxdata.payload, modbus_rxdata.count);
    
    return modbus_rxdata;
}

static bool rdBnrGateMbIdIsPresentFromNodeList(uint16_t a_mbid)
{
	for(uint8_t i = 0; i <= NODE_COUNT; i++)
	{
		if(DEVICE_INFO_LIST[i].rad_modbus_id == a_mbid)
			return true;
	}
	return false;
}

#pragma optimize=none
RdStatEnum rdBnrGateNetworkInit(void)
{
    RdStatEnum status;
    AttachedNodeStruct attached_nodes;    
    RadioNodeStruct radio_node;
    ModbusHeadStruct modbus_header;
    ModbusRxDataStruct modbus_rxdata;
    uint16_t dev_addr_arr[RDBNRGATE_MAX_NODE_COUNT];
    uint16_t mb_id_arr[RDBNRGATE_MAX_NODE_COUNT];
    uint16_t attached_node_count = 0;
    uint8_t attached_registered_node_count = 0;
	
    logWrite("\n***Syncing Radio Network START***\n");
	
    memset(&attached_nodes, 0xFF, sizeof(attached_nodes));
    memset(&dev_addr_arr, 0xFF, sizeof(dev_addr_arr));
    memset(&mb_id_arr, 0xFF, sizeof(mb_id_arr));
    MB_CLEAR(modbus_header);
    modbus_header.modbus_id = RDBNR_MULTIHOPRADIO_ID;
    
    //(1) Validate Available Modbus ID
    logWrite("Available Modbus ID = [0x%02x]\n", AVAILABLE_MODBUS_ID);
    logWrite("(1) Validating available Modbus Id...");
    status = RDSTAT_OK;
    for (uint8_t i=RDBNR_BASE_MODBUSID; i<AVAILABLE_MODBUS_ID; i++)
    {
        if(!rdBnrGateMbIdIsPresentFromNodeList(i))
        {
            AVAILABLE_MODBUS_ID = i;
            status = RDSTAT_ERROR;
            break;
        }
    }
    
    if(status == RDSTAT_OK)
        logWrite("OK\n");
    else
        logWrite("Failed! Update available Modbus Id into [0x%02x]\n", AVAILABLE_MODBUS_ID);
            
    //(2) Update Modbus Id in Node List and set Inactive status to all
    logWrite("Node count from Node list = [%d]\n", NODE_COUNT);
    logWrite("(2) Updating Modbus Ids in the Node list:\n");
    for (uint8_t i=1; i<=NODE_COUNT; i++)
    {
        logWrite("Node [%d] of [%d] - [%s]...", i, NODE_COUNT, (&DEVICE_INFO_LIST[i])->device_sn);
        if(DEVICE_INFO_LIST[i].rad_modbus_id < RDBNR_BASE_MODBUSID
           || DEVICE_INFO_LIST[i].rad_modbus_id > AVAILABLE_MODBUS_ID)
        {
            DEVICE_INFO_LIST[i].rad_modbus_id = AVAILABLE_MODBUS_ID;
            ++AVAILABLE_MODBUS_ID;
        }
        DEVICE_INFO_LIST[i].network_add_status = NETWORK_ADD_EMPTY;
        logWrite("Modbus Id [0x%02x]\n", DEVICE_INFO_LIST[i].rad_modbus_id);
    }
    
    //(3) Get number of attached node
    logWrite("(3) ");
    attached_node_count = rdBnrGateGetNumberOfAttachedRadio();
    
    //(4) Fetch list of radio device address from Banner radio then save to a rad_dev_addr list
    logWrite("(4) ");
    status = rdBnrGateListAllAttachedNodes(attached_nodes.rad_dev_addr, attached_node_count);
    delayMillis(RD_TRANSMIT_DELAY);
    if (status != RDSTAT_ERROR)
    {
        //(5) Get device serial number from each node 
        logWrite("(5-6) Getting serial number from each Node's Banner radio...\n");
        rdBnrGateSetHighPowerMode(true);
        for (uint8_t i=0; i<attached_node_count; i++)
        {
            logWrite("\nAttached Node [%d] of [%d]...", i+1, attached_node_count);
            modbus_rxdata = rdBnrGateGetNodeDeviceSn(attached_nodes.rad_dev_addr[i]);
            delayMillis(RD_TRANSMIT_DELAY);
            if(modbus_rxdata.status != MODBUS_OK)
            {
                logWrite("FAILED!\n");
                continue;
            }
            
            memset(&attached_nodes.rad_dev_sn[i], 0x00, sizeof(attached_nodes.rad_dev_sn[i]));
            memcpy(&attached_nodes.rad_dev_sn[i], &modbus_rxdata.payload[0], modbus_rxdata.count);
            logWrite("SUCCESS - [%s]\n", attached_nodes.rad_dev_sn[i]);
        
            //(6) Update Radio Device Address and State in Node List referring by device serial number
            radio_node.rad_dev_addr = attached_nodes.rad_dev_addr[i];
            memcpy(radio_node.rad_dev_sn, attached_nodes.rad_dev_sn[i], RADIONODE_MAX_DEVSN_CHARSZ);
            updateDeviceInfoListDevAddAndState(radio_node);
        }
        rdBnrGateSetHighPowerMode(false);
        logWrite("...DONE\n");
    }
    else
        logWrite("Skip process (5-6)\n");
    
    //(7) Copy some details from Node list into buffer arrays
    attached_registered_node_count = 0;
    if(NODE_COUNT > 0)
    {
        for(uint8_t i=1; i<=NODE_COUNT; i++)
        {
            dev_addr_arr[i-1] = DEVICE_INFO_LIST[i].rad_dev_add;
            mb_id_arr[i-1] = DEVICE_INFO_LIST[i].rad_modbus_id;
            if(DEVICE_INFO_LIST[i].network_add_status == NETWORK_ADD_OK)
                attached_registered_node_count++;
        }
    }
    logWrite("(7) [%d] out of [%d] Attached Node is Registered from Node List\n"
             , attached_registered_node_count, attached_node_count);    
    
    //(8) Update Modbus to Device Register Table in Banner radio
    logWrite("(8) Updating Modbus to device register table in Banner radio...\n");
    rdBnrGateSortRegisteredDevAddrList(dev_addr_arr, mb_id_arr, NODE_COUNT);
    rdBnrGateUpdateModbusToDeviceTable(dev_addr_arr, NODE_COUNT); //TODO: confirm by test if 0xFFFF in between will still work
	
    //(9) Update Modbus ID of each Node
    logWrite("(9) Updating Modbus Id of each Node in Node List:\n");
	delayMillis(500);
	for(uint8_t i=1; i<=NODE_COUNT; i++)
	{
        logWrite("Node [%d] of [%d] - SN:[%s] MBID:[0x%02x]...", i, NODE_COUNT
                 , (&DEVICE_INFO_LIST[i])->device_sn, DEVICE_INFO_LIST[i].rad_modbus_id);
		if(DEVICE_INFO_LIST[i].network_add_status == NETWORK_ADD_OK)
		{
			if(rdBnrGateSetModbusOfNode(DEVICE_INFO_LIST[i].rad_modbus_id) != RDSTAT_OK)
			{
				/*Node did not respone, try again*/
				delayMillis(RD_TRANSMIT_DELAY);
                logWrite("\nTry Again... Node [%d] of [%d] - SN:[%s] MBID:[0x%02x]..."
                         , i, NODE_COUNT
                         , (&DEVICE_INFO_LIST[i])->device_sn
                         , DEVICE_INFO_LIST[i].rad_modbus_id);
				if(rdBnrGateSetModbusOfNode(DEVICE_INFO_LIST[i].rad_modbus_id) != RDSTAT_OK)
				{
                    logWrite("FAILED\n");
					DEVICE_INFO_LIST[i].network_add_status = NETWORK_ADD_EMPTY;
				}
                else
                {
                    logWrite("SUCCESS\n");
                }
			}
			else
			{
				logWrite("SUCCESS\n");
			}
			delayMillis(RD_TRANSMIT_DELAY);
            continue;
		}
        logWrite("STOPPED! - Inactive Node\n");
	}
	
    logWrite("Available Modbus ID = [0x%02x]\n", AVAILABLE_MODBUS_ID);
    logWrite("***Syncing Radio Network END***\n");    
    return RDSTAT_OK;
}

#pragma optimize=none
static void rdBnrGateMaintenanceModeCmdToNode(RdNetMaintenanceModeTypEnum a_cmd, uint8_t a_id)
{
	RdHeadStruct head_buffer;
    uint8_t attempt_cnt = 2;

    logWrite("rdBnrGateMaintenanceModeCmdToNode(%s)", a_cmd);
    
    rdClear(head_buffer);
	head_buffer.mb_id = a_id;
    head_buffer.function = RDFUNC_MAINTENANCE;
	head_buffer.type = a_cmd;
	head_buffer.value2 = sleep_attr.nmm_timer_min;
	
    while(attempt_cnt > 0)
    {
        rdBnrMainHeadTx(head_buffer);
        rdClear(head_buffer);
        if(rdBnrMainBlankFrRx(&head_buffer, HEADERRX_TIMEOUT) == RDSTAT_OK)
        {
            if(head_buffer.mb_id == a_id 
               && head_buffer.function == RDFUNC_MAINTENANCE
               && head_buffer.type == a_cmd 
               &&(a_cmd == RDNETMAINTENANCEMODETYP_STARTCMD
                   && head_buffer.value1 == RADNETMAINTENANCEMODE_ENABLE)
               || (a_cmd == RDNETMAINTENANCEMODETYP_STOPCMD
                   && head_buffer.value1 == RADNETMAINTENANCEMODE_DISABLE))
            {
                break;
            }
        }
        attempt_cnt--;
        delayMillis(RD_TRANSMIT_DELAY);
    }	
}
                
#pragma optimize=none
static void rdBnrGateMaintenanceModeCmdToAllNodes(RdNetMaintenanceModeTypEnum a_cmd)
{
    for(uint8_t i = 1; i <= NODE_COUNT; i++)
	{
		if(DEVICE_INFO_LIST[i].network_add_status == NETWORK_ADD_OK)
		{
            rdBnrGateMaintenanceModeCmdToNode(a_cmd, DEVICE_INFO_LIST[i].rad_modbus_id);
			delayMillis(RD_TRANSMIT_DELAY);
		}
	}
}

#pragma optimize=none
static void rdBnrGateClearSiteSurveyCounts(uint16_t a_rad_dev_addr)
{
    ModbusExtendedHeadStruct modbus_extheader;
    ModbusStatEnum rx_state;
    uint16_t cleared_value[4];
    
    logWrite("rdBnrGateClearSiteSurveyCounts[0x%x]...", a_rad_dev_addr);
    MB_CLEAR(cleared_value);
    MB_CLEAR(modbus_extheader);
    modbus_extheader.rad_dev_addr = a_rad_dev_addr;
    modbus_extheader.start_addr = RDBNRGATE_REGADDR_SITESURVEYCOUNT;
    modbus_extheader.reg_cnt = 4;
    for( uint8_t counter=1; counter<=3; counter++ )
    {
        rx_state = modbusExtendedWriteMultiple(&RD_BANNER_HANDLER, modbus_extheader, cleared_value);
        delayMillis(RD_TRANSMIT_DELAY);
        if(rx_state == MODBUS_OK)
        {
            logWrite("SUCCESS\n");
            return;
        }
    }
    logWrite("FAILED!\n");
}

#pragma optimize=none
static void rdBnrGateReadSiteSurveyCounts(uint16_t a_rad_dev_addr, uint16_t* a_result)
{
    uint16_t const SITE_SURVEY_COUNT_REGSIZE = 4;
    ModbusExtendedHeadStruct modbus_extheader;
    ModbusRxDataStruct rx_data;
    
    logWrite("rdBnrGateReadSiteSurveyCounts[0x%x]...", a_rad_dev_addr);
    MB_CLEAR(rx_data);
    MB_CLEAR(a_result);
    MB_CLEAR(modbus_extheader);
    modbus_extheader.rad_dev_addr = a_rad_dev_addr;
    modbus_extheader.start_addr = RDBNRGATE_REGADDR_SITESURVEYCOUNT;
    modbus_extheader.reg_cnt = SITE_SURVEY_COUNT_REGSIZE;
    for( uint8_t counter=1; counter<=3; counter++ )
    {
        rx_data = modbusExtendedRead(&RD_BANNER_HANDLER, &modbus_extheader);
        delayMillis(RD_TRANSMIT_DELAY);
        if(rx_data.status == MODBUS_OK && rx_data.count == modbus_extheader.reg_cnt)
        {
            memcpy(&a_result, rx_data.payload, modbus_extheader.reg_cnt);
            logWrite("SUCCESS\n");
            return;
        }
    }
    logWrite("FAILED!\n");
}

#pragma optimize=none
static RdStatEnum rdBnrGateExecuteSiteSurvey(uint16_t a_rad_dev_addr)
{
    ModbusExtendedHeadStruct modbus_extheader;
    ModbusStatEnum rx_state;
    uint16_t ENABLE_VALUE = 1;
    
    logWrite("rdBnrGateClearSiteSurveyCounts[0x%x]...", a_rad_dev_addr);
    MB_CLEAR(modbus_extheader);
    modbus_extheader.rad_dev_addr = a_rad_dev_addr;
    modbus_extheader.start_addr = RDBNRGATE_REGADDR_SITESURVEYCONTROL;
    modbus_extheader.reg_cnt = sizeof(ENABLE_VALUE)/sizeof(uint16_t);
    for( uint8_t counter=1; counter<=3; counter++ )
    {
        rx_state = modbusExtendedWriteMultiple(&RD_BANNER_HANDLER, modbus_extheader, &ENABLE_VALUE);
        delayMillis(RD_TRANSMIT_DELAY);
        if(rx_state == MODBUS_OK)
        {
            logWrite("SUCCESS\n");
            return RDSTAT_OK;
        }
    }
    logWrite("FAILED!\n");
    return RDSTAT_ERROR;
}

#pragma optimize=none
static uint8_t rdBnrGateSiteSurveyToNode(uint16_t a_rad_dev_addr)
{
    uint16_t survey_count[4];
    uint8_t signal_strength_percentage = 0;
    enum surveycount_register{GREEN, YELLOW, RED, MISSED};
    
    rdBnrGateClearSiteSurveyCounts(a_rad_dev_addr);
    rdBnrGateExecuteSiteSurvey(a_rad_dev_addr); //TODO: Inside this function, test in banner radio if its needed some delay or the response is enough if there is have
    rdBnrGateReadSiteSurveyCounts(a_rad_dev_addr, survey_count);
    signal_strength_percentage = ((survey_count[GREEN]+survey_count[YELLOW]
                                   +survey_count[RED]+survey_count[MISSED])
                                  -survey_count[MISSED])*100; //TODO: R&D the proper computation of signal strength percentage
    
    return signal_strength_percentage;
}

#pragma optimize=none
static void rdBnrGateSendRssToNode(uint8_t a_mbid, uint8_t a_rss)
{
	RdHeadStruct header;
    RdStatEnum rx_state = RDSTAT_ERROR;
    
    rdClear(header);
	header.mb_id = a_mbid;
	header.function = RDFUNC_MAINTENANCE;
    header.type = RDNETMAINTENANCEMODETYP_RSSIDATA;
    header.value2 = a_rss;
    
	logWrite("rdBnrGateSendRssToNode[%d] to Node[0x%x]...", a_rss, a_mbid);
	
    for( uint8_t counter=1; counter<=3; counter++ )
    {
        rdBnrMainHeadTx(header);
        rdClear(header);
        rx_state = rdBnrMainRxResp(&header, DATARX_TIMEOUT);
        delayMillis(RD_TRANSMIT_DELAY);
        if(rx_state == RDSTAT_OK
           && header.mb_id == a_mbid 
           && header.function == RDFUNC_MAINTENANCE
           && header.type == RDNETMAINTENANCEMODETYP_RSSIDATA)
        {
            logWrite("SUCCESS\n");
            return;
        }
    }
    logWrite("FAILED!\n");
}

#pragma optimize=none
static void rdBnrGateSiteSurveyToAllNodes(void)
{
    const uint8_t BASE_MODBUS_ID = 0x0C;
    AttachedNodeStruct attached_nodes;
    ModbusExtendedHeadStruct modbus_extheader;
    uint16_t attached_node_count;
    uint8_t signal_strength;
    

    //Get # of active nodes
    attached_node_count = rdBnrGateGetNumberOfAttachedRadio();
    
    //List the device addresses of all active nodes
    rdBnrGateListAllAttachedNodes(attached_nodes.rad_dev_addr, attached_node_count);
    
    //Write the list to Modbus Id registered table
    rdBnrGateUpdateModbusToDeviceTable(attached_nodes.rad_dev_addr, attached_node_count);
    
    //Enable high power mode    
    rdBnrGateSetHighPowerMode(true);
    
    //Execute external site survey per node //Then, Update Node RSSI
    for(uint8_t index = BASE_MODBUS_ID; index<=BASE_MODBUS_ID+attached_node_count; index++)
    {
        signal_strength = rdBnrGateSiteSurveyToNode(index);
        rdBnrGateSendRssToNode(index, signal_strength);
    }
        
    //Disable high power mode
    rdBnrGateSetHighPowerMode(false);
}
                
#pragma optimize=none            
                
#pragma optimize=none
static void rdBnrGateMaintenanceModeProcess(void)
{
    uint32_t t_end;
    
    if(sleep_attr.nmm_status != RADNETMAINTENANCEMODE_ENABLE)
    {
        return;
    }
    
    
    t_end = HAL_GetTick() + (sleep_attr.nmm_timer_min * 60000);
    while(sleep_attr.nmm_timer_min > 0)         //do the loop process
    {
        rdBnrMainUpdateNmmTimer(t_end);
        if(sleep_attr.nmm_status == RADNETMAINTENANCEMODE_DISABLE)
        {
            sleep_attr.nmm_timer_min = 0;
            break;
        }
        rdBnrGateSiteSurveyToAllNodes();
    }
    
    HAL_NVIC_SystemReset();
}

#pragma optimize=none
static void rdBnrGateEnablingDisablingNmm(void)
{
    switch(sleep_attr.nmm_status)
    {
    case RADNETMAINTENANCEMODE_TOBEENABLED:
        rdBnrGateMaintenanceModeCmdToAllNodes(RDNETMAINTENANCEMODETYP_STARTCMD);
        break;
    case RADNETMAINTENANCEMODE_TOBEDISABLED:
        rdBnrGateMaintenanceModeCmdToAllNodes(RDNETMAINTENANCEMODETYP_STOPCMD);
        break;
    }
    rdBnrMainSwitchMaintenanceMode();
}
                
#pragma optimize=none              
void rdBnrGateRxFrameNmmChecker(RdHeadStruct a_header)
{
    if(a_header.function == RDFUNC_MAINTENANCE) //FOR MAINTENANCE MODE
    {
        if(a_header.type == RDNETMAINTENANCEMODETYP_STARTCMD 
           && sleep_attr.nmm_status == RADNETMAINTENANCEMODE_DISABLE)
        {
            sleep_attr.nmm_status = RADNETMAINTENANCEMODE_TOBEENABLED;
            sleep_attr.nmm_timer_min = a_header.value2;
            rdBnrGateEnablingDisablingNmm();
            rdBnrGateMaintenanceModeProcess();
        }
        else if(a_header.type == RDNETMAINTENANCEMODETYP_STOPCMD 
           && sleep_attr.nmm_status == RADNETMAINTENANCEMODE_ENABLE)
        {
            sleep_attr.nmm_status = RADNETMAINTENANCEMODE_TOBEDISABLED;
            sleep_attr.nmm_timer_min = 0;
            rdBnrGateEnablingDisablingNmm();
        }
    }
}
                
#endif //DEVICE_TYPE
/******************************************************************************
  * Revision History
  *	@file      	rdBnrGate.c
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
