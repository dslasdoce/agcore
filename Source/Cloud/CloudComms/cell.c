/*******************************************************************************
  * @file      	cell.c
  * @author     Hardware Team
  * @version    v2.2.0
  * @date       04/08/16
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

/* Includes ==================================================================*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <ctype.h>
#include <deployAssert.h>

#include "productCfg.h"
#include "agBoardInit.h"
#include "utilMem.h"
#include "cell.h"
#include "http.h"
#include "delay.h"
#include "rtc.h"
#include "agdatim.h"
#include "extFlash.h"

/* Defines ==================================================================*/
#define DEFAULT_CONTEXTID 		((uint32_t)0x01)
#define SIZE_CNTXT_ACTIVATION 	((uint8_t)15)
#define FW_MAXLOOP		((uint32_t)200)


#define RSSI_NOTDETECTABLE			(99)
#define RSSI_SIGNALMAX				(31)
#define RSSI_SIGNALMIN				(0)
#define RSSI_PERCENTMAX				(100)

/* Globals ==================================================================*/
uint32_t cell_idx_buff;
uint8_t cell_rx_buff_single;
/*setup parameters*/
static uint8_t active_cid = DEFAULT_CONTEXTID;

/*data activation command/respnse */
#if CELLTYPE == CELLTYPE_HSPA
	const uint8_t available_cid[] = {1};
#else
	const uint8_t available_cid[] = {1};
#endif

/*at common*/
const CellStatusEnum cell_common_stats[] = {CELL_AT_OK, CELL_AT_ERROR};
const char *cell_common_resps[] = {"OK","ERROR"};

/* Functions ==================================================================*/
static CellStatusEnum cellIsNetworkReady(void);
static CellStatusEnum cellSetRoaming(void);

/*=============================================================================
 * @brief	look for a string in an array
 * @param	refstr_ptr: string to find
 * @param	patternstr_arr_ptr: array of string to be searched
 * @param	pattern_count: count of elements in patternstr_arr_ptr
 * @param	idx_found_ptr: pointer to storage of index of matched pattern
 * ==========================================================================*/
static inline bool matchStringFromArray(char *refstr_ptr,
										const char **patternstr_arr_ptr,
										uint8_t pattern_count,
										uint8_t *idx_found_ptr)
{
	for(uint32_t idx_pattern=0; idx_pattern<pattern_count; idx_pattern++)
	{
		if(strstr(refstr_ptr,patternstr_arr_ptr[idx_pattern]))
		{
			*idx_found_ptr = idx_pattern;
			return true;
		}
	}
	return false;
}

/*==============================================================================
 * @brief 
 * @param
 * @retval
 *=============================================================================*/
CellStatusEnum serialReadUntil(char *resp, uint32_t resp_maxsize ,char *end_str,
									  CellStatusEnum success_response, uint32_t timeout)
{
	uint32_t t_start = HAL_GetTick();
	uint32_t timeout_millis = (uint32_t)(timeout*1000);
	uint8_t rx_char = 0;
	uint32_t idx_resp = 0;
	uint32_t t_current = HAL_GetTick();
	while (delayTimeDiff(t_start, t_current) < timeout_millis)
	{

		if(idx_resp >= resp_maxsize)
		{
			return CELL_BUFFEROVERFLOW;
		}

		if (uartRxCell(&rx_char, sizeof(rx_char)) == HAL_OK)
		{
			resp[idx_resp++] = rx_char;
		}
		else
		{
			HAL_UART_DeInit(&UART1HandlerDef);
			HAL_UART_Init(&UART1HandlerDef);
		}

		gpioDigitalWrite(&CELL_RTS, HIGH);
		if(strstr(resp,end_str))
		{
			return success_response;
		}
		gpioDigitalWrite(&CELL_RTS, LOW);

		t_current = HAL_GetTick();
	}

	if(strstr(resp,end_str))
	{
		return success_response;
	}
	return CELL_ERROR;
}

/*=============================================================================
 * @brief	reads serial until one of the patterns is found
 * @param	refstr_ptr: string to find
 * @param	patternstr_arr_ptr: array of string to be searched
 * @param	pattern_count: count of elements in patternstr_arr_ptr
 * @param	statmap_ptr: array of possible status to be returned
 * @param	timeout: reception timeout
 * @retval	CellStatus
  ===========================================================================*/
CellStatusEnum serialReadUntilMultiple(char *refstr_ptr,
									   const char **patternstr_arr_ptr,
									   uint8_t pattern_count,
									   const CellStatusEnum *statmap_ptr,
									   uint32_t timeout)
{
	uint32_t t_start = HAL_GetTick();
	uint32_t timeout_millis = (uint32_t)(timeout*1000);
	uint32_t idx_resp = 0;
	uint32_t t_prevcomp = HAL_GetTick();
	uint32_t t_current = HAL_GetTick();
	uint8_t idx_pattern_found = 0;
	uint8_t rx_char = 0;

	while (delayTimeDiff(t_start, HAL_GetTick()) < timeout_millis)
	{
		if (uartRxCell(&rx_char, sizeof(rx_char)) == HAL_OK)
		{
			refstr_ptr[idx_resp++] = rx_char;
			if(idx_resp == AT_MAXSIZE)
				return CELL_BUFFEROVERFLOW;
		}
		else
		{
			HAL_UART_DeInit(&UART1HandlerDef);
			HAL_UART_Init(&UART1HandlerDef);
		}

		if(delayTimeDiff(t_prevcomp, t_current) > INTERVAL_MULTIPLE_SEARCH)
		{
			gpioDigitalWrite(&CELL_RTS, HIGH);
			t_prevcomp = HAL_GetTick();
			if(matchStringFromArray(refstr_ptr, patternstr_arr_ptr,
									pattern_count, &idx_pattern_found))
				return statmap_ptr[idx_pattern_found];
			gpioDigitalWrite(&CELL_RTS, LOW);

		}
		t_current = HAL_GetTick();
	}

	if(matchStringFromArray(refstr_ptr, patternstr_arr_ptr, pattern_count,
							&idx_pattern_found))
		return statmap_ptr[idx_pattern_found];
	return CELL_NO_RESPONSE;
}

/*=============================================================================
 * @brief	send simple at command
 * ==========================================================================*/
static CellStatusEnum cellSendAT(void)
{
	volatile CellStatusEnum current_stat;
	char response[AT_MAXSIZE];
	memset(response,0,AT_MAXSIZE);
	const char *atgen = "AT\r\n";
	if (uartTxCell((uint8_t *)atgen, strlen(atgen)) == HAL_OK)
	{
		current_stat = serialReadUntilMultiple(response, cell_common_resps,
					   sizeof(cell_common_resps)/sizeof(const char*),
					   cell_common_stats, CELL_COMMON_TIMEOUT);
	}
	delayMillis(200);
	return current_stat;
}

#pragma GCC push_options
#pragma optimize=none
CellStatusEnum cellSignalQuality(uint8_t *signal_strength)
{
  	CellStatusEnum current_stat;
	char response[AT_MAXSIZE];
	const char *atcsq = "AT+CSQ\r\n";
	const char *atcsq_key = "+CSQ:";
	char *csq_resp_addr = NULL;
	*signal_strength = 0;
//	for(uint8_t i = 0; i < 3; i++)
//	{
	  	for(uint8_t i = 0; i < 3; i++)
		{
			current_stat = cellIsNetworkReady();
			if(current_stat == CELL_NET_HOME 
			   || current_stat == CELL_NET_ROAMING 
			   || current_stat == CELL_NET_SEARCHING)
				break;
			delayMillis(500);
		}
		
		if (uartTxCell((uint8_t *)atcsq, strlen(atcsq)) == HAL_OK)
		{
			current_stat = serialReadUntilMultiple(response, cell_common_resps,
						   sizeof(cell_common_resps)/sizeof(const char*),
						   cell_common_stats, CELL_COMMON_TIMEOUT);
		}
		
		if(current_stat == CELL_AT_OK)
		{
		  	//logWrite(response);
			csq_resp_addr = strstr(response, atcsq_key);
			csq_resp_addr += strlen(atcsq_key);
			*signal_strength = (uint8_t)strtol(csq_resp_addr,NULL,10);
			
			if(*signal_strength >= RSSI_SIGNALMIN 
			   && *signal_strength <= RSSI_SIGNALMAX)
					*signal_strength =  (uint8_t)((float)*signal_strength
										*RSSI_PERCENTMAX/RSSI_SIGNALMAX);
			
			else if (*signal_strength == RSSI_NOTDETECTABLE)
			  	*signal_strength = 1;
			
			else
				*signal_strength = 0;
			//break;
		}
		else
		{
			cellForcedShutDown();
			cellON_OFFBypass(true);
		}
	//}
	return current_stat;
}
#pragma GCC pop_options

/* ============================================================================
 * @brief	turn on/off cell modem
 * @param	state: true = ON, false = off
 * ==========================================================================*/
void cellON_OFF(bool state)
{
	const char *at_off = "AT#SHDN\r\n";
	CellStatusEnum current_stat = CELL_NO_RESPONSE;
	char response[AT_MAXSIZE];
		
	if(state)
	{
	  	#if PRINT_PROCESSES_IN_UART
		logWrite("+++++CELL TURN ON+++++\n");
		#endif
		
		for(uint8_t i = 0; i < 3; i++)
		{
		  	current_stat = cellSendAT();
			if(current_stat == CELL_AT_OK)
			  	break;
			else
			  	delayMillis(500);
		}
	  	if (current_stat != CELL_AT_OK)
		{
		  	gpioDigitalWrite(&CELL_ON_OFF, HIGH);
			delayMillis(3000);
			gpioDigitalWrite(&CELL_ON_OFF, LOW);
		}
		else
		{
		  	#if PRINT_PROCESSES_IN_UART
				logWrite("+++++CELL ALREADY ON+++++\n");
			#endif
		}
		delayMillis(11000);
        //cellFTPTO();
		//if(cellFTPTO() != CELL_AT_OK)
		//	assert(0);
	}
	else
	{
	  	#if PRINT_PROCESSES_IN_UART
		logWrite("+++++CELL TURN OFF+++++\n");
		#endif
	  
		if(uartTxCell((uint8_t *)at_off,  strlen(at_off)) == HAL_OK)
		{
			current_stat = serialReadUntilMultiple(response, cell_common_resps,
						   sizeof(cell_common_resps)/sizeof(const char*),
						   cell_common_stats, 25);
			if(current_stat != CELL_AT_OK)
			{
			  	cellForcedShutDown();
			}
		}

	}
}

/* ============================================================================
 * @brief	turn on/off cell modem
 * @param	state: true = ON, false = off
 * ==========================================================================*/
void cellON_OFFBypass(bool state)
{
	const char *at_off = "AT#SHDN\r\n";
	volatile CellStatusEnum current_stat;
	char response[AT_MAXSIZE];
		
	if(state)
	{
	  	#if PRINT_PROCESSES_IN_UART
		logWrite("+++++CELL TURN ON+++++\n");
		#endif
		
	  	if (cellSendAT() != CELL_AT_OK)
		{
		  	gpioDigitalWrite(&CELL_ON_OFF, HIGH);
			delayMillis(3000);
			gpioDigitalWrite(&CELL_ON_OFF, LOW);
		}
		else
		{
		  	#if PRINT_PROCESSES_IN_UART
				logWrite("+++++CELL ALREADY ON+++++\n");
			#endif
		}
	}
	else
	{
	  	#if PRINT_PROCESSES_IN_UART
		logWrite("+++++CELL TURN OFF+++++\n");
		#endif
	  
		if(uartTxCell((uint8_t *)at_off,  strlen(at_off)) == HAL_OK)
		{
			current_stat = serialReadUntilMultiple(response, cell_common_resps,
						   sizeof(cell_common_resps)/sizeof(const char*),
						   cell_common_stats, 25);
			if(current_stat != CELL_AT_OK)
			{
			}
		}

	}
}

/* ============================================================================
 * @brief	initialize specific context id settings
 * ==========================================================================*/
#if CELLTYPE == CELLTYPE_HSPA
	static CellStatusEnum cellSetupContext(uint8_t cid, char *cell_apn)
#else
	static CellStatusEnum cellSetupContext(void)
#endif
{
	char printout[50];
	char response[AT_MAXSIZE];
	volatile CellStatusEnum current_stat;
	/*socket and connection settings at command*/
	#if CELLTYPE == CELLTYPE_HSPA
		char *at_cntxsetting_frmt = "AT+CGDCONT=%d,\"IP\",\"%s\",\"0.0.0.0\",0,0\r";
		char *at_cgqmin_frmt = "AT+CGQMIN=%d,0,1,0,0,31\r\n";
		char *at_cgqreq_frmt = "AT+CGQREQ=%d,0,0,3,0,0\r\n";
		char *at_cgeqmin_frmt = "AT+CGEQMIN=%d,0,0,0,0,0,0,0,\"0E0\",\"0E0\",0,0,1,0,0\r\n";
		char *at_cgeqreq_frmt = "AT+CGEQREQ=%d,4,0,0,0,0,0,1500,\"1E4\",\"1E5\",0,0,0,0,0\r\n";
		char *at_scfg_frmt = "AT#SCFG=%d,%d,300,90,300,50\r\n";

		char at_cntxsetting[MAXSIZE_CELL_AT];
		char at_cgqmin[MAXSIZE_CELL_AT];
		char at_cgqreq[MAXSIZE_CELL_AT];
		char at_cgeqmin[MAXSIZE_CELL_AT];
		char at_cgeqreq[MAXSIZE_CELL_AT];
		char at_scfg[MAXSIZE_CELL_AT];

		snprintf(at_cntxsetting, MAXSIZE_CELL_AT, at_cntxsetting_frmt,
				 cid, cell_apn);
		snprintf(at_cgqmin,MAXSIZE_CELL_AT,at_cgqmin_frmt, cid);
		snprintf(at_cgqreq,MAXSIZE_CELL_AT,at_cgqreq_frmt, cid);
		snprintf(at_cgeqmin,MAXSIZE_CELL_AT,at_cgeqmin_frmt, cid);
		snprintf(at_cgeqreq,MAXSIZE_CELL_AT,at_cgeqreq_frmt, cid);
		snprintf(at_scfg,MAXSIZE_CELL_AT,at_scfg_frmt, cid, cid);

		const char *setup_list[] = {at_cntxsetting, at_cgqmin, at_cgqreq,
								at_cgeqmin, at_cgeqreq, at_scfg};
	#else
		char *at_scfg = "AT#SCFG=1,1,300,90,300,50\r\n";
		const char *setup_list[] = {at_scfg};
	#endif

	for(uint8_t i = 0; i<sizeof(setup_list)/sizeof(const char*); i++)
	{
		memset(response,0,AT_MAXSIZE);
		if(uartTxCell((uint8_t *)setup_list[i],  strlen(setup_list[i])) ==
						HAL_OK)
		{
			current_stat = serialReadUntilMultiple(response, cell_common_resps,
						   sizeof(cell_common_resps)/sizeof(const char*),
						   cell_common_stats, CELL_COMMON_TIMEOUT);
			
			#if PRINT_PROCESSES_IN_UART
				logWrite("%s",response);
			#endif
				
			if ( current_stat != CELL_AT_OK)
				return CELL_AT_ERROR;
		}
		else
			return CELL_NO_RESPONSE;
	}
	return CELL_AT_OK;
}

static CellStatusEnum cellSetRoaming(void)
{
  	CellStatusEnum current_stat = CELL_NO_RESPONSE;
	const char *roam_set = "AT+CLCK=\"AB\",0\r\n";
	char response[AT_MAXSIZE];
	memset(response,0,AT_MAXSIZE);
	
//	delayMillis(500);
//	if(uartTxCell((uint8_t *)roam_set,  strlen(roam_set)) == HAL_OK)
//	{
//		current_stat = serialReadUntilMultiple(response, cell_common_resps,
//					   sizeof(cell_common_resps)/sizeof(const char*),
//					   cell_common_stats, CELL_COMMON_TIMEOUT);
//		
//		#if PRINT_PROCESSES_IN_UART
//			sprintf(printout, "%s",response);
//			logWrite((uint8_t*)printout);
//		#endif
//			
//	}
	
	return current_stat;

}
/*=============================================================================
 * @brief	cellular modem settings
 * ==========================================================================*/
void cellSetupParameters(void)
{
	CellStatusEnum current_stat = CELL_NO_RESPONSE;
	#if PRINT_PROCESSES_IN_UART
		logWrite("+++++CELL SETUP+++++\n");
	#endif
		
	#if CELLTYPE == CELLTYPE_HSPA
		uint8_t cid = 1;
		char response[AT_MAXSIZE];
		const char *at_spn = "AT#SPN\r\n";
		char cell_apn[20];
		delayMillis(500);
		const char *spn_sun = "SUN";
		const char *spn_smart = "SMART";
		const char *spn_globe = "GLOBE";
		memset(response, 0, AT_MAXSIZE);

		/*set apn*/
		if(uartTxCell((uint8_t *)at_spn,  strlen(at_spn)) == HAL_OK)
		{
			current_stat = serialReadUntilMultiple(response, cell_common_resps,
						   sizeof(cell_common_resps)/sizeof(const char*),
						   cell_common_stats, CELL_COMMON_TIMEOUT);
				#if PRINT_PROCESSES_IN_UART
					//logWrite((uint8_t*)response);
				#endif
			if(current_stat == CELL_AT_OK)
			{
				#if PRINT_PROCESSES_IN_UART
					logWrite("%s",response);
				#endif
			  	
				if(strstr(response, spn_sun))
					strcpy(cell_apn,"minternet");
				else if(strstr(response, spn_smart))
					strcpy(cell_apn,"internet");
				else if(strstr(response, spn_globe))
				  	strcpy(cell_apn,"internet.globe.com.ph");
				else
				  	strcpy(cell_apn,"iot.aer.net");
					//strcpy(cell_apn,"globe.com.ph");
			}
			
			else
			{
				uartTxCell((uint8_t *)at_spn,  strlen(at_spn));
				current_stat = serialReadUntilMultiple(response, cell_common_resps,
							   sizeof(cell_common_resps)/sizeof(const char*),
							   cell_common_stats, CELL_COMMON_TIMEOUT);

				if(current_stat == CELL_AT_OK)
				{
				  	#if PRINT_PROCESSES_IN_UART
						logWrite("%s",response);
					#endif
					if(strstr(response, spn_sun))
						strcpy(cell_apn,"minternet");
					else if(strstr(response, spn_smart))
						strcpy(cell_apn,"internet");
					else if(strstr(response, spn_globe))
				  		strcpy(cell_apn,"internet.globe.com.ph");
					else
						strcpy(cell_apn,"iot.aer.net");
						//strcpy(cell_apn,"globe.com.ph");
				}
				else
				  	strcpy(cell_apn,"iot.aer.net");
			}
		}
		
		
		delayMillis(100);

		/*set context parameters*/
		for(cid = 1; cid <= MAXSIZE_CID; cid++)
		{
			current_stat = cellSetupContext(cid, cell_apn);
			if(current_stat != CELL_AT_OK)
			{
				 assert(0);
			}
		}
	#else
		current_stat = cellSetupContext();
		if(current_stat != CELL_AT_OK)
		{
			/*log here*/
		   assert(0);
		}
	#endif
}



/*=============================================================================
 * @brief	activate data on a specific context id
 * @param	cid: context id to be activated
 * ==========================================================================*/
static CellStatusEnum cellActivateContext(uint8_t cid)
{

	volatile CellStatusEnum current_stat;
	char response[AT_MAXSIZE];
	memset(response,0,AT_MAXSIZE);

	#ifdef CELL == CELLTYPE_HSPA
		char *at_activatedata_frmt 	= "AT#SGACT=%d,1\r\n";
		char *rsp_contextactive_frmt 	= "#SGACT: %d,1";

		char at_activatedata[SIZE_CNTXT_ACTIVATION];
		char rsp_contextactive[SIZE_CNTXT_ACTIVATION];
		snprintf(rsp_contextactive, SIZE_CNTXT_ACTIVATION,
				 rsp_contextactive_frmt, cid);

		snprintf(at_activatedata,SIZE_CNTXT_ACTIVATION,
				 at_activatedata_frmt, cid);
	#else
		const char *at_activatedata 	= "AT#CDMADC=1\r\n";
	#endif

	cellSendAT();
	delayMillis(500);

	memset(response, 0, AT_MAXSIZE);
	if (uartTxCell((uint8_t *)at_activatedata,
				   strlen(at_activatedata)) == HAL_OK)
	{
		current_stat = serialReadUntilMultiple(response, cell_common_resps,
					   sizeof(cell_common_resps)/sizeof(const char*),
					   cell_common_stats, TIMEOUT_CELL_DATAACTIVATION);
		if(current_stat == CELL_AT_OK)
		{
			/* Activation successful*/
			#if PRINT_PROCESSES_IN_UART
				logWrite("+++++DATA ACTIVATION SUCCESSS+++++\n");
			#endif
			active_cid = cid;
			return CELL_DATA_ON;
		}
		else
		{
			#if PRINT_PROCESSES_IN_UART
				logWrite("+++++DATA ACTIVATION FAILED+++++\n");
			#endif
			return CELL_DATA_OFF;
		}
	}
	else
		return CELL_NO_RESPONSE;
}

/*=============================================================================
 * @brief 	read data coming from host
 * @param	resp: ptr to data storage
 * @param	resp_maxsize: limit of storage
 * @param	timeout: reception timeout
 * ==========================================================================*/
static DataStatusEnum cellReadWebData(char *resp, uint32_t resp_maxsize,
									  uint32_t timeout)
{
	DataStatusEnum data_stat = DATA_PENDING;
	char *httpkey_content_length = "Content-Length: ";
	char *httpkey_conlen_addr = NULL;
	char *httpkey_conlen_endaddr = NULL;
	char *data_start_ptr = NULL;
	const char *no_carrier = "\r\n\r\nNO CARRIER";
	uint8_t rx_char = 0;
	uint32_t content_length = 0;
	uint32_t t_start = HAL_GetTick();
	uint32_t timeout_millis = (uint32_t)(timeout*1000);
	uint32_t idx_resp = 0;
	uint32_t t_prevcomp = HAL_GetTick();
	uint32_t t_current = HAL_GetTick();
	gpioDigitalWrite(&CELL_RTS, LOW);
	while (delayTimeDiff(t_start, t_current) < timeout_millis)
	{
		if(idx_resp >= resp_maxsize)
		{
			resp[idx_resp - 1] = 0;
			return DATA_BUFFEROVERFLOW;
		}

		if (uartRxCell(&rx_char, sizeof(rx_char)) == HAL_OK)
		{
			resp[idx_resp++] = rx_char;
		}
		else
		{
			HAL_UART_DeInit(&UART1HandlerDef);
			HAL_UART_Init(&UART1HandlerDef);
		}

		if(delayTimeDiff(t_prevcomp, t_current) > 300)
		{
			t_prevcomp = HAL_GetTick();
			if(data_stat == DATA_PENDING)
			{
				/*look for Content-Length string*/
				httpkey_conlen_addr = strstr(resp, httpkey_content_length);
				if(httpkey_conlen_addr)
					/*check if content lenght is complete*/
					httpkey_conlen_endaddr = strstr(httpkey_conlen_addr,
													"\r\n");
				/*content length is complete*/
				if(httpkey_conlen_addr && httpkey_conlen_endaddr)
				{
					/*get value of content length*/
					content_length =
								strtoul((const char*)httpkey_conlen_addr +
								strlen(httpkey_content_length), NULL, 10);
					/*if there is data after http header*/
					if(content_length)
						data_stat = DATA_STARTCH_WAIT;
					else
						data_stat = DATA_STARTCH_READY;
				}
			}

			if(data_stat == DATA_STARTCH_WAIT)
			{
				/*wait for the start character to be found*/
				data_start_ptr = strstr(resp, "{");
				if(data_start_ptr)
					/*add header at data length to actual expected data*/
					content_length += (data_start_ptr - resp);
					data_stat = DATA_STARTCH_READY;
			}

			/*check if data is complete*/
			if((data_stat == DATA_STARTCH_READY) && (idx_resp >= content_length))
			{
				if(resp[content_length -1] == '}')
					return DATA_VALID;
				else
					return DATA_INVALID;
				delayMillis(10);

			}

			if(strstr(resp, no_carrier))
			{
				if(idx_resp - strlen(no_carrier) == content_length)
									return DATA_VALID;
				return data_stat;
			}
		}
		t_current = HAL_GetTick();
	}

	if((content_length == idx_resp) && (idx_resp != 0))
	{
		return DATA_VALID;
	}

	return DATA_INVALID;
}

void cellForcedShutDown(void)
{
  	gpioDigitalWrite(&CELL_NRESET, HIGH);
    delayMillis(500);
    gpioDigitalWrite(&CELL_NRESET, LOW);
    delayMillis(1000);
}
/*=============================================================================
 * @brief	reset cell modem
 * ==========================================================================*/
void cellReset(void)
{
	#if PRINT_PROCESSES_IN_UART
		logWrite("\n+++++CELL RESET+++++\n");
	#endif
		cellForcedShutDown();
		//cellON_OFF(false);
		cellON_OFF(true);
	#if PRINT_PROCESSES_IN_UART
		logWrite("\n+++++CELL RESET DONE+++++\n");
	#endif
}
CellStatusEnum cellModemTestMode()
{
  	char rx_char;
  	char at_command[AT_MAXSIZE];
	char response[AT_MAXSIZE];
	CellStatusEnum current_stat;
	uint8_t i = 0;
	memset(at_command, 0, AT_MAXSIZE);
	while(1)
	{
		if(uartRxSys(&rx_char, 1) == HAL_OK)
		{
		  	at_command[i++] = rx_char;
			if(rx_char == '\n')
			{
			  	logWrite((uint8_t*)at_command);
				if(uartTxCell((uint8_t *)at_command, strlen(at_command) ) == HAL_OK)
				{
					current_stat = serialReadUntilMultiple(response, cell_common_resps,
								   sizeof(cell_common_resps)/sizeof(const char*),
								   cell_common_stats, 20);
				}
				logWrite((uint8_t*)response);
				logWrite("done\n");
				HAL_UART_DeInit(&UART5HandlerDef);
				HAL_UART_Init(&UART5HandlerDef);
				memset(response, 0, AT_MAXSIZE);
				memset(at_command, 0, AT_MAXSIZE);
				i = 0;
			}
		}
		else
		{
		  	HAL_UART_DeInit(&UART5HandlerDef);
			HAL_UART_Init(&UART5HandlerDef);
		}

	}
	return current_stat;
}
/*=============================================================================
 * @brief	check network status
 * ==========================================================================*/
static CellStatusEnum cellIsNetworkReady(void)
{
	char response[AT_MAXSIZE];
	char printout[50];
	/*network relared command/response*/
	const char *at_netstatus = "AT+CREG?\r\n";
	const char *rsp_net_home = "+CREG: 0,1";
	const char *rsp_net_roam = "+CREG: 0,5";
	const char *rsp_net_notreg = "+CREG: 0,0";
	const char *rsp_net_searching = "+CREG: 0,2";

	const CellStatusEnum cell_net_stats[] = {CELL_NET_HOME, CELL_NET_ROAMING,
										CELL_NET_NOTREG, CELL_NET_SEARCHING};
	const char *cell_net_resps[] = {rsp_net_home, rsp_net_roam,
									rsp_net_notreg,rsp_net_searching};
	volatile CellStatusEnum current_stat;

	memset(response,0,AT_MAXSIZE);
	if(uartTxCell((uint8_t *)at_netstatus, strlen(at_netstatus) ) == HAL_OK)
	{
		current_stat = serialReadUntilMultiple(response, cell_net_resps,
					   sizeof(cell_net_resps)/sizeof(const char*),
					   cell_net_stats, 2);
	}
	else
		current_stat = CELL_NO_RESPONSE;

	#if PRINT_PROCESSES_IN_UART
		if(current_stat == CELL_NET_HOME)
			sprintf(printout,"+++++CELL Network Home...\n");
		else if (current_stat == CELL_NET_ROAMING)
		{
		  	sprintf(printout,"+++++CELL Network Roaming...\n");
		}
		else if (current_stat == CELL_NET_SEARCHING)
		  	sprintf(printout,"+++++CELL Network Searching...\n");
		else
		  	sprintf(printout,"+++++CELL No Response...\n");
		logWrite((uint8_t*)printout);
	#endif
	
	return current_stat;
}

void cellSetDateTime(void)
{
	#if PRINT_PROCESSES_IN_UART
		logWrite("\n+++++DATE TIME SETTING+++++\n");
	#endif
	
	volatile CellStatusEnum  current_stat = CELL_NET_NOTREG;
	char *url;
	const char *url_main = CLOUDHOST;
	const char *url_loc = "api/v0.1/telemetry/?limit=1";
	url = memAlloc(strlen(url_main) + strlen(url_loc));
	sprintf(url, "%s%s", url_main,url_loc);
	uint32_t failcount = 0;
	while(1)
	{
		current_stat = cellIsNetworkReady();
		if(current_stat == CELL_NET_HOME ||
			current_stat == CELL_NET_ROAMING)
			break;
		if(failcount == 10)
		{
			cellForcedShutDown();
			cellON_OFF(true);
		}
		if(failcount == 20)
			assert(0);
		delayMillis(3000);
		failcount++;
	}
//	#if PRINT_PROCESSES_IN_UART
//		if(current_stat == CELL_NET_HOME || current_stat == CELL_NET_ROAMING)
//		{
//			const char * printout = "\n+++++CELL NETWORK FOUND+++++\n";
//			logWrite((uint8_t*)printout);
//		}
//	#endif
	HTTPStruct httpstruct;
	RTCDateTimeStruct date_time;
	for(uint8_t i = 0; i < 5; i++)
	{
		cellGet(url, AUTHENTICATION, &httpstruct);

		if(httpstruct.data_stat == DATA_VALID)
		{
			agdatimGETRspParser(httpstruct.http_date, &date_time);
			rtcSetDateTime(&date_time);
			exRTCWriteTime(&date_time);
			#if PRINT_PROCESSES_IN_UART
				logWrite("+++++DATE TIME SETTING COMPLETE+++++\n");
			#endif
			return;
		}
		delayMillis(1000);
	}
	#if PRINT_PROCESSES_IN_UART
		logWrite("\n+++++FAILED: NO INTERNET+++++\n");
	#endif
}


void cellProvision(void)
{
	const char *cell_common_resps2[] = {"NO CARRIER","ERROR"};
	volatile CellStatusEnum current_stat = CELL_AT_ERROR;
	char response[AT_MAXSIZE];
	memset(response,0,AT_MAXSIZE);
	const char *atgen = "ATD*22899;\r\n";
	#if PRINT_PROCESSES_IN_UART
		logWrite("\n+++++PROVISIONING+++++\n");
	#endif
	uint32_t try_count = 0;
	while(1)
	{
		if(try_count > 20)
			 assert(0);
		current_stat = cellIsNetworkReady();
		if(current_stat == CELL_NET_HOME ||
			current_stat == CELL_NET_ROAMING)
			break;
		delayMillis(3000);
		try_count++;
	}

	#if PRINT_PROCESSES_IN_UART
		if(current_stat == CELL_NET_HOME || current_stat == CELL_NET_ROAMING)
		{
			logWrite("\n+++++CELL NETWORK FOUND+++++\n");
		}
	#endif

	if (uartTxCell((uint8_t *)atgen, strlen(atgen)) == HAL_OK)
	{
		current_stat = serialReadUntilMultiple(response, cell_common_resps2,
					   sizeof(cell_common_resps2)/sizeof(const char*),
					   cell_common_stats, 300);
	}
	if(current_stat == CELL_AT_OK)
	{
		logWrite((uint8_t*)response);
		logWrite((uint8_t*)"\n+++++CELL PR COMPLETE+++++\n");
	}
	delayMillis(200);
}

/*=============================================================================
 * @brief	check data connection status
 * ==========================================================================*/
static CellStatusEnum cellIsDataActive(void)
{
	char response[AT_MAXSIZE];

	#if CELLTYPE == CELLTYPE_HSPA
		const char *at_contexstat 		= "AT#SGACT?\r\n";
		const char* cell_data_resps[] = {"#SGACT: 1,1", "#SGACT: 2,1",
										 "#SGACT: 3,1", "#SGACT: 4,1",
										 "#SGACT: 5,1"};
	#else
		const char* cell_data_resps[] = {"#CDMADC: 1"};
		const char *at_contexstat 		= "AT#CDMADC?\r\n";
	#endif
	volatile CellStatusEnum current_stat;

	uint8_t idx_found = 0;
	memset(response,0,AT_MAXSIZE);
	if(uartTxCell((uint8_t *)at_contexstat, strlen(at_contexstat)) == HAL_OK)
	{
		current_stat = serialReadUntilMultiple(response, cell_common_resps,
					   sizeof(cell_common_resps)/sizeof(const char*),
					   cell_common_stats, CELL_COMMON_TIMEOUT);
		if(current_stat == CELL_AT_OK)
		{
			if (matchStringFromArray(response, cell_data_resps,
				sizeof(cell_data_resps)/sizeof(const char*), &idx_found))
			{
				#if PRINT_PROCESSES_IN_UART
				const char *msg_act = "+++++DATA ACTIVE+++++\n";
				logWrite((uint8_t*)msg_act);
				#endif
				active_cid = ++idx_found;
				current_stat = CELL_DATA_ON;
			}
			else
			{
				#if PRINT_PROCESSES_IN_UART
				const char *msg_inact = "+++++DATA INACTIVE+++++\n";
				logWrite((uint8_t*)msg_inact);
				#endif
				current_stat = CELL_DATA_OFF;
			}
		}
		return current_stat;
	}
	else
		return CELL_NO_RESPONSE;
}

/*=============================================================================
 * @brief	activate cellular data
 * ==========================================================================*/
CellStatusEnum cellActData(void)
{
	uint8_t trial = 0;
	volatile CellStatusEnum current_stat;
	uint8_t no_response_cnt = 1;
	
	for(trial=1; trial<=10;trial++)
	{
		/* check if cell is connected to network */
		current_stat = cellIsNetworkReady();
		if(current_stat != CELL_NET_HOME && current_stat != CELL_NET_ROAMING)
		{
			delayMillis(1000);
			current_stat = cellIsNetworkReady();
		}
		if(current_stat == CELL_NET_HOME || current_stat == CELL_NET_ROAMING)
		{
		/* Network carrier is active, check if data is still active */
			current_stat = cellIsDataActive();
			if(current_stat == CELL_DATA_OFF)
			{
				/* Activate cellular data */
				for(uint32_t idx_cid=0;
					idx_cid<sizeof(available_cid)/sizeof(uint8_t);
					idx_cid++)//(sizeof(available_cid)/sizeof(uint8_t)); idx_cid++)
				{
					current_stat = cellActivateContext(available_cid[idx_cid]);//available_cid[idx_cid]);
					if(current_stat == CELL_DATA_ON)
						return current_stat;
					delayMillis(1000);
				}
			}
			else if(current_stat == CELL_DATA_ON)
				return current_stat;
		}

		else if (current_stat == CELL_NET_NOTREG ||
				 current_stat == CELL_NET_SEARCHING)
			/*No network but cell modem is responding, try again*/
			delayMillis(5000);

		else
		{
			/*Cell modem is not responding, reset it*/
		  	no_response_cnt++;
			if(no_response_cnt == CELL_NO_RESP_LIMIT)
			{
			  	no_response_cnt = 1;
				cellReset();
			}
			delayMillis(1000);
		}
	}
	return current_stat;
}

/*=============================================================================
 * @brief	turnof cellular data
 * ==========================================================================*/
CellStatusEnum cellDeactData(void)
{
	#if CELLTYPE == CELLTYPE_HSPA
			char *at_deactivatedata_frmt 	= "AT#SGACT=%d,0\r\n";
			char at_deactivatedata[SIZE_CNTXT_ACTIVATION];
			snprintf(at_deactivatedata,SIZE_CNTXT_ACTIVATION,
					 at_deactivatedata_frmt, active_cid);
	#else
			const char *at_deactivatedata 	= "AT#CDMADC=0\r\n";
	#endif

	char response[AT_MAXSIZE];
	volatile CellStatusEnum current_stat;
	uint32_t t_start = HAL_GetTick();

	while(delayTimeDiff(t_start,HAL_GetTick()) <
					TIMEOUT_CELL_DATAACTIVATION*1000)
	{
		/* Issue data deactivation command*/
		current_stat = cellIsDataActive();
		if(current_stat == CELL_DATA_OFF)
			return CELL_DATA_OFF;

		memset(response,0,AT_MAXSIZE);
		if(uartTxCell((uint8_t *)at_deactivatedata,
					  strlen(at_deactivatedata)) == HAL_OK)
		{
			current_stat = serialReadUntilMultiple(response, cell_common_resps,
						   sizeof(cell_common_resps)/sizeof(const char*),
						   cell_common_stats, TIMEOUT_CELL_DATAACTIVATION);
			if(current_stat != CELL_AT_OK)
			{
				/* error log here */
			}
			 delayMillis(2000);
		}
		else
			return CELL_NO_RESPONSE;
	}
	return CELL_DATA_ON;
}

/*=============================================================================
 * @brief	transmit and receive data to and from host
 * @param	webresponse: storage for data from host
 * @param	urlstruct_ptr: struc of host url
 * @param	httpstruct:	struct of parsed web response
 * @param	data: ptr to data to be transmited
 * ==========================================================================*/
static CellStatusEnum cellTxRxWeb(char *webresponse, URLStruct *urlstruct_ptr,
								  HTTPStruct *httpstruct, char *data)
{

	CellStatusEnum current_stat = CELL_ERROR;
	DataStatusEnum data_stat = DATA_EMPTY;
	char *socketdial = memAlloc(sizeof(char)*HEADER_FIELD_SIZE );
	char socketclose[20];
	char *socketdial_frmt =  "AT#SD=%d,0,80,\"%s\",0,0,0\r\n";
	char *socketclose_frmt = "AT#SH=%d\r\n";
	char response[AT_MAXSIZE];
	const char *no_carrier = "NO CARRIER";
	char *no_carrier_ptr = NULL;
	const char *msg_socket_con = "+++++SOCKET CONNECTED+++++\n";
	const char *msg_socket_fail = "+++++SOCKET DIAL FAILED+++++\n";
	const char *msg_connecting =  "+++++Connecting to server: ";
	const char *msg_disconnecting =  "+++++Closing socket+++++\n";
	memset(httpstruct->http_body, 0, MAXSIZE_HTTPBODY);
	memset(httpstruct->http_code, 0, MAXSIZE_HTTPCODE);
	memset(httpstruct->http_date, 0, MAXSIZE_HTTPDATE);
	memset(response, 0, AT_MAXSIZE);
	httpstruct->data_stat = DATA_EMPTY;
	
	#if PRINT_PROCESSES_IN_UART
		logWrite((uint8_t*)msg_connecting);
		logWrite(urlstruct_ptr->host);
		logWrite("\n");
	#endif
		
	/*socket dial response*/
	const CellStatusEnum cell_host_stats[] = {CELL_HOST_CONNECTED,
										  CELL_HOST_ERROR};
	const char *cell_host_resps[] = {"CONNECT","ERROR"};

	/*format socket dial string*/
	snprintf(socketdial, strlen(urlstruct_ptr->host) + RAWSIZE_SOCKETDIAL,
			 socketdial_frmt, active_cid, urlstruct_ptr->host);
	memset(webresponse, 0, MAXSIZE_WEBRESPONSE);

	/*connect to host*/
	if(uartTxCell((uint8_t *)socketdial, strlen(socketdial) ) == HAL_OK)
	{
		current_stat = serialReadUntilMultiple(response,
					   cell_host_resps, sizeof(cell_host_resps)/sizeof(const char*),
					   cell_host_stats, 30);
		if(current_stat == CELL_HOST_CONNECTED)
		{
			#if PRINT_PROCESSES_IN_UART
			logWrite((uint8_t*)msg_socket_con);
			#endif

			/*transmit data to host*/
			if(uartTxCell((uint8_t *)data, strlen(data) ) == HAL_OK)
			{
				/*read data from host*/
				data_stat = cellReadWebData(webresponse,
							   MAXSIZE_WEBRESPONSE, TIMEOUT_HOST_RESPONSE);
				/*data is complete*/
				if(data_stat == DATA_VALID)
					current_stat = CELL_WEBTRNSCT_SUCCESS;
				/*errors occured in between reception*/
				else
					current_stat = CELL_WEBTRNSCT_ERROR;
				/*remove "NO CARRIER" string*/
				no_carrier_ptr = strstr(webresponse, no_carrier);
				if(no_carrier_ptr)
					memset(no_carrier_ptr, 0, strlen(no_carrier));
				/*parse the web data*/
				httpParseWebResponse(webresponse, httpstruct);
				httpstruct->data_stat = data_stat;
				webresponse[MAXSIZE_WEBRESPONSE-1] = 0;
				#if PRINT_PROCESSES_IN_UART
					logWrite((uint8_t*)webresponse);
					logWrite((uint8_t*)"\n");
				#endif
				delayMillis(1000);
			}
			else
				current_stat = CELL_NO_RESPONSE;
			
			/*close socket connection*/
			snprintf(socketclose, 20, socketclose_frmt, active_cid);
			memset(response, 0, AT_MAXSIZE);
			if(uartTxCell((uint8_t *)socketclose, strlen(socketclose) ) !=
								HAL_OK)
			{
				current_stat = serialReadUntilMultiple(response,
						   cell_common_resps, sizeof(cell_common_resps)/sizeof(const char*),
						   cell_common_stats, 5);
				logWrite(response);
			}
			delayMillis(2000);
		}
		
	#if PRINT_PROCESSES_IN_UART
		else if (current_stat == CELL_HOST_ERROR)
		{
			logWrite(response);
			logWrite((uint8_t*)msg_socket_fail);
		}
		else
		{
			memset(response, 0, AT_MAXSIZE);
			sprintf(response, "+++++CELL_NO_RESPONSE+++++\n");
			logWrite(response);
		}
	#endif

	}
	else
		return CELL_NO_RESPONSE;

	free(socketdial);
	return current_stat;
}

/* ============================================================================
 *	@brief
 * ==========================================================================*/
CellStatusEnum cellPost(char *urlstr, const char *authorization, char *datastr,
						HTTPStruct *httpstruct)
{
	char *data = NULL;
	char webresponse[MAXSIZE_WEBRESPONSE];
	//char *webresponse = NULL;

	/*clear buffers*/
	memset(httpstruct->http_body, 0, MAXSIZE_HTTPBODY);
	memset(httpstruct->http_code, 0, MAXSIZE_HTTPCODE);
	memset(httpstruct->http_date, 0, MAXSIZE_HTTPDATE);
	httpstruct->data_stat = DATA_EMPTY;

	/*HTTP Header Fields*/
	char *header = NULL;
	char hdrfld_method[HEADER_FIELD_SIZE];
	char hdrfld_authorization[HEADER_FIELD_SIZE];
	char hdrfld_contentlength[HEADER_FIELD_SIZE];
	char hdrfld_host[HEADER_FIELD_SIZE];
	char *hdrfld_connection = "Connection: close\r\n";
	char *hdrfld_accept = "Accept: application/json\r\n";
	char *hdrfld_contenttype = "Content-Type: application/json\r\n";

	uint32_t data_length = strlen(datastr);
	URLStruct urlstruct;

	size_t header_size = 0;
	volatile CellStatusEnum current_stat = CELL_ERROR;
	current_stat = cellActData();

	if(current_stat == CELL_DATA_ON)
	{
		if (httpParseUrl(&urlstruct, urlstr))
		{
			//webresponse =  memAlloc(MAXSIZE_WEBRESPONSE);
			/*Set header field values*/
			snprintf(hdrfld_method, strlen(urlstruct.path) +
					 RAWSIZE_HDRFLD_METHOD, "POST %s HTTP/1.1\r\n",
					 urlstruct.path);
			snprintf(hdrfld_host,strlen(urlstruct.host) + RAWSIZE_HDRFLD_HOST,
					 "Host: %s\r\n", urlstruct.host);
			snprintf(hdrfld_authorization,strlen(authorization) +
					 RAWSIZE_HDRFLD_AUTH,
					 "Authorization: %s\r\n", authorization);
			snprintf(hdrfld_contentlength,MAXSIZE_HDRFLD_CONLEN,
					 "Content-Length: %lu\r\n", data_length);

			header_size = sizeof(char)*( strlen(hdrfld_method) +
					strlen(hdrfld_authorization) + strlen(hdrfld_contentlength)
					+ strlen(hdrfld_host) + strlen(hdrfld_connection) +
					strlen(hdrfld_accept) + strlen(hdrfld_contenttype) + 4);
			header = memAlloc(header_size);
			snprintf(header,header_size,"%s%s%s%s%s%s%s",hdrfld_method,
					 hdrfld_host, hdrfld_authorization, hdrfld_contentlength,
					 hdrfld_accept, hdrfld_contenttype,hdrfld_connection);

			data = memAlloc(sizeof(char)*(strlen(header) + strlen(datastr) + 7));
			sprintf(data,"%s\r\n%s\r\n\r\n",header,datastr);

			current_stat = cellTxRxWeb(webresponse, &urlstruct,
									   httpstruct, data);

			if (current_stat != CELL_WEBTRNSCT_SUCCESS)
			{
				/*log errors*/
			}

			free(urlstruct.host);
			free(urlstruct.path);
			free(header);
			free(data);
			//free(webresponse);
		}
	}
	else
		current_stat = CELL_AT_ERROR;
	return current_stat;
}

/*=============================================================================
 *
 * ==========================================================================*/
CellStatusEnum cellGet(char *urlstr, const char *authorization,
			   HTTPStruct *httpstruct)
{
	char *webresponse = NULL;
	memset(httpstruct->http_body, 0, MAXSIZE_HTTPBODY);
	memset(httpstruct->http_code, 0, MAXSIZE_HTTPCODE);
	memset(httpstruct->http_date, 0, MAXSIZE_HTTPDATE);
	httpstruct->data_stat = DATA_EMPTY;

	/*HTTP Header Fields*/
	char *header = NULL;
	char *hdrfld_method = NULL;
	char hdrfld_host[HEADER_FIELD_SIZE];
	char hdrfld_authorization[HEADER_FIELD_SIZE];
	char *get_method = "GET %s HTTP/1.1\r\n";
	const char *hdrfld_connection = "Connection: close\r\n";
	const char *hdrfld_accept = "Accept: application/json\r\n";
	size_t header_size = 0;

	URLStruct urlstruct;
	volatile CellStatusEnum current_stat = CELL_ERROR;

	current_stat = cellActData();
	if(current_stat == CELL_DATA_ON)
	{
		if (httpParseUrl(&urlstruct, urlstr) )
		{
			webresponse =  memAlloc(MAXSIZE_WEBRESPONSE);
			/*Set header field values*/
			hdrfld_method = memAlloc(sizeof(char)*(strlen(urlstruct.path) +
							strlen(get_method)) + 1);
			snprintf(hdrfld_method,HEADER_FIELD_SIZE,get_method,
					 urlstruct.path);
			snprintf(hdrfld_host,HEADER_FIELD_SIZE ,"Host: %s\r\n",
					 urlstruct.host);

			snprintf(hdrfld_authorization,HEADER_FIELD_SIZE,
					"Authorization: %s\r\n", authorization);

			header_size =  sizeof(char)*( strlen(hdrfld_method) +
					strlen(hdrfld_host) + strlen(hdrfld_accept) + strlen(hdrfld_authorization) +
					strlen(hdrfld_connection) + 4);
			header = memAlloc(header_size);

			snprintf(header,header_size,"%s%s%s%s%s\r\n\r\n",hdrfld_method,
					 hdrfld_host, hdrfld_authorization, hdrfld_accept,
					 hdrfld_connection);

			current_stat = cellTxRxWeb(webresponse, &urlstruct,
									   httpstruct, header);


			if (current_stat != CELL_WEBTRNSCT_SUCCESS)
			{

			}

			free(hdrfld_method);
			free(urlstruct.host);
			free(urlstruct.path);
			free(header);
			free(webresponse);
		}
	}

	return current_stat;
}

/*=============================================================================
 *
 * ==========================================================================*/
CellStatusEnum cellAccessToken(char *urlstr, char *security_data,
							   HTTPStruct *httpstruct)
{
	char *data = NULL;
	char *webresponse = NULL;
	memset(httpstruct->http_body, 0, MAXSIZE_HTTPBODY);
	memset(httpstruct->http_code, 0, MAXSIZE_HTTPCODE);
	memset(httpstruct->http_date, 0, MAXSIZE_HTTPDATE);

	/*HTTP Header Fields*/
	char *header = NULL;
	char *hdrfld_method = NULL;
	char hdrfld_contentlength[HEADER_FIELD_SIZE];
	char hdrfld_host[HEADER_FIELD_SIZE];
	char *hdrfld_connection = "Connection: close\r\n";
	char *hdrfld_accept = "Accept: application/json\r\n";
	char *hdrfld_contenttype =
					"Content-Type: application/x-www-form-urlencoded\r\n";
	char *post_method =  "POST %s HTTP/1.1\r\n";
	uint32_t data_length = strlen(security_data);
	URLStruct urlstruct;
	size_t header_size = 0;
	volatile CellStatusEnum current_stat = CELL_ERROR;

	current_stat = cellActData();
	if(current_stat == CELL_DATA_ON)
	{
		if (httpParseUrl(&urlstruct, urlstr) )
		{
			webresponse =  memAlloc(MAXSIZE_WEBRESPONSE);
			/*Set header field values*/
			hdrfld_method = memAlloc(sizeof(char)*(strlen(urlstruct.path) +
							strlen(post_method)) + 1);
			snprintf(hdrfld_method, strlen(urlstruct.path) +
					 RAWSIZE_HDRFLD_METHOD,
					post_method, urlstruct.path);
			snprintf(hdrfld_host,strlen(urlstruct.host) + RAWSIZE_HDRFLD_HOST,
					 "Host: %s\r\n", urlstruct.host);
			snprintf(hdrfld_contentlength,MAXSIZE_HDRFLD_CONLEN,
					 "Content-Length: %lu\r\n", data_length);
			header_size = sizeof(char)*( strlen(hdrfld_method) +
					strlen(hdrfld_contentlength) +
					strlen(hdrfld_host) + strlen(hdrfld_connection) +
					strlen(hdrfld_accept) + strlen(hdrfld_contenttype) + 4);
			header = memAlloc(header_size);
			snprintf(header,header_size,"%s%s%s%s%s%s",hdrfld_method,
					 hdrfld_host, hdrfld_contentlength, hdrfld_accept,
					 hdrfld_contenttype,hdrfld_connection);

			/*format the data to be transmitted to cloud*/
			data = memAlloc(sizeof(char)*(strlen(header) +
						  strlen(security_data) + 7));
			sprintf(data,"%s\r\n%s\r\n\r\n",header,security_data);

			/*transmit and receive data from cloud*/
			current_stat = cellTxRxWeb(webresponse, &urlstruct,
									   httpstruct, data);

			if (current_stat != CELL_WEBTRNSCT_SUCCESS)
			{

			}

			free(hdrfld_method);
			free(urlstruct.host);
			free(urlstruct.path);
			free(data);
			free(header);
			free(webresponse);
		}
	}
	return current_stat;
}

/*==============================================================================
 * @brief 
 * @param
 * @retval
 *=============================================================================*/
CellStatusEnum cellSetBaud(uint32_t baud)
{
	CellStatusEnum current_stat = CELL_AT_ERROR;
	char at_baud[20];
	char response[AT_MAXSIZE];
	memset(response, 0, AT_MAXSIZE);
	snprintf(at_baud,20,"AT+IPR=%lu\r\n",baud);
	if (uartTxCell((uint8_t *)at_baud, strlen(at_baud)) == HAL_OK)
	{
	current_stat = serialReadUntilMultiple(response, cell_common_resps,
			sizeof(cell_common_resps)/sizeof(const char*),
			cell_common_stats, 5);

	}
	return current_stat;
}

/*==============================================================================
 * @brief 
 * @param
 * @retval
 *=============================================================================*/
CellStatusEnum cellFTPTO()
{
    CellStatusEnum current_stat = CELL_AT_ERROR;
    char response[100];
    const char *ftp_to = "AT#FTPTO=2400\r\n";
    memset(response, 0, AT_MAXSIZE);
    if (uartTxCell((uint8_t *)ftp_to, strlen(ftp_to)) == HAL_OK)
    {
    current_stat = serialReadUntilMultiple(response, cell_common_resps,
                    sizeof(cell_common_resps)/sizeof(const char*),
                    cell_common_stats, 5);

    }
    return current_stat;
}

/*==============================================================================
 * @brief 
 * @param
 * @retval
 *=============================================================================*/
bool cellWriteSPI(uint8_t *buffer, uint32_t addr,uint32_t loop_count)
{
	for(uint32_t i = 0; i < loop_count; i++)
	{
		if(extFlWrite((int8_t*)buffer, addr, SPI_WRMAX) != SUCCESSFUL)
			assert(0);
		addr += SPI_WRMAX;
		buffer += SPI_WRMAX;
        delayMillis(1);
	}
    return true;
}

/*==============================================================================
 * @brief 
 * @param
 * @retval
 *=============================================================================*/
void cellReadSPI(uint8_t *buffer, uint32_t addr,uint32_t loop_count)
{
	for(uint32_t i = 0; i < loop_count; i++)
	{
		if(extFlRead((int8_t*)buffer, addr, SPI_WRMAX) != SUCCESSFUL)
			assert(0);
		buffer += SPI_WRMAX;
		addr += SPI_WRMAX;
        delayMillis(10);
	}
}

/*==============================================================================
 * @brief 
 * @param
 * @retval
 *=============================================================================*/
static CellStatusEnum cellFTPGet(uint32_t *fw_size)
{
    extFlSectorErase(5, 0);
    const char *at_ftpget_frmt = "AT#FTPGET=\"%s\"\r\n";
    const char *no_carrier = "\r\nNO CARRIER\r\n";
    char *cell_host_resps = "CONNECT\r\n";
    char *at_ftpget = NULL;
    char file[FTP_BUFFSIZE];// = NULL;
    char response[AT_MAXSIZE];
    uint8_t rx_char = 0;
    uint32_t ftpcmd_size = 0;
    uint32_t fw_next_free_address = NEW_FW_START_SEC_ADDR;
    uint32_t timeout_millis = (uint32_t)(300000);
    uint32_t idx_resp = 0;
    uint32_t t_start = HAL_GetTick();
    uint32_t t_prevcomp = HAL_GetTick();
    uint32_t t_current = HAL_GetTick();
    uint32_t buff_loop_count = 0;
    CellStatusEnum current_stat = CELL_DOWNLOAD_ER;

    ftpcmd_size = strlen(at_ftpget_frmt) + strlen(FTP_FILE);
    at_ftpget = memAlloc(ftpcmd_size);
    memset(at_ftpget, 0, ftpcmd_size);
    snprintf(at_ftpget, ftpcmd_size, at_ftpget_frmt, FTP_FILE);

    t_start = HAL_GetTick();
    t_prevcomp = HAL_GetTick();
    //file = memAlloc(FTP_BUFFSIZE);
    memset(file, 0, FTP_BUFFSIZE);

    /*send download command*/
    memset(response, 0, AT_MAXSIZE);
    if(uartTxCell((uint8_t *)at_ftpget, strlen(at_ftpget)) == HAL_OK)
    {
        current_stat = serialReadUntil(response, AT_MAXSIZE,
                      cell_host_resps, CELL_AT_OK, 60);
    }

    if(current_stat == CELL_AT_OK)
    {
        #if PRINT_PROCESSES_IN_UART
        gpioDigitalWrite(&CELL_RTS, HIGH);
        logWrite("Downloading File...\n\n");
        gpioDigitalWrite(&CELL_RTS, LOW);
        #endif

        t_current = HAL_GetTick();
        t_prevcomp = HAL_GetTick();
        while (delayTimeDiff(t_start, t_current) < timeout_millis)
        {
            if (uartRxCell(&rx_char, 1) == HAL_OK)
            {
                file[idx_resp++] = rx_char;
            }
			else
			{
				HAL_UART_DeInit(&UART1HandlerDef);
				HAL_UART_Init(&UART1HandlerDef);
			}

            t_current = HAL_GetTick();
            if(idx_resp == FTP_BUFFSIZE ||
                  delayTimeDiff(t_prevcomp, t_current) > 5000)
            {
                gpioDigitalWrite(&CELL_RTS, HIGH);
                /*save to flash*/
                if(idx_resp == FTP_BUFFSIZE)
                {
                    idx_resp = 0;
                    ++buff_loop_count;
                    if (cellWriteSPI((uint8_t*)file, fw_next_free_address, 20) != true)
                    {
                        delayMillis(1000);
                         break;
                    }
                    //cellReadSPI((uint8_t*)file_test, fw_next_free_address, 20);
                    
                    fw_next_free_address += FTP_BUFFSIZE;
                    #if PRINT_PROCESSES_IN_UART
                    logWrite("%lu\n", buff_loop_count);
                    //uartSHOW((uint8_t*)file_test, FTP_BUFFSIZE);
                    #endif
                }

                if(strstr(file, no_carrier))
                {
                      break;
                }

                t_prevcomp = HAL_GetTick();
                gpioDigitalWrite(&CELL_RTS, LOW);
            }
        }

        if(strstr(file, no_carrier))
        {
            /*save to flash*/
            idx_resp = idx_resp - strlen(no_carrier) - 1;
            cellWriteSPI((uint8_t*)file, fw_next_free_address, 20);
            *fw_size =  buff_loop_count*FTP_BUFFSIZE + idx_resp + 1;
            //uartSHOW((uint8_t*)file, idx_resp);
            uartSHOW((uint8_t*)file, idx_resp + 1);
            memset(file, 0, FTP_BUFFSIZE);
            current_stat = CELL_DOWNLOAD_OK;
        }
        else
        {
            #if PRINT_PROCESSES_IN_UART
            logWrite("FTP TIMEOUT...\n\n");
            #endif
        }
    }

    else
    {
        #if PRINT_PROCESSES_IN_UART
        logWrite("Failed to Start Download...\n\n");
        #endif
    }

    //free(file);
    free(at_ftpget);
    return current_stat;
}

/*==============================================================================
 * @brief 
 * @param
 * @retval
 *=============================================================================*/
static CellStatusEnum cellFTPDownload(uint32_t *fw_size)
{
	const char *at_ftptype = "AT#FTPTYPE=0\r\n";
	const char *at_ftpopen_frmt = "AT#FTPOPEN=\"%s\",\"%s\",\"%s\",0\r\n";
	const char *at_ftpclose = "AT#FTPCLOSE\r\n";
	char *at_ftpopen = NULL;
	char response[AT_MAXSIZE];
	uint32_t ftpcmd_size = 0;
 	CellStatusEnum current_stat = CELL_AT_ERROR;
	CellStatusEnum ftp_stat[] = {CELL_FTPSERVER_OK, CELL_FTPSERVER_ER};

	/*clear spi flash*/

	/*allow data rx*/
	gpioDigitalWrite(&CELL_RTS, LOW);

	/*activate cell*/
	current_stat = cellActData();
	if(current_stat != CELL_DATA_ON)
	{
		return current_stat;
	}
	delayMillis(100);

	#if PRINT_PROCESSES_IN_UART
		logWrite("DATA ACTIVE\n\n");
	#endif

	/*open file transfer port*/
	ftpcmd_size = strlen(at_ftpopen_frmt) + strlen(FTP_ADD) +
			strlen(FTP_USER) + strlen(FTP_PASS);
	at_ftpopen = memAlloc(ftpcmd_size);
	memset(at_ftpopen, 0, ftpcmd_size);
	snprintf(at_ftpopen, ftpcmd_size, at_ftpopen_frmt, FTP_ADD, FTP_USER,
			FTP_PASS);
	memset(response, 0, AT_MAXSIZE);
	if (uartTxCell((uint8_t *)at_ftpopen, strlen(at_ftpopen)) == HAL_OK)
	{
		current_stat = serialReadUntilMultiple(response, cell_common_resps,
					   sizeof(cell_common_resps)/sizeof(const char*),
					   ftp_stat, 60);
	}
	free(at_ftpopen);

	if(current_stat != CELL_FTPSERVER_OK)
	{
		#if PRINT_PROCESSES_IN_UART
			if(current_stat == CELL_FTPSERVER_ER)
				logWrite("Failed to Connect to Server\n\n");
			else
				logWrite("Connection Time Out\n\n");
		#endif
		return current_stat;
	}
	delayMillis(500);

	#if PRINT_PROCESSES_IN_UART
		logWrite("Connected to Server\n\n");
	#endif

	/*select ftp type*/
	memset(response, 0, AT_MAXSIZE);
	if (uartTxCell((uint8_t *)at_ftptype, strlen(at_ftptype)) == HAL_OK)
	{
		current_stat = serialReadUntilMultiple(response, cell_common_resps,
				sizeof(cell_common_resps)/sizeof(const char*),
				cell_common_stats, 20);

	}
	delayMillis(500);

	if(current_stat == CELL_AT_OK)
	{
		#if PRINT_PROCESSES_IN_UART
			logWrite("Starting Download...\n\n");
		#endif
		/*get file from server*/
		current_stat = cellFTPGet(fw_size);
	}
    
	else
	{
		#if PRINT_PROCESSES_IN_UART
			logWrite("Failed to set FTP Type...\n\n");
		#endif
	}

	/*close ftp connection*/
	memset(response, 0, AT_MAXSIZE);
	if (uartTxCell((uint8_t *)at_ftpclose, strlen(at_ftpclose)) == HAL_OK)
	{
		if(serialReadUntilMultiple(response, cell_common_resps,
				sizeof(cell_common_resps)/sizeof(const char*),
				cell_common_stats, 5) != CELL_AT_OK)
			cellReset();
	}

	return current_stat;
}


/*==============================================================================
 * @brief 
 * @param
 * @retval
 *=============================================================================*/
static inline bool isEndofFile(uint8_t buff[], uint32_t size)
{
	const char * end_of_file = ":00000001FF\r\n";
	if(size != strlen(end_of_file))
		return false;
	for(uint32_t i = 0; i < strlen(end_of_file); i++)
	{
	  if ((buff[i] != end_of_file[i]))
		 return false;
	}
   return true;
}

/*==============================================================================
 * @brief 
 * @param
 * @retval
 *=============================================================================*/
FWStatEnum fileCheck(uint32_t fw_size)
{
	uint8_t file_buff[FTP_BUFFSIZE];
	//uint8_t *file_buff = agMalloc(FTP_BUFFSIZE);
	uint8_t starf_of_line = 0x3A;
	uint8_t end_of_line = 0x0A;
	uint8_t carriage_return = 0x0D;
	bool is_sol_found = false;
	bool is_msb = true;
	uint16_t check_sum_rx = 0;
	uint32_t check_sum_calc = 0;
	uint32_t last_checked_index = 0;
	volatile uint32_t next_address_to_read = NEW_FW_START_SEC_ADDR;

	for(uint32_t loopcount = 0; loopcount < FW_MAXLOOP; loopcount++)
	{
		/*load fw bytes to buffer*/
        memset(file_buff, 0, FTP_BUFFSIZE);
		cellReadSPI((uint8_t*)file_buff, next_address_to_read, 20);

		/*check if start of line is correct*/
		if(file_buff[0] != starf_of_line)
			return FWSTAT_ER_SOL;//assert(0);
			//return false;

		check_sum_calc = 0;
		last_checked_index = 0;
		is_msb = true;

		for(uint32_t i = 0; i < FTP_BUFFSIZE; i++)
		{
			if(file_buff[i] == starf_of_line)
				is_sol_found = true;
			else if(file_buff[i] == end_of_line)
			{
				if(!is_sol_found)
					return FWSTAT_ER_SOL; //assert(0);
				if(file_buff[i - 1] == carriage_return)
				{
					check_sum_rx =
						((uint16_t)strtol((char[]){file_buff[i - 2], 0}, NULL, 16) |
						((uint16_t)strtol((char[]){file_buff[i - 3], 0}, NULL, 16)<<4));
					/*subtract the checksum byte from total checksum*/
                    check_sum_calc =
							(~(check_sum_calc - check_sum_rx) + 1) & 0xFF;

					if(check_sum_calc != check_sum_rx)
						return FWSTAT_ER_CHKSUM;//assert(0);

					check_sum_calc = 0;
					is_sol_found = false;
					if(isEndofFile(file_buff + last_checked_index + 1,
						i - last_checked_index))
					{
						delayMillis(10);
						uartSHOW((uint8_t*)file_buff, i);
						if(fw_size == (i + 1 + next_address_to_read - NEW_FW_START_SEC_ADDR))
							return FWSTAT_OK;
                        else
                            return FWSTAT_ER_SIZE;
					}
					last_checked_index = i;
				}
				else
					return FWSTAT_ER_CR;//assert(0);

			}
			else if(isxdigit(file_buff[i]))
			{
				check_sum_calc += (is_msb == true ?
					strtol((char[]){file_buff[i], 0}, NULL, 16)<<4 :
					strtol((char[]){file_buff[i], 0}, NULL, 16));
				is_msb = !is_msb;
			}
			else
			{
				if(file_buff[i] != carriage_return)
					return FWSTAT_ER_CR;//assert(0);
			}
		}
		/*save last check address*/
		//uartSHOW((uint8_t*)file_buff, last_checked_index + 1);
		next_address_to_read += (last_checked_index + 1);
	}
	return FWSTAT_ERROR;
}

/*==============================================================================
 * @brief 
 * @param
 * @retval
 *=============================================================================*/
uint32_t cellDownloadFW(void)
{
    //const char *test = ":10A390000000000001020304060708090024F4007D\r\n:10A3A0000008024000000000000802400100000018\r\n:10A3B0000008024002000000000802400300000004";
    //char xxx[200];
	uint32_t fw_size = 0;
	uint32_t fw_size_checked = 0;
    if(extFlSectorErase(4, NEW_FW_START_SEC_ADDR) != SUCCESSFUL)
		assert(0);
    //fileCheck(fw_size);
	cellON_OFF(true);
	for(uint8_t i = 0; i < 10; i++)
	{
		if(cellFTPDownload(&fw_size) == CELL_DOWNLOAD_OK)
		{
			if(fileCheck(fw_size) == FWSTAT_OK)
			{
			  	fw_size_checked = fw_size;
			  	logWrite("File Check OK\n\n");
				break;
			}
		}

		delayMillis(5000);
	}
	cellON_OFF(false);

	return fw_size_checked;
}
#if DEVICE_TYPE == DEVICETYPE_GATE
#pragma optimize=none
CellStatusEnum cellUpdateCheck(void)
{
  	HTTPStruct httpstruct;
	FWVersionStruct fwver;
	uint32_t fw_size = 0;
	const char *httpkey_fwver =  "\"firmware_version\":";
  	const char *fw_path = "api/v0.1/firmware_update/%s";
	char *fw_path_specific = NULL;
	char *http_fw_start_addr = NULL;
	uint32_t path_size = strlen(fw_path) + strlen(DEVICE_INFO_LIST[0].device_sn + 1);
	fw_path_specific = memAlloc(path_size);
	CellStatusEnum retval = CELL_AT_ERROR;
	snprintf(fw_path_specific, path_size, fw_path, DEVICE_INFO_LIST[0].device_sn);
	//cellGet(fw_path_specific, AUTHENTICATION, &httpstruct);
	
	/*TESTING ONLY*/
	const char *test_body = "\"firmware_version\": \"3.1.2\",\"firmware_path\": \"http://dev-core.agtechindustries.com/media/firmware/firmwareV1.0__77ngHmD.txt\"";
	httpstruct.data_stat = DATA_VALID;
	memcpy(httpstruct.http_body, test_body, strlen(test_body));
	/*TESTING ONLY*/
	
	if(httpstruct.data_stat == DATA_VALID)
	{
		http_fw_start_addr = strstr(httpstruct.http_body, httpkey_fwver);
		if(http_fw_start_addr)
		{
			http_fw_start_addr += strlen(httpkey_fwver) + 2;	
			fwver.fwver_main = strtoul(http_fw_start_addr, NULL, 10);
			
			http_fw_start_addr = strstr(http_fw_start_addr, ".");
			http_fw_start_addr++;
			fwver.fwver_submain = strtoul(http_fw_start_addr, NULL, 10);
			
			http_fw_start_addr = strstr(http_fw_start_addr, ".");
			http_fw_start_addr++;
			fwver.fwver_minor = strtoul(http_fw_start_addr, NULL, 10);
			
			
			if(fwver.fwver_main != FW_VERSION.fwver_main
				|| fwver.fwver_minor != FW_VERSION.fwver_minor
				|| fwver.fwver_submain !=  FW_VERSION.fwver_submain)
			{
			  	fw_size = cellDownloadFW();
				if(fw_size)
					retval = CELL_FWUPDATE_READY;
			}
		}
	}
	return retval;
}
#endif

