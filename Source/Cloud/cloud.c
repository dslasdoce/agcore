/********************************************************************************
 * @file      	:	cloud.c
 * @author		:	Hardware Team
 * @version		:	v2.2.0
 * @date		:	04/08/16
 * @brief		:	Module of Post/Get to cloud server
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
#include <string.h>

#include "productCfg.h"
#include "agBoardInit.h"
#include "cloud.h"
#include "api.h"
#include "utilMem.h"
#include "cell.h"
#include "wifi.h"
#include "tempAcctDev.h"
#include "tempApiCodedList.h"
#include "sdMain.h"

#define HTTPCODE_GETOK			((const char*)"200")
#define HTTPCODE_POSTOK			((const char*)"201")
bool cloud_post_try = false;

/* Functions ================================================================*/
/******************************************************************************
 * @brief	data POST procedure to api server
 * @param	apitype: api type (ex: telemetry, device-registrations, etc)
 * @retval  status
 ******************************************************************************/
CloudStat cloudPost(uint8_t apitype)
{
	const uint8_t POST_TRY_CNT = 5;
	char *apidata_ptr;
	char *url;
	HTTPStruct httpstruct;
	uint32_t posted_rec_cnt;
	temp_posted_rec_cnt = 0;
	apidata_ptr = NULL;
	CLEAR(httpstruct);
	CellStatusEnum cell_stat = CELL_NO_RESPONSE;
	cloud_post_try = false;
	char printout[50];
	
	/*conversion of data into JSON format*/
	if(apiToJson(apitype, &apidata_ptr) == APISTAT_FAILED)
	{
		#if PRINT_PROCESSES_IN_UART
			const char *print_no_data = "\n###NO DATA TO POST###\n";
			uartSHOW((uint8_t*)print_no_data, strlen(print_no_data));
			#endif
		free(apidata_ptr);
		return CLOUDSTAT_FAILED;
	}

	/*posting of data*/
	url = memAlloc(strlen(URLHOST) + strlen(URLPOSTPATH[apitype]));
	sprintf(url, "%s%s", URLHOST, URLPOSTPATH[apitype]);
	//#ifdef CELL
	uartSHOW(apidata_ptr, strlen(apidata_ptr));
	uartSHOW("\n\n\n", 3);
	uartSHOW((uint8_t*)url, strlen(url));
	uartSHOW("\n\n", 2);
	
	#if ENABLE_CELLPOST
		for(uint8_t i = 0; i < POST_TRY_CNT; i++ )
		{
			#if PRINT_PROCESSES_IN_UART
			snprintf(printout, 50, "###POST ATTEMPT %d###\n", i+1);
			uartSHOW((uint8_t*)printout, strlen(printout));
			#endif
			cell_stat = cellPost(url, AUTHENTICATION, (char *)apidata_ptr, &httpstruct);
			if(strcmp(httpstruct.http_code,HTTPCODE_POSTOK) == 0)
				break;
			else
			{	
				if(cell_stat == CELL_AT_ERROR)
				{
					cellForcedShutDown();
					//cellON_OFF(true);
				}
				else
					delayMillis(2000);
			}
		}
	#endif
		
	free(url);
	free(apidata_ptr);

	/*POST results from cloud server*/
	if((httpstruct.data_stat != DATA_VALID)
			|| (strcmp(httpstruct.http_code, HTTPCODE_POSTOK) != 0))
	{
		return CLOUDSTAT_FAILED;
	}

	if(apitype == API_SENSOR_REG
			|| apitype == API_VALVE_REG)
	{
		return CLOUDSTAT_OK;
	}

	posted_rec_cnt = strtol(httpstruct.http_body, NULL, 10);
	loggerSaveRec((uint32_t*)NULL, temp_posted_rec_cnt, apitype);
	cloud_post_try = true;
	if(posted_rec_cnt <= 0)
	{
		return CLOUDSTAT_FAILED;
	}

	/* Save Posted Records to FL*/	
	/* to be added by mimlorica */
	
//
//	switch(apitype)
//	{
//	case API_TELEMETRY:
//		telemResetSramRec(posted_rec_cnt);
//		break;
//	case API_DEV_REG:
//		sysStatResetSramRec(posted_rec_cnt);
//		break;
//	case API_VALVE_CMD:
//		break;
//	default:
//		return CLOUDSTAT_FAILED;
//	}
	return CLOUDSTAT_OK;
}

/******************************************************************************
 * @brief	data GET procedure from api server
 * @param	httpstruct: response from api server
 * 			urlfilter: filtering parameters on getting data from api server
 * @retval  status
 ******************************************************************************/
CloudStat cloudVlvSchedGet(HTTPStruct *httpstruct, char *urlfilter)
{
	char * url;
	uint32_t urlsize;
	HTTPStruct tempstruct;

	memset(httpstruct, 0, sizeof(HTTPStruct));

	urlsize = strlen(URLHOST) + strlen(URLGETPATH[VALVE_SCHED])
			+ strlen(urlfilter) + 1;
	url = memAlloc(urlsize);
	snprintf(url, urlsize, "%s%s%s", URLHOST, URLGETPATH[VALVE_SCHED], urlfilter);

	wifiGet(url, AUTHENTICATION, &tempstruct);
	free(url);
	if(!strcmp(tempstruct.http_code,HTTPCODE_GETOK) && tempstruct.data_stat == DATA_VALID)
	{
		memcpy(httpstruct, &tempstruct, sizeof(HTTPStruct));
		return CLOUDSTAT_OK;
	}
	else
		return CLOUDSTAT_FAILED;
}
#if DEVICE_TYPE == DEVICETYPE_GATE
#pragma optimize=none
/******************************************************************************
 * @brief	data GET procedure from api server
 * @param	httpstruct: response from api server
 * 			urlfilter: filtering parameters on getting data from api server
 * @retval  status
 ******************************************************************************/
CloudStat cloudGateInfoGet(char *gate_sn)
{
	ReturnStatusEnum sd_stat;
  	CloudStat retval = CLOUDSTAT_FAILED;
	CellStatusEnum cell_stat = CELL_NO_RESPONSE;
	HTTPStruct httpstruct;
	char *url = NULL;
	const uint8_t GET_TRY_CNT = 5;
	memset(&httpstruct, 0, sizeof(HTTPStruct));
	char printout[100];
	uint8_t i = 0;
	
	led_pattern = LED_RUNMODE_ACCNTID_SET;
	buzzEnable(5);
	cellReset();
	
	url = memAlloc(strlen(URLHOST) + strlen(URLGETPATH[API_GATEINFO]) + strlen(gate_sn)  + 2);
	sprintf(url, "%s%s%s\/", URLHOST, URLGETPATH[API_GATEINFO], gate_sn);

	#if PRINT_PROCESSES_IN_UART
		snprintf(printout, 100, "###FETCHING GATE INFO: %s###\n", gate_sn);
		uartSHOW((uint8_t*)printout, strlen(printout));
		
		snprintf(printout, 100, "###URL: %s###\n", url);
		uartSHOW((uint8_t*)printout, strlen(printout));
	#endif
		
	while(1)
	{
		#if PRINT_PROCESSES_IN_UART
			snprintf(printout, 100, "###FETCH ATTEMPT %d###\n", i+1);
			uartSHOW((uint8_t*)printout, strlen(printout));
		#endif
		cell_stat = cellGet(url, AUTHENTICATION, &httpstruct);
		if(strcmp(httpstruct.http_code,HTTPCODE_GETOK) == 0 
		   && httpstruct.data_stat == DATA_VALID)
		{
		  	retval = CLOUDSTAT_OK;
			cloudParseGateInfo(httpstruct.http_body);			
			break;
		}
		else
		{	
			if(cell_stat == CELL_AT_ERROR)
			{
				cellForcedShutDown();
			}
			else
				delayMillis(2000);
		}
		i++;
	}
	
	
	#if PRINT_PROCESSES_IN_UART
		snprintf(printout, 100, "###GATE INFO DONE\nResetting System...", gate_sn);
		uartSHOW((uint8_t*)printout, strlen(printout));
	#endif
	sd_stat = sdCfgGetAcctDev(); 
	if(sd_stat)
		sd_stat = sdGetNodeSNList();
	
	if(!sd_stat)
		assert(0);
	
	free(url);
	
	logWrite("DEVICE_SN: %s\n",DEVICE_INFO->device_sn);
	logWrite("\nACCOUNT_ID = %d\n", ACCOUNT_ID);
	
	logWrite("NODE_COUNT = %d\n", NODE_COUNT);
	
	for(uint8_t i = 1; i <= NODE_COUNT; i++)
	{
		logWrite("%s\n", DEVICE_INFO_LIST[i].device_sn);
	}

	return retval;
}

#pragma optimize=none
void cloudParseGateInfo(char *gate_info_str)
{
  	const char *respkey_accountid = "{\"account_id\":";
	const char *respkey_nodes = "\"nodes\":[";
	const char *respkey_closebracket = "]}";
	uint32_t account_id = 0;
	char *str_buff = NULL;
	char *str_buff_end = NULL;
	char *str_stripped_copy = NULL;
	uint32_t str_stripped_index = 0;
	char *test_char = NULL;
	char *parsed_strings[MAXNODE];
	uint32_t temp_nodecount = 0;
	for(uint32_t i = 0; i< MAXNODE; i++)
	{
		parsed_strings[i] = NULL;
	}
	
	/*Get Account ID*/
	str_buff = strstr(gate_info_str, respkey_accountid);
	str_buff += strlen(respkey_accountid);
	account_id = strtoul(str_buff, NULL, 10);
	
	/*Get Node List*/
	/*Search for node keyword*/
	str_buff = strstr(gate_info_str, respkey_nodes);
	str_buff += strlen(respkey_nodes);
	
	/*search for end of data chars*/
	str_buff_end = strstr(str_buff, respkey_closebracket);
	
	/*Check if there is a node present*/
	if(str_buff == str_buff_end)
	{
		sdUpdateDeviceNodeList(NULL, 0);
		sdUpdateAcctID(account_id);
	}
	else
	{
	  	/*Parse Nodes*/
	  	memset(str_buff_end, 0, strlen(respkey_closebracket));
		str_stripped_copy = (char*)memAlloc(strlen(str_buff));
		memset(str_stripped_copy, 0, strlen(str_buff));
		for(uint32_t i = 0; i < strlen(str_buff); i++)
		{
		  	if( isalnum(str_buff[i]) || str_buff[i] == '-' 
			   || str_buff[i] == '_' || str_buff[i] == ',')
			{
				str_stripped_copy[str_stripped_index] = str_buff[i];
				str_stripped_index++;
			}
		}
		
		temp_nodecount = eqStringSplit(parsed_strings, str_stripped_copy,",");
		if(str_stripped_copy)
			free(str_stripped_copy);
		
		sdUpdateDeviceNodeList(parsed_strings, temp_nodecount);
		sdUpdateAcctID(account_id);
		
		for(uint32_t i = 0; i< MAXNODE; i++)
		{
			if(parsed_strings[i])
				free(parsed_strings[i]);
		}
	}
}
#endif
/******************************************************************************
 * Revision History
 *	@file      	cloud.c
 ******************************************************************************
 * @version	:	v2.2.0
 * @date	:	04/08/16
 * @author	:	J. Fajardo
 * @changes	:	Initial release for Beta version 2.2
 ******************************************************************************
 */
