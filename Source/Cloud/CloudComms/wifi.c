/*******************************************************************************
  * @file      	wifi.c
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

#include "agBoardInit.h"
#include "utilMem.h"
#include "http.h"
#include "delay.h"
#include "wifi.h"

/* Defines ==================================================================*/
#define AT_MAXSIZE2				((uint32_t)100)
#define AT_TIMEOUT_COMMON 		((uint32_t)2)
#define TIMEOUT_WIFIPOST 		((uint32_t)20)

/* Functions ================================================================*/

/*==============================================================================
 * @brief 
 * @param
 * @retval
 *=============================================================================*/
static inline void wifiSendCRLF(void)
{
	uartTxWifi((uint8_t*)"\r\n\r\n", 4);
}

/*==============================================================================
 * @brief 
 * @param
 * @retval
 *=============================================================================*/
void wifiHReset()
{
	gpioDigitalWrite(&WIFI_NRESET, LOW);
	delayMillis(100);
	gpioDigitalWrite(&WIFI_NRESET, HIGH);
	delayMillis(2000);
	wifiSendCRLF();
	delayMillis(5000);
}
/*==============================================================================
 * @brief 
 * @param
 * @retval
 *=============================================================================*/
static inline bool matchStringFromArrayz(char *refstr_ptr, const char **patternstr_arr_ptr,
										uint8_t pattern_count, uint8_t *idx_found_ptr)
{
	uint32_t idx_pattern = 0;
	for(idx_pattern=0; idx_pattern<pattern_count; idx_pattern++)
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
static WifiStatusEnum serialReadUntilMultiple(char *refstr_ptr, const char **patternstr_arr_ptr,
											  uint8_t pattern_count,const WifiStatusEnum *statmap_ptr,
											  uint32_t timeout)
{
	uint32_t t_start = HAL_GetTick();
	uint32_t timeout_millis = (uint32_t)(timeout*1000);
	uint8_t rx_char = 0;
	uint32_t idx_resp = 0;
	uint8_t idx_pattern_found = 0;
	uint32_t t_prevcomp = HAL_GetTick();
	uint32_t t_current = HAL_GetTick();
	while (delayTimeDiff(t_start, HAL_GetTick()) < timeout_millis)
	{
		//uartRxWifi(&rx_char, sizeof(rx_char));
		if(idx_resp >= AT_MAXSIZE2)
		{
			return WIFI_BUFFEROVERFLOW;
		}
		if (uartRxWifi(&rx_char, sizeof(rx_char)) == HAL_OK)
		{
			refstr_ptr[idx_resp++] = rx_char;
		}

		if(delayTimeDiff(t_prevcomp, t_current) > INTERVAL_MULTIPLE_SEARCH)
		{
			t_prevcomp = HAL_GetTick();
			if(matchStringFromArray(refstr_ptr, patternstr_arr_ptr, pattern_count,
									&idx_pattern_found))
				return statmap_ptr[idx_pattern_found];

		}
		t_current = HAL_GetTick();
	}
	if(matchStringFromArray(refstr_ptr, patternstr_arr_ptr, pattern_count, &idx_pattern_found))
				return statmap_ptr[idx_pattern_found];
	return WIFI_NO_RESPONSE;
}

/*==============================================================================
 * @brief 
 * @param
 * @retval
 *=============================================================================*/
static DataStatusEnum wifiReadWebData(char *resp, uint32_t resp_maxsize,
									  uint32_t timeout)
{
	DataStatusEnum data_stat = DATA_PENDING;
	char *httpkey_content_length = "Content-Length: ";
	char *httpkey_conlen_addr = NULL;
	char *httpkey_conlen_endaddr = NULL;
	//char *httpkey_content_length_endaddr = NULL;
	char *data_start_ptr = NULL;
	volatile uint32_t content_length = 0;
	uint32_t t_start = HAL_GetTick();
	uint32_t timeout_millis = (uint32_t)(timeout*1000);
	uint8_t rx_char = 0;
	uint32_t idx_resp = 0;
	uint32_t t_prevcomp = HAL_GetTick();
	uint32_t t_current = HAL_GetTick();

	while (delayTimeDiff(t_start, t_current) < timeout_millis)
	{
		if(idx_resp >= resp_maxsize)
		{
			resp[idx_resp - 1] = 0;
			return DATA_BUFFEROVERFLOW;
		}
		if (uartRxWifi(&rx_char, sizeof(rx_char)) == HAL_OK)
		{
			resp[idx_resp++] = rx_char;
		}

		if(delayTimeDiff(t_prevcomp, t_current) > 500)
		{
			t_prevcomp = HAL_GetTick();
			if(data_stat == DATA_PENDING)
			{
				/*look for Content-Length string*/
				httpkey_conlen_addr = strstr(resp, httpkey_content_length);
				if(httpkey_conlen_addr)
					/*check if content lenght is complete*/
					httpkey_conlen_endaddr = strstr(httpkey_conlen_addr, "\r\n");
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
			if(data_stat == DATA_STARTCH_READY && content_length == idx_resp)
			{
				delayMillis(10);
				return DATA_VALID;
			}
			/*
			if(strstr(resp,end_str))
			{
				return success_response;
			}
			*/
		}
		t_current = HAL_GetTick();
	}

	if(content_length == idx_resp)
	{
		return DATA_VALID;
	}

	return DATA_INVALID;
}

/*==============================================================================
 * @brief 
 * @param
 * @retval
 *=============================================================================*/
static WifiStatusEnum serialReadUntil(char *resp, uint32_t resp_maxsize ,char *end_str,
									  uint32_t timeout, WifiStatusEnum success_response)
{
	uint32_t t_start = HAL_GetTick();
	uint32_t timeout_millis = (uint32_t)(timeout*1000);
	uint8_t rx_char = 0;
	uint32_t idx_resp = 0;
	uint32_t t_prevcomp = HAL_GetTick();
	uint32_t t_current = HAL_GetTick();
	while (delayTimeDiff(t_start, t_current) < timeout_millis)
	{

		if(idx_resp >= resp_maxsize)
		{
			return WIFI_BUFFEROVERFLOW;
		}
		if (uartRxWifi(&rx_char, sizeof(rx_char)) == HAL_OK)
		{
			resp[idx_resp++] = rx_char;
		}

		if(delayTimeDiff(t_prevcomp, t_current) > 1000)
		{
			t_prevcomp = HAL_GetTick();
			if(strstr(resp,end_str))
			{
				return success_response;
			}
		}
		t_current = HAL_GetTick();
	}

	if(strstr(resp,end_str))
	{
		return success_response;
	}
	return WIFI_ERROR;
}

/*==============================================================================
 * @brief 
 * @param
 * @retval
 *=============================================================================*/
static WifiStatusEnum wifiThroughMode(void)
{
	const char *at_throughmode = "AT+ENTM\r\n";
	char response[AT_MAXSIZE2];
	const char *at_resps[] = {"+ok","ERR"};
	WifiStatusEnum at_stats[] = {WIFI_AT_OK,WIFI_ERROR};
	volatile WifiStatusEnum current_stat = WIFI_ERROR;

	memset(response,0,AT_MAXSIZE2);
	if(uartTxWifi( (uint8_t *)at_throughmode, strlen(at_throughmode)) == HAL_OK)
	{
		current_stat = serialReadUntilMultiple(response, at_resps,
											   sizeof(at_resps)/sizeof(const char*), at_stats, 5);
	}
	delayMillis(500);
	return current_stat;
}

/*==============================================================================
 * @brief 
 * @param
 * @retval
 *=============================================================================*/
WifiStatusEnum wifiCommandMode(void)
{
	const char *at_cmdmode = "+++";
	const char *at_resptocmd = "a";
	const char *at_end = "\r\n";
	char response[AT_MAXSIZE2];
	const char *at_resps[] = {"+ok","ERR"};
	WifiStatusEnum at_stats[] = {WIFI_AT_OK,WIFI_ERROR};
	volatile WifiStatusEnum current_stat = WIFI_NO_RESPONSE;

	memset(response,0,AT_MAXSIZE2);
	if(uartTxWifi( (uint8_t *)at_cmdmode, strlen(at_cmdmode)) == HAL_OK)
	{
		current_stat = serialReadUntil(response, AT_MAXSIZE2,"a", AT_TIMEOUT_COMMON, WIFI_AT_OK);
		delayMillis(100);
		if(current_stat == WIFI_AT_OK)
		{
			memset(response,0,AT_MAXSIZE2);
			if(uartTxWifi( (uint8_t *)at_resptocmd, strlen(at_resptocmd)) == HAL_OK)
			{
				current_stat = serialReadUntilMultiple(response, at_resps,
													   sizeof(at_resps)/sizeof(const char*),
													   at_stats, AT_TIMEOUT_COMMON);
			}
			else
				current_stat = WIFI_ERROR;
		}
		else
		{
			if(uartTxWifi( (uint8_t *)at_end, strlen(at_end)) == HAL_OK)
			{
				current_stat = WIFI_AT_OK;
			}
			else
				current_stat = WIFI_ERROR;
		}
	}
	delayMillis(1000);
	return current_stat;
}

/*==============================================================================
 * @brief 
 * @param
 * @retval
 *=============================================================================*/
WifiStatusEnum wifiReset(void)
{
	const char *at_reset = "AT+Z\r";
	//const char *at_echo = "AT+E=off\r";
	//char response[AT_MAXSIZE2];
	//volatile WifiStatusEnum current_stat = WIFI_AT_OK;

	if (wifiCommandMode() != WIFI_AT_OK)
	{
		/*logs here*/
	}
	if(uartTxWifi( (uint8_t *)at_reset, strlen(at_reset)) != HAL_OK)
	{
		return WIFI_ERROR;
	}
	delayMillis(5000);
	/*
	if(uartTxWifi( (uint8_t *)at_echo, strlen(at_echo)) == HAL_OK)
	{
		current_stat = serialReadUntil(response, "+ok", AT_TIMEOUT_COMMON, WIFI_AT_OK);
	}
	else
		return WIFI_ERROR;
	*/

	return WIFI_AT_OK;
}

/*==============================================================================
 * @brief 
 * @param
 * @retval
 *=============================================================================*/
WifiStatusEnum wifiSleep(void)
{
	volatile WifiStatusEnum current_stat;
	char response[AT_MAXSIZE2];
	const char *at_sleep = "AT+MSLP=standby\r\n\r\n";
	wifiReset();
	wifiCommandMode();
	uartTxWifi((uint8_t*)at_sleep,strlen(at_sleep));
	current_stat = serialReadUntil(response, AT_MAXSIZE2 ,"+ok",
										   AT_TIMEOUT_COMMON, WIFI_AT_OK);
	/*gpioDigitalWrite(&WIFI_SLEEP, LOW);
	delayMillis(4000);
	gpioDigitalWrite(&WIFI_SLEEP, HIGH);*/
	return current_stat;
}

/*==============================================================================
 * @brief 
 * @param
 * @retval
 *=============================================================================*/
WifiStatusEnum wifiConfig(void)
{
	//wifiReset();
	const char *at_netp = "AT+NETP=tcp,client,80,beta-api.agtechind.com\r\n";
	const char *at_ssid = "AT+WSSSID=AGTECHLABS00\r\n";
	const char *at_wskey = "AT+WSKEY=wpa2psk,aes,agt3chl@bs\r\n";
	const char *at_wmode = "AT+WMODE=sta\r\n";

	wifiHReset();
	volatile WifiStatusEnum current_stat;
	const char *setup_list[] = {at_netp, at_ssid,at_wskey,at_wmode};
	char response[AT_MAXSIZE2];
	uint32_t i;

	if (wifiCommandMode() != WIFI_AT_OK)
	{
		/*logs here*/
	}
	for(i=0; i<(sizeof(setup_list)/sizeof(const char*)); i++)
	{
		memset(response,0,AT_MAXSIZE2);
		if(uartTxWifi((uint8_t *)setup_list[i],  strlen(setup_list[i])) == HAL_OK)
		{
			current_stat = serialReadUntil(response, AT_MAXSIZE2 ,"+ok",
										   AT_TIMEOUT_COMMON, WIFI_AT_OK);
			if(current_stat != WIFI_AT_OK)
			{
				//return WIFI_ERROR;
			}
		}
	}

	wifiHReset();
	return current_stat;
}

/*==============================================================================
 * @brief 
 * @param
 * @retval
 *=============================================================================*/
static WifiStatusEnum wifiTxRxWeb(char *webresponse, char *data, HTTPStruct *httpstruct)
{
	volatile WifiStatusEnum current_stat = WIFI_ERROR;
	volatile DataStatusEnum data_stat = DATA_EMPTY;
	memset(webresponse,0,MAXSIZE_WEBRESPONSE);
	//delayMillis(100);
	if(uartTxWifi((uint8_t *)"\r\n", 2 ) != HAL_OK)
	{

	}
	//delayMillis(100);
	if(uartTxWifi((uint8_t *)data, strlen(data) ) == HAL_OK)
	{
		data_stat = wifiReadWebData(webresponse, MAXSIZE_WEBRESPONSE,
									   TIMEOUT_WIFIPOST);
		if(data_stat == DATA_VALID)
					current_stat = WIFI_WEBTRNSCT_SUCCESS;
		else
			current_stat = WIFI_WEBTRNSCT_ERROR;

		httpParseWebResponse(webresponse, httpstruct);
		httpstruct->data_stat = data_stat;
		/*
		if(current_stat == WIFI_HOST_DONE)
			current_stat = WIFI_WEBTRNSCT_SUCCESS;
		else
			current_stat = WIFI_WEBTRNSCT_ERROR;
		*/
	}
	else
		return WIFI_NO_RESPONSE;
	return current_stat;
}

/*==============================================================================
 * @brief 
 * @param
 * @retval
 *=============================================================================*/
static WifiStatusEnum wifiPing()
{
	char response[AT_MAXSIZE2];
	volatile WifiStatusEnum current_stat = WIFI_DATA_OFF;
	const char *data = "AT+PING=www.google.com\r\n";
	const char *rsp_success = "+ok=Success";
	const char *rsp_unknown = "+ok=Unknown host";
	const char *rsp_rto = "+ok=Timeout";
	const char *rsp_err = "+ERR";

	const WifiStatusEnum wifi_data_stats[] = {WIFI_DATA_ON, WIFI_HOST_ERROR, WIFI_RTO, WIFI_ERROR};
	const char *wifi_data_resps[] = {rsp_success, rsp_unknown, rsp_rto, rsp_err};

	if (wifiCommandMode() != WIFI_AT_OK)
	{
		/*logs here*/
	}

	if(uartTxWifi((uint8_t *)data, strlen(data) ) == HAL_OK)
	{
		//current_stat = serialReadUntilMultiple(response, "+ok=Success", TIMEOUT_HOST_RESPONSE, WIFI_DATA_ON);
		current_stat = serialReadUntilMultiple(response, wifi_data_resps,
					   sizeof(wifi_data_resps)/sizeof(const char*), wifi_data_stats,
					   TIMEOUT_WIFIPOST);
		if(current_stat != WIFI_DATA_ON)
		{
			current_stat = WIFI_DATA_OFF;
		}
	}
	if (wifiThroughMode() != WIFI_AT_OK)
	{
		/*logs here*/
	}
	#if PRINT_PROCESSES_IN_UART
		uartSHOW((uint8_t*)response, strlen(response));
	#endif
	return current_stat;
}

/*
 * @brief 	Post data to api
 * @param	urlstr: string of url where data will be posted
 * @param	authorization: string that contains security credential of HW
 * @param	datastr: data to be posted
 * @param 	httpstruct: ptr to formatted web response
 * @retval	wifi status
 * */
WifiStatusEnum wifiPost(char *urlstr, const char *authorization, char *datastr, HTTPStruct *httpstruct)
{
	char *data = NULL;
	char webresponse[MAXSIZE_WEBRESPONSE];
	memset(webresponse,0,MAXSIZE_WEBRESPONSE);
	memset(httpstruct->http_body, 0, MAXSIZE_HTTPBODY);
	memset(httpstruct->http_code, 0, MAXSIZE_HTTPCODE);
	memset(httpstruct->http_date, 0, MAXSIZE_HTTPDATE);

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
	volatile WifiStatusEnum current_stat = WIFI_DATA_OFF;

	current_stat = wifiPing();
	if(current_stat == WIFI_DATA_ON)
	{
		if (httpParseUrl(&urlstruct, urlstr) )
		{
			/*Set header field values*/
			snprintf(hdrfld_method, strlen(urlstruct.path) + RAWSIZE_HDRFLD_METHOD,
					 "POST %s HTTP/1.1\r\n", urlstruct.path);
			snprintf(hdrfld_host,strlen(urlstruct.host) + RAWSIZE_HDRFLD_HOST,
					 "Host: %s\r\n", urlstruct.host);
			snprintf(hdrfld_authorization,strlen(authorization) + RAWSIZE_HDRFLD_AUTH,
					 "Authorization: %s\r\n", authorization);
			snprintf(hdrfld_contentlength,MAXSIZE_HDRFLD_CONLEN,
					 "Content-Length: %lu\r\n", data_length);

			header_size = sizeof(char)*( strlen(hdrfld_method) +
					strlen(hdrfld_authorization) + strlen(hdrfld_contentlength) +
					strlen(hdrfld_host) + strlen(hdrfld_connection) +
					strlen(hdrfld_accept) + strlen(hdrfld_contenttype) + 4);
			header = memAlloc(header_size);
			snprintf(header,header_size,"%s%s%s%s%s%s%s",hdrfld_method, hdrfld_host,
					 hdrfld_authorization, hdrfld_contentlength, hdrfld_accept, hdrfld_contenttype,
					 hdrfld_connection);

			data = memAlloc(sizeof(char)*(strlen(header) + strlen(datastr) + 6));
			sprintf(data,"%s\r\n%s\r\n\r\n",header,datastr);

			current_stat = wifiTxRxWeb(webresponse, data, httpstruct);
			if (current_stat != WIFI_WEBTRNSCT_SUCCESS)
			{
				httpParseWebResponse(webresponse, httpstruct);
			}

			free(urlstruct.host);
			free(urlstruct.path);
			free(header);
			free(data);
		}
	}
	return current_stat;
}

/*==============================================================================
 * @brief 
 * @param
 * @retval
 *=============================================================================*/
WifiStatusEnum wifiGet(char *urlstr, const char *authorization, HTTPStruct *httpstruct)
{
	char webresponse[MAXSIZE_WEBRESPONSE];
	memset(httpstruct->http_body, 0, MAXSIZE_HTTPBODY);
	memset(httpstruct->http_code, 0, MAXSIZE_HTTPCODE);
	memset(httpstruct->http_date, 0, MAXSIZE_HTTPDATE);

	/*HTTP Header Fields*/
	char *header = NULL;
	char hdrfld_method[HEADER_FIELD_SIZE];
	char hdrfld_host[HEADER_FIELD_SIZE];
	char hdrfld_authorization[HEADER_FIELD_SIZE];
	const char *hdrfld_connection = "Connection: close\r\n";
	const char *hdrfld_accept = "Accept: application/json\r\n";
	size_t header_size = 0;

	URLStruct urlstruct;
	volatile WifiStatusEnum current_stat = WIFI_ERROR;

	current_stat = wifiPing();
	if(current_stat == WIFI_DATA_ON)
	{
		if (httpParseUrl(&urlstruct, urlstr) )
		{
			/*Set header field values*/
			snprintf(hdrfld_method,HEADER_FIELD_SIZE,"GET %s HTTP/1.1\r\n", urlstruct.path);
			snprintf(hdrfld_host,HEADER_FIELD_SIZE ,"Host: %s\r\n", urlstruct.host);
			snprintf(hdrfld_authorization,HEADER_FIELD_SIZE, "Authorization: %s\r\n", authorization);

			header_size =  sizeof(char)*( strlen(hdrfld_method) +
					strlen(hdrfld_host) + strlen(hdrfld_accept) + strlen(hdrfld_authorization) +
					strlen(hdrfld_connection) + 4);
			header = memAlloc(header_size);

			snprintf(header,header_size, "%s%s%s%s%s\r\n\r\n", hdrfld_method, hdrfld_host,
					 hdrfld_authorization, hdrfld_accept, hdrfld_connection);


			current_stat = wifiTxRxWeb(webresponse, header, httpstruct);
			if (current_stat != WIFI_WEBTRNSCT_SUCCESS)
			{

			}

			free(urlstruct.host);
			free(urlstruct.path);
			free(header);
		}
	}
	return current_stat;
}

/*********************************************************************************
  * Revision History
  *	@file      	wifi.c
  ********************************************************************************
  * @version    v2.2.0
  * @date       04/08/16
  * @author     D.Lasdoce
  * @changes
  ********************************************************************************  
  * @version    v2.1.0
  * @date       12/23/2015
  * @author     D.Lasdoce
  * @changes
  ********************************************************************************
  */