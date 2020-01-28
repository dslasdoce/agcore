/*******************************************************************************
  * @file      	wifi.h
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

#ifndef WIFI_H_
#define WIFI_H_

/* Includes ==================================================================*/
#include "http.h"

/* Typedefs ==================================================================*/
typedef enum
{
	WIFI_NO_RESPONSE,
	WIFI_AT_OK,
	WIFI_AT_ERROR,
	WIFI_RTO,
	WIFI_HOST_CONNECTED,
	WIFI_HOST_ERROR,
	WIFI_HOST_DONE,
	WIFI_DATA_ON,
	WIFI_DATA_OFF,
	WIFI_ERROR,
	WIFI_WEBTRNSCT_SUCCESS,
	WIFI_WEBTRNSCT_ERROR,
	WIFI_BUFFEROVERFLOW
}WifiStatusEnum;

/* Function Protypes =============================================================*/
WifiStatusEnum wifiPost(char *urlstr, const char *authorization, char *datastr, HTTPStruct *httpstruct_ptr);
WifiStatusEnum wifiGet(char *urlstr, const char *authorization, HTTPStruct *httpstruct_ptr);
WifiStatusEnum wifiAccessToken(char *urlstr, char *security_data, HTTPStruct *httpstruct_ptr);
WifiStatusEnum wifiConfig(void);
WifiStatusEnum wifiCommandMode(void);
void wifiHReset();
WifiStatusEnum wifiPing();
WifiStatusEnum wifiSleep(void);
#endif

/*********************************************************************************
  * Revision History
  *	@file      	wifi.h
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