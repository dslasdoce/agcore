/*******************************************************************************
  * @file      	cell.h
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

#ifndef CELL_H_
#define CELL_H_
/* Includes ==================================================================*/
#include "http.h"
/* Defines  ==================================================================*/
#define TIMEOUT_SOCKETDIAL 					((uint32_t)20)
#define TIMEOUT_CELL_DATAACTIVATION 		((uint32_t)30)

#define SOCKET_DIAL_SIZE 					((uint32_t)100)
#define AT_MAXSIZE							((uint32_t)100)
#define CELL_COMMON_TIMEOUT 				((uint32_t)2)
#define	CELL_NO_RESP_LIMIT					((uint32_t)3)
#define MAXSIZE_CID							((uint32_t)1)

#define FTP_ADD		((const char*)"54.183.39.108")
#define FTP_USER	((const char*)"ian")
#define FTP_PASS	((const char*)"q1w2e3r4")
//#define FTP_FILE	((const char*)"hardware\\iaptest.hex")
#define FTP_FILE	((const char*)"hardware\\Agricapture_Node.hex")
//#define FTP_FILE	((const char*)"hardware\\AgcaptureFW_ver2.0.hex")
#define FTP_BUFFSIZE 	((uint32_t)2560)
#define SPI_WRMAX	 	((uint32_t)128)

/* Typedefs ==================================================================*/
typedef enum
{
	FWSTAT_ERROR,
	FWSTAT_OK,
	FWSTAT_ER_SOL,
    FWSTAT_ER_CHKSUM,
    FWSTAT_ER_SIZE,
    FWSTAT_ER_CR
}FWStatEnum;

typedef enum
{
	CELL_NO_RESPONSE,
	CELL_AT_OK,
	CELL_AT_ERROR,
	CELL_NET_HOME,
	CELL_NET_ROAMING,
	CELL_NET_SEARCHING,
	CELL_NET_NOTREG,
	CELL_HOST_CONNECTED,
	CELL_HOST_ERROR,
	CELL_HOST_DONE,
	CELL_DATA_ON,
	CELL_DATA_OFF,
	CELL_ERROR,
	CELL_WEBTRNSCT_SUCCESS,
	CELL_WEBTRNSCT_ERROR,
	CELL_BUFFEROVERFLOW,
	CELL_FTPSERVER_OK,
	CELL_FTPSERVER_ER,
	CELL_DOWNLOAD_OK,
	CELL_DOWNLOAD_ER,
	CELL_FWUPDATE_READY
}CellStatusEnum;

/* Function Protypes =============================================================*/
CellStatusEnum cellAccessToken(char *urlstr, char *security_data, HTTPStruct *httpstruct);
CellStatusEnum cellPost(char *urlstr, const char *authorization, char *datastr, HTTPStruct *httpstruct);
CellStatusEnum cellGet(char *urlstr, const char *authorization, HTTPStruct *httpstruct);
void cellSetupParameters(void);
//CellStatusEnum cellSetupContext(uint8_t cid);
void cellSetDateTime(void);
void cellON_OFF(bool state);
void cellReset(void);
void cellProvision(void);
CellStatusEnum cellSignalQuality(uint8_t *signal_strength);
CellStatusEnum serialReadUntil(char *resp, uint32_t resp_maxsize ,char *end_str,
									  CellStatusEnum success_response, uint32_t timeout);
CellStatusEnum serialReadUntilMultiple(char *refstr_ptr,
									   const char **patternstr_arr_ptr,
									   uint8_t pattern_count,
									   const CellStatusEnum *statmap_ptr,
									   uint32_t timeout);
//CellStatusEnum serialReadUntil(char *resp, regex_t *pattern_ptr, uint32_t timeout);
CellStatusEnum cellActData(void);
CellStatusEnum cellDeactData(void);
CellStatusEnum cellDeactDataAllContext(void);

uint32_t cellDownloadFW(void);
FWStatEnum fileCheck();
void cellReadSPI(uint8_t *buffer, uint32_t addr,uint32_t loop_count);
bool cellWriteSPI(uint8_t *buffer, uint32_t addr,uint32_t loop_count);
CellStatusEnum cellFTPTO();
void cellForcedShutDown(void);
CellStatusEnum cellUpdateCheck(void);
void cellON_OFFBypass(bool state);
CellStatusEnum cellModemTestMode();
#endif

/*********************************************************************************
  * Revision History
  *	@file      	cell.h
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