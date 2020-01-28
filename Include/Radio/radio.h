/******************************************************************************
  * @file    	radio.h
  * @author  	Hardware Team, Agtech Labs Inc.
  * @version 	v2.2.2
  * @date    	04/08/16
  * @brief   	Header File for Radio Function Module
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

/* Define to prevent recursive inclusion ====================================*/
#ifndef __RADIO_H_
#define __RADIO_H_

/* Includes =================================================================*/
#include "modbus.h"
#include "agBoardInit.h"

/* Typedefs =================================================================*/
typedef enum
{
	RDSTAT_NULL,
	RDSTAT_ERROR,
	RDSTAT_OK
} RdStatEnum;

typedef enum
{
	RDTYPE_NULL,
	RDTYPE_BANNER,
	RDTYPE_XBEE,
	RDTYPE_BOTH
} RdTypeEnum;

typedef enum
{
	RDFUNC_NULL,
	RDFUNC_GET,
	RDFUNC_UPDATE,
	RDFUNC_SLEEP,
	RDFUNC_STAT_CHK,
	RDFUNC_FWVER_CHK,
	RDFUNC_NETDEVNUMSET,
    RDFUNC_NETID,
    RDFUNC_MAINTENANCE
} RdFuncEnum;

typedef enum
{
	RDRESP_NULL,
	RDRESP_GET,
	RDRESP_UPDATE,
	RDRESP_REC_REPORT,
	RDRESP_ACKNOWLEDGE,
	RDRESP_STOP
} RdRespEnum;

typedef enum
{
	RDAPI_TELEM,		//for get
	RDAPI_DEVREG,		//for get
	RDAPI_VLVSCHED,	    //for update
	RDAPI_VLVCMD,		//for get
	RDAPI_VLVREG,		//for get
	RDAPI_VLVDRV,		//for update
	RDAPI_SENREG,		//for get
	RDAPI_SENDRV,		//for update
	RDAPI_RTC,			//for update
	RDAPI_RSSI,		    //for update
	RDAPI_FW,			//for update
	RDAPI_NETDEVNUM
} RdApiEnum;

typedef enum
{
	RDNETMAINTENANCEMODETYP_STARTCMD,
    RDNETMAINTENANCEMODETYP_STOPCMD,
    RDNETMAINTENANCEMODETYP_EDITCMD,
    RDNETMAINTENANCEMODETYP_CHECKCMD,
    RDNETMAINTENANCEMODETYP_RSSIDATA
} RdNetMaintenanceModeTypEnum;

typedef struct
{
	uint8_t		mb_id;		//radio network address
	uint8_t		function; 	//indicator of the process to execute
	uint8_t		type;
	uint8_t		byt_sz;		//payload size (header and crc not included)
	uint8_t		value1;		//misc value1
	uint8_t		value2;		//misc value 2
} RdHeadStruct; //BNRRADIO_MAX_DTABYT size depends on this header size

typedef struct
{
	RdHeadStruct 	    header;
	ModbusCrcStruct		crc;
} RdFrameStruct;

/* Object-like Macros =======================================================*/
#define RDFRAME_SZ		    sizeof(RdFrameStruct)
#define RDHEAD_SZ			sizeof(RdHeadStruct)
#define RDCRC_SZ			sizeof(ModbusCrcStruct)
#define RD_BANNER_HANDLER   UART2HandlerDef
#define RD_XBEE_HANDLER     UART4HandlerDef

/* Function-like Macros =====================================================*/
#define rdClear(data)	    memset(&data, 0, sizeof(data))

/* External Declarations ====================================================*/
extern RdFrameStruct rxbuff_banner;

/* Functions ================================================================*/
RdStatEnum radioInit(void);
RdStatEnum radioNetSet(uint32_t arg_netid);

#if DEVICE_TYPE == DEVICETYPE_GATE
	RdStatEnum radioQuery(uint8_t mb_id, RdFuncEnum arg_func,...);
	RdStatEnum radioPollQuery(RdFuncEnum arg_func,...);
#else
	void radioResponse(RdTypeEnum arg_rdtyp, RdFrameStruct arg_frame);
#endif /*Unit Type*/

#endif /* __RADIO_H_ */

/******************************************************************************
  * Revision History
  *	@file      	radio.h
  *****************************************************************************
   * @version	v2.2.0
  * @date		04/08/16
  * @author		J. Fajardo
  * @initial 	 
  *****************************************************************************
  * @version	v2.1
  * @date		01/12/2015
  * @author		JWF
  * @initial release for V2.1 board
  *****************************************************************************
  */