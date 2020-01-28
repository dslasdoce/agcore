/******************************************************************************
  * @file    	rdBnrMain.h
  * @author  	Hardware Team, Agtech Labs Inc.
  * @version 	v2.2.0
  * @date    	04/08/16
  * @brief   	Header File for Banner Radio Function Module
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
#ifndef __RDBNRMAIN_H_ 
#define __RDBNRMAIN_H_

/* Includes =================================================================*/
#include "agBoardInit.h"
#include "radio.h"

/* Object-Like Macros =======================================================*/
#define RDBNR_MULTIHOPRADIO_ID      ((uint8_t)0x01)
#define RDBNR_BASE_MODBUSID         ((uint8_t)0x0C)
#define RDBNR_MAX_DATASZ		    ((uint32_t)164) // divisible by 4 for flash
#define BNRRADIO_RXDLY 				((uint32_t)5000)
#define BNRRADIO_DLY				((uint32_t)100)
#define RDBNR_OTA_PACKETSZ		    ((uint32_t)128)

#define	RECSZ_PER_POLL_TELEM		((uint8_t)100)
#define	RECSZ_PER_POLL_SYSSTAT		((uint8_t)50)
#define	RECSZ_PER_POLL_VLVSCHED		((uint8_t)1)
#define	RECSZ_PER_POLL_VLVCMD		((uint8_t)1)
#define	RECSZ_PER_POLL_VLVREG		((uint8_t)1)
#define	RECSZ_PER_POLL_VLVDRV		((uint8_t)1)
#define RECSZ_PER_POLL_SENREG		((uint8_t)4)
#define	RECSZ_PER_POLL_SENDRV		((uint8_t)1)

#define	MAX_REC_PER_FRAME_TELEM		((uint8_t)4)
#define	MAX_REC_PER_FRAME_SYSSTAT	((uint8_t)3)

#define DATARX_TIMEOUT		(3000)
#define HEADERRX_TIMEOUT	(1000)
#define RDBNR_DEFAULT_NETMAINTENANCEMODE_TIME_MIN   ((uint8_t)120) //120 minutes

// Banner radio register addresses
#define RDBNR_REGADDR_DEV_MULTIHOP_RADIO_ID     ((uint16_t)0x1A97)
#define RDBNRGATE_REGADDR_NUM_OF_DEV            ((uint16_t)0x1B58)
#define RDBNRGATE_REGADDR_DEV_ADDR_LIST         ((uint16_t)0x1B59) + 1 //exclude Master
#define RDBNRGATE_REGADDR_MODBUS_TODEVICE_LIST  ((uint16_t)0x1967) + 1 //exclude Master
#define RDBNRGATE_REGADDR_AGDEVICESN            ((uint16_t)0x1068)
#define RDBNRGATE_REGADDR_POWERMODE             ((uint16_t)0x18B8)
#define RDBNRGATE_REGADDR_SITESURVEYCOUNT       ((uint16_t)0x1933)
#define RDBNRGATE_REGADDR_SITESURVEYCONTROL     ((uint16_t)0x1932)

/* Extern ===================================================================*/
extern const uint16_t POLL_REC_COUNT[];
extern const uint8_t BYTESIZE[];
extern const uint8_t MAX_FRAME_REC[];
extern bool is_devnum_set;
#if DEVICE_TYPE == DEVICETYPE_NODE
extern uint8_t m_radio_signal_strength;
#endif
/* Banner Radio Common Functions ============================================*/
RdStatEnum rdBnrMainDataRx(RdHeadStruct arg_headptr, uint8_t *arg_dataptr);
RdStatEnum rdBnrMainHeadRx(RdHeadStruct *arg_headptr);
RdStatEnum rdBnrMainRxResp(RdHeadStruct *a_headptr, uint32_t a_timeout_ms);
RdStatEnum rdBnrMainDataTx(RdHeadStruct arg_head
                           , uint8_t *arg_dataptr
                           , uint32_t arg_bytsz);
RdStatEnum rdBnrMainHeadTx(RdHeadStruct arg_head);
uint8_t rdBnrMainGetLedStat(void);

/* Banner Radio Setup Functions =============================================*/
void rdBnrMainReset(void);
RdStatEnum rdBnrMainInit(void);
RdStatEnum rdBnrMainSetNetId(uint32_t arg_netid);
RdStatEnum rdBnrMainGetNetId(uint32_t *arg_netid_ptr);
RdStatEnum rdBnrMainBlankFrRx(RdHeadStruct *arg_headptr, uint32_t timeout);
RdStatEnum rdBnrGateSetModbusOfNode(uint8_t rad_mb_id);
RdStatEnum rdBnrGateNetworkInit(void);
RdStatEnum rdBnrMainUpdateMaintenanceModeState(RdNetMaintenanceModeTypEnum a_code
                                            , uint8_t a_remaining_time);
void rdBnrMainSwitchMaintenanceMode(void);
void rdBnrMainUpdateNmmTimer(uint32_t a_endtime);
#endif /* __RDBNRMAIN_H_ */

/******************************************************************************
  * Revision History
  *	@file      	rdBnrMain.h
  *****************************************************************************
  * @version	v2.1
  * @date		2.2.0
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
