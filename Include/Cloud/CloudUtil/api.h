/********************************************************************************
 * @file      	:	api.h
 * @author		:	Hardware Team
 * @version		:	v2.0.2
 * @date		:	04/08/16
 * @brief		:	Header file of api.c
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

#ifndef __API_H_
#define __API_H_

/* Includes =================================================================*/
#include <deployAssert.h>
#include <stdlib.h>

#include "http.h"
#include "parson.h"
#include "productCfg.h"
#include "agdatim.h"

/* Macro Definitions ========================================================*/
#define MAXSIZE_DEVICESN		((uint8_t)20)
#define MAXSIZE_VALVECODE		((uint8_t)50)
#define MAXSIZE_VALVETYPE		((uint8_t)6)
#define MAXSIZE_INRAWVAL		((uint8_t)6)
#define MAXSIZE_INRAWUOM		((uint8_t)15)
#define MAXSIZE_EXECDATE		((uint8_t)25)
#define MAXSIZE_PORT			((uint8_t)5)
#define REC_COUNT				((uint8_t)5)	//temporary

/* Typedefs =================================================================*/
typedef enum
{
	APISTAT_FAILED,
	APISTAT_OK
}ApiStat;

typedef enum
{
	API_TELEMETRY		= TELEMETRY,
	API_DEV_REG			= SYSTEM_STAT,
	API_VALVE_SCHED		= VALVE_SCHED,
	API_VALVE_CMD		= VALVE_CMD,
	API_VALVE_REG		= VALVE_REG,
	API_VALVE_DRIVER	= VALVE_DRIVER,
	API_SENSOR_REG		= SENSOR_REG,
	API_SENSOR_DRIVER	= SENSOR_DRIVER,
	API_FIRMWARE_UPDATE = FIRMWARE_UPDATE,
	API_GATEINFO		= GATEINFO
} ApiTypeEnum;

typedef struct
{
	char 		dev_sn[MAXSIZE_DEVICESN];
	char 		vlv_cde[MAXSIZE_VALVECODE];
	char 		vlv_typ[MAXSIZE_VALVETYPE];
	char 		in_val[MAXSIZE_INRAWVAL];
	char 		in_uom[MAXSIZE_INRAWUOM];
	char 		exec_date[MAXSIZE_EXECDATE];
	char 		port[MAXSIZE_PORT];
	char		reserved;
	uint32_t	acct_id;
	uint32_t	sched_id;
	uint32_t	activity_id;
	uint32_t	command;
} ValveSchedAPIStruct;

/* External Declarations ====================================================*/
//extern PortConfigStruct PortConfig;

/* Function Prototypes ======================================================*/
ApiStat apiToJson(uint8_t apitype, char **apidata_ptr);
extern uint32_t temp_posted_rec_cnt;
#endif /*__API_H_*/
/******************************************************************************
 * Revision History
 *	@file      	api.h
 ******************************************************************************
 * @version		v2.0.2
 * @date		04/08/16
 * @author		J. Fajardo
 * @changes	 	initial release
 ******************************************************************************
 */

