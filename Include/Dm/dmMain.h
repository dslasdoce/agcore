/********************************************************************************
  * @file		dmMain.h
  * @author     Hardware Team
  * @version    v2.2.2
  * @date       04/08/16
  * @brief
  * 	Contains all exception handler related to timers.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DMMAIN_H_
#define __DMMAIN_H_

/* Includes ==================================================================*/
#include <stdint.h>
#include "rtc.h"
#include "agBoardInit.h"
#include "productCfg.h"
  

/* Typedefs ==================================================================*/

/* Telemetry Data Struct*/
	  
typedef enum
{
	SYS_RESET,
	POST_FLASH_ERASE
} FlashStateEnum;

/* Defines ===================================================================*/
#define TELEM_REC_SIZE_BYTES			(sizeof(TelemetryStruct))
#define SYSSTAT_REC_SIZE_BYTES			(sizeof(SystemStatusStruct))
#define SENREG_REC_SIZE_BYTES			(sizeof(SensorRegistrationStruct))

#define REC_SZ(rec_typ)					((rec_typ == TELEMETRY) ? \
											TELEM_REC_SIZE_BYTES: \
											SYSSTAT_REC_SIZE_BYTES)	

#define REC_TYP_CNT_TO_LOG				2	/*telemetry and system stat*/
		  
 

/* Function Prototypes =======================================================*/
ReturnStatusEnum dmInitialize();

#endif /* DMMAIN_H_ */


/*******************************************************************************
  * Revision History
  * @file         dmMain.h
  ******************************************************************************
  * @version    v2.2.0
  * @date       
  * @author     M.I.Lorica
  * @changes	rename from dataCommon.h to dmMain.h and migrated structs from
  *				other DM header file
  ******************************************************************************
  * @version
  * @date
  * @author
  * @changes     - created file
  * @version    v2.1.0
  * @date       12/23/2015
  * @author     M.I.Lorica
  * @changes
  ********************************************************************************
  * @version
  * @date
  * @author
  * @changes     - created file
  ********************************************************************************
  */

