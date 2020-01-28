/********************************************************************************
  * @file      	daqSysStat.h
  * @author		Hardware Team
  * @version	v2.2.0
  * @date		04/08/16
  * @brief
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

#ifndef __DAQSYSSTAT_H_
#define __DAQSYSSTAT_H_

/* Includes ==================================================================*/
#include <stdint.h>

#include "productCfg.h"

/* External Declarations =====================================================*/

/* Function Prototypes =======================================================*/
ReturnStatusEnum sysPwrMntrRead(uint8_t *batt_stat_ptr);//float *pwr_stat);
ReturnStatusEnum sysRadioSigStat(uint8_t *radio_sig_stat_ptr);
ReturnStatusEnum sysCellSigStat(uint8_t *cell_sig_stat_ptr);
ReturnStatusEnum sysAzimuth(uint8_t *acc_sig_stat_ptr);//float *angle);
ReturnStatusEnum sysMemSize(uint16_t *orig_mem_size_ptr,
							uint16_t *avail_mem_ptr);
ReturnStatusEnum sysInternalTemp(float *temp_f);
ReturnStatusEnum sysExternalTemp(float *temp_f);
ReturnStatusEnum sysGPSLoc(float *latitude,
						   float *longitude);
uint8_t sysChargingStat(void);
void sysSleepAcce(void);

#endif
	
/******************************************************************************
  * Revision History
  *	@file      	daqSysStat.h
  *****************************************************************************
  * @version	v2.2.0
  * @date		04/08/16
  * @author		D. Lasdoce
  * @changes	- created file
  *****************************************************************************
  */