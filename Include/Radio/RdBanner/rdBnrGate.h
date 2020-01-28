/******************************************************************************
  * @file    	rdBnrGate.h
  * @author  	Hardware Team
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
#ifndef __RDBNRGATE_H_
#define __RDBNRGATE_H_

/* Object-Like Macros =======================================================*/
#define RDBNRGATE_MAX_NODE_COUNT    ((uint8_t)100)

/* Functions ================================================================*/
RdStatEnum rdBnrGateQuery(uint8_t arg_id, RdFuncEnum arg_func, va_list vlist);
RdStatEnum rdBnrGateNetworkAutoConfig(uint32_t arg_netid);
void rdBnrGateRxFrameNmmChecker(RdHeadStruct a_header);

#endif /* __RDBNRGATE_H_ */

/******************************************************************************
  * Revision History
  *	@file      	rdBnrGate.h
  *****************************************************************************
  * @version	v2.2.0
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