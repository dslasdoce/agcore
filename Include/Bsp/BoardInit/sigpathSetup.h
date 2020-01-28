/**
  ******************************************************************************
  * @file      	sigpathSetup.h
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


#ifndef SIGPATH_SETUP_H_
#define SIGPATH_SETUP_H_

/* Includes =================================================================*/
#include "agBoardInit.h"

/* Defines ==================================================================*/
//#define IS_INTERFACE(INTERFACE)		(INTERFACE == ANALOG || INTERFACE == ANALOG_W_CCTR || \
//									INTERFACE == SDI12 || INTERFACE == C_COUNTER ||\
//									INTERFACE == P_COUNTER || INTERFACE == ANALOG_W_PCTR \
//									|| INTERFACE == RS485 || INTERFACE == RS485_DAVIS \
//									|| INTERFACE == DUAL_ANALOG)
#define IS_PORT(PORT)				(PORT == 1 || PORT == 2 || PORT == 3 \
									|| PORT == 4)
#define IS_CODE_AMUX(CODE)			(CODE == 0 || CODE == 1 || CODE == 2 \
									|| CODE == 3)
#define IS_DPOT(RES)				(RES >= 0 && RES<=90000)

/* Function Prototypes ======================================================*/
InterfaceStatusEnum sigpathSet(uint8_t port, int rescode, 
								InterfaceCodeEnum intcode,int dpot_res);
void sigpathAnalogEn(uint8_t port, bool enable);
void sigpathCtrEn(uint8_t port,  InterfaceCodeEnum code_interface);

#endif
/*********************************************************************************
  * Revision History
  * @file         
  ********************************************************************************
  * @version    v2.2.0
  * @date       04/08/16
  * @author     D.Lasdoce
  * @changes
  ********************************************************************************
  * @version
  * @date
  * @author         
  * @changes     - created file
  ********************************************************************************
  */
