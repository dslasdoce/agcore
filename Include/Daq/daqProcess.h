/********************************************************************************
  * @file      	daqProcess.h
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

#ifndef __DAQPROCESS_H_
#define __DAQPROCESS_H_

#include "gpio.h"
#include "agBoardInit.h"
#include "equations.h"

/* External Declarations =====================================================*/

/* Function Prototypes =======================================================*/
void daqShowInUart(float *raw_values, double *con_values,
				   uint32_t size, uint8_t *data_sequence,
				   uint8_t *depth_values);
void daqPrint(float raw_value, uint8_t raw_uom, 
			  double con_value, uint8_t con_uom, 
			  uint8_t depth_value, uint8_t depth_uom, 
			  uint16_t sensor_code, uint8_t sensor_type);
/* Telemetry Data Acquisition*/
DaqStatusEnum daqExecute(SensorRegistrationStruct *sensordetails_ptr);
DaqStatusEnum daqExecuteDummy(uint32_t i);
void daqSetProcessingParams(DataProcessingStruct *params_ptr, 
							SensorRegistrationStruct *sensordetails_ptr);

/*for System Status Acquisition*/
DaqStatusEnum daqSysAcquire(void);
DaqStatusEnum sysAcquireDummy(void);
DaqStatusEnum daqSysAcquireApp(SystemStatusStruct *sys_stat_ptr);
#endif
	
/******************************************************************************
  * Revision History
  *	@file      	daqProcess.h
  *****************************************************************************
  * @version	v2.2.0
  * @date		04/08/16
  * @author		D. Lasdoce
  * @changes	- created file
  *****************************************************************************
  */