/********************************************************************************
 * @file      	:	agps.h
 * @author		:	Hardware Team, Agtech Labs Inc.
 * @version		:	V2.0.1
 * @date		:	10/27/2015
 * @brief		:	Global Positioning System Module
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

/* Define to prevent recursive inclusion ====================================*/
#ifndef __GPS_H_
#define __GPS_H_

/* Includes =================================================================*/
#include "agps.h"
#include "rtc.h"

/* Constants ================================================================*/
#define GPS_BOOTIME			    (uint32_t)60000
#define GPS_ONCHCK_PROCSTIME	(uint32_t)300000
#define GPS_ONBOOT_PROCSTIME	(uint32_t)900000  /*15mins process time*/
#define GPS_RESTIME			    (uint32_t)60000   /*1mins rest time*/

#define gpsOnBoot(void)		    (gpsExec(GPS_ONBOOT, GPS_ONBOOT_PROCSTIME))
#define gpsOnCheck(void)		(gpsExec(GPS_ONCHECK, GPS_ONCHCK_PROCSTIME))

/* Typedefs =================================================================*/
typedef struct
{
	float 					latitude;
	float	 				longitude;
	RTCDateTimeStruct		 datetime;
}GpsSysStatStruct;

typedef enum
{
	GPS_FAILED,
	GPS_SUCCESSFUL,
	GPS_RUNNING
}GpsStat;

typedef enum
{
	GPS_ONCHECK,
	GPS_ONBOOT
}GpsMode;

/* Function Prototypes ======================================================*/
GpsStat gpsExec(GpsMode mode,uint32_t process_time);
GpsStat gpsProcess(uint32_t process_time);
GpsStat gpsPowerON(bool state);
extern GpsSysStatStruct gps_systat_data;
//GpsStat gpsBoot(void);

#endif /* agps.h */

/******************************************************************************
 * Revision History
 *	@file      	agps.c
 ******************************************************************************
 * @version	v2.0.0
 * @date	10/13/2015
 * @author	JWF
 * @changes	- initial version
 * 			- done on booting up gps module
 * 			- done on acquiring data
 ******************************************************************************
 * @version	v2.0.1
 * @date	10/27/2015
 * @author	JWF
 * @changes	- full-functional release
 * 			- internal function:
 * 				+ agpsBoot
 * 				+ agpsShutDown
 * 				+ agpsValidateNmea
 * 				+ agpsLog
 * 				+ agpsNmeaParser
 *				+ agpsProcess
 * 			- external function:
 * 				+ agpsExec
 *****************************************************************************
 * @version	Beta Version 2.3.0
 * @date		11-April-2016
 * @author		JWF
 * @initial release for version 2.3
 *****************************************************************************
 */

