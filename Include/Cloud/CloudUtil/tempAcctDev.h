/********************************************************************************
 * @file      	:	tempAcctDev.h
 * @author		:	Hardware Team, Agtech Labs Inc.
 * @version		:	V2.2.0
 * @date		:	04/08/16
 * @brief		:	Header file of cloud
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

#ifndef __TEMPACCTDEV_H_
#define __TEMPACCTDEV_H_

/* Includes =================================================================*/
#include <stddef.h>

/* External Declarations ====================================================*/
const char *URLHOST = CLOUDHOST;

//char *AUTHENTICATION = "Bearer 08e937186390a12ed16294706e974830ccbc749d"; //temporary

const char *URLPOSTPATH[] =
{
	"api/v0.1/telemetry-bulk/",
	"api/v0.1/device-registrations-bulk/",
	NULL,
	"api/v0.1/valve-commands-bulk/",
	"api/v0.1/valve-registrations-bulk/",
	NULL,
	"api/v0.1/sensor-registrations-bulk/",
	NULL,
	NULL,
	NULL
};

const char *URLGETPATH[] =
{
	NULL,
	NULL,
	"api/v0.1/valve-schedules/?limit=1&account_id=11&sort-=date_created",
	NULL,
	NULL,
	"api/v0.1/valve-drivers/",
	NULL,
	"api/v0.1/sensor-drivers/",
	"",
	"api/v0.1/device-account-list/"
};

#endif /*__TEMPACCTDEV_H_.h*/
