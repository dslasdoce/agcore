 /********************************************************************************
  * @file      	http.h
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

#ifndef HTTP_H_
#define HTTP_H_
#include <stdint.h>
#include <stdbool.h>

#define MAXSIZE_HTTPCODE					((uint32_t)4)
#define MAXSIZE_HTTPDATE					((uint32_t)30)
#define MAXSIZE_HTTPBODY					((uint32_t)2000)
#define RAWSIZE_SOCKETDIAL 					((uint32_t)25)
#define RAWSIZE_HDRFLD_METHOD 				((uint32_t)18)
#define RAWSIZE_HDRFLD_HOST 				((uint32_t)10)
#define RAWSIZE_HDRFLD_AUTH 				((uint32_t)19)
#define RAWSIZE_HDRFLD_CONLEN 				((uint32_t)21)
#define MAXSIZE_HDRFLD_CONLEN 				((uint32_t)(RAWSIZE_HDRFLD_CONLEN + 7))
#define MAXSIZE_WEBRESPONSE 				((uint32_t)2000)
#define MAXSIZE_CELL_AT						((uint32_t)60)
#define HEADER_FIELD_SIZE					((uint32_t)100)
#define	TIMEOUT_HOST_RESPONSE 				((uint32_t)90)
#define INTERVAL_MULTIPLE_SEARCH			((uint32_t)200)

typedef enum
{
	DATA_EMPTY,
	DATA_VALID,
	DATA_INVALID,
	DATA_PENDING,
	DATA_STARTCH_READY,
	DATA_STARTCH_WAIT,
	DATA_BUFFEROVERFLOW

}DataStatusEnum;

typedef struct
{
	char http_code[MAXSIZE_HTTPCODE];
	char http_body[MAXSIZE_HTTPBODY];
	char http_date[MAXSIZE_HTTPDATE];
	DataStatusEnum data_stat;
}HTTPStruct;

typedef struct
{
	char *host;
	char *path;
}URLStruct;

void httpParseWebResponse(char *webresponse, HTTPStruct *httpstruct_ptr);
bool httpParseUrl(URLStruct *urlstruct_ptr, char *url_str);
void httpResetStruct(HTTPStruct *httpstruct_ptr);

#endif

/*********************************************************************************
  * Revision History
  *	@file      	http.h
  ********************************************************************************
  * @version    v2.2.0
  * @date       04/08/16
  * @author     D.Lasdoce
  * @changes
  ********************************************************************************  
  * @version    v2.1.0
  * @date       12/23/2015
  * @author     D.Lasdoce
  * @changes
  ********************************************************************************
  */