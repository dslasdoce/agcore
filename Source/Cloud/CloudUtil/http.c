 /********************************************************************************
  * @file      	http.c
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
/* Includes ==================================================================*/
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "http.h"

/*==============================================================================
 * @brief 
 * @param
 * @retval
 *=============================================================================*/
static char * strndup (const char *s, size_t n)
{
  char *result;
  size_t len = strlen (s);

  if (n < len)
    len = n;

  result = (char *) malloc (len + 1);
  if (!result)
    return 0;

  result[len] = '\0';
  return (char *) memcpy (result, s, len);
}

/*==============================================================================
 * @brief 
 * @param
 * @retval
 *=============================================================================*/
void httpParseWebResponse(char *webresponse, HTTPStruct *httpstruct_ptr)
{
	//char *response = "HTTP/1.1 201 CREATED\r\nServer: nginx/1.8.0\r\nContent-Type: application/json\r\nContent-Length: 14\r\nConnection: close\r\nVary: Accept\r\nAllow: POST\r\nDate: Fri, 02 Oct 2015 07:22:29 GMT\r\nX-Frame-Options: SAMEORIGIN\r\n\r\n{\"created\": 12128}\r\nNO CARRIER\r\n";
	const char *httpcode_created = "201";
	const char *httpkey_version = "HTTP/1.1 ";
	char *httpkey_version_addr = NULL;

	char *httpkey_bodystart = NULL;
	char *httpkey_bodystart_addr = NULL;

	//char *httpkey_bodyend = "}\r\n";
	char *httpkey_bodyend_addr = NULL;

	char *httpkey_date = "Date: ";
	char *http_date_start_addr = NULL;

    /*DATE*/
	http_date_start_addr = strstr(webresponse,httpkey_date);
	if(http_date_start_addr)
	{
		memcpy(httpstruct_ptr->http_date, http_date_start_addr +
			   strlen(httpkey_date), MAXSIZE_HTTPDATE - 1);
	}

    /*copy http code*/
    httpkey_version_addr = strstr(webresponse,httpkey_version);
	if(http_date_start_addr)
		memcpy(httpstruct_ptr->http_code, httpkey_version_addr + strlen(httpkey_version), 3);
	else
		return;

    /*look for httpkey_bodyend in webresponse*/
    /*
    httpkey_bodyend_addr = strstr(webresponse,httpkey_bodyend);
    if(!httpkey_bodyend_addr)
    {
    	httpkey_bodyend_addr = strstr(webresponse,"}]}") + 4;
    }
	*/
	httpkey_bodyend_addr = webresponse + strlen(webresponse);
    if(!httpkey_bodyend_addr)
    	return;

    /*httpcode = 201*/
    if (strcmp(httpstruct_ptr->http_code,httpcode_created) == 0)
    {
    	httpkey_bodystart = "{\"created\": ";
    	httpkey_bodystart_addr = strstr(webresponse,httpkey_bodystart);
    	httpkey_bodyend_addr = strstr(webresponse,"}");
    	//--httpkey_bodyend_addr;
    	/*copy http body*/
    	if(httpkey_bodystart_addr)
    		memcpy(httpstruct_ptr->http_body, httpkey_bodystart_addr +
				   strlen(httpkey_bodystart),httpkey_bodyend_addr -
				   httpkey_bodystart_addr - strlen(httpkey_bodystart) );
    }
    /*other http codes*/
    else
    {
    	httpkey_bodystart = "\r\n\r\n";
    	httpkey_bodystart_addr = strstr(webresponse,httpkey_bodystart);
    	if(httpkey_bodystart_addr)
    		memcpy(httpstruct_ptr->http_body, httpkey_bodystart_addr,
				   strlen(httpkey_bodystart_addr));
    }
}

/*==============================================================================
 * @brief 
 * @param
 * @retval
 *=============================================================================*/
bool httpParseUrl(URLStruct *urlstruct_ptr, char *url_str)
{
	char *resource_name = NULL;
	resource_name = strstr(url_str,"/");
	if(resource_name)
	{
		urlstruct_ptr->host = (char*)strndup(url_str,resource_name - url_str);
		urlstruct_ptr->path = strdup(resource_name);
		return true;
	}
	return false;
}

/*==============================================================================
 * @brief 
 * @param
 * @retval
 *=============================================================================*/
void httpResetStruct(HTTPStruct *httpstruct_ptr)
{
	memset(httpstruct_ptr->http_date, 0, MAXSIZE_HTTPDATE);
	memset(httpstruct_ptr->http_body,0,MAXSIZE_HTTPBODY);
	memset(httpstruct_ptr->http_code,0,MAXSIZE_HTTPCODE);
}


/*********************************************************************************
  * Revision History
  *	@file      	http.c
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