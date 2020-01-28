/*
 ********************************************************************************
 * @file      	:	agdatim.c
 * @author		:	Hardware Team, Agtech Labs Inc.
 * @version		:	V2.0.1xx
 * @date		:	11/23/2015
 * @brief		:	SCRAP VERSION!!!!!!!..... but working
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

/* Includes =================================================================*/
#include "agdatim.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "agstring.h"
#include "rtc.h"

/* Functions ================================================================*/
/******************************************************************************
 * @brief	leap year checker
 * @param	year: last 2 digit of year  to be checked (2000+ only)
 * @retval  true/false status
 ******************************************************************************/
static bool agdatimLeapYrChck(uint8_t year)
{
	const int16_t MILLENIA = 2000;
	int16_t varyear;

	varyear = MILLENIA + year;
	if((varyear % 400 == 0) || ((varyear % 4 == 0) && (varyear % 100 != 0)))
		return true;
	else
		return false;
}

/* ============================================================================
 * @brief	subtract 2 datetime
 * @param	datimeA: minuend in rtcdatetime format
 * @param	datimeB: subtrahend in rtcdatetime format
 * @retval  difference in rtcdatetime format
 * ==========================================================================*/
static RTCDateTimeStruct agdatimSubtract(RTCDateTimeStruct datimeA, RTCDateTimeStruct datimeB)
{
	RTCDateTimeStruct timediff;
	memset(&timediff, 0, sizeof(RTCDateTimeStruct));
	timediff.Year = datimeA.Year - datimeB.Year;
	timediff.Month = datimeA.Month - datimeB.Month;
	timediff.Date = datimeA.Date - datimeB.Date;
	timediff.Hours = datimeA.Hours - datimeB.Hours;
	timediff.Minutes = datimeA.Minutes - datimeB.Minutes;
	timediff.Seconds = datimeA.Seconds - datimeB.Seconds;
	timediff.Millis = datimeA.Millis - datimeB.Millis;
	return timediff;
}

/******************************************************************************
 * @brief	comparing 2 datetime
 * @param	datimeA: first datetime in rtcdatetime format
 * @param	datimeB: second datetime in rtcdatetime format
 * @retval  Difference status of first datetime from second datetime
 ******************************************************************************/
AgdatimDiff agdatimCompare(RTCDateTimeStruct datimeA, RTCDateTimeStruct datimeB)
{
	RTCDateTimeStruct timediff;
	uint8_t rtcdt_idx;

	memset(&timediff, 0, sizeof(RTCDateTimeStruct));
	timediff = agdatimSubtract(datimeA, datimeB);
	int32_t diffvalbuf[] = {timediff.Year, timediff.Month, timediff.Date,
							timediff.Hours, timediff.Minutes, timediff.Seconds,
							timediff.Millis};

	for(rtcdt_idx = 0; rtcdt_idx < sizeof(diffvalbuf); rtcdt_idx++)
	{
		if(diffvalbuf[rtcdt_idx] < 0)
			return ADT_NEGATIVE;
		else if(diffvalbuf[rtcdt_idx] > 0)
			return ADT_POSITIVE;
	}
	return ADT_EQUAL;
}

/******************************************************************************
 * @brief	Adding of minute into time with the range of 16bit
 * 			range of -32,768 to 32,768 minutes or -22 to 22 days
 * @param	datime: base datetime value
 * 			operand: adjusting value
 * @retval  adjusted value of datetime
 ******************************************************************************/
RTCDateTimeStruct agdatimMinAdjuster(RTCDateTimeStruct datime, int16_t operand)
{
	RTCDateTimeStruct datime_buffer;
	const int8_t MININHR = 60, HRINDAY = 24, MONTHINYR = 12;
	int8_t dayinmonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30 ,31};
	int8_t month_idx;

	memset(&datime_buffer, 0, sizeof(RTCDateTimeStruct));
	datime_buffer = datime;
	month_idx = datime_buffer.Month;
	//datime_buffer.Year = 16; //<-- leap year testing

	if(agdatimLeapYrChck(datime_buffer.Year)) {
		dayinmonth[1] = 29;
	}

	datime_buffer.Minutes += operand;

	if(datime_buffer.Minutes >= MININHR) {
		datime_buffer.Minutes -= MININHR;
		datime_buffer.Hours++;
	} else if(datime_buffer.Minutes < 0) {
		datime_buffer.Minutes += MININHR;
		datime_buffer.Hours--;
	}

	if(datime_buffer.Hours >= HRINDAY) {
		datime_buffer.Hours -= HRINDAY;
		datime_buffer.Date++;
	} else if(datime_buffer.Hours < 0) {
		datime_buffer.Hours += HRINDAY;
		datime_buffer.Date--;
	}

	if(datime_buffer.Date >= dayinmonth[month_idx]) {
		datime_buffer.Date -= dayinmonth[month_idx];
		datime_buffer.Month++;
	} else if(datime_buffer.Date <= 0) {
		month_idx = (datime_buffer.Month - 1 < 0)
				? MONTHINYR - 1: datime_buffer.Month--;
		datime_buffer.Date = dayinmonth[month_idx];
		datime_buffer.Month--;
	}

	if(datime_buffer.Month >= MONTHINYR)
	{
		datime_buffer.Month -= MONTHINYR;
		datime_buffer.Year++;
	} else if (datime_buffer.Month <= 0) {
		datime_buffer.Month = MONTHINYR;
		datime_buffer.Year--;
	}

	return datime_buffer;
}

/******************************************************************************
 * @brief	api (content) to rtcdatetime conversion
 * @param	datim_ptr: datetime string to be parsed
 *			conv_datim: converted datetime in rtcdatetime format
 * @retval  none
 ******************************************************************************/
void agdatimAPIParser(char * datim_ptr, RTCDateTimeStruct * conv_datim)
{
	const char DELIM[] = "-T:Z";
	char *datim_token;
	uint8_t token_idx;
	enum {YEAR, MONTH, DATE, HOUR, MIN, SEC};

	memset(conv_datim, 0, sizeof(RTCDateTimeStruct));
	datim_token = strtok(datim_ptr, DELIM);
	for(token_idx = 0; token_idx < 6; token_idx++)
	{
		switch(token_idx)
		{
		case YEAR:
			conv_datim->Year = agstringStrToNum(datim_token + 2, 2);
			break;
		case MONTH:
			conv_datim->Month = agstringStrToNum(datim_token, strlen(datim_token));
			break;
		case DATE:
			conv_datim->Date = agstringStrToNum(datim_token, strlen(datim_token));
			break;
		case HOUR:
			conv_datim->Hours = agstringStrToNum(datim_token, strlen(datim_token));
			break;
		case MIN:
			conv_datim->Minutes = agstringStrToNum(datim_token, strlen(datim_token));
			break;
		case SEC:
			conv_datim->Seconds = agstringStrToNum(datim_token, strlen(datim_token));
			break;
		}
		datim_token = strtok(NULL, DELIM);
	}
}

/******************************************************************************
 * @brief	rtcdatetime to api conversion (for url-filter or api content)
 * @param	dt: rtcdatetime format datetime to be parse
 * 			type: type of format (DTEnc_URLFILTER or DTEnc_APICONTENT)
 * @retval  converted string
 ******************************************************************************/
char * agdatimEncoder(RTCDateTimeStruct dt, DTEncType type)
{
	char *dt_delim, *tzone;
	static char encoded_dt[30];

	memset(encoded_dt, '\0', sizeof(encoded_dt));
	switch(type)
	{
	case DTEnc_URLFILTER:
		dt_delim = " ";
		tzone ="\0";
		break;
	default:
		dt_delim = "T";
		tzone = "Z";
		break;
	}
	snprintf(encoded_dt, sizeof(encoded_dt), "%d-%02d-%02d%s%02d:%02d:%02d.%03d%s",
			2000 + dt.Year, dt.Month, dt.Date, dt_delim, dt.Hours, dt.Minutes,
			dt.Seconds, dt.Millis, tzone);
	return encoded_dt;
}

/******************************************************************************
 * @brief	HTTP GET response datetime to rtcdatetime conversion
 * @param	datim_ptr: datetime string (from HTTP Get response) to be parsed
 * 			conv_datim: converted datetime in rtcdatetime format
 * @retval  none
 ******************************************************************************/
void agdatimGETRspParser(char * datim_ptr, RTCDateTimeStruct * conv_datim)
{
	const char DELIM[] = " :,";
	const uint8_t MAXMONTH = 12;
	const char *HTTPMonth[] = {"Nan", "Jan", "Feb", "Mar", "Apr", "May", "Jun",
								"Jul", "Aug","Sep", "Oct", "Nov", "Dec"};
	static char *datim_token,  token_idx;
	uint8_t month_idx;
	enum {WEEKDAY, DATE, MONTH, YEAR, HOUR, MIN, SEC, TIMEZONE};

	memset(conv_datim, 0, sizeof(RTCDateTimeStruct));
	datim_token = strtok(datim_ptr, DELIM);
	for(token_idx = 0; token_idx < 8; token_idx++)
	{
		switch(token_idx)
		{
		case DATE:
			conv_datim->Date = agstringStrToNum(datim_token, strlen(datim_token));
			break;
		case MONTH:
			for(month_idx = 0; month_idx <= MAXMONTH; month_idx++)
			{
				if (!strcmp(datim_token, HTTPMonth[month_idx]))
				{
					conv_datim->Month = month_idx;
					break;
				}
			}
			break;
		case YEAR:
			conv_datim->Year = agstringStrToNum(datim_token+2, (uint8_t)2);
			break;
		case HOUR:
			conv_datim->Hours = agstringStrToNum(datim_token, strlen(datim_token));
			break;
		case MIN:
			conv_datim->Minutes = agstringStrToNum(datim_token, strlen(datim_token));
			break;
		case SEC:
			conv_datim->Seconds = agstringStrToNum(datim_token, strlen(datim_token));
			break;
		}
		datim_token = strtok(NULL, DELIM);
	}
}

/******************************************************************************
 * @brief	api (content) to rtcdatetime conversion
 * @param	datim_ptr: datetime string to be parsed
 *			conv_datim: converted datetime in rtcdatetime format
 * @retval  none
 ******************************************************************************/
void agdatimGpsParser(char * date_ptr, char * time_ptr, RTCDateTimeStruct * conv_datim)
{
	const char DELIM[] = ".";
	char *time_token;
	uint8_t token_idx;
	enum {YEAR, MONTH, DATE, HOUR, MIN, SEC};

	memset(conv_datim, 0, sizeof(RTCDateTimeStruct));
	time_token = strtok(time_ptr, DELIM);
	for(token_idx = 0; token_idx < 2; token_idx++)
	{
		if(token_idx == 1)
			conv_datim->Millis = agstringStrToNum(time_token, strlen(time_token));
		else
		{
			conv_datim->Hours = agstringStrToNum(&time_token[0], 2);
			conv_datim->Minutes = agstringStrToNum(&time_token[2], 2);
			conv_datim->Seconds = agstringStrToNum(&time_token[4], 2);
		}
		time_token = strtok(NULL, DELIM);
	}
	conv_datim->Date = agstringStrToNum(&date_ptr[0], 2);
	conv_datim->Month = agstringStrToNum(&date_ptr[2], 2);
	conv_datim->Year = agstringStrToNum(&date_ptr[4], 2);
}

/******************************************************************************
 * Revision History
 *	@file	agdatim.c
 ******************************************************************************
 * @version	v2.0.1
 * @date	11/13/2015
 * @author	JWF
 * @changes	- initial release
 * 			- internal function:
 * 				+ agdatimLeapYrChck
 * 				+ agdatimSubtract
 * 			- external function:
 * 				+ agdatimCompare
 * 				+ agdatimMinAdjuster
 * 				+ agdatimAPIParser
 * 				+ agdatimAPIEncoder
 * 				+ agdatimGETRspParser
 ******************************************************************************
 */
