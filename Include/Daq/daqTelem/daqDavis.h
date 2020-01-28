/********************************************************************************
  * @file:			daqDavis.h
  * @author:		Agtech Hardware
  * @version	v2.2.0
  * @date		04/08/16
  * @brief:			Davis Data Module Packets.
  ******************************************************************************
  * @attention
  *
  * COPYRIGHT(c) 2015 AgTech Labs, Inc.
  *
  * are permitted provided that the following conditions are met:
  * Redistribution and use in source and binary forms, with or without modification,
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


#ifndef __DAQDAVIS_H_
#define __DAQDAVIS_H_

#include <stdint.h>
#include "gpio.h"
#include "equations.h"

/* External Declarations =====================================================*/
extern bool davis_resetflags;
extern uint8_t davis_weather_port;
/* Typedefs ==================================================================*/
typedef struct
{
	uint8_t			uv_ad;
	uint8_t			w_state			:1;
	uint8_t			dir				:1;
	uint8_t			w_invalid		:1;
	uint8_t			sensor_type		:1;
	uint8_t			uv_res_LSB		:2;
	uint8_t			uv_ad_ext		:2;
	uint8_t			reserved;
}UvPacketStruct;

typedef struct
{
	uint8_t			rain_rate;
	uint8_t			w_state			:1;
	uint8_t			dir				:1;
	uint8_t			w_invalid		:1;
	uint8_t			reserved_1		:1;
	uint8_t			rain_rate_ext	:2;
	uint8_t			rts				:1;
	uint8_t			reserved_2		:1;
	uint8_t			reserved;

}RainRatePacketStruct;

typedef struct
{
	uint8_t			solar_radn_ad;
	uint8_t			w_state			:1;
	uint8_t			dir				:1;
	uint8_t			w_invalid		:1;
	uint8_t			sensor_type		:1;
	uint8_t			solar_radn_ad_ext:4;
	uint8_t			reserved;

}SolarRadiationPacketStruct;

typedef struct
{
	uint8_t			temp_ad;
	uint8_t			w_state			:1;
	uint8_t			dir				:1;
	uint8_t			w_invalid		:1;
	uint8_t			sensor_type		:1;
	uint8_t			temp_ad_ext		:4;
	uint8_t			reserved;

}TemperaturePacketStruct;

typedef struct
{
	uint8_t			max_wind_speed;
	uint8_t			w_state			:1;
	uint8_t			dir				:1;
	uint8_t			w_invalid		:1;
	uint8_t			reserved		:5;
	uint8_t			dir_sector		:4;
	uint8_t			age_max_wind	:4;

}MaxWindSpeedPacketStruct;

typedef struct
{
	uint8_t			humidity;
	uint8_t			w_state			:1;
	uint8_t			dir				:1;
	uint8_t			w_invalid		:1;
	uint8_t			sensor_type		:1;
	uint8_t			humidity_ext	:4;
	uint8_t			reserved;

}HumidityMeasurementPacketStruct;

typedef struct
{
	uint8_t			rain_count		:7;
	uint8_t			rain_reset		:1;
	uint8_t			w_state			:1;
	uint8_t			dir				:1;
	uint8_t			w_invalid		:1;
	uint8_t			reserved_1		:5;
	uint8_t			reserved;

}RainPacketStruct;

typedef enum
{
	DAVIS_UV_DATA		= 0x04,
	DAVIS_RAIN_RATE	= 0x05,
	DAVIS_SOLAR_RADN	= 0x06,
	DAVIS_TEMP		= 0x08,
	DAVIS_MAX_WIND	= 0x09,
	DAVIS_HUMIDITY	= 0x0A,
	DAVIS_RAIN		= 0x0E,
	DAVIS_EXT_DATAPACKET	=	0x0F

}DavisWeatherIDEnum;

typedef struct
{
	uint8_t						unit_id 		:3;
	uint8_t						battery 		:1;
	DavisWeatherIDEnum			packet_ID 		:4;
	uint8_t						wind_speed;
	uint8_t						wind_dir;

	union
	{
		UvPacketStruct						uv;
		RainRatePacketStruct 				rain_rate;
		SolarRadiationPacketStruct 			solar_radn;
		TemperaturePacketStruct 			temp;
		MaxWindSpeedPacketStruct 			max_wind;
		HumidityMeasurementPacketStruct 	humidity;
		RainPacketStruct					rainfall;
	}PacketDataUnion;

	uint8_t			crc_msb;
	uint8_t			crc_lsb;

}DavisMainStruct;

/* Function Prototypes ======================================================*/
void davisUpdateStat(bool stat);
bool davisIsComplete(void);
void davis(DavisMainStruct *packet_ptr, uint8_t port);
ReturnStatusEnum davisDTHReadTemp(float *temp_c, AgTechIOStruct *io_data, 
								  AgTechIOStruct *io_clk);
ReturnStatusEnum davisDTHReadRelHum(float *relhum, AgTechIOStruct *io_data, 
								AgTechIOStruct *io_clk);
SensorRegistrationStruct *davisGetPortDetails();
AgTechIOStruct *davisGetEnablePin(SensorRegistrationStruct *davis_config);

#endif

/*******************************************************************************
  * Revision History
  * @file       daqDavis.h  
  ******************************************************************************
  * @version	v2.2.0
  * @date		04/08/16
  * @author		D. Lasdoce
  * @changes	- created file
  ******************************************************************************
  */
