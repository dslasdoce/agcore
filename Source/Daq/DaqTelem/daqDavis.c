/********************************************************************************
  * @file      	daqDavois.h
  * @author		Hardware Team
  * @version	v2.2.0
  * @date		04/08/16
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
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "stdint.h"

#include "agBoardInit.h"
#include "utilMem.h"
#include "daqPrintUtil.h"
#include "productCfg.h"
#include "loggerMain.h"
#include "rtc.h"
#include "daqDavis.h"
#include "delay.h"
#include "port.h"
#include "isrMain.h"
#include "gpio.h"
#include "utilCalc.h"

static uint8_t davisDTHReadBytes(AgTechIOStruct *io_data, AgTechIOStruct *io_clk);
static void davisDTHInitLines(AgTechIOStruct *io_data, AgTechIOStruct *io_clk);
static bool davisWaitUntilTimeout(void);
static bool tx_start(AgTechIOStruct *io_data, AgTechIOStruct *io_clk);
static ReturnStatusEnum davisDTHReadBytes2(uint16_t *databyte, AgTechIOStruct *io_data, AgTechIOStruct *io_clk);
static bool davisDTHSendCmd(uint8_t cmd, AgTechIOStruct *io_data, AgTechIOStruct *io_clk);
static ReturnStatusEnum davisDTHReadSR(uint8_t *databyte, AgTechIOStruct *io_data, AgTechIOStruct *io_clk);
static ReturnStatusEnum davisDTHWriteSR(uint8_t databyte, AgTechIOStruct *io_data, AgTechIOStruct *io_clk);


#define IS_PACKET_ID_VALID(packet_id)   ((packet_id == DAVIS_UV_DATA) || \
                                        (packet_id == DAVIS_RAIN_RATE) || \
                                        (packet_id == DAVIS_SOLAR_RADN) || \
                                        (packet_id == DAVIS_TEMP) || \
                                        (packet_id == DAVIS_HUMIDITY) || \
					(packet_id == DAVIS_RAIN))

#define	DTH_COEF_TP_14bit			((float)0.01)
#define	DTH_COEF_TP_12bit			((float)0.04)

#define	DTH_COEF_RH_12bit_C1		((double)-2.0468)
#define	DTH_COEF_RH_12bit_C2		((double)0.0367)
//#define	DTH_COEF_RH_12bit_C3		((float)-1.5955E-6)
#define	DTH_COEF_RH_12bit_C3		((double)-1.5955E-6)


/* Global Declarations =======================================================*/
static bool davis_stat;

/* External Declarations =====================================================*/
//extern PortConfigStruct PortConfig;
bool davis_resetflags;
uint8_t davis_weather_port;
/* Functions =================================================================*/

/*==============================================================================
 * @brief
 * @param
 * @retval
==============================================================================*/
void davisUpdateStat(bool stat)
{
	davis_stat = stat;
}

/*==============================================================================
 * @brief
 * @param
 * @retval
==============================================================================*/
bool davisIsComplete(void)
{
	return davis_stat;
}

/*==============================================================================
 * @brief
 * @param
 * @retval
==============================================================================*/
SensorRegistrationStruct *davisGetPortDetails(void)
{
	if(PortConfig.P1.interface == IFC_DAVIS_ISS)
	{
		return &PortConfig.P1;
	}
	else if(PortConfig.P2.interface == IFC_DAVIS_ISS)
	{
		return &PortConfig.P2;
	}
	else if(PortConfig.P3.interface == IFC_DAVIS_ISS)
	{
		return &PortConfig.P3;
	}
	else if(PortConfig.P4.interface == IFC_DAVIS_ISS)
	{
		return &PortConfig.P4;
	}
	else
		return NULL;
}

/*==============================================================================
 * @brief
 * @param
 * @retval
==============================================================================*/
void davis(DavisMainStruct *packet_ptr, uint8_t port)
{
	static bool is_uv = true;
	static bool is_rain_rate = false;
	static bool is_rain = false;
	static bool is_solar = false;
	static bool is_temp = false;
	static bool is_humidity = false;
	static bool is_wind_dir = false;
	static bool is_wind_speed = false;
	bool will_log = true;
	uint8_t *addr_step_ptr = NULL;
	volatile uint16_t temp_data = 0;
	float wind_dir = 0;
	volatile uint16_t temperature;
	float uv_data = 0;
	float rain_rate = 0;
	float rain = 0;
	float solar_radiation = 0;
	float humidity = 0;
	TelemetryStruct *telem_data_ptr;
	RTCDateTimeStruct date_time;
    	uint8_t stat;
	addr_step_ptr = (uint8_t*)packet_ptr;

	if(davis_resetflags)
	{
		/*is_uv = */is_rain = is_rain_rate = is_solar = is_temp = 
						is_humidity = is_wind_dir =
						is_wind_speed = false;
		davis_resetflags = false;
	}
	if(is_uv & is_rain & is_rain_rate & is_solar & is_temp &
		is_humidity & is_wind_dir & is_wind_speed)
	{
		davisUpdateStat(true);
		delayMillis(20);
		return;
	}

	packet_ptr->wind_speed = ((float)(packet_ptr->wind_speed/2.25));
	
	if(!is_wind_speed)
	{

		if(packet_ptr->packet_ID != DAVIS_EXT_DATAPACKET)
		{
		  	is_wind_speed = true;
		  	telem_data_ptr = (TelemetryStruct *)loggerWrRecToRingBuff(&stat, 1, TELEMETRY);
			telem_data_ptr->raw_uom = UOM_MPH;
			telem_data_ptr->raw_value = packet_ptr->wind_speed;
			telem_data_ptr->base_uom = UOM_MPH;
			telem_data_ptr->base_value =packet_ptr->wind_speed;
			telem_data_ptr->sensor_type = ST_WS;
			telem_data_ptr->port = port;
			telem_data_ptr->depth_uom = UOM_IN;
			telem_data_ptr->depth_value = 0x00;
			telem_data_ptr->sensor_code = SC_WS_WD;
			telem_data_ptr->rad_modbus_id = DEVICE_INFO->rad_modbus_id;
			rtcGetDateTime(&date_time);
			telem_data_ptr->date_time = date_time;
			
			daqPrint(telem_data_ptr->raw_value, telem_data_ptr->raw_uom,
						 telem_data_ptr->base_value, telem_data_ptr->base_uom,
						 telem_data_ptr->depth_value, telem_data_ptr->depth_uom,
						 telem_data_ptr->sensor_code, telem_data_ptr->sensor_type);
		}
	}

	temp_data = ((((uint16_t)packet_ptr->wind_dir) << 1) |
					(((*(addr_step_ptr + 4)) & 0x02) >> 1));

	if(!is_wind_dir)
	{
		if(temp_data == 0)
			wind_dir = 360;
		else if (temp_data == 1)
			wind_dir = 5;
		else if(temp_data > 1 && temp_data < 509)
			wind_dir = 8 + (float)(temp_data - 1)*344/509.4;
		else
			wind_dir = 355;
        
		//++wsctr;
		if(packet_ptr->packet_ID != DAVIS_EXT_DATAPACKET)
		{
		  	is_wind_dir = true;
		  	telem_data_ptr = (TelemetryStruct *)loggerWrRecToRingBuff(&stat, 1, TELEMETRY);
			telem_data_ptr->raw_uom = UOM_DEG;
			telem_data_ptr->raw_value = wind_dir;
			telem_data_ptr->base_uom = UOM_DEG;
			telem_data_ptr->base_value = wind_dir;
			telem_data_ptr->sensor_type = ST_WD;
			telem_data_ptr->port = port;
			telem_data_ptr->depth_uom = UOM_IN;
			telem_data_ptr->depth_value = 0x00;
			telem_data_ptr->sensor_code = SC_WS_WD;
			telem_data_ptr->rad_modbus_id = DEVICE_INFO->rad_modbus_id;
			rtcGetDateTime(&date_time);
			telem_data_ptr->date_time = date_time;
			daqPrint(telem_data_ptr->raw_value, telem_data_ptr->raw_uom,
						 telem_data_ptr->base_value, telem_data_ptr->base_uom,
						 telem_data_ptr->depth_value, telem_data_ptr->depth_uom,
						 telem_data_ptr->sensor_code, telem_data_ptr->sensor_type);
		}
	}
    
    if(!IS_PACKET_ID_VALID(packet_ptr->packet_ID))
        return;
    //telem_data_ptr = (TelemetryStruct *)loggerWrRecToRingBuff(&stat, 1, TELEMETRY);
	switch(packet_ptr->packet_ID)
	{
		case DAVIS_UV_DATA: //UOM: UV Index
			if (is_uv)
				will_log = false;
			else
			{
			 	telem_data_ptr = (TelemetryStruct *)loggerWrRecToRingBuff(&stat, 1, TELEMETRY);
				if (packet_ptr->PacketDataUnion.uv.sensor_type == 0)
					uv_data = ((((uint16_t)packet_ptr->PacketDataUnion.uv.uv_ad)
							  << 2) + (packet_ptr->PacketDataUnion.uv.uv_ad_ext))*
								((float)3/1024)*(16/2.4);
				else
					uv_data = (packet_ptr->PacketDataUnion.uv.uv_ad)*
							  ((float)3/1024)*(16/2.4);
				telem_data_ptr->sensor_code = SC_UV;
				telem_data_ptr->raw_uom = UOM_MEDS;
				telem_data_ptr->raw_value = uv_data;
				telem_data_ptr->base_uom = UOM_MEDS;
				telem_data_ptr->base_value = uv_data;
				telem_data_ptr->sensor_type = ST_UV;
				is_uv = true;
			}
			break;
		case DAVIS_RAIN_RATE: //UOM: tips/hr
			if(is_rain_rate)
				will_log = false;
			
			else
			{
			 	telem_data_ptr = (TelemetryStruct *)loggerWrRecToRingBuff(&stat, 1, TELEMETRY);
				if (packet_ptr->PacketDataUnion.rain_rate.rts == 0)
					rain_rate = (float)57600/
								(packet_ptr->PacketDataUnion.rain_rate.rain_rate_ext |
								packet_ptr->PacketDataUnion.rain_rate.rain_rate << 8);
				else
					rain_rate = (float)3600/
								(packet_ptr->PacketDataUnion.rain_rate.rain_rate_ext |
								packet_ptr->PacketDataUnion.rain_rate.rain_rate << 8);
				telem_data_ptr->sensor_code = SC_RG;
				telem_data_ptr->raw_uom = UOM_IN_HR;
				telem_data_ptr->raw_value = rain_rate;
				telem_data_ptr->base_uom = UOM_IN_HR;
				telem_data_ptr->base_value = roundf(rain_rate)/100;
				telem_data_ptr->sensor_type = ST_RG;
				is_rain_rate = true;
			}
			break;

		case DAVIS_SOLAR_RADN: //UOM: W/m^2
			if(is_solar)
				will_log = false;
			
			else
			{
			 	telem_data_ptr = (TelemetryStruct *)loggerWrRecToRingBuff(&stat, 1, TELEMETRY);
				if (packet_ptr->PacketDataUnion.solar_radn.sensor_type == 0)
				{
						temp_data = (uint16_t)(
									packet_ptr->
									PacketDataUnion.solar_radn.solar_radn_ad << 2 |
									packet_ptr->
									PacketDataUnion.solar_radn.solar_radn_ad_ext >> 2); //10bit MSB, 2bit set to 00
						solar_radiation = (float)temp_data*1800/1024;

				}
				else
				{
					temp_data = (uint16_t)(
									packet_ptr->
									PacketDataUnion.solar_radn.solar_radn_ad << 4 |
									packet_ptr->
									PacketDataUnion.solar_radn.solar_radn_ad_ext); //10bit MSB, 2bit set to 00
					solar_radiation = temp_data & ~(1<<12);
				}

				telem_data_ptr->sensor_code = SC_SR;
				telem_data_ptr->raw_uom = UOM_W_M2;
				telem_data_ptr->raw_value = solar_radiation;
				telem_data_ptr->base_uom = UOM_W_M2;
				telem_data_ptr->base_value = solar_radiation;
				telem_data_ptr->sensor_type = ST_SR;
				is_solar = true;
			}
			break;

		case DAVIS_TEMP: //UOM: Digital temp >> already converted to degree F x 10
			if(is_temp)
				will_log = false;
			else
			{
			 	telem_data_ptr = (TelemetryStruct *)loggerWrRecToRingBuff(&stat, 1, TELEMETRY);
				if (packet_ptr->PacketDataUnion.temp.sensor_type == 0)
					temperature = ((((uint16_t)packet_ptr->
								  PacketDataUnion.temp.temp_ad) << 2)
								  + ((packet_ptr->PacketDataUnion.
								  temp.temp_ad_ext) & 0x0c)); //10 MSB, 2bit set to 00
				else
					temperature = ((((uint16_t)packet_ptr->
								  PacketDataUnion.temp.temp_ad) << 4)
								  + (packet_ptr->PacketDataUnion.
								  temp.temp_ad_ext)); //already converter to degree Fx10 (12bit)
				telem_data_ptr->sensor_code = SC_AT;
				telem_data_ptr->raw_uom = UOM_C;
				telem_data_ptr->raw_value = ( (float)temperature/10 - 32)*5/9;
				telem_data_ptr->base_uom = UOM_F;
				telem_data_ptr->base_value = (float)temperature/10;
				telem_data_ptr->sensor_type = ST_TP;
				is_temp = true;
			}
			break;

		case DAVIS_MAX_WIND: // revolution/2.25 seconds
			will_log = false;
			break;

		case DAVIS_HUMIDITY: //percentage
			if(is_humidity)
				will_log = false;
			
			else
			{
			 	telem_data_ptr = (TelemetryStruct *)loggerWrRecToRingBuff(&stat, 1, TELEMETRY);
				if (packet_ptr->PacketDataUnion.humidity.sensor_type == 1)
					humidity = (((uint16_t)packet_ptr->PacketDataUnion.
							   humidity.humidity_ext) << 8) + (packet_ptr->
							   PacketDataUnion.humidity.humidity);
				telem_data_ptr->sensor_code = SC_RH;
				telem_data_ptr->raw_uom = UOM_PERCENT;
				telem_data_ptr->raw_value = humidity/10;
				telem_data_ptr->base_uom = UOM_PERCENT;
				telem_data_ptr->base_value = humidity/10;
				telem_data_ptr->sensor_type = ST_RH;
				is_humidity = true;
			}
			break;

		case DAVIS_RAIN:
		  	if(is_rain)
				will_log = false;
			else
			{
			  	telem_data_ptr = (TelemetryStruct *)loggerWrRecToRingBuff(&stat, 1, TELEMETRY);
				rain = packet_ptr->PacketDataUnion.rainfall.rain_count;
			  	telem_data_ptr->sensor_code = SC_RG;
				telem_data_ptr->raw_uom = UOM_PULSE;
				telem_data_ptr->raw_value = rain;
				telem_data_ptr->base_uom = UOM_PULSE;
				telem_data_ptr->base_value = roundf(rain)/100;
				telem_data_ptr->sensor_type = ST_RG;
			  	is_rain = true;
			}
			break;

		case DAVIS_EXT_DATAPACKET:
			will_log = false;
			break;

		default:
			//no data packet, unknown packet id, (error detection)
			will_log = false;
			break;
	}


    //telem_data_ptr = (TelemetryStruct *)loggerWrRecToRingBuff(&stat, 1, TELEMETRY);
    //loggerWrRecToRingBuff((uint32_t *)telem_data_ptr, 1, TELEMETRY);
	if(will_log)
	{
		telem_data_ptr->port = port;
		telem_data_ptr->depth_uom = UOM_IN;
		telem_data_ptr->depth_value = 0x00;
		rtcGetDateTime(&date_time);
		telem_data_ptr->date_time = date_time;
		telem_data_ptr->backup_flag = false;
		telem_data_ptr->unsent_flag = true;
		telem_data_ptr->rad_modbus_id = DEVICE_INFO->rad_modbus_id;
		daqPrint(telem_data_ptr->raw_value, telem_data_ptr->raw_uom,
				 telem_data_ptr->base_value, telem_data_ptr->base_uom,
				 telem_data_ptr->depth_value, telem_data_ptr->depth_uom,
				 telem_data_ptr->sensor_code, telem_data_ptr->sensor_type);
	}
	//free(telem_data_ptr);

}

ReturnStatusEnum davisDTHReadTemp(float *temp_c, AgTechIOStruct *io_data, AgTechIOStruct *io_clk)
{
		uint32_t t_start = 0;
		uint16_t databyte = 0;
		uint8_t cmd = 0x03;
		uint8_t sr = 0;
		ReturnStatusEnum status = FAILED;
		delayMillis(200);
		//davisDTHWriteSR(0x01, io_data, io_clk);
		if(davisDTHReadSR(&sr, io_data, io_clk) != SUCCESSFUL)
		{
		  	HAL_NVIC_DisableIRQ(TIM4_IRQn);
		  	return FAILED;
		}
			
		davisDTHInitLines(io_data, io_clk);
		delayMillis(100);
		
		for(uint8_t i = 0; i < 5; i++)
		{
			if (tx_start(io_data, io_clk))
			{
				if(davisDTHSendCmd(cmd, io_data, io_clk))
				{
					if (davisDTHReadBytes2(&databyte, io_data, io_clk) == SUCCESSFUL)
					{
						if( (sr & 0x01) == 0)
						{
							databyte &= 0x3FFF;
							*temp_c = (float)databyte*DTH_COEF_TP_14bit - 40.1;
						}
						else
						{
							databyte &= 0x0FFF;
							*temp_c = (float)databyte*DTH_COEF_TP_12bit - 40.1;
						}
						status = SUCCESSFUL;
						break;
					}
				}
			}
		}
		HAL_NVIC_DisableIRQ(TIM4_IRQn);
		return status;
}

ReturnStatusEnum davisDTHReadRelHum(float *relhum, AgTechIOStruct *io_data, AgTechIOStruct *io_clk)

//						databyte &= 0x0FFF;
//						*relhum =  (double)-2.0468 
//								  + (double)0.0367*(double)databyte ;
{
		uint32_t t_start = 0;
		uint16_t databyte = 0;
		uint8_t cmd = 0x05;
		uint8_t sr = 0;
		ReturnStatusEnum status = FAILED;
		delayMillis(200);
		//davisDTHWriteSR(0x01, io_data, io_clk);
		if(davisDTHReadSR(&sr, io_data, io_clk) != SUCCESSFUL)
		{
		  	HAL_NVIC_DisableIRQ(TIM4_IRQn);
		  	return FAILED;
		}
				
		davisDTHInitLines(io_data, io_clk);
		delayMillis(1);
		
		for(uint8_t i = 0; i < 5; i++)
		{
			if (tx_start(io_data, io_clk))
			{
				if(davisDTHSendCmd(cmd, io_data, io_clk))
				{
					if (davisDTHReadBytes2(&databyte, io_data, io_clk) == SUCCESSFUL)
					{
						if( (sr & 0x01) == 0)
						{
							databyte &= 0x0FFF;
							*relhum = (float)(DTH_COEF_RH_12bit_C1 
									  + DTH_COEF_RH_12bit_C2*databyte 
									  + DTH_COEF_RH_12bit_C3*pow(databyte, 2));
						}
						else
						{
							databyte &= 0x00FF;
							//*temp_c = (float)databyte*DTH_COEF_TP_12bit - 40.1;
						}
						status = SUCCESSFUL;
						break;
					}
				}
			}
		}
		
		*relhum = utilCalcRoundDec(*relhum, 2);
		HAL_NVIC_DisableIRQ(TIM4_IRQn);
		gpioDettachInt(io_clk);
		return status;
}

static void davisDTHInitLines(AgTechIOStruct *io_data, AgTechIOStruct *io_clk)
{
	HAL_NVIC_DisableIRQ(TIM4_IRQn);
	gpioDigitalWrite(io_clk, 0);
	davisdth_sck_state = 1;
	gpioAttachInt(io_clk, IT_MODE_RISING);
	gpioMode(io_data, MODE_OUTPUT);
	gpioDigitalWrite(io_data, 1);
	delayMillis(100);
	HAL_NVIC_EnableIRQ(TIM4_IRQn);
	delayMillis(1);
}
#define WAITFORTIMEOUT()		if(!davisWaitUntilTimeout()) return 0;
static bool davisWaitUntilTimeout(void)
{
  	uint32_t t_start = HAL_GetTick();
	davisdth_flasgset = false;
	while(!davisdth_flasgset)
	{
	  	if(davisdth_flasgset == true)
		{
		  	davisdth_flasgset = false;
		  	return true;
		}
		
		if (delayTimeDiff(t_start, HAL_GetTick()) > 10)
			return false;
	}
}

static bool tx_start(AgTechIOStruct *io_data, AgTechIOStruct *io_clk)
{
  	HAL_NVIC_EnableIRQ(TIM4_IRQn);
  	gpioDettachInt(io_clk);
	gpioAttachInt(io_clk, IT_MODE_RISING);
	WAITFORTIMEOUT();
  	delayMicros(25);				//wait for clock to center
	/*Transmission Start Sequence*/
	gpioDigitalWrite(io_data, 0);	//pull DATA low
	davisWaitUntilTimeout();
	delayMicros(25);			//wait for 1 period
	gpioDigitalWrite(io_data, 1);	//pull data HIGH wait for 1 period

	gpioDettachInt(io_clk);
	gpioAttachInt(io_clk, IT_MODE_FALLING);
	davisWaitUntilTimeout();
	delayMicros(25);
	gpioDigitalWrite(io_data, 0);	//pull DATA low
	
//	gpioDettachInt(io_clk);
//	gpioAttachInt(io_clk, IT_MODE_RISING);

	HAL_NVIC_DisableIRQ((IRQn_Type)TIM4_IRQn);
//	gpioDigitalWrite(io_clk, 0);
//	davisdth_sck_state = 1;
	delayMicros(200);
	HAL_NVIC_EnableIRQ((IRQn_Type)TIM4_IRQn);
}

static bool davisDTHSendCmd(uint8_t cmd, AgTechIOStruct *io_data, AgTechIOStruct *io_clk)
{
  	bool status = false;
	for(uint8_t i = 0x40; i > 0; i >>= 1)
	{
		davisWaitUntilTimeout();
		delayMicros(25);
		if( i & cmd )
		  gpioDigitalWrite(io_data, HIGH);
		else
		  gpioDigitalWrite(io_data, LOW);
	}
	
	//delayMicros(40);
	gpioDettachInt(io_clk);
	gpioAttachInt(io_clk, IT_MODE_FALLING);
	davisWaitUntilTimeout();
	gpioMode(io_data, MODE_INPUT);
	//while(gpioDigitalRead(io_clk));
	delayMicros(25);
			
	status = !gpioDigitalRead(io_data);
	davisWaitUntilTimeout();
	return status;
}

static ReturnStatusEnum davisDTHReadBytes2(uint16_t *databyte, AgTechIOStruct *io_data, AgTechIOStruct *io_clk)
{
  	uint32_t t_start = 0;
	uint8_t databyte0 = 0;
	uint8_t databyte1 = 0;
	ReturnStatusEnum stat = FAILED;
	
	HAL_NVIC_DisableIRQ(TIM4_IRQn);
	gpioDigitalWrite(io_clk, 0);
	davisdth_sck_state = 1;
	
	t_start = HAL_GetTick();
	while(gpioDigitalRead(io_data))
	{	
		if (delayTimeDiff(t_start, HAL_GetTick()) > 1000)
			return 0;
	}
	
	davisdth_flasgset = false;
	gpioAttachInt(io_clk, IT_MODE_FALLING);
	__HAL_TIM_CLEAR_IT(&timer4_handle, TIM_IT_UPDATE);
	timer4_handle.Instance->CNT = 0;
	HAL_NVIC_EnableIRQ(TIM4_IRQn);
	
	for(uint8_t i = 0; i < 8; i++)
	{
		databyte1 |= (gpioDigitalRead(io_data) << (7 - i));
		davisWaitUntilTimeout();
		delayMicros(25);
	}
	
	gpioMode(io_data, MODE_OUTPUT);
	gpioDigitalWrite(io_data, 0);
	gpioDettachInt(io_clk);
	gpioAttachInt(io_clk, IT_MODE_RISING);
	davisWaitUntilTimeout();

	
	gpioDettachInt(io_clk);
	gpioAttachInt(io_clk, IT_MODE_FALLING);
	davisWaitUntilTimeout();
	gpioMode(io_data, MODE_INPUT);
	
	for(uint8_t i = 0; i < 8; i++)
	{
		
		delayMicros(25);
		databyte0 |= (gpioDigitalRead(io_data) << (7 - i));
		davisWaitUntilTimeout();
	}
	gpioDettachInt(io_clk);
	*databyte = (databyte1 << 8) | databyte0;
	stat = SUCCESSFUL;
	return stat;
}

static uint8_t davisDTHReadBytes(AgTechIOStruct *io_data, AgTechIOStruct *io_clk)
{
  	uint32_t t_start = 0;
	uint8_t databyte = 0;
	
	HAL_NVIC_DisableIRQ(TIM4_IRQn);
	gpioDigitalWrite(io_clk, 0);
	davisdth_sck_state = 1;
	
	t_start = HAL_GetTick();
	while(gpioDigitalRead(io_data))
	{	
		if (delayTimeDiff(t_start, HAL_GetTick()) > 1000)
			return 0;
	}
	
	davisdth_flasgset = false;
	gpioAttachInt(io_clk, IT_MODE_FALLING);
	__HAL_TIM_CLEAR_IT(&timer4_handle, TIM_IT_UPDATE);
	timer4_handle.Instance->CNT = 0;
	HAL_NVIC_EnableIRQ(TIM4_IRQn);
	
	for(uint8_t i = 0; i < 8; i++)
	{
		databyte |= (gpioDigitalRead(io_data) << (7 - i));
		davisWaitUntilTimeout();
		delayMicros(25);
	}
	
	gpioDettachInt(io_clk);
	gpioAttachInt(io_clk, IT_MODE_RISING);
	davisWaitUntilTimeout();
	gpioMode(io_data, MODE_OUTPUT);
	gpioDigitalWrite(io_data, 0);
	
	gpioDettachInt(io_clk);
	gpioAttachInt(io_clk, IT_MODE_FALLING);
	davisWaitUntilTimeout();
	gpioMode(io_data, MODE_INPUT);
	
	gpioDettachInt(io_clk);
	
	return databyte;
}

static ReturnStatusEnum davisDTHReadSR(uint8_t *databyte, AgTechIOStruct *io_data, AgTechIOStruct *io_clk)
{
		*databyte = 0;
		uint8_t cmd = 0x07;
		uint32_t t_start = 0;
		ReturnStatusEnum stat = FAILED;
		
		davisDTHInitLines(io_data, io_clk);
		delayMillis(1);
		if (tx_start(io_data, io_clk))
		{
			if(davisDTHSendCmd(cmd, io_data, io_clk))
			{
				*databyte = davisDTHReadBytes(io_data, io_clk);	
				stat = SUCCESSFUL;
			}
		}
		gpioDettachInt(io_clk);
		return stat;
}

static ReturnStatusEnum davisDTHWriteSR(uint8_t databyte, AgTechIOStruct *io_data, AgTechIOStruct *io_clk)
{
		uint8_t cmd = 0x06;
		uint32_t t_start = 0;
		ReturnStatusEnum stat = FAILED;
		
		davisDTHInitLines(io_data, io_clk);
		delayMillis(1);
		tx_start(io_data, io_clk);

		if(davisDTHSendCmd(cmd, io_data, io_clk))
		{
			for(uint8_t i = 0x80; i > 0; i >>= 1)
			{
				davisWaitUntilTimeout();
				delayMicros(25);
				if( i & databyte )
				  gpioDigitalWrite(io_data, HIGH);
				else
				  gpioDigitalWrite(io_data, LOW);
			}
		}

		return stat;
}
/*******************************************************************************
  * Revision History
  * @file         daqDavis.c
  ******************************************************************************
  * @version	v2.2.0
  * @date		04/08/16
  * @author		D. Lasdoce
  * @changes     - created file
  ******************************************************************************
  */