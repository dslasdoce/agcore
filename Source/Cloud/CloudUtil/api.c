/********************************************************************************
 * @file      	:	api.c
 * @author		:	Hardware Team
 * @version		:	v2.0.2
 * @date		:	04/08/16
 * @brief		:	Module of Post/Get to cloud server
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
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "productCfg.h"
#include "agBoardinit.h"
#include "api.h"
#include "utilMem.h"
#include "loggerMain.h"
#include "agBoardInit.h"
#include "tempApiCodedList.h"
#include "port.h"
#include "daqPrintUtil.h"

uint32_t temp_posted_rec_cnt;
/* Definition ===============================================================*/
#define GEOMTYPE		"Point"
#define GEOJSONTYPE		"Feature"

/* External Declarations ====================================================*/
ValveSchedAPIStruct vlvsched_api;
RTCDateTimeStruct vlv_datexecuted;

/* Global Declarations ======================================================*/
HTTPStruct httpstruct;
//TelemetryStruct cloud_telem_buff[MAX_REC_PER_POST];
//SystemStatusStruct cloud_systat_buff[MAX_REC_PER_POST];

/* MACROS ===================================================================*/

/* Typedefs =================================================================*/

/* Functions ================================================================*/
/* API FORMATTING */
#if DEVICE_TYPE == DEVICETYPE_GATE
#pragma optimize=none
static bool checkSensorCode(uint16_t sensor_code)
{
  	uint32_t i = 0;
	while(SENSOR_CODE_LIST[i].sc_code != 0xFFFF)
	{
		if(SENSOR_CODE_LIST[i].sc_code == sensor_code)
			return true;
		i++;
	}
	return false;
}

#pragma optimize=none
static bool checkSensorType(uint8_t sensor_type)
{
  	uint32_t i = 0;
	while(SENSOR_TYPE_LIST[i].hexcode != 0xFF)
	{
		if(SENSOR_TYPE_LIST[i].hexcode == sensor_type)
			return true;
		i++;
	}
	return false;
}

#pragma optimize=none
static bool checkSensorUom(uint8_t sensor_uom)
{
  	uint32_t i = 0;
	while(SENSOR_UOM_LIST[i].hexcode != 0xFF)
	{
		if(SENSOR_UOM_LIST[i].hexcode == sensor_uom)
			return true;
		i++;
	}
	return false;
}

static bool checkDeviceSN(uint16_t modbus_id)
{
	for(uint8_t i = 0; i <= NODE_COUNT; i++)
	{
		if(DEVICE_INFO_LIST[i].rad_modbus_id == modbus_id
		   && modbus_id != MODBUS_DEFAULT)
			return true;
	}
	
	#if PRINT_PROCESSES_IN_UART
		char output[50];
		sprintf(output, "\n\nERROR: DeviceSN with Modbus: 0x%02X Does Not Exist\n\n"
					, modbus_id);
		uartSHOW((uint8_t*)output, strlen(output));
	#endif
	return false;
}

#pragma optimize=none
static bool dataFilter(uint8_t modbus_id, uint8_t port, uint16_t sensor_code
					   ,uint8_t sensor_type, uint8_t sensor_uom)
{
	if(checkDeviceSN(modbus_id) != true)
		return false;
	if((port < 1) || (port > 4))
		return false;
	if( checkSensorCode(sensor_code) != true)
		return false;
	if( checkSensorType(sensor_type) != true)
		return false;
	if( checkSensorUom(sensor_uom) != true)
		return false;
	return true;
}

#pragma optimize=none
static const char *getSensorCode(uint32_t sensor_code)
{
	uint32_t i = 0;
	while(SENSOR_CODE_LIST[i].sc_code != 0xFFFF)
	{
		if(SENSOR_CODE_LIST[i].sc_code == sensor_code)
			return SENSOR_CODE_LIST[i].sc_name;
		i++;
	}
	assert(0);
}

#pragma optimize=none
static const char *getSensorType(uint8_t sensor_type)
{
	uint32_t i = 0;
	while(SENSOR_TYPE_LIST[i].hexcode != 0xFF)
	{
		if(SENSOR_TYPE_LIST[i].hexcode == sensor_type)
			return SENSOR_TYPE_LIST[i].name;
		i++;
	}
	assert(0);
}

#pragma optimize=none
static const char *getSensorUom(uint8_t sensor_uom)
{
	uint32_t i = 0;
	while(SENSOR_UOM_LIST[i].hexcode != 0xFF)
	{
		if(SENSOR_UOM_LIST[i].hexcode == sensor_uom)
			return SENSOR_UOM_LIST[i].name;
		i++;
	}
	assert(0);
}

ApiStat apiTelemToJson(char **apidata_ptr)
{
	/** declarations & assignments **/
	char *serialized_string_ptr;							/*JSON format into a string buffer*/														/*buffer for API formatted bulk-data*/
	uint16_t cloud_rec_count = 0;
	uint32_t jsonsize;										/*size of data into object buffer "root_obj_ptr*/
	uint32_t datasize;										/*variable size of "apidata_ptr" buffer*/
	JSON_Value *root_value;									/*buffer of GEOJSON data*/
	JSON_Object *root_obj_ptr;								/*object buffer for GEOJSON formatting data*/
	RTCDateTimeStruct date_time;
	rtcGetDateTime(&date_time);
	
	/** initialization **/
	temp_posted_rec_cnt = 0;
	datasize = 7;
	*apidata_ptr = memAlloc(datasize);
	TelemetryStruct *cloud_telem_buff;
	
	/** retrieve data **/
	loggerFetchRecFromRingBuff((uint8_t **)&cloud_telem_buff
                                     , TELEMETRY
                                     , MAX_REC_PER_POST
                                     ,(uint16_t *)&cloud_rec_count);
	
	cloud_telem_buff = telem_ptr + (rb_handle[TELEMETRY].latest_idx - 1);
	
	if(cloud_rec_count <=0)
	{
		return APISTAT_FAILED;
	}
	temp_posted_rec_cnt = cloud_rec_count;
	#if PRINT_PROCESSES_IN_UART
		char printout[30];
		snprintf(printout, 30, "\n### TELEM : %d ###\n",cloud_rec_count);
		uartSHOW((uint8_t*)printout, strlen(printout));
	#endif
	/** format data into api form **/
	strcpy(*apidata_ptr, "[");
	for(uint16_t i = 0; i < cloud_rec_count; i++)
	{
		if(dataFilter(cloud_telem_buff->rad_modbus_id, cloud_telem_buff->port,
				cloud_telem_buff->sensor_code, cloud_telem_buff->sensor_type,
				cloud_telem_buff->raw_uom))
		{
		  	#if PRINT_PROCESSES_IN_UART
				snprintf(printout, 30, "\nPORT = %d  ",cloud_telem_buff->port);
				uartSHOW((uint8_t*)getDeviceSN(cloud_telem_buff->rad_modbus_id)
						 , strlen(getDeviceSN(cloud_telem_buff->rad_modbus_id)));
				uartSHOW((uint8_t*)" ", 1);
				uartSHOW((uint8_t*)agdatimEncoder(cloud_telem_buff->date_time, DTEnc_APICONTENT),
						strlen(agdatimEncoder(cloud_telem_buff->date_time, DTEnc_APICONTENT)));
				uartSHOW((uint8_t*)printout, strlen(printout));
				daqPrint(cloud_telem_buff->raw_value, cloud_telem_buff->raw_uom,
					cloud_telem_buff->base_value, cloud_telem_buff->base_uom,
					cloud_telem_buff->depth_value, cloud_telem_buff->depth_uom,
					cloud_telem_buff->sensor_code, cloud_telem_buff->sensor_type);
			#endif
		/** initialize json object **/
			root_value = json_value_init_object();
			root_obj_ptr = json_value_get_object(root_value);

			/** enumerate api value in json object **/
			json_object_dotset_string(root_obj_ptr, "device_sn", getDeviceSN(cloud_telem_buff->rad_modbus_id)); //temp
			json_object_dotset_string(root_obj_ptr, "device_tx", DEVICE_INFO->device_sn); //temp
			json_object_dotset_string(root_obj_ptr, "sensor_code", getSensorCode(cloud_telem_buff->sensor_code));
			json_object_dotset_string(root_obj_ptr, "sensor_type", getSensorType(cloud_telem_buff->sensor_type));
			json_object_dotset_number(root_obj_ptr, "account_id", ACCOUNT_ID); //temp
			json_object_dotset_number(root_obj_ptr, "port", PORT[cloud_telem_buff->port]);
			json_object_dotset_number(root_obj_ptr, "raw_value", cloud_telem_buff->raw_value);
			json_object_dotset_string(root_obj_ptr, "raw_uom", getSensorUom(cloud_telem_buff->raw_uom));
			json_object_dotset_number(root_obj_ptr, "base_value", cloud_telem_buff->base_value);
			json_object_dotset_string(root_obj_ptr, "base_uom", getSensorUom(cloud_telem_buff->base_uom));
			json_object_dotset_number(root_obj_ptr, "misc_value", cloud_telem_buff->depth_value);
			json_object_dotset_string(root_obj_ptr, "misc_uom", getSensorUom(cloud_telem_buff->depth_uom));

			if(cloud_telem_buff->date_time.Year < 17)
			{
				json_object_dotset_string(root_obj_ptr, "read_date",
						agdatimEncoder(date_time, DTEnc_APICONTENT));
			}
			else
				json_object_dotset_string(root_obj_ptr, "read_date",
						agdatimEncoder(cloud_telem_buff->date_time, DTEnc_APICONTENT));

			/** append json object to api data string **/
			jsonsize = json_serialization_size(root_value);
			serialized_string_ptr = json_serialize_to_string(root_value);
			datasize += jsonsize + 1;
			*apidata_ptr = memReAlloc(*apidata_ptr, datasize);
			strncat(*apidata_ptr, serialized_string_ptr, jsonsize);
			strncat(*apidata_ptr, ",", 1);

			/** free json object **/
			json_free_serialized_string(serialized_string_ptr);
			json_value_free(root_value);
		}
		//uartSHOW(*apidata_ptr, strlen(*apidata_ptr));
		//const char* printthis = "\r\n\r\n\r\n";
		//uartSHOW((uint8_t*) "\r\n\r\n\r\n", 6);
		cloud_telem_buff--;
		//uartSHOW(serialized_string_ptr, strlen(serialized_string_ptr));
	}
	strncpy(*apidata_ptr + strlen(*apidata_ptr) - 1, "]", 1);
	strcat(*apidata_ptr, "\r\n\r\n");
	//*apidata_ptr = realloc(*apidata_ptr, datasize + 2);
	//(*apidata_ptr)[]
	//strncpy(*apidata_ptr + strlen(*apidata_ptr) - 1, "\r", 1);
	//strncpy(*apidata_ptr + strlen(*apidata_ptr) - 1, "\n", 1);
	return APISTAT_OK;
}

ApiStat apiDevRegToJson(char **apidata_ptr)
{
	/** declarations & assignments **/
	char *serialized_string_ptr;							/*JSON format into a string buffer*/														/*buffer for API formatted bulk-data*/
	uint16_t cloud_rec_count = 0;
	uint32_t jsonsize;										/*size of data into object buffer "root_obj_ptr*/
	uint32_t datasize;										/*variable size of "apidata_ptr" buffer*/
	JSON_Value *root_value;									/*buffer of GEOJSON data*/
	JSON_Object *root_obj_ptr;								/*object buffer for GEOJSON formatting data*/
	RTCDateTimeStruct date_time;
	rtcGetDateTime(&date_time);
	SystemStatusStruct *cloud_systat_buff;
	/** initialization **/
	temp_posted_rec_cnt = 0;
	datasize = 7;
	*apidata_ptr = memAlloc(datasize);
	char fversion[10];
	char drversion[10];
	/** retrieve data **/
	loggerFetchRecFromRingBuff((uint8_t **)&cloud_systat_buff
                                     , SYSTEM_STAT
                                     , MAX_REC_PER_POST
                                     ,(uint16_t *)&cloud_rec_count);
	
	//cloud_systat_buff = sysstat_ptr + (cloud_rec_count - 1);
	cloud_systat_buff = sysstat_ptr + (rb_handle[SYSTEM_STAT].latest_idx - 1);
//	loggerGetRecFromRingBuff((uint32_t*)cloud_systat_buff, 
//							 SYSTEM_STAT, MAX_REC_PER_POST, &cloud_rec_count);
	if(cloud_rec_count <=0)
	{
		return APISTAT_FAILED;
	}
	temp_posted_rec_cnt = cloud_rec_count;
	#if PRINT_PROCESSES_IN_UART
		char printout[30];
		snprintf(printout, 30, "\n### SYS : %d### \n",cloud_rec_count);
		uartSHOW((uint8_t*)printout, strlen(printout));
	#endif

	/** format data into api form **/
	strcpy(*apidata_ptr, "[");
	for(uint16_t i = 0; i < cloud_rec_count; i++)
	{
		if( checkDeviceSN(cloud_systat_buff->rad_modbus_id))
		{
		  	#if PRINT_PROCESSES_IN_UART
				snprintf(printout, 30, "\nDEVREG\n");
				uartSHOW((uint8_t*)getDeviceSN(cloud_systat_buff->rad_modbus_id)
						 , strlen(getDeviceSN(cloud_systat_buff->rad_modbus_id)));
				uartSHOW((uint8_t*)"\n", 1);
				uartSHOW((uint8_t*)agdatimEncoder(cloud_systat_buff->date_time, DTEnc_APICONTENT),
						strlen(agdatimEncoder(cloud_systat_buff->date_time, DTEnc_APICONTENT)));
				uartSHOW((uint8_t*)printout, strlen(printout));
				sysShow(cloud_systat_buff);
				uartSHOW("\n", 1);
			#endif
		/** initialize json object **/
			root_value = json_value_init_object();
			root_obj_ptr = json_value_get_object(root_value);
			
			/*Convert FW Version to string*/
			snprintf(fversion, 10, "%d.%d.%d.%d", cloud_systat_buff->fwver.fwver_main, 
					cloud_systat_buff->fwver.fwver_submain
					, cloud_systat_buff->fwver.fwver_minor
					, cloud_systat_buff->fwver.fwver_dev);
			
			/*Convert DR Version to string*/
			snprintf(drversion, 10, "%d.%d.%d", cloud_systat_buff->drver.drver_main, 
					 cloud_systat_buff->drver.drver_sub,  cloud_systat_buff->drver.drver_minor);
			
			/** enumerate api value in json object **/
			json_object_dotset_string(root_obj_ptr, "device_sn", getDeviceSN(cloud_systat_buff->rad_modbus_id)); //temp
			json_object_dotset_string(root_obj_ptr, "registered_tx", DEVICE_INFO->device_sn); //temp
			json_object_dotset_number(root_obj_ptr, "account_id", ACCOUNT_ID); //temp
			json_object_dotset_string(root_obj_ptr, "firmware_version", fversion);
			json_object_dotset_string(root_obj_ptr, "driver_version", drversion);
			json_object_dotset_number(root_obj_ptr, "battery_status", cloud_systat_buff->batt_stat);
			json_object_dotset_number(root_obj_ptr, "radio_signal_status", cloud_systat_buff->radio_sig_stat);
			json_object_dotset_number(root_obj_ptr, "cell_signal_status", cloud_systat_buff->cell_sig_stat);
			json_object_dotset_number(root_obj_ptr, "memory_orig_size", cloud_systat_buff->orig_mem_size);
			json_object_dotset_number(root_obj_ptr, "memory_available_size", cloud_systat_buff->avail_mem);
			json_object_dotset_number(root_obj_ptr, "internal_temp_status", cloud_systat_buff->int_temp);
			json_object_dotset_number(root_obj_ptr, "external_temp_status", cloud_systat_buff->ext_temp);
			json_object_dotset_number(root_obj_ptr, "charging_status", cloud_systat_buff->charging_stat);
			json_object_dotset_number(root_obj_ptr, "accelerometer_status", cloud_systat_buff->acc_sig_stat);
			json_object_dotset_string(root_obj_ptr, "registration_type", "I");
			json_object_dotset_string(root_obj_ptr, "registered_ip", "0.0.0.0"); //<--error if value = "", it shoul be "null" or not include this at all!!!!
			json_object_dotset_string(root_obj_ptr, "status", "A");
			
			if(cloud_systat_buff->date_time.Year < 17)
			{
				json_object_dotset_string(root_obj_ptr, "registration_date",
						agdatimEncoder(date_time, DTEnc_APICONTENT));
				//json_object_dotset_value(root_obj_ptr, "gps_location.coordinates",
            	//	json_parse_string("[0,0]"));
			}
			else
			{
			  	char gps_data[30];
				char gps_lon[15];
				char gps_lat[15];
				double intpart, fracpart_tmp, fracpart;
				json_object_dotset_string(root_obj_ptr, "registration_date",
						agdatimEncoder(cloud_systat_buff->date_time, DTEnc_APICONTENT));
				fracpart_tmp = modf(cloud_systat_buff->longitude,&intpart);
				modf(fracpart_tmp*1000000, &fracpart);
				fracpart = abs(fracpart);
				intpart = abs(intpart);
				if(cloud_systat_buff->longitude < 0)
					snprintf(gps_lon,15,"-%d.%06d", (int)intpart, (int)fracpart);
				else
					snprintf(gps_lon,15,"%d.%06d", (int)intpart, (int)fracpart);

				fracpart_tmp = modf(cloud_systat_buff->latitude, &intpart);
				modf(fracpart_tmp*1000000, &fracpart);
				fracpart = abs(fracpart);
				intpart = abs(intpart);
				if(cloud_systat_buff->latitude < 0)
					snprintf(gps_lat,15,"-%d.%06d", (int)intpart, (int)fracpart);
				else
					snprintf(gps_lat,15,"%d.%06d", (int)intpart, (int)fracpart);

				snprintf(gps_data,30,"[%s,%s]",gps_lon, gps_lat);
				//uartSHOW(gps_data,strlen(gps_data));
				json_object_dotset_string(root_obj_ptr, "gps_location.type", "Point");
				json_object_dotset_value(root_obj_ptr, "gps_location.coordinates",
				json_parse_string(gps_data));
			}

			/** append json object to api data string **/
			jsonsize = json_serialization_size(root_value);
			serialized_string_ptr = json_serialize_to_string(root_value);
			datasize += jsonsize + 1;
			*apidata_ptr = realloc(*apidata_ptr, datasize);
			strncat(*apidata_ptr, serialized_string_ptr, jsonsize);
			strncat(*apidata_ptr, ",", 1);

			/** free json object **/
			json_free_serialized_string(serialized_string_ptr);
			json_value_free(root_value);
		}
		cloud_systat_buff--;
	}
	strncpy(*apidata_ptr + strlen(*apidata_ptr) - 1, "]", 1);
	strcat(*apidata_ptr, "\r\n\r\n");

	return APISTAT_OK;
}

ApiStat apiVlvCmdToJson(char **apidata_ptr)
{
	static JSON_Value *root_value = NULL;										/*buffer of GEOJSON data*/
	static JSON_Object *root_obj_ptr = NULL;									/*object buffer for GEOJSON formatting data*/
	char *serialized_string_ptr = NULL;														/*JSON format into a string buffer*/														/*buffer for API formatted bulk-data*/
	uint32_t jsonsize = 0;													/*size of data into object buffer "root_obj_ptr*/

	root_value = json_value_init_object();
	root_obj_ptr = json_value_get_object(root_value);

	json_object_set_string(root_obj_ptr, "type", "Feature");
	json_object_set_string(root_obj_ptr, "geometry", "");
	json_object_dotset_string(root_obj_ptr, "properties.device_sn", "PUP2014800001"); //temp
	json_object_dotset_string(root_obj_ptr, "properties.device_rx", "PUP2014800001"); //temp
	json_object_dotset_string(root_obj_ptr, "properties.device_tx", "PUP2014800001"); //temp
	json_object_dotset_string(root_obj_ptr, "properties.valve_code", vlvsched_api.vlv_cde);
	json_object_dotset_number(root_obj_ptr, "properties.account_id", vlvsched_api.acct_id); //temp
	json_object_dotset_number(root_obj_ptr, "properties.schedule_id", vlvsched_api.sched_id);
	json_object_dotset_string(root_obj_ptr, "properties.port", vlvsched_api.port);
	json_object_dotset_string(root_obj_ptr, "properties.valve_type", vlvsched_api.vlv_typ);
	json_object_dotset_number(root_obj_ptr, "properties.input_raw_value", agstringStrToNum(vlvsched_api.in_val, sizeof(vlvsched_api.in_val)));
	json_object_dotset_string(root_obj_ptr, "properties.input_raw_uom", "ON");//vlvsched_api.in_uom); //temp
	json_object_dotset_number(root_obj_ptr, "properties.input_base_value", agstringStrToNum(vlvsched_api.in_val, sizeof(vlvsched_api.in_val)));
	json_object_dotset_string(root_obj_ptr, "properties.input_base_uom", "ON");//vlvsched_api.in_uom); //temp
	json_object_dotset_string(root_obj_ptr, "properties.date_executed", agdatimEncoder(vlv_datexecuted, DTEnc_APICONTENT));

	serialized_string_ptr = json_serialize_to_string(root_value);	//_pretty to be readable
	jsonsize = json_serialization_size(root_value); 				//_pretty to be readable
	*apidata_ptr = (char *)memAlloc(jsonsize + 5);
	assert(*apidata_ptr);

	//strncpy(apidata_ptr, "[", 1);
	memset(*apidata_ptr, '\0', jsonsize + 5);
	strcpy(*apidata_ptr, "[");
	strncat(*apidata_ptr, serialized_string_ptr, jsonsize);
	strncat(*apidata_ptr, "]", 1);

	json_free_serialized_string(serialized_string_ptr);
	json_value_free(root_value);

	return APISTAT_OK;
}

ApiStat apiSenRegToJson(char **apidata_ptr)
{
	/** declarations & assignments **/
	const uint8_t PORT_CNT = 4;
	SensorRegistrationStruct *senport;
	uint8_t senport_id;
	JSON_Value *root_value;									/*buffer of GEOJSON data*/
	JSON_Object *root_obj_ptr;								/*object buffer for GEOJSON formatting data*/
	char *serialized_string_ptr;							/*JSON format into a string buffer*/

	uint32_t jsonsize;										/*size of data into object buffer "root_obj_ptr*/
	uint32_t datasize;										/*variable size of "apidata_ptr" buffer*/
	RTCDateTimeStruct date_time;
	rtcGetDateTime(&date_time);

	/** initialization **/
	senport = (SensorRegistrationStruct*)&PortConfig;
	datasize = 3;
	*apidata_ptr = memAlloc(datasize);
	
	/** format data into api form **/
	strcpy(*apidata_ptr, "[");
	for(senport_id = 0; senport_id < PORT_CNT; senport_id++)
	{
		/** initialize json object **/
		root_value = json_value_init_object();
		root_obj_ptr = json_value_get_object(root_value);

		/** enumerate api value in json object **/
		json_object_set_string(root_obj_ptr, "type", GEOJSONTYPE);
		json_object_dotset_string(root_obj_ptr, "geometry.type", "Point");
		json_object_dotset_value(root_obj_ptr, "geometry.coordinates",
		json_parse_string(DEVICE_COORDINATES[senport_id]));
		json_object_dotset_string(root_obj_ptr, "properties.device_sn", DEVICE_INFO_LIST[0].device_sn); //temp
		json_object_dotset_string(root_obj_ptr, "properties.registered_tx", DEVICE_INFO_LIST[0].device_sn); //temp
		json_object_dotset_string(root_obj_ptr, "properties.sensor_code", getSensorCode(senport->sensor_code));
		json_object_dotset_string(root_obj_ptr, "properties.sensor_alias", "");
		json_object_dotset_number(root_obj_ptr, "properties.account_id", ACCOUNT_ID); //temp
		json_object_dotset_string(root_obj_ptr, "properties.sensor_type", senport->sensor_type);
		json_object_dotset_number(root_obj_ptr, "properties.port", senport_id+1);
		json_object_dotset_string(root_obj_ptr, "properties.registration_type", "B");
		json_object_dotset_string(root_obj_ptr, "properties.registered_ip", "0.0.0.0");//SERIAL_NUM[0]); //temp
		json_object_dotset_string(root_obj_ptr, "properties.notes", "");
		json_object_dotset_string(root_obj_ptr, "properties.raw_uom", senport->raw_uom);
		json_object_dotset_string(root_obj_ptr, "properties.base_uom", senport->base_uom);
		json_object_dotset_string(root_obj_ptr, "properties.conversion", senport->conversion);
		json_object_dotset_string(root_obj_ptr, "properties.conversion", "");
		json_object_dotset_string(root_obj_ptr, "properties.misc_value", senport->depth_value);
		json_object_dotset_string(root_obj_ptr, "properties.misc_uom", getSensorUom(senport->depth_uom));
		json_object_dotset_string(root_obj_ptr, "properties.command", senport->command);
		json_object_dotset_string(root_obj_ptr, "properties.data_sequence", senport->data_sequence);
		json_object_dotset_number(root_obj_ptr, "properties.tolerance", 0);
		json_object_dotset_number(root_obj_ptr, "properties.interval", senport->interval);
		json_object_dotset_number(root_obj_ptr, "properties.excitation", senport->excite_time);
		json_object_dotset_string(root_obj_ptr, "properties.driver_version", "");
		json_object_dotset_string(root_obj_ptr, "properties.status", "A");
		json_object_dotset_string(root_obj_ptr
								, "properties.registration_date"
								, "2016-02-23T14:00:00Z");
//		json_object_dotset_string(root_obj_ptr, "properties.registration_date",
//						agdatimEncoder(date_time, DTEnc_APICONTENT));

		/** append json object to api data string **/
		jsonsize = json_serialization_size(root_value);
		serialized_string_ptr = json_serialize_to_string(root_value);
		datasize += jsonsize + 1;
		*apidata_ptr = realloc(*apidata_ptr, datasize);
		assert(*apidata_ptr);
		strncat(*apidata_ptr, serialized_string_ptr, jsonsize);
		strncat(*apidata_ptr, ",", 1);

		/** free json object **/
		json_free_serialized_string(serialized_string_ptr);
		json_value_free(root_value);
		senport++;
	}
	strncpy(*apidata_ptr + strlen(*apidata_ptr) - 1, "]", 1);

	return APISTAT_OK;
}

ApiStat apiToJson(uint8_t apitype, char **apidata_ptr)
{
	ApiStat retval;

	switch(apitype)
	{
	case TELEMETRY:
		retval = apiTelemToJson(apidata_ptr);
		break;
	case SYSTEM_STAT:
		retval = apiDevRegToJson(apidata_ptr);
		break;
	case VALVE_CMD:
		retval = apiVlvCmdToJson(apidata_ptr);
		break;
	case SENSOR_REG:
		retval = apiSenRegToJson(apidata_ptr);
		break;
	default:
		retval = APISTAT_FAILED;
	}

	return retval;
}

#endif
/******************************************************************************
 * Revision History
 *	@file      	api.c
 ******************************************************************************
 * @version		v2.0.2
 * @date		04/08/16
 * @author		J. Fajardo
 * @changes	 	initial release
  ******************************************************************************
 */
