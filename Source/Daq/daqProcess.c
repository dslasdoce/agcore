/********************************************************************************
  * @file      	daqProcess.c
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
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <deployAssert.h>

#include "productCfg.h"
#include "daqPrintUtil.h"
#include "utilMem.h"
#include "delay.h"
#include "agBoardInit.h"
#include "port.h"
#include "equations.h"
#include "utilCalc.h"
#include "loggerMain.h"
#include "daqDavis.h"
#include "daqSensInterface.h"
#include "daqSysStat.h"
#include "daqProcess.h"
#include "daqAdcSpi.h"
#include "ble.h"
#include "mspSpi.h"

/* Typedefs ==================================================================*/


/* Defines ===================================================================*/
#define TIMEOUT_DAVIS			((uint32_t)60)
#define DATACNT_FIX_SINGLE		((uint8_t)1)
#define DATACNT_FIX_DOUBLE		((uint8_t)2)

typedef enum
{
	STANDARD_DEV,
	PERCENT_DIFF
}SpikeDetectMethodEnum;

/* Global Declarations =======================================================*/

/* Functions =================================================================*/

/* ============================================================================
 * @brief generate 3 bit spike code to represent which data are valid(1) or invalid(0)
 *
 * ==========================================================================*/
static uint8_t daqGetSpikeCode(float value_1, float value_2, float value_3,
							   SpikeDetectMethodEnum detection_method)
{
	uint8_t spike_code = 0;
	float desired_percentage = 0.1;
	bool d1_d2, d1_d3, d2_d3, d1, d2, d3;
	float mean_value = utilCalcMeanValue(3, value_1, value_2, value_3);
	float std_dev = utilCalcStandardDev(value_1, value_2, value_3);

	if(detection_method == PERCENT_DIFF)
	{
		d1_d2 = (utilCalcPercentDiff(value_1, value_2) <= desired_percentage);
		d1_d3 = (utilCalcPercentDiff(value_1, value_3) <= desired_percentage);
		d2_d3 = (utilCalcPercentDiff(value_2, value_3) <= desired_percentage);
		spike_code = (d2_d3<<2) | (d1_d3<<1) | (d1_d2);
	}
	else
	{
		d1 = (fabs(value_1 - mean_value) <= std_dev);
		d2 = (fabs(value_2 - mean_value) <= std_dev);
		d3 = (fabs(value_3 - mean_value) <= std_dev);
		spike_code = (d3<<2) | (d2<<1) | (d1);
	}
	return spike_code;
}

/* ============================================================================
 * @brief averages valid data and save it to raw_values
 * ==========================================================================*/
bool daqSpikeDetection(SpikeDetectMethodEnum detection_method,
					   uint32_t count_per_dataset, float *dataset_1,
					   float *dataset_2, float *dataset_3, float *dataset_ave)
{
	uint32_t idx;
	uint8_t spike_code = 0;
	bool is_data_valid = true;

	for(idx = 0; idx < count_per_dataset; idx++)
	{
		spike_code = daqGetSpikeCode(dataset_1[idx], dataset_2[idx],
									 dataset_3[idx], detection_method);
		if(detection_method == PERCENT_DIFF)
		{
			switch(spike_code)
			{ 
				case 0x01: /*d1 and d2 are valid*/
					dataset_ave[idx] = (dataset_1[idx] + dataset_2[idx])/2;
					break;
				case 0x02: /*d1 and d3 are valid*/
					dataset_ave[idx] = (dataset_1[idx] + dataset_3[idx])/2;
					break;
				case 0x04: /*d2 and d3 are valid*/
					dataset_ave[idx] = (dataset_2[idx] + dataset_3[idx])/2;
					break;
				case 0x07: /*all data are valid*/
					dataset_ave[idx] = (dataset_1[idx] + dataset_2[idx] +
										dataset_3[idx])/3;
					break;
				default:
					return false;
			}
		}

		else if(detection_method == STANDARD_DEV)
		{
			switch(spike_code)
			{
				case 0x01:	/*d1 is valid*/
					dataset_ave[idx] = dataset_1[idx];
						break;
				case 0x02: /*d2 is valid*/
					dataset_ave[idx] = dataset_2[idx];
					break;
				case 0x03: /*d1 and d2 are valid*/
					dataset_ave[idx] = (dataset_2[idx] + dataset_1[idx])/2;
					break;
				case 0x04: /*d3 is valid*/
					dataset_ave[idx] = dataset_3[idx];
					break;
				case 0x05: /*d1 and d3 are valid*/
					dataset_ave[idx] = (dataset_3[idx] + dataset_1[idx])/2;
					break;
				case 0x06: /*d2 and d3 are valid*/
					dataset_ave[idx] = (dataset_3[idx] + dataset_2[idx])/2;
					break;
				case 0x07: /*all data are valid*/
					dataset_ave[idx] = (dataset_1[idx] + dataset_2[idx] +
										dataset_3[idx])/3;
					break;
				default:
					return false;
			}
		}
		else
			return false;
	}
	return is_data_valid;
}

/******************************************************************************
 * @brief
 * @param
 * @retval
 ******************************************************************************/
static void daqDavis(SensorRegistrationStruct *davis_config)
{

	static bool is_pwren = false;
	//AgTechIOStruct *en_davis = NULL;
	//SensorRegistrationStruct *davis_config = davisGetPortDetails(en_davis);
	uint32_t start_time, current_time;
	delayMillis(10000);
	delayMillis(10000);
	delayMillis(10000);
	davis_resetflags = true;
	daqRS485EN(davis_config->port, true);
	davis_weather_port = davis_config->port;
	uartRS485ITEnable(rxbuff_rs485);
//	if(!is_pwren)
//	{
//		portPowerSet(davis_config->port,davis_config->power_supply);
//		is_pwren = true;
//	}
	//delayMillis(2000);

	start_time = current_time = HAL_GetTick();
	while((!davisIsComplete()) && (delayTimeDiff(start_time, current_time) 
            < TIMEOUT_DAVIS*1000))
	{
		delayMillis(1000);
		current_time = HAL_GetTick();
	}
	//gpioDigitalWrite(en_davis, HIGH);
	//gpioDigitalWrite(&RS485_EN, HIGH);
	daqRS485EN(davis_config->port, false);
	//portPowerReset(davis_config->port);

	#if PRINT_PROCESSES_IN_UART
		char display[30];
		const char *temp_display = "TIME ELLAPSED = %d\n";
		snprintf(display, 30, temp_display,
		delayTimeDiff(start_time, current_time)/1000);
		uartSHOW((uint8_t*)display, strlen(display));
	#endif


	if(davisIsComplete())
	{
		davisUpdateStat(false);
		#if PRINT_PROCESSES_IN_UART
			const char *temp_display1 = "=====DAVIS COMPLETE DATA=====\n";
			uartSHOW((uint8_t*)temp_display1, strlen(temp_display1));
		#endif

	}
	#if PRINT_PROCESSES_IN_UART
		else
		{
			const char *temp_display2 = "=====DAVIS INCOMPLETE DATA=====\n";
			uartSHOW((uint8_t*)temp_display2, strlen(temp_display2));
		}
	#endif

}
#pragma optimize=none
/* ============================================================================
 * @brief	read 3 data from designated sensor
 * @param	gpio:	pointer to IOstruct
 * @param	sensordetails_ptr:	pointer to port configuration struct
 * @param	exp_datacount: expected data to be received in one reading
 * @param	raw_values: pointer to array that will contain readings
 * ==========================================================================*/
static bool daqSensorRead(SensorRegistrationStruct *sensordetails_ptr,
					 uint8_t exp_datacount, float *raw_values)
{
	float dataset_1[MAX_SINGLE_READ];
	float dataset_2[MAX_SINGLE_READ];
	float dataset_3[MAX_SINGLE_READ];
	bool is_data_valid = false;
	ReturnStatusEnum adc_stat = FAILED;
	uint32_t datacount_1,datacount_2,datacount_3;

	switch(sensordetails_ptr->interface){
	  	case IFC_ANALOG_V:
			datacount_1 = daqSensorAnalogV(sensordetails_ptr->port, dataset_1);
			datacount_2 = daqSensorAnalogV(sensordetails_ptr->port, dataset_2);
			datacount_3 = daqSensorAnalogV(sensordetails_ptr->port, dataset_3);
			break;
		case IFC_ANALOG_I:
			dataset_1[0] = daqSensAnalogI(sensordetails_ptr->port);
			dataset_2[0] = daqSensAnalogI(sensordetails_ptr->port);
			dataset_3[0] = daqSensAnalogI(sensordetails_ptr->port);
			datacount_1 = datacount_2 = datacount_3 = DATACNT_FIX_SINGLE;
			break;
		case IFC_ANALOG_R:
			/*TODO: Read from MSP430*/
			datacount_1 = datacount_2 = datacount_3 = DATACNT_FIX_SINGLE;
			break;
		case IFC_ANALOG_V_R:
			dataset_1[0] = daqSensorAnalogV(sensordetails_ptr->port, dataset_1);
			dataset_2[0] = daqSensorAnalogV(sensordetails_ptr->port, dataset_2);
			dataset_3[0] = daqSensorAnalogV(sensordetails_ptr->port, dataset_3);
			
			dataset_1[1] = daqSensTherm(sensordetails_ptr->port);
			dataset_2[1] = daqSensTherm(sensordetails_ptr->port);
			dataset_3[1] = daqSensTherm(sensordetails_ptr->port);
			/*TODO: Read from MSP430*/
			datacount_1 = datacount_2 = datacount_3 = DATACNT_FIX_DOUBLE;
			break;
		case IFC_ANALOG_V_V:
//			dataset_1[0] = daqSensorAnalogV(sensordetails_ptr->port, &adc_stat);
//			dataset_2[0] = daqSensorAnalogV(sensordetails_ptr->port, &adc_stat);
//			dataset_3[0] = daqSensorAnalogV(sensordetails_ptr->port, &adc_stat);
			/*TODO: Read from MSP430*/
			datacount_1 = datacount_2 = datacount_3 = DATACNT_FIX_DOUBLE;
			break;
		case IFC_DAVIS_DTH:
		  	delayMillis(5000);
		  	datacount_1 = daqSensorDavisDTH(sensordetails_ptr, dataset_1);
			datacount_2 = daqSensorDavisDTH(sensordetails_ptr, dataset_2);
			datacount_3 = daqSensorDavisDTH(sensordetails_ptr, dataset_3);
	  		break;
		case IFC_COUNTER_PU_C: 
		case IFC_COUNTER_PD_C:
		case IFC_COUNTER_NP_C:
		  	datacount_1 = datacount_2 = datacount_3 = DATACNT_FIX_SINGLE;
			return true;
		case IFC_COUNTER_PD_P:
		case IFC_COUNTER_PU_P:
		case IFC_COUNTER_NP_P:
			/*TODO: Read from MSP430*/
		  	mspStartSensorRead(sensordetails_ptr->port, 2.5);
			delayMillis(2500);
			raw_values[0] = mspSendSensorData(sensordetails_ptr->port);
			mspClearSensorData(sensordetails_ptr->port);
			datacount_1 = datacount_2 = datacount_3 = DATACNT_FIX_SINGLE;
			return true;
		case IFC_SDI12:
			datacount_1 = daqSensorSDI(sensordetails_ptr, dataset_1);
			datacount_2 = daqSensorSDI(sensordetails_ptr, dataset_2);
			datacount_3 = daqSensorSDI(sensordetails_ptr, dataset_3);
			break;
		case IFC_RS485:
			//TODO:
			break;
		case IFC_RS232:
			//TODO:
			break;
		case IFC_DAVIS_ISS:
			daqDavis(sensordetails_ptr);
			return false; //return false and bypass the conversion part
		case IFC_DAVIS_TEMP:
			//TODO: read from davis proprietary temp
			datacount_1 = datacount_2 = datacount_3 = DATACNT_FIX_SINGLE;
			break;
		case IFC_THERM:
			dataset_1[0] = daqSensTherm(sensordetails_ptr->port);
			dataset_2[0] = daqSensTherm(sensordetails_ptr->port);
			dataset_3[0] = daqSensTherm(sensordetails_ptr->port);
			datacount_1 = datacount_2 = datacount_3 = DATACNT_FIX_SINGLE;
			break;
		case IFC_AV_CPUC:
		case IFC_AV_CPUP:
		case IFC_AV_CPDC:
		case IFC_AV_CPDP:
		case IFC_AV_CNPC:
		case IFC_AV_CNPP:
			/*TODO: Read from MSP430*/
			datacount_1 = datacount_2 = datacount_3 = DATACNT_FIX_DOUBLE;
			return true;
		case IFC_BLE:
		  //TODO: 
		  	BleStat stat;
			datacount_1 = datacount_2 = datacount_3 = DATACNT_FIX_SINGLE;
			stat = bleReadPressureSensor(dataset_1, dataset_2, dataset_3);
			if((stat != BLE_SUCCESSFUL))	  	
		  		return false;
			break;
		default:
			return false;
	}

	if ( (datacount_1 == datacount_2) && (datacount_1 == datacount_3)
	   && (datacount_1 == exp_datacount))
	{
		/*Do not include counter in spike detection*/
	  //TODO: add exception for counter
//		if(sensordetails_ptr->interface == ANALOG_W_CCTR ||
//		   sensordetails_ptr->interface == ANALOG_W_PCTR)
//			--exp_datacount;
		
		is_data_valid = daqSpikeDetection(STANDARD_DEV, exp_datacount,
						dataset_1, dataset_2, dataset_3, raw_values);
	}
	return is_data_valid;
}

/* ============================================================================
 * @brief	Converts all required data processing parameters from string
 * 			to array
 * @param	params_ptr:	pointer to DataProcessingStruct that should be updated
 * @param	sensordetails_ptr:	pointer SensorRegistrationStruct where
 * 								sensor details for each port are stored
 * ==========================================================================*/
void daqSetProcessingParams(DataProcessingStruct *params_ptr,
							SensorRegistrationStruct *sensordetails_ptr)
{
	char *string_holder = NULL;
	char *parsed_strings[20];
	uint8_t idx;
	uint8_t type_count;
	uint8_t raw_uom_count;
	uint8_t base_uom_count;
	uint8_t depth_values_count;
	uint8_t idx_active_eq;

	/* Convert data sequence from string to array of coded values */
	string_holder = strdup(sensordetails_ptr->data_sequence);
	params_ptr->exp_datacount = eqStringSplit(parsed_strings,string_holder,",");
	for(idx=0; idx<params_ptr->exp_datacount; idx++)
	{
		params_ptr->datasequence[idx] 
				= (uint8_t)strtoul(parsed_strings[idx],NULL,16);
	}
	eqStringSplitFree(parsed_strings, params_ptr->exp_datacount);
	free(string_holder);

	/* Convert sensor type from string to array of coded values */
	string_holder = strdup(sensordetails_ptr->sensor_type);
	type_count = eqStringSplit(parsed_strings,string_holder,",");
	for(idx=0; idx<type_count; idx++)
	{
		params_ptr->sensor_type[idx] =
						(uint8_t)strtoul(parsed_strings[idx],NULL,16);
	}
	eqStringSplitFree(parsed_strings, type_count);
	free(string_holder);

	/* Convert raw_uom from string to array of coded values */
	string_holder = strdup(sensordetails_ptr->raw_uom);
	raw_uom_count = eqStringSplit(parsed_strings,string_holder,",");
	for(idx=0; idx<raw_uom_count; idx++)
	{
		params_ptr->raw_uoms[idx] =
						(uint8_t)strtoul(parsed_strings[idx],NULL,16);
	}
	eqStringSplitFree(parsed_strings, raw_uom_count);
	free(string_holder);

	/* Convert base_uom from string to array of coded values */
	string_holder = strdup(sensordetails_ptr->base_uom);
	base_uom_count = eqStringSplit(parsed_strings,string_holder,",");
	for(idx=0; idx<base_uom_count; idx++)
	{
		params_ptr->base_uoms[idx] =
						(uint8_t)strtoul(parsed_strings[idx],NULL,16);
	}
	eqStringSplitFree(parsed_strings, base_uom_count);
	free(string_holder);

	/* Convert misc_uom from string to array of coded values */
	string_holder = strdup(sensordetails_ptr->depth_value);
	depth_values_count =
					eqStringSplit(parsed_strings,string_holder,",");
	for(idx=0; idx<depth_values_count; idx++)
	{
		params_ptr->depth_values[idx] =
					(uint8_t)strtoul(parsed_strings[idx],NULL,10);
	}
	free(string_holder);
	eqStringSplitFree(parsed_strings, depth_values_count);

	/* Convert equation codes from string to array of equation address to be used */

	for(idx_active_eq=0; idx_active_eq<MAX_CMN_DPSIZE; idx_active_eq++)
	{

		(params_ptr->ActiveEquations[idx_active_eq]).equation_ptr = NULL;
	}
	eqCodeParser(sensordetails_ptr->conversion, params_ptr->ActiveEquations);
	//daqSetIO(sensordetails_ptr);
}

/* ============================================================================
 * @brief	read, convert, and log data from sensor
 * @param	gpio:	pointer to AgTechIOStruct that indicates the details of
 * 					pin to be used in reading
 * @param	sensordetails_ptr:	pointer to SensorRegistrationStruct that
 * 								contains driver details of the sensor
 * ==========================================================================*/
DaqStatusEnum daqExecute(SensorRegistrationStruct *sensordetails_ptr)
{
	#if PRINT_PROCESSES_IN_UART
		char port_str[40];
		snprintf(port_str,40,"\n===========DAQ PORT %d===========\n",sensordetails_ptr->port);
		uartSHOW((uint8_t*)port_str, strlen(port_str));
	#endif	
	
	//uartSHOW((uint8_t*)"\n===== ENTRY====\n",20);
	TelemetryStruct *telem_data_ptr;
	TelemetryStruct telem_data_ptr_test;
	RTCDateTimeStruct date_time;
	/* Convert each data to respective unit of measures */
	uint8_t idx_rawdata;
	uint8_t idx_ref;
	uint8_t current_raw_uom;
	uint8_t current_sensor_type;
    uint8_t stat;
	
	//TODO: rename the interval member of the struct to pwr_mode
	switch( (sensordetails_ptr->interval & 0x0FFF) ){
	case PWR_MODE_DISCRETE:
		/*Power Set via ARM*/
		portPowerSet(sensordetails_ptr->port, sensordetails_ptr->power_supply);
		break;
	case PWR_MODE_CONTINUOUS:
		/*Power Set via MSP*/
	  	if ( sensordetails_ptr->interval & PWR_INT_MASK_SET)
		{
		  	/*Power Already ON*/
		}
		else
		{	
		  	sensordetails_ptr->interval |= PWR_INT_MASK_SET; 	
		 	mspSetPortPower(sensordetails_ptr->port, sensordetails_ptr->power_supply);
		}
		break;
	case PWR_MODE_OFF:
		break;
	default:
		const char *msg_wrongmode = "!!!!!! WRONG POWER MODE !!!!!!";
		uartSHOW((uint8_t*)msg_wrongmode, strlen(msg_wrongmode));
		assert(0);
		break;
	}
		
	delayMillis(2000);

	#ifdef FW_DUMMY_DATA
		float raw_values[MAX_SINGLE_READ] = {0,2,4,6,8,10,20,21,22,23,24,25};
		uint8_t datacount = 12;
        bool is_data_valid = true;
	#else
		float raw_values[MAX_SINGLE_READ];
		memset(raw_values, 0, MAX_SINGLE_READ);
		bool is_data_valid = false;
	#endif

	int idx_splvalues;
	double con_values[MAX_SINGLE_READ];

	/*struct to temporarily hold the processing details
	 * of the port where sensor was read*/
	DataProcessingStruct *dataproc_ptr;

	/*Struct for input and output parameters of equation to be used*/
	EquationParameterStruct EqParams;

	/*Select data processing details based on port*/
    dataproc_ptr = (DataProcessingStruct *)(&PortDataProcessing)
                    + (sensordetails_ptr->port - 1);
//	switch(sensordetails_ptr->port)
//	{
//		case 1:
//			dataproc_ptr = &Port1Processing;
//			break;
//		case 2:
//			dataproc_ptr = &Port2Processing;
//			break;
//		case 3:
//			dataproc_ptr = &Port3Processing;
//			break;
//		case 4:
//			dataproc_ptr = &Port4Processing;
//			break;
//		default:
//			return DAQ_PORT_ERROR;
//	}

	/* Execute data reading based on sensor interface */
	#ifndef FW_DUMMY_DATA
		is_data_valid = daqSensorRead(sensordetails_ptr,dataproc_ptr->exp_datacount,
								 raw_values);
	#endif
	if(sensordetails_ptr->interval == PWR_MODE_DISCRETE)
		portPowerReset(sensordetails_ptr->port);
	
	if(!is_data_valid)
	{
		if(sensordetails_ptr->interface != IFC_DAVIS_ISS)
		{
			#if PRINT_PROCESSES_IN_UART
				const char * printout = "=====DATA INVALID====\n";
				uartSHOW((uint8_t*)printout, strlen(printout));
			#endif
		}
		return DAQ_DATA_ERROR;
	}
	
	rtcGetDateTime(&date_time);
	/**/
	for(idx_rawdata=0; idx_rawdata<dataproc_ptr->exp_datacount; idx_rawdata++)
	{
		/*Get uom of current data*/
		current_raw_uom = dataproc_ptr->datasequence[idx_rawdata];

		/*get reference index for raw_uom, converted_uom, sensor_type, and equation*/
		for(idx_ref = 0; idx_ref <= MAX_CMN_DPSIZE; idx_ref++)
		{
			if( dataproc_ptr->raw_uoms[idx_ref] ==  current_raw_uom)
				break;
			if(idx_ref == MAX_CMN_DPSIZE)
			{
				assert(0);
				return DAQ_RAW_UOM_ERROR;
			}
		}

		/*get sensor type based on raw_uom index*/
		current_sensor_type = dataproc_ptr->sensor_type[idx_ref];

		/*set new raw value*/
		EqParams.raw_value = raw_values[idx_rawdata];

		/*check if there is an equation to be applied*/
		if(dataproc_ptr->ActiveEquations[idx_ref].equation_ptr)
		{
			/*Set equation parameters*/
			for(idx_splvalues=0; idx_splvalues<MAX_CMN_DPSIZE; idx_splvalues++)
			{
				EqParams.supplementary_values[idx_splvalues] =
					(dataproc_ptr->ActiveEquations[idx_ref]).
					 supplementary_values[idx_splvalues];
			}
			/*execute equation*/
			if(dataproc_ptr->ActiveEquations[idx_ref].equation_ptr != NULL);
				dataproc_ptr->ActiveEquations[idx_ref].equation_ptr(&EqParams);
			raw_values[idx_rawdata] = EqParams.raw_value;
		}
		else
			EqParams.base_value = raw_values[idx_rawdata];

		con_values[idx_rawdata] = EqParams.base_value;

        telem_data_ptr = (TelemetryStruct *)loggerWrRecToRingBuff(&stat, 1, TELEMETRY);

		telem_data_ptr->rad_modbus_id = DEVICE_INFO->rad_modbus_id;
		telem_data_ptr->sensor_type	= current_sensor_type;
		telem_data_ptr->port = sensordetails_ptr->port;
		telem_data_ptr->sensor_code	= sensordetails_ptr->sensor_code;
		telem_data_ptr->date_time = date_time;
		telem_data_ptr->raw_value = EqParams.raw_value;
		telem_data_ptr->base_value = EqParams.base_value;;
		telem_data_ptr->depth_value = dataproc_ptr->depth_values[idx_rawdata];
		telem_data_ptr->raw_uom= dataproc_ptr->raw_uoms[idx_ref];
		telem_data_ptr->base_uom = dataproc_ptr->base_uoms[idx_ref];
		telem_data_ptr->depth_uom = sensordetails_ptr->depth_uom;
		telem_data_ptr->backup_flag = false;
		telem_data_ptr->unsent_flag = true;
		
        /*if(loggerWrRecToRingBuff((uint32_t *)telem_data_ptr, 1, TELEMETRY) 
		   							!= SUCCESSFUL)
		{
			return DAQ_DATA_ERROR;
		}*/
                
		if(loggerGetRecFromRingBuff((uint32_t *)&telem_data_ptr_test, TELEMETRY, 
                    1, NULL) != SUCCESSFUL)
		{
			return DAQ_DATA_ERROR;
		}
		
		#if PRINT_PROCESSES_IN_UART
//		daqPrint(telem_data_ptr_test.raw_value, telem_data_ptr_test.raw_uom,
//				 telem_data_ptr_test.base_value, telem_data_ptr_test.base_uom,
//				 telem_data_ptr_test.depth_value, telem_data_ptr_test.depth_uom,
//				 telem_data_ptr_test.sensor_code);
        daqPrint(telem_data_ptr->raw_value, telem_data_ptr->raw_uom,
                    telem_data_ptr->base_value, telem_data_ptr->base_uom,
                    telem_data_ptr->depth_value, telem_data_ptr->depth_uom,
                    telem_data_ptr->sensor_code, telem_data_ptr->sensor_type);
		#endif
		//free(telem_data_ptr);
	}

	#if PRINT_PROCESSES_IN_UART
		const char *done = "==========DAQ COMPLETE!==========\n\n";
		uartSHOW((uint8_t*)done, strlen(done));
	#endif
	return DAQ_OK;
}



/*==============================================================================
 * @brief
 * @param
 * @retval
==============================================================================*/
DaqStatusEnum daqExecuteDummy(uint32_t i)
{
	TelemetryStruct *telem_data_ptr;
	RTCDateTimeStruct date_time;

	while(i--)
	{
		/*get telem rec buffer to use*/
		telem_data_ptr = (TelemetryStruct *)memAlloc(TELEM_REC_SIZE_BYTES);
//		if(loggerWrRecToRingBuff((uint32_t *)telem_data_ptr, 1, TELEMETRY) 
//							!= SUCCESSFUL)
//		{
//			return DAQ_DATA_ERROR;
//		}
		
		telem_data_ptr->rad_modbus_id = DEVICE_INFO->rad_modbus_id;
		telem_data_ptr->sensor_type	= 0x2;
		telem_data_ptr->port = 0xAA;
		telem_data_ptr->sensor_code	= 0x01;
		telem_data_ptr->raw_value = -1;
		telem_data_ptr->base_value =  -2;
		telem_data_ptr->depth_value = 0;
		telem_data_ptr->raw_uom = 0x01;
		telem_data_ptr->base_uom	= 0x01;
		telem_data_ptr->depth_uom = 0xF;
		telem_data_ptr->backup_flag = false;
		telem_data_ptr->unsent_flag = true;
		rtcGetDateTime(&date_time);
		telem_data_ptr->date_time = date_time;
		
	}
	return DAQ_OK;
}


/******************************************************************************
 * @brief
 * @param
 * @retval
 ******************************************************************************/
DaqStatusEnum daqSysAcquire(void)
{
	SystemStatusStruct *sys_stat_ptr;
    SystemStatusStruct sys_stat_ptr_test;
	RTCDateTimeStruct date_time;
    uint8_t stat;
	const char *printout = "\n==========sysAcquire start==========\n";
	uartSHOW((uint8_t*)printout, strlen(printout));
	//utilPwrModeCont(&PM_MAIN, true);
//	sys_stat_ptr = (SystemStatusStruct *)memAlloc(SYSSTAT_REC_SIZE_BYTES);
//	if(loggerWrRecToRingBuff((uint32_t *)sys_stat_ptr, 1, SYSTEM_STAT) 
//						!= SUCCESSFUL)
//	{
//		return DAQ_DATA_ERROR;
//	}
	
	utilPwrModeCont(&PM_MAIN, true);
    sys_stat_ptr = (SystemStatusStruct *)loggerWrRecToRingBuff(&stat, 1, SYSTEM_STAT);
	sysPwrMntrRead(&sys_stat_ptr->batt_stat);
	sysInternalTemp(&sys_stat_ptr->int_temp);
	sysExternalTemp(&sys_stat_ptr->ext_temp);
	utilPwrModeCont(&PM_MAIN, false);
	
	sysAzimuth(&sys_stat_ptr->acc_sig_stat);

	sysMemSize(&sys_stat_ptr->orig_mem_size, &sys_stat_ptr->avail_mem);

	sysGPSLoc(&sys_stat_ptr->latitude, &sys_stat_ptr->longitude);

	rtcGetDateTime(&date_time);
	sys_stat_ptr->date_time = date_time;
	sys_stat_ptr->rad_modbus_id = DEVICE_INFO->rad_modbus_id;
	sys_stat_ptr->charging_stat = sysChargingStat();

	sys_stat_ptr->reg_typ = 0x07;
	sys_stat_ptr->backup_flag = false;
	sys_stat_ptr->unsent_flag = true;

	sys_stat_ptr->rsvd_12b = 0x00;
	
	sys_stat_ptr->fwver = FW_VERSION;
	sys_stat_ptr->drver = DR_VERSION;
	 
	#if DEVICE_TYPE == DEVICETYPE_GATE
		sysCellSigStat(&sys_stat_ptr->cell_sig_stat);
		sys_stat_ptr->radio_sig_stat = 0;
	#else
		sysRadioSigStat(&sys_stat_ptr->radio_sig_stat);
		sys_stat_ptr->cell_sig_stat = 0;
	#endif
		
    #if PRINT_PROCESSES_IN_UART
		sysShow(sys_stat_ptr);
	#endif
	
	

		
	const char *printout1 = "==========sysAcquire end==========\n";
	uartSHOW((uint8_t*)printout1, strlen(printout1));
	//utilPwrModeCont(&PM_MAIN, false);
	return DAQ_OK;
}

/******************************************************************************
 * @brief
 * @param
 * @retval
 ******************************************************************************/
DaqStatusEnum daqSysAcquireApp(SystemStatusStruct *sys_stat_ptr)
{
	//SystemStatusStruct *sys_stat_ptr;
    //SystemStatusStruct sys_stat_ptr_test;
	RTCDateTimeStruct date_time;
    uint8_t stat;
	const char *printout = "\n==========sysAcquire start==========\n";
	uartSHOW((uint8_t*)printout, strlen(printout));
	//utilPwrModeCont(&PM_MAIN, true);
	//sys_stat_ptr = (SystemStatusStruct *)memAlloc(SYSSTAT_REC_SIZE_BYTES);
//	if(loggerWrRecToRingBuff((uint32_t *)sys_stat_ptr, 1, SYSTEM_STAT) 
//						!= SUCCESSFUL)
//	{
//		return DAQ_DATA_ERROR;
//	}
	
	utilPwrModeCont(&PM_MAIN, true);
    //sys_stat_ptr = (SystemStatusStruct *)loggerWrRecToRingBuff(&stat, 1, SYSTEM_STAT);
	sysPwrMntrRead(&sys_stat_ptr->batt_stat);

	sysInternalTemp(&sys_stat_ptr->int_temp);
	sysExternalTemp(&sys_stat_ptr->ext_temp);

	sysAzimuth(&sys_stat_ptr->acc_sig_stat);

	sysMemSize(&sys_stat_ptr->orig_mem_size, &sys_stat_ptr->avail_mem);

	sysGPSLoc(&sys_stat_ptr->latitude, &sys_stat_ptr->longitude);

	rtcGetDateTime(&date_time);
	sys_stat_ptr->date_time = date_time;
	sys_stat_ptr->rad_modbus_id = DEVICE_INFO->rad_modbus_id;
	sys_stat_ptr->charging_stat = sysChargingStat();

	sys_stat_ptr->reg_typ = 0x07;
	sys_stat_ptr->backup_flag = false;
	sys_stat_ptr->unsent_flag = true;

	sys_stat_ptr->rsvd_12b = 0x00;
	
	sys_stat_ptr->fwver = FW_VERSION;
	 
	#if DEVICE_TYPE == DEVICETYPE_GATE
		sysCellSigStat(&sys_stat_ptr->cell_sig_stat);
		sys_stat_ptr->radio_sig_stat = 0;
	#else
		sysRadioSigStat(&sys_stat_ptr->radio_sig_stat);
		sys_stat_ptr->cell_sig_stat = 0;
	#endif
		
    #if PRINT_PROCESSES_IN_UART
		sysShow(sys_stat_ptr);
	#endif
	utilPwrModeCont(&PM_MAIN, false);
	
//	if(sys_stat_ptr)
//		free(sys_stat_ptr);
//		
	const char *printout1 = "==========sysAcquire end==========\n";
	uartSHOW((uint8_t*)printout1, strlen(printout1));
	//utilPwrModeCont(&PM_MAIN, false);
	return DAQ_OK;
}

/******************************************************************************
 * @brief
 * @param
 * @retval
 ******************************************************************************/
//DaqStatusEnum sysAcquireDummy(void)
//{
//	SystemStatusStruct *sys_stat_ptr;
//	RTCDateTimeStruct date_time;
//
////	sys_stat_ptr = (SystemStatusStruct *)memAlloc(SYSSTAT_REC_SIZE_BYTES);
////	if(loggerWrRecToRingBuff((uint32_t *)sys_stat_ptr, 1, SYSTEM_STAT) 
////						!= SUCCESSFUL)
////	{
////		return DAQ_DATA_ERROR;
////	}
//	
//	sys_stat_ptr->batt_stat = 0x11;
//
//	sys_stat_ptr->radio_sig_stat = 0x22;
//	sys_stat_ptr->cell_sig_stat = 0x33;
//
//	sys_stat_ptr->acc_sig_stat = 0x44;
//
//	sys_stat_ptr->orig_mem_size = 0x55;
//	sys_stat_ptr->avail_mem = 0x66;
//
//		rtcGetDateTime(&date_time);
//	sys_stat_ptr->date_time = date_time;
//	sys_stat_ptr->reg_typ = 0x07;
//	sys_stat_ptr->backup_flag = false;
//	sys_stat_ptr->unsent_flag = true;
//
//	sys_stat_ptr->rsvd_12b = 0x00;
//
//
//	return DAQ_OK;
//}


/******************************************************************************
  * Revision History
  *	@file      	daqProcess.c
  *****************************************************************************
  * @version	v2.2.0
  * @date		04/08/16
  * @author		D. Lasdoce
  * @changes	- created file
  *****************************************************************************
  */