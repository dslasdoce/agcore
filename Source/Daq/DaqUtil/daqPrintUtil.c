/********************************************************************************
  * @file      	daqPrintUtil.c
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
	  
#include "agBoardInit.h"	  
#include "daqPrintUtil.h"
#include "dmMain.h"
#include "cell.h"

/* Function ==================================================================*/


/*==============================================================================
 * @brief
 * @param
 * @retval
==============================================================================*/
void daqShowInUart(float *raw_values, double *con_values,
				   uint32_t size, uint8_t *data_sequence,
				    uint8_t *depth_values)
{
	char output[50];
	uint32_t idx;

	for(idx = 0; idx <size; idx++)
	{
		memset(output, 0, 50);
		double intpart, fracpart_tmp, fracpart;
		fracpart_tmp = modf(raw_values[idx],&intpart);
		modf(fracpart_tmp*100000, &fracpart);
		snprintf(output,50,"raw = %6d.%-5d    ", (int)intpart, (int)fracpart);
		uartSHOW((uint8_t*)output, strlen(output));

		memset(output, 0, 50);
		fracpart_tmp = modf(con_values[idx],&intpart);
		modf(fracpart_tmp*100000, &fracpart);
		snprintf(output,50,"con = %6d.%-5d   ", (int)intpart, (int)fracpart);
		uartSHOW((uint8_t*)output, strlen(output));

		memset(output, 0, 50);
		snprintf(output,50,"uom = 0x%02x    depth = %2d\n",(unsigned int)data_sequence[idx],
						(unsigned int)depth_values[idx]);
		uartSHOW((uint8_t*)output, strlen(output));
	}
	const char *done = "==========DAQ COMPLETE==========\n\n";
	uartSHOW((uint8_t*)done, strlen(done));
}

/*==============================================================================
 * @brief
 * @param
 * @retval
==============================================================================*/
void daqPrint(float raw_value, uint8_t raw_uom,
			  double con_value, uint8_t con_uom,
			  uint8_t depth_value, uint8_t depth_uom, 
			  uint16_t sensor_code, uint8_t sensor_type)
{
	char output[50];
	/*RAW*/
	memset(output, 0, 50);
	double intpart, fracpart_tmp, fracpart;
	fracpart_tmp = modf(raw_value,&intpart);
	modf(fracpart_tmp*100000, &fracpart);
	fracpart = abs(fracpart);
	if(raw_value < 0 && intpart == 0)
	{
		snprintf(output,50,"raw= -%6d.%05d  0x%02x     ", (int)intpart, (int)fracpart,
				(unsigned int)raw_uom);
	}
	else
		snprintf(output,50,"raw= %6d.%05d  0x%02x     ", (int)intpart, (int)fracpart,
				(unsigned int)raw_uom);
	uartSHOW((uint8_t*)output, strlen(output));

	/*CON*/
	memset(output, 0, 50);
	fracpart_tmp = modf(con_value,&intpart);
	modf(fracpart_tmp*100000, &fracpart);
	fracpart = abs(fracpart);
	if(con_value < 0 && intpart == 0)
	{
		snprintf(output,50,"con= -%6d.%05d  0x%02x     ", (int)intpart, (int)fracpart,
				(unsigned int)con_uom);
	}
	else
		snprintf(output, 50, "con= %6d.%05d  0x%02x     ", (int)intpart, abs((int)fracpart),
				(unsigned int)con_uom);

	uartSHOW((uint8_t*)output, strlen(output));

	/*depth*/
	memset(output, 0, 50);
	snprintf(output,50,"dep= %02d  0x%02x 0x%04x 0x%02x\n",(unsigned int)depth_value,
			(unsigned int)depth_uom, sensor_code, sensor_type);
	uartSHOW((uint8_t*)output, strlen(output));
}

void sysShow(SystemStatusStruct *sys_stat_ptr)
{
	char printout[50];
	
	snprintf(printout, 50, "RadModBusID:    0x%02X\n",sys_stat_ptr->rad_modbus_id);
	uartSHOW((uint8_t*)printout, strlen(printout));
	memset(printout, 0, 50);
	
	snprintf(printout, 50, "Battery:    %d\n",sys_stat_ptr->batt_stat);
	uartSHOW((uint8_t*)printout, strlen(printout));
	memset(printout, 0, 50);

	snprintf(printout, 50, "Charging: %d\n",sys_stat_ptr->charging_stat);
	uartSHOW((uint8_t*)printout, strlen(printout));
	memset(printout, 0, 50);

	#if DEVICE_TYPE == DEVICETYPE_GATE
		snprintf(printout, 50, "RSSI: %d\n",sys_stat_ptr->cell_sig_stat);
	#else
		snprintf(printout, 50, "RSSI: %d\n",sys_stat_ptr->radio_sig_stat);
	#endif
	uartSHOW((uint8_t*)printout, strlen(printout));
	memset(printout, 0, 50);
	
	double intpart, fracpart_tmp, fracpart;
	fracpart_tmp = modf(sys_stat_ptr->int_temp,&intpart);
	modf(fracpart_tmp*100, &fracpart);
	snprintf(printout,50,"INT TEMP: %4d.%02d\n", (int)intpart, (int)fracpart);
	uartSHOW((uint8_t*)printout, strlen(printout));
	memset(printout, 0, 50);

	fracpart_tmp = modf(sys_stat_ptr->ext_temp,&intpart);
	modf(fracpart_tmp*100, &fracpart);
	snprintf(printout,50,"EXT TEMP: %4d.%02d\n", (int)intpart, (int)fracpart);
	uartSHOW((uint8_t*)printout, strlen(printout));
	memset(printout, 0, 50);

	fracpart_tmp = modf(sys_stat_ptr->acc_sig_stat,&intpart);
	modf(fracpart_tmp*100, &fracpart);
	snprintf(printout,50,"AZIMUTH: %4d.%02d\n", (int)intpart, (int)fracpart);
	uartSHOW((uint8_t*)printout, strlen(printout));
	memset(printout, 0, 50);

	fracpart_tmp = modf(sys_stat_ptr->longitude,&intpart);
	modf(fracpart_tmp*1000000, &fracpart);
	fracpart = abs(fracpart);
	if(sys_stat_ptr->longitude < 0 && intpart == 0)
	{
		snprintf(printout,50,"LONGITUDE: -%d.%06d\n", (int)intpart, (int)fracpart);
	}
	else
		snprintf(printout, 50, "LONGITUDE: %d.%06d\n", (int)intpart, abs((int)fracpart));
		uartSHOW((uint8_t*)printout, strlen(printout));
	memset(printout, 0, 50);

	fracpart_tmp = modf(sys_stat_ptr->latitude,&intpart);
	modf(fracpart_tmp*1000000, &fracpart);
	fracpart = abs(fracpart);
	if(sys_stat_ptr->latitude < 0 && intpart == 0)
	{
		snprintf(printout,50,"LATITUDE: -%d.%06d\n", (int)intpart, (int)fracpart);
	}
	else
		snprintf(printout, 50, "LATITUDE: %d.%06d\n", (int)intpart, abs((int)fracpart));
		uartSHOW((uint8_t*)printout, strlen(printout));
	memset(printout, 0, 50);
}




/******************************************************************************
  * Revision History
  *	@file      	daqPrintUtil.h
  *****************************************************************************
  * @version	v2.2.0
  * @date		04/08/16
  * @author		D. Lasdoce
  * @changes	- created file
  *****************************************************************************
  */