/********************************************************************************
  * @file      	daqSysStat.c
  * @author     Hardware Team
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

/* Includes =================================================================*/
#include <math.h>
#include <stdlib.h>

#include "daqSysStat.h"
#include "delay.h"
#include "i2c.h"
#include "oneWire.h"
#include "agBoardInit.h"
#include "intFlash.h"
#include "utilPwrMonitor.h"
#include "agps.h"
#include "cell.h"
#include "rdBnrMain.h"

/* Constants ================================================================*/
#define SYSSTAT_FLASH_START_SECTOR			FLASH_SECTOR_9

#define BAT_MAX		(8.3)
#define BAT_MIN		(6)
#define BATOFFSET_MAX		((float)0.25)
#define BATOFFSET_MIN		((float)0.19)
#define PWRSAMPLES			((uint8_t)3)
/* External Declarations =====================================================*/  
//extern gpsSysStatStruct gps_systat_data;

/******************************************************************************
 * @brief	prepares battery status
 * @param
 * @retval
 ******************************************************************************/
ReturnStatusEnum sysPwrMntrRead(uint8_t *batt_stat_ptr)//float *pwr_stat)
{
	double vbat = BAT_MIN;
	double  power_voltage;
	float vbat_percent = 0;
	
    delayMillis(200);
	vbat = utilPwrGetVoltage(&PM_MAIN);

		
	vbat_percent = (vbat - BAT_MIN)*
	         (100)/(BAT_MAX - BAT_MIN);

	if(vbat_percent > 100)
		vbat_percent = 100;
	if(vbat_percent < 0)
		vbat_percent = 0;
	modf(vbat_percent, &power_voltage);

	*batt_stat_ptr  = (uint8_t)power_voltage;
	return SUCCESSFUL;
}

/******************************************************************************
 * @brief	prepares data for radio signal status
 * @param
 * @retval
 ******************************************************************************/
ReturnStatusEnum sysRadioSigStat(uint8_t *radio_sig_stat_ptr)
{
	*radio_sig_stat_ptr = rdBnrMainGetLedStat();
	return SUCCESSFUL;
}


/******************************************************************************
 * @brief	prepares data for cell signal status
 * @param
 * @retval
 ******************************************************************************/
ReturnStatusEnum sysCellSigStat(uint8_t *cell_sig_stat_ptr)
{
	cellSignalQuality(cell_sig_stat_ptr);
	return SUCCESSFUL;
}

#pragma optimize=none
/******************************************************************************
 * @brief	prepares data for accelerometer signal status
 * @param
 * @retval
 ******************************************************************************/
ReturnStatusEnum sysAzimuth(uint8_t *acc_sig_stat_ptr)//float *angle)
{
	double angle;
	uint8_t i2c_buff[I2C_BUFFSIZE];
	uint8_t i2c_regstart = 0x00;
	volatile int16_t x_val,y_val, z_val;
	uint8_t x_regd0 = 0x32;
	uint8_t x_regd1 = 0x33;
	uint8_t y_regd0 = 0x34;
	uint8_t y_regd1 = 0x35;
	uint8_t z_regd0 = 0x36;
	uint8_t z_regd1 = 0x37;
	uint8_t sensitivity = 255;
	uint8_t cmd_measure[] = {0x2d, 0x08};
	uint8_t cmd_sleep[] = {0x2d, 0x04};
	float x_acce, y_acce, z_acce;
	volatile double azimuth = 0;
	InterfaceStatusEnum i2c_stat = 0;
	
	gpioDigitalWrite(&ACCEL_ENB, HIGH);
	i2c_stat = (i2c1Write((uint8_t)I2CADD_ACCE,(uint8_t *)cmd_measure,
					 (uint16_t)sizeof(cmd_measure)));
	if (i2c_stat != INTERFACE_OK)
		return FAILED;
	delayMillis(100);

	if ( i2c1Read((uint8_t)I2CADD_ACCE,i2c_regstart,i2c_buff,I2C_BUFFSIZE)
					== INTERFACE_ERROR)
	{
		return FAILED;
	}

	x_val = ((i2c_buff[x_regd1]<< 8) | i2c_buff[x_regd0]);
	y_val = ((i2c_buff[y_regd1]<< 8) | i2c_buff[y_regd0]);
	z_val = ((i2c_buff[z_regd1]<< 8) | i2c_buff[z_regd0]);
	int quad = abs(x_val>>15)<<2 | abs(y_val>>15)<<1 | abs(z_val>>15);
	x_acce = (float)x_val/sensitivity;
	y_acce = (float)y_val/sensitivity;
	z_acce = (float)z_val/sensitivity;
	azimuth = acos(z_acce/sqrt(x_acce*x_acce + y_acce*y_acce + z_acce*z_acce)) *180/3.1416;
	i2c1Write(I2CADD_ACCE,cmd_sleep,sizeof(cmd_sleep));
	//angle = azimuth;

	//DUMMY
	modf(azimuth, &angle);
	*acc_sig_stat_ptr = (uint8_t)angle;
	
	/*Put Accelerometer to sleep*/
	if (i2c1Write(0x1D,cmd_sleep, 2) != INTERFACE_OK)
	  	assert(0);
	if (i2c1Read((uint8_t)0x1D,0x00,i2c_buff,58)
					== INTERFACE_ERROR)
	   	assert(0); 
	if(i2c_buff[0x2D] != 0x04)
	   	assert(0);
	gpioDigitalWrite(&ACCEL_ENB, LOW);
	
	return SUCCESSFUL;
}

/******************************************************************************
 * @brief	prepares data for orig mem size and available memory size
 * @param
 * @retval
 ******************************************************************************/
void sysSleepAcce(void)
{
  	uint8_t i2c_buff[I2C_BUFFSIZE];
  	uint8_t cmd_sleep[] = {0x2d, 0x04};
  	gpioDigitalWrite(&ACCEL_ENB, HIGH);
	delayMillis(50);
  	if (i2c1Write(0x1D,cmd_sleep, 2) != INTERFACE_OK)
	  	assert(0);
	if (i2c1Read((uint8_t)0x1D,0x00,i2c_buff,58)
					== INTERFACE_ERROR)
	   	assert(0); 
	if(i2c_buff[0x2D] != 0x04)
	   	assert(0);
	gpioDigitalWrite(&ACCEL_ENB, LOW);
}

/******************************************************************************
 * @brief	prepares data for orig mem size and available memory size
 * @param
 * @retval
 ******************************************************************************/
ReturnStatusEnum sysMemSize(uint16_t *orig_mem_size_ptr,
								   uint16_t *avail_mem_ptr)
{
	//fl_sysstat_handle.last_rec_addr
	*orig_mem_size_ptr = ORIG_FLASH_MEM_SIZE / 1024;	// IN MB
	*avail_mem_ptr = (ORIG_FLASH_MEM_SIZE - (CODE_FLASH_MEM_SIZE
						+ (intFlRetLastAddr(TELEMETRY)
								- SYSSTAT_TEMP_BUFF_START_ADDR)
								+ (intFlRetLastAddr(SYSTEM_STAT)
										- TELEM_TEMP_BUFF_START_ADDR))) / 1024;
	*orig_mem_size_ptr = 2048;
	*avail_mem_ptr = 1890;
	return SUCCESSFUL;
}

/******************************************************************************
 * @brief	prepares data for internal temperature
 * @param
 * @retval
 ******************************************************************************/
ReturnStatusEnum sysInternalTemp(float *temp_f)
{
    delayMillis(200);
	*temp_f = (float)(utilPwrGetTemp(&PM_MAIN)*9/5 + 32);      
	return SUCCESSFUL;
}

/******************************************************************************
 * @brief	prepares data for external temperature
 * @param
 * @retval
 ******************************************************************************/
ReturnStatusEnum sysExternalTemp(float *temp_f)
{
	uint8_t addr[8];
	uint8_t data[12];
	volatile uint8_t search_result;
	uint8_t crc_code;
	char chip_name[20];
	uint8_t type_s = 0;
	uint8_t celsius = 0;
	AgTechIOStruct *gpio_ptr = &TEMP_EXT;
	int i;
	uint8_t cfg;
	int16_t raw = 0;

	search_result = oneWireSearch(gpio_ptr,addr);
	if (search_result)
	{
		crc_code = crc8(addr, 7);
		if ( crc_code != addr[7])
		{
			*temp_f = -1;
			return FAILED;
		}

		switch (addr[0])
		{
			case 0x10:
				snprintf(chip_name,20, "DS18S20");
				type_s = 1;
				break;
			case 0x28:
				snprintf(chip_name,20, "DS18B20");
				type_s = 0;
				break;
			case 0x22:
				snprintf(chip_name,20, "DS1822");
				type_s = 0;
				break;
			default:
				snprintf(chip_name,20, "NONE");
				*temp_f = -1;
				//return FAILED;
		}

		oneWireReset(gpio_ptr);
		oneWireSelect(gpio_ptr, addr);
		oneWireWrite(gpio_ptr,0x44, OWIRE_PARASITIC_POWERON);        // start conversion, with parasite power on at the end
		delayMillis(1000);

		oneWireReset(gpio_ptr);
		oneWireSelect(gpio_ptr, addr);
		oneWireWrite(gpio_ptr,0xBE, OWIRE_PARASITIC_POWEROFF);


		for ( i = 0; i < 9; i++)
		{           // we need 9 bytes
			data[i] = oneWireRead(gpio_ptr);
		}

		crc_code = crc8(data, 8);
		if ( crc_code != data[8])
		{
			*temp_f = -1;
			return FAILED;
		}

		// Convert the data to actual temperature
		// because the result is a 16 bit signed integer, it should
		// be stored to an "int16_t" type, which is always 16 bits
		// even when compiled on a 32 bit processor.
		raw = (data[1] << 8) | data[0];
		if (type_s)
		{
			raw = raw << 3; // 9 bit resolution default
			if (data[7] == 0x10)
				raw = (raw & 0xFFF0) + 12 - data[6]; // "count remain" gives full 12 bit resolution
		}

		else
		{
			cfg = (data[4] & 0x60);
			// at lower res, the low bits are undefined, so let's zero them
			if (cfg == 0x00)
				raw = raw & ~7;  // 9 bit resolution, 93.75 ms
			else if
				(cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
			else if
				(cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
			//// default is 12 bit resolution, 750 ms conversion time
		}

		celsius = (float)raw / 16.0;
		*temp_f = celsius * 1.8 + 32.0;
		return SUCCESSFUL;
	}
	else
		*temp_f = -1;
	return FAILED;
}


/******************************************************************************
 * @brief	prepares data for latest GPS Location 1 and 2 data
 * @param
 * @retval
 ******************************************************************************/
ReturnStatusEnum sysGPSLoc(float *latitude,
						   float *longitude)
{
	*latitude = gps_systat_data.latitude;
	*longitude = gps_systat_data.longitude;
	return SUCCESSFUL;
}

/******************************************************************************
 * @brief
 * @param
 * @retval
 ******************************************************************************/
uint8_t sysChargingStat(void)
{
 AgTechIOStruct *gpio_ptr = &CHARGE_OK;

 return !gpioDigitalRead(gpio_ptr);
}



/*********************************************************************************
  * Revision History
  *	@file      	daqSysStat.c
 ********************************************************************************
  * @version	v2.2.0
  * @date		04/08/16
  * @author		D. Lasdoce
  * @changes	- created file
  ********************************************************************************
  */
