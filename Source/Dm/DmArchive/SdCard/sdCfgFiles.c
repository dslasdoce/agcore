/********************************************************************************
  * @file      	sdCfgFiles.c
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
#include <stdlib.h>
#include <stdbool.h>
#include <deployAssert.h>

#include "productCfg.h"
#include "agBoardInit.h"
#include "delay.h"
#include "utilMem.h"
#include "utilCalc.h"
#include "sdMain.h"
#include "tempApiCodedList.h"
#include "port.h"
#include <math.h>

/* Contants ==================================================================*/
const char *CFGFOLDER_NAMES[]
			 = {"/Cfg", "/Cfg/SensorDrvr", "/Cfg/CodedList",
				"/Cfg/NodeList", "/Cfg/AcctDev", "/Cfg/SensRegn", "/Cfg/DeviceInfo",
				"/Cfg/Sleep"};
const char *CFGTXTDIR_NAMES[]
			 = {"CodedListDir.txt", "NodeListDir.txt"};

/* List of existing coded list text files, UPDATE if necessary*/
char *CD_TYP_FNAMES[]
			= {"SensCode.txt", "SensTyp.txt", "Port.txt", "RawUom.txt", 
			   "BaseUom.txt", "MiscUom.txt", "Intrfce.txt", "PowSup.txt", 
			   "VlvTyp.txt", "VlvCode.txt", "PowReqmt.txt", "LogTyp.txt", 
			   "Module.txt"};
char *NL_TYP_FNAMES[]
			= {"DevSn.txt", "ModBusId.txt", "BnrRdSn.txt", "XbRdSn.txt"};

/* Defines ===================================================================*/
#define SD_INIT_MALLOC_SZ		((uint8_t) 100)
#define SD_ALLOC_SMALL_SZ		((uint8_t) 15)
#define SD_NEWLINECHAR_CNT		((uint8_t) 2)
#define	DEVINFO_GATE			(0)	  
/* Typedefs ==================================================================*/
typedef enum
{
	CFG_MAINDIR,
	CFG_SENSRDRVR,
	CFG_CODEDLIST,
	CFG_NODELIST,
	CFG_ACCTDEV,
	CFG_SENSREGN,
	CFG_DEVINFO,
	CFG_SLEEPINFO
} CfgFolderNameEnum;


/* External Declarations =====================================================*/
/*global storage of strings for cfg files from SDCARD*/
CfgStringStruct *cfg_liststr_ptr[SD_CFGLISTTYP_CNT];


/* Global Definitions ========================================================*/


/* Function Prototypes =======================================================*/
static ReturnStatusEnum sdReadCfgTxtFile(FIL *file_obj_ptr,
										 CfgStringStruct* hdl_ptr,
										 uint8_t file_num);
static ReturnStatusEnum sdCfgCopyList(CfgTypNameEnum cfg_typ);
static ReturnStatusEnum sdGetDeviceInfo(void);
static ReturnStatusEnum sdGetSleepTime(void);

#if DEVICE_TYPE == DEVICETYPE_GATE
static ReturnStatusEnum sdGetNodeRadDevAddrList(void);
static ReturnStatusEnum sdGetSensList(void);
static ReturnStatusEnum sdGetSensTypeList(void);
static ReturnStatusEnum sdGetSensUomList(void);
#endif

#if DEVICETYPE == DEVICETYPE_NODE
static ReturnStatusEnum sdGetWaitTimeout(void)
#endif

/* Functions =================================================================*/

/*==============================================================================
* @brief	Configure folder for Config files
* @param
* @retval
*=============================================================================*/
ReturnStatusEnum sdCfgMkDir(void)
{
	FRESULT f_res;

	f_res = f_mkdir(CFGFOLDER_NAMES[CFG_MAINDIR]);
	if((f_res != FR_OK) && (f_res != FR_EXIST))
	{
		return FAILED;
	}

	f_res = f_mkdir(CFGFOLDER_NAMES[CFG_CODEDLIST]);
	if((f_res != FR_OK) && (f_res != FR_EXIST))
	{
		return FAILED;
	}

	f_res = f_mkdir(CFGFOLDER_NAMES[CFG_SENSRDRVR]);
	if((f_res != FR_OK) && (f_res != FR_EXIST))
	{
		return FAILED;
	}
	
	#if DEVICE_TYPE == DEVICETYPE_GATE
	f_res = f_mkdir(CFGFOLDER_NAMES[CFG_NODELIST]);
	if((f_res != FR_OK) && (f_res != FR_EXIST))
	{
		return FAILED;
	}

	f_res = f_mkdir(CFGFOLDER_NAMES[CFG_ACCTDEV]);
	if((f_res != FR_OK) && (f_res != FR_EXIST))
	{
		return FAILED;
	}
	#endif
	
	f_res = f_mkdir(CFGFOLDER_NAMES[CFG_SENSREGN]);
	if((f_res != FR_OK) && (f_res != FR_EXIST))
	{
		return FAILED;
	}

	
	return SUCCESSFUL;
}


/*==============================================================================
* @brief	
* @param
* @retval
*=============================================================================*/
ReturnStatusEnum sdCfgCopy(void)
{
	ReturnStatusEnum sd_stat;

	sd_stat = sdCfgCopyList(CODEDLIST);
	if(sd_stat)
	  	sd_stat = sdGetDeviceInfo();
	if(sd_stat)
		sd_stat = sdCfgGetRadNetInfo();
	if(sd_stat)
		sd_stat = sdGetSleepTime();
#if DEVICETYPE == DEVICETYPE_NODE
	if(sd_stat)
		sd_stat = sdGetWaitTimeout();
#endif

#if DEVICE_TYPE == DEVICETYPE_GATE
	if(sd_stat)
		sd_stat = sdCfgGetAcctDev(); 
	if(sd_stat)
		sd_stat = sdGetNodeSNList();
    if(sd_stat)
		sdGetNodeRadDevAddrList();
	if(sd_stat)
	  	sd_stat = sdGetSensList();
	if(sd_stat)
		sd_stat = sdGetSensTypeList();
	if(sd_stat)
		sd_stat = sdGetSensUomList();
#endif
	
	if(sd_stat != SUCCESSFUL)
	{
		const char *printout =
		"\n\n\n***SD INIT FAILED!***\n\n\n";
		uartSHOW((uint8_t*)printout, strlen(printout));
		assert(0);
	}
	return sd_stat;
}
 	

/*==============================================================================
* @brief	Read text directories of each type of cfg files then save dir bitmap
* @param
* @retval
*=============================================================================*/
ReturnStatusEnum sdCfgReadTxtDir(CfgTypNameEnum cfg_typ, uint64_t *bitmap_ptr,
								 uint16_t *content_numptr)
{	
	ReturnStatusEnum sd_stat;
	FRESULT f_res;
	FIL file_obj;	
	char *file_name_ptr;
	char *rtext_ptr;		/*buff to contain read text*/
	
	/*param check for congfig type with dir txt only*/
	if(cfg_typ > SD_CFGLISTTYP_CNT)
	{	
		sd_stat = NOT_AVAIL;
		f_res = FR_DENIED;
	}
	else
		sd_stat = SUCCESSFUL;
	
	rtext_ptr = (char *)memAlloc(SD_INIT_MALLOC_SZ);

	/*change folder dir to "/Cfg/Sensor_Driver" and assign corresponding filename*/
	if(sd_stat) 
	{	
		file_name_ptr = (char *)CFGTXTDIR_NAMES[cfg_typ - 1];
	}
	
	if(!f_res) 
		sd_stat = sdReadInit(file_name_ptr, &file_obj);			
	
	while(sd_stat & (!f_res))
	{		
		//read first line which corresponds to bitmap
		f_gets(rtext_ptr, SD_INIT_MALLOC_SZ, &file_obj);
		*bitmap_ptr = (uint64_t)strtol(rtext_ptr, NULL, 10);
		//read first line which corresponds to number of txt files
		f_gets(rtext_ptr, SD_INIT_MALLOC_SZ, &file_obj);
		*content_numptr = (uint16_t)strtol(rtext_ptr, NULL, 10);
		
		/*close file*/
		f_res = f_close(&file_obj);
		break;
	}

	free(rtext_ptr);
	
	return sd_stat;
}


/*==============================================================================
* @brief example fxn reading of sensor driver from SD 
* @param
* @retval
*=============================================================================*/
ReturnStatusEnum sdCfgTestCopySensDrvr(uint16_t sensor_code, 
										SensorRegistrationStruct *buff_ptr)
{
	ReturnStatusEnum sd_stat;
	FRESULT f_res;
	uint16_t dev_ver_num;
	char* fname_ptr = NULL;
	fname_ptr = (char *)memAlloc(SD_ALLOC_SMALL_SZ);
	/*get sensor driver version*/
	dev_ver_num = 1; // temp
	
	/*go to Sensor Driver Directory*/
	f_res = f_chdir(CFGFOLDER_NAMES[CFG_SENSRDRVR]);
	if(!f_res)
	{
		sd_stat = SUCCESSFUL;
		//
	}
	
    /*prepare file name (sensor code + version number: ie, 0x0B_ver1.txt)*/
    sprintf(fname_ptr, "0x%04X_ver%d.txt", sensor_code, dev_ver_num);
    
    /*main fxn for getting sens driver based from sens_code and ver_num requested*/
    sd_stat = sdGetSensDrvrOrRegn(SENSRDRVR, buff_ptr, fname_ptr);
	
	//if(fname_ptr)
		free(fname_ptr);
	
	//TODO: ADD error handling if text file for requested sensor code 
	//and version num does not exist
		
	return sd_stat;
}

/*==============================================================================
* @brief	Copy Sensor Driver Data from SDCARD text files to SRAM
* @param
* @retval
*=============================================================================*/
ReturnStatusEnum sdGetSensDrvrOrRegn(CfgTypNameEnum cfg_typ,
									 SensorRegistrationStruct *buff,
									 char *fname_ptr)
{
	ReturnStatusEnum sd_stat;
	FRESULT f_res;	
	FIL file_obj;
	char *rtext_ptr;		/*buff to contain read text*/

	sd_stat = FAILED;
	rtext_ptr = (char *)memAlloc(SD_INIT_MALLOC_SZ);

	switch(cfg_typ)
	{
	case SENSRDRVR:
		break;
	case SENSREGN:
		/*go to Sensor Driver Directory*/
		f_res = f_chdir(CFGFOLDER_NAMES[CFG_SENSREGN]);	
		fname_ptr = memAlloc(SD_ALLOC_SMALL_SZ);
		sprintf(fname_ptr, "port%d.txt", buff->port);
		sd_stat = SUCCESSFUL;
		break;
	default:
		sd_stat = FAILED;	
		break;
	}	
	
	sd_stat = sdReadInit(fname_ptr, &file_obj);
	
	if(sd_stat)
	{	
		/*read and assign data to corresponding struct member of  the buff given*/
		/*status*/
		f_gets(rtext_ptr, SD_INIT_MALLOC_SZ, &file_obj);
		buff->status = (uint8_t)strtoll(rtext_ptr, NULL, 10);
		/*port*/
		f_gets(rtext_ptr, SD_INIT_MALLOC_SZ, &file_obj);
		//buff->port = (uint8_t)strtoll(rtext_ptr, NULL, 10);
		/*interface*/
		f_gets(rtext_ptr, SD_INIT_MALLOC_SZ, &file_obj);
		buff->interface = (uint8_t)strtoll(rtext_ptr, NULL, 10);
		 /*power_supply*/
		f_gets(rtext_ptr, SD_INIT_MALLOC_SZ, &file_obj);
		buff->power_supply = (uint8_t)strtoll(rtext_ptr, NULL, 10);
		/*sensor_code*/
		f_gets(rtext_ptr, SD_INIT_MALLOC_SZ, &file_obj);
		buff->sensor_code = (uint16_t)strtoll(rtext_ptr, NULL, 16);
		
		if((buff->sensor_code == 0) && (cfg_typ == SENSREGN))
		{
			sd_stat = FAILED;	
		}
		
		if (sd_stat)
		{
		  	f_gets(rtext_ptr, SD_INIT_MALLOC_SZ, &file_obj);
			buff->interval = (uint16_t)strtoll(rtext_ptr, NULL, 10);	
			/*excite time*/
			f_gets(rtext_ptr, SD_INIT_MALLOC_SZ, &file_obj);
			buff->excite_time = (uint32_t)strtoll(rtext_ptr, NULL, 10);
			/*depth uom*/
			f_gets(rtext_ptr, SD_INIT_MALLOC_SZ, &file_obj);
			buff->depth_uom = (uint32_t)strtoll(rtext_ptr, NULL, 16);
			/*raw min*/
			f_gets(rtext_ptr, SD_INIT_MALLOC_SZ, &file_obj);
			buff->raw_min = (uint32_t)strtoll(rtext_ptr, NULL, 10);
			/*raw max*/
			f_gets(rtext_ptr, SD_INIT_MALLOC_SZ, &file_obj);
			buff->raw_max = (uint32_t)strtoll(rtext_ptr, NULL, 10);
			/*lower_lim*/
			f_gets(rtext_ptr, SD_INIT_MALLOC_SZ, &file_obj);
			buff->lower_lim = (uint32_t)strtoll(rtext_ptr, NULL, 10);
			/*upper_lim*/
			f_gets(rtext_ptr, SD_INIT_MALLOC_SZ, &file_obj);
			buff->upper_lim = (uint32_t)strtoll(rtext_ptr, NULL, 10);
			/*sensor_type*/
			f_gets(rtext_ptr, SD_INIT_MALLOC_SZ, &file_obj);
			memcpy((char*)(buff->sensor_type), (char*)rtext_ptr,
				   strlen(rtext_ptr) - 1);
			/*raw_uom*/
			f_gets(rtext_ptr, SD_INIT_MALLOC_SZ, &file_obj);
			memcpy((char*)(buff->raw_uom), (char*)rtext_ptr, 
				   strlen(rtext_ptr) - 1);
			/*base_uom*/
			f_gets(rtext_ptr, SD_INIT_MALLOC_SZ, &file_obj);
			memcpy((char*)(buff->base_uom), (char*)rtext_ptr, 
				   strlen(rtext_ptr) - 1);
			/*depth_value*/
			f_gets(rtext_ptr, SD_INIT_MALLOC_SZ, &file_obj);
			memcpy((char*)(buff->depth_value), (char*)rtext_ptr, 
				   strlen(rtext_ptr) - 1);
			/*data_sequence*/
			f_gets(rtext_ptr, SD_INIT_MALLOC_SZ, &file_obj);
			memcpy((char*)(buff->data_sequence), (char*)rtext_ptr, 
				   strlen(rtext_ptr) - 1);
			/*command*/
			f_gets(rtext_ptr, SD_INIT_MALLOC_SZ, &file_obj);
			memcpy((char*)(buff->command), (char*)rtext_ptr, 
				   strlen(rtext_ptr) - 1);
			/*conversion*/
			f_gets(rtext_ptr, SD_INIT_MALLOC_SZ, &file_obj);
			memcpy((char*)(buff->conversion), (char*)rtext_ptr, 
				   strlen(rtext_ptr) - 1);
		}
			
		/*close file*/
		f_res = f_close(&file_obj);
		if(f_res)
			sd_stat = FAILED;
	}
	
	free(rtext_ptr);
	
	return sd_stat;
}

/*==============================================================================
* @brief	
* @param
* @retval
*=============================================================================*/
ReturnStatusEnum sdUpdateSensDrvrOrRegn(CfgTypNameEnum cfg_typ,
										SensorRegistrationStruct *buff_ptr, 
										char *fname_ptr)
{
	ReturnStatusEnum sd_stat = FAILED;
	FRESULT f_res;
	FIL file_obj;
	bool bu_flag = false;
	char *fname_bu_ptr = "backup.txt";
	
	float integer;
	float fractional;
		
	switch(cfg_typ)
	{
	case SENSRDRVR:
		/*go to Sensor Driver Directory*/
		f_res = f_chdir(CFGFOLDER_NAMES[CFG_SENSRDRVR]);
		sd_stat = SUCCESSFUL;		
		break;
	case SENSREGN:
		/*go to Sensor Driver Directory*/
		f_res = f_chdir(CFGFOLDER_NAMES[CFG_SENSREGN]);	
		fname_ptr = memAlloc(SD_ALLOC_SMALL_SZ);
		sprintf(fname_ptr, "port%d.txt", buff_ptr->port);
		sd_stat = SUCCESSFUL;
		break;
	default:
		sd_stat = FAILED;	
		break;
	}
	
	if(!f_res && sd_stat)
	{
		f_res = f_open(&file_obj, fname_ptr, FA_CREATE_NEW | FA_WRITE);
		/*if file already exist, create back-up first before creating new file 
		with same fname*/
		if(!f_res)
		{
			sd_stat = SUCCESSFUL;
		}	
		else if(f_res == FR_EXIST)
		{
                        f_res = f_unlink(fname_bu_ptr);
			f_res = f_rename(fname_ptr, fname_bu_ptr);

			/*reopen file*/
			f_res = f_open(&file_obj, fname_ptr, FA_CREATE_NEW|FA_WRITE);
			if(!f_res)
			{
				bu_flag = true;
				sd_stat = SUCCESSFUL;
			}
		}
	}

	/*start updating created file*/
	if(!f_res && sd_stat)
	{	
		f_printf(&file_obj, "%d\n", buff_ptr->status);
		f_printf(&file_obj, "%d\n", buff_ptr->port);
		f_printf(&file_obj, "%d\n", buff_ptr->interface);
		f_printf(&file_obj, "%d\n", buff_ptr->power_supply);
		f_printf(&file_obj, "0x%02X\n", buff_ptr->sensor_code);
		f_printf(&file_obj, "%d\n", buff_ptr->interval);
		f_printf(&file_obj, "%d\n", buff_ptr->excite_time);
		f_printf(&file_obj, "0x%02X\n", buff_ptr->depth_uom);
		
		fractional = FLOAT_TO_INT(buff_ptr->raw_min, &integer);
		f_printf(&file_obj, "%d.%d\n", (int32_t)integer, (int32_t)fractional);
		fractional = FLOAT_TO_INT(buff_ptr->raw_max, &integer);
		f_printf(&file_obj, "%d.%d\n", (int32_t)integer, (int32_t)fractional);
		fractional = FLOAT_TO_INT(buff_ptr->lower_lim, &integer);
		f_printf(&file_obj, "%d.%d\n", (int32_t)integer, (int32_t)fractional);
		fractional = FLOAT_TO_INT(buff_ptr->upper_lim, &integer);
		f_printf(&file_obj, "%d.%d\n", (int32_t)integer, (int32_t)fractional);		
		
		f_printf(&file_obj, "%s\n", buff_ptr->sensor_type);
		f_printf(&file_obj, "%s\n", buff_ptr->raw_uom);
		f_printf(&file_obj, "%s\n", buff_ptr->base_uom);
		f_printf(&file_obj, "%s\n", buff_ptr->depth_value);
		f_printf(&file_obj, "%s\n", buff_ptr->data_sequence);
		f_printf(&file_obj, "%s\n", buff_ptr->command);
		f_printf(&file_obj, "%s\n", buff_ptr->conversion);
	
		if(bu_flag)
		{	/*delete orig file if backup was created*/
			f_res = f_unlink(fname_bu_ptr);
		}
		f_res = f_close(&file_obj);
		if(f_res)	
			sd_stat = FAILED;
	}
	
	return sd_stat;
}


/*==============================================================================
* @brief	copy cfg list files to sram upon boot-up
* @param	
* @retval	
*=============================================================================*/
static ReturnStatusEnum sdCfgCopyList(CfgTypNameEnum cfg_typ)
{
	ReturnStatusEnum sd_stat;
	FRESULT f_res;
	char *fname_ptr = NULL;
	FIL file_obj;
	uint8_t num_file, j;	
		
	/* check cfg type if correct*/
	if((cfg_typ == CODEDLIST) || (cfg_typ == NODELIST))
		sd_stat = SUCCESSFUL;
	
	/* change dir*/
	f_res = f_chdir(CFGFOLDER_NAMES[cfg_typ + 1]);
	
	if(sd_stat && (!f_res))
	{		
		num_file = SD_LSTTYP_FILENUM(cfg_typ);
		cfg_liststr_ptr[cfg_typ - 1] 
			= (CfgStringStruct *)memAlloc(sizeof(uint32_t) * num_file);
		
		//TODO: error handling				
		for(j = 0; j < num_file; j++)
		{
			fname_ptr = ((cfg_typ == CODEDLIST) 
							? CD_TYP_FNAMES[j]: NL_TYP_FNAMES[j]);
			/*open file*/
			sd_stat = sdReadInit(fname_ptr, &file_obj);
			/*copy contents of file to global container*/			
			if(sd_stat)
			{
				sd_stat = sdReadCfgTxtFile(&file_obj, 
										   &cfg_liststr_ptr[cfg_typ - 1][j],
										   j);

				/*close file*/
				f_res = f_close(&file_obj);
			}				
		}
	}
	
	return sd_stat;
}

/*==============================================================================
* @brief
* @param
* @retval
*=============================================================================*/
static ReturnStatusEnum sdReadCfgTxtFile(FIL *file_obj_ptr,
										 CfgStringStruct* hdl_ptr,
										 uint8_t file_num)
{
	char *rtext_ptr;
	uint32_t str_addr;
	uint16_t file_sz, offset = 0;
	uint16_t len_inc = 0;
	int8_t i;

	/* read first row for the number of code types*/
	rtext_ptr = (char*)memAlloc(SD_INIT_MALLOC_SZ);
	
	f_gets(rtext_ptr, sizeof(rtext_ptr), file_obj_ptr);
	hdl_ptr->strline_num = strtol(rtext_ptr, NULL, 10);
	
	hdl_ptr->offset_ptr = memAlloc((hdl_ptr->strline_num + 1) * sizeof(uint16_t));
	file_sz = f_size(file_obj_ptr) - (strlen(rtext_ptr) - 1)
				- (hdl_ptr->strline_num * SD_NEWLINECHAR_CNT);
	hdl_ptr->str_ptr = memAlloc(file_sz);
	
	str_addr = (uint32_t)hdl_ptr->str_ptr;
	
	/* read succeeding lines from text file*/
	for(i = 0; i < hdl_ptr->strline_num; i++)
	{
		f_gets(rtext_ptr, SD_INIT_MALLOC_SZ, file_obj_ptr);
		//cfg_str.str_ptr = (uint8_t*) addr;
		strcpy((char *)str_addr, rtext_ptr);
		*((uint16_t *)hdl_ptr->offset_ptr + len_inc) = offset;
		len_inc++;
		offset += (strlen(rtext_ptr) - 1);
		str_addr = (uint32_t)(hdl_ptr->str_ptr + offset);	
	}
	
	/*also for last string, get offset and store*/
	if(i == hdl_ptr->strline_num)
	{
		*((uint16_t *)hdl_ptr->offset_ptr + len_inc) = offset + 1;
	}	
	
	return SUCCESSFUL;
}

/*==============================================================================
* @brief	Deallocate pointers used as containers of cfg data
* @param
* @retval
*=============================================================================*/
void sdDeallocHandle(void)
{
	uint8_t i;
		
	for(i = 0; i < SD_CFGLISTTYP_CNT; i++)
	{
		free(cfg_liststr_ptr[i]->offset_ptr);
		free(cfg_liststr_ptr[i]->str_ptr);
		free(cfg_liststr_ptr[i]);
	}
}

/*==============================================================================
* @brief 	writes to he givern buffer the requested string from list txt file
			used to access member strings of a Coded List and Node List txt file	
* @param
* @retval	len
*=============================================================================*/
uint8_t sdGetCfgListVal(CfgTypNameEnum cfg_typ, 
						uint8_t list_typ,
						uint16_t hexcode, uint8_t *buff_ptr)
{
	uint16_t offset0, offset1;
	uint16_t len;
	
	offset0 = *(cfg_liststr_ptr[cfg_typ][list_typ].offset_ptr 
				 		+ hexcode);
	offset1 = *(cfg_liststr_ptr[cfg_typ][list_typ].offset_ptr 
				 		+ (hexcode + 1));
	len = offset1 - offset0;
	
	memcpy((char *)buff_ptr, 
		   (uint8_t *)cfg_liststr_ptr[cfg_typ][list_typ].str_ptr + offset0,
		   len);

	return len;
}

ReturnStatusEnum sdCfgGetRadNetInfo(void)
{
	ReturnStatusEnum sd_stat;
	volatile FRESULT f_res;
	char *fname_ptr = (char *)SD_FNAME_RADMBID;
	FIL file_obj;
	char *mb_id;
	
	/*malloc poiner for account ID */
	mb_id = NULL; /*Null value initially*/
	
	f_res = f_chdir(CFGFOLDER_NAMES[CFG_ACCTDEV]);
	if(!f_res)
		/*open file*/
		sd_stat = sdReadInit(fname_ptr, &file_obj);

	if(sd_stat)
	{
		mb_id = (char *)memAlloc(SD_INIT_MALLOC_SZ);	
		f_gets(mb_id, SD_INIT_MALLOC_SZ, &file_obj);	
		mb_id = (char *)memReAlloc(mb_id , strlen(mb_id));
				
		if( isdigit(mb_id[strlen(mb_id) - 1]) == 0)
			mb_id[strlen(mb_id) - 1] = '\0';
	}
	else
	{
	  	f_res = f_close(&file_obj);
		
	  	sd_stat = sdWriteInit(fname_ptr, &file_obj);
		if(sd_stat != SUCCESSFUL)
			sd_stat = sdWriteInit(fname_ptr, &file_obj);
		if(sd_stat == SUCCESSFUL)
		  	f_res = f_printf(&file_obj, "%d\n", 0);
		
		if(f_res != FR_OK)
		{
		  	f_res = f_close(&file_obj);
			if(f_res != FR_OK)
				sd_stat = FAILED;
		}
		else
		  	sd_stat = FAILED;
	}
	/*close file*/
	f_res = f_close(&file_obj);
	
	if(f_res)
		sd_stat = FAILED;
	
	#if DEVICE_TYPE == DEVICETYPE_GATE
		DEVICE_INFO->rad_modbus_id = MODBUS_GATE;		/*Look for associated account number based on property*/
	#else
		DEVICE_INFO->rad_modbus_id = (uint32_t)strtoul(mb_id, NULL, 10);
		if(DEVICE_INFO->rad_modbus_id == 0)
		  	DEVICE_INFO->rad_modbus_id = MODBUS_DEFAULT;
	#endif
        
    /* james: get network id */
    sd_stat = sdCfgGetNetworkId();
        
	if(mb_id)
	  	free(mb_id);
	return sd_stat;
}

#pragma optimize=none
static ReturnStatusEnum sdGetDeviceInfo(void)
{
	ReturnStatusEnum sd_stat;
	FRESULT f_res;	
	FIL file_obj;
	char *rtext_ptr;		/*buff to contain read text*/
	char *fname_ptr = (char *)SD_FNAME_INFOSERIAL;
	
	DEVICE_INFO =( DeviceInfoStruct*)memAlloc(sizeof(DeviceInfoStruct));

	f_res = f_chdir(CFGFOLDER_NAMES[CFG_DEVINFO]);
	
	if(!f_res)
		/*open file*/
		sd_stat = sdReadInit(fname_ptr, &file_obj);
	
	if(sd_stat)
	{
		rtext_ptr = (char *)memAlloc(SD_INIT_MALLOC_SZ);
		f_gets(rtext_ptr, SD_INIT_MALLOC_SZ, &file_obj);
				if( isalnum(rtext_ptr[strlen(rtext_ptr) - 1]) == 0 )
					rtext_ptr[strlen(rtext_ptr) - 1] = '\0';
				DEVICE_INFO->device_sn = strdup(rtext_ptr);
		
	}
	
	f_res = f_close(&file_obj);
	if(f_res)
		sd_stat = FAILED;
	DEVICE_INFO->arm_fwver = FW_VERSION;
	return sd_stat;
}

static ReturnStatusEnum sdGetSleepTime(void)
{
	ReturnStatusEnum sd_stat;
	FRESULT f_res;	
	FIL file_obj;
	char *sleep_info_ptr = NULL;		/*buff to contain read text*/
	char *fname_ptr = (char *)SD_FNAME_SLEEPTIME;
	

	f_res = f_chdir(CFGFOLDER_NAMES[CFG_SLEEPINFO]);
	
	if(!f_res)
		/*open file*/
		sd_stat = sdReadInit(fname_ptr, &file_obj);
	
	if(sd_stat)
	{
		sleep_info_ptr = (char *)memAlloc(SD_INIT_MALLOC_SZ);	
		f_gets(sleep_info_ptr, SD_INIT_MALLOC_SZ, &file_obj);	
		sleep_info_ptr = (char *)memReAlloc(sleep_info_ptr , strlen(sleep_info_ptr));
		
		if( isdigit(sleep_info_ptr[strlen(sleep_info_ptr) - 1]) == 0)
			sleep_info_ptr[strlen(sleep_info_ptr) - 1] = '\0';
	}
	
	f_res = f_close(&file_obj);
	if(f_res)
		sd_stat = FAILED;
	RTC_SLEEPTIME = (uint32_t)strtoul(sleep_info_ptr, NULL, 10);
	
	if(sleep_info_ptr)
		free(sleep_info_ptr);
	return sd_stat;
}

#if DEVICETYPE == DEVICETYPE_NODE
static ReturnStatusEnum sdGetWaitTimeout(void)
{
	ReturnStatusEnum sd_stat;
	FRESULT f_res;	
	FIL file_obj;
	char *sleep_info_ptr = NULL;		/*buff to contain read text*/
	char *fname_ptr = (char *)SD_FNAME_WAITTIMEOUT;
	
	f_res = f_chdir(CFGFOLDER_NAMES[CFG_SLEEPINFO]);
	
	if(!f_res)
		/*open file*/
		sd_stat = sdReadInit(fname_ptr, &file_obj);
	
	if(sd_stat)
	{
		sleep_info_ptr = (char *)memAlloc(SD_INIT_MALLOC_SZ);	
		f_gets(sleep_info_ptr, SD_INIT_MALLOC_SZ, &file_obj);	
		sleep_info_ptr = (char *)memReAlloc(sleep_info_ptr , strlen(sleep_info_ptr));
		
		if( isdigit(sleep_info_ptr[strlen(sleep_info_ptr) - 1]) == 0)
			sleep_info_ptr[strlen(sleep_info_ptr) - 1] = '\0';
	}
	
	f_res = f_close(&file_obj);
	if(f_res)
		sd_stat = FAILED;
	NODEWAIT_TIMEOUT = (uint32_t)strtoul(sleep_info_ptr, NULL, 10);
	
	if(sleep_info_ptr)
		free(sleep_info_ptr);
	return sd_stat;
}
#endif

#if DEVICE_TYPE == DEVICETYPE_GATE
/*==============================================================================
* @brief	Copy Account Device Information to SRAM to be used later for
* 			posting (only applicable for GATE)
* @param
* @retval	return status 
*=============================================================================*/
ReturnStatusEnum sdCfgGetAcctDev(void)
{
	ReturnStatusEnum sd_stat;
	FRESULT f_res;
	char *fname_ptr = (char *)SD_FNAME_ACCNTID;
	FIL file_obj;
	char *acct_id_ptr;
	
	/*malloc poiner for account ID */
	acct_id_ptr = NULL; /*Null value initially*/
	f_res = f_chdir(CFGFOLDER_NAMES[CFG_ACCTDEV]);
	if(!f_res)
		/*open file*/
		sd_stat = sdReadInit(fname_ptr, &file_obj);
	
	if(sd_stat)
	{
		acct_id_ptr = (char *)memAlloc(SD_INIT_MALLOC_SZ);
		f_gets(acct_id_ptr, SD_INIT_MALLOC_SZ, &file_obj);
		acct_id_ptr = (char *)memReAlloc(acct_id_ptr , strlen(acct_id_ptr));

		if( isdigit(acct_id_ptr[strlen(acct_id_ptr) - 1]) == 0)
			acct_id_ptr[strlen(acct_id_ptr) - 1] = '\0';
	}
	else
	{
	  	f_res = f_close(&file_obj);
		
	  	sd_stat = sdWriteInit(fname_ptr, &file_obj);
		if(sd_stat != SUCCESSFUL)
			sd_stat = sdWriteInit(fname_ptr, &file_obj);
		if(sd_stat == SUCCESSFUL)
		  	f_res = f_printf(&file_obj, "%d\n", 0);
		
		if(f_res != FR_OK)
		{
		  	f_res = f_close(&file_obj);
			if(f_res != FR_OK)
				sd_stat = FAILED;
		}
		else
		  	sd_stat = FAILED;
	}
	
	/*close file*/
	f_res = f_close(&file_obj);
	if(f_res)
		sd_stat = FAILED;
	
	#ifdef __HWTEST__
		ACCOUNT_ID = 9;		/*Look for associated account number based on property*/
	#else
		ACCOUNT_ID = (uint32_t)strtoul(acct_id_ptr, NULL, 10);
	#endif
	if(acct_id_ptr)
		free(acct_id_ptr);
	return sd_stat;
}

#pragma optimize=none
static void clearDeviceInfoList()
{
	if(DEVICE_INFO_LIST)
	{
		for(uint8_t i = 1; i <= NODE_COUNT; i++)
		{
			if(DEVICE_INFO_LIST[i].device_sn)
				free(DEVICE_INFO_LIST[i].device_sn);
		}
		free(DEVICE_INFO_LIST);
	}
	
		
}

#pragma optimize=none
ReturnStatusEnum sdGetNodeSNList(void)
{
	ReturnStatusEnum sd_stat;
	FRESULT f_res;	
	FIL file_obj;
	char *rtext_ptr;		/*buff to contain read text*/
	char *fname_ptr = (char *)SD_FNAME_NODELIST;
	uint32_t i = 1;
	
	/*copy gate's info*/
	clearDeviceInfoList();
	DEVICE_INFO_LIST =( DeviceInfoStruct*)memAlloc(sizeof(DeviceInfoStruct));
	memcpy(DEVICE_INFO_LIST, DEVICE_INFO, sizeof(DeviceInfoStruct));
	
	f_res = f_chdir(CFGFOLDER_NAMES[CFG_NODELIST]);
	
	if(!f_res)
		/*open file*/
		sd_stat = sdReadInit(fname_ptr, &file_obj);
	
	if(sd_stat)
	{
		rtext_ptr = (char *)memAlloc(SD_INIT_MALLOC_SZ);

			while(f_gets(rtext_ptr, SD_INIT_MALLOC_SZ, &file_obj) != 0)
			{
				DEVICE_INFO_LIST = (DeviceInfoStruct *)memReAlloc( DEVICE_INFO_LIST, 
							  sizeof(DeviceInfoStruct) * (i + 1) );
				if( isalnum(rtext_ptr[strlen(rtext_ptr) - 1]) == 0 )
					rtext_ptr[strlen(rtext_ptr) - 1] = '\0';
				DEVICE_INFO_LIST[i].device_sn = strdup(rtext_ptr);
				DEVICE_INFO_LIST[i].rad_modbus_id = MODBUS_DEFAULT;
				DEVICE_INFO_LIST[i].network_add_status = NETWORK_ADD_EMPTY;
				i++;
			}
	}
	
	NODE_COUNT = i - 1;
	
//	for(int x = 0; x <= NODE_COUNT; x++)
//	{
//		SERIAL_NUMTEST[x] = strdup(DEVICE_INFO_LIST[x].device_sn);
//	}
//	
	f_res = f_close(&file_obj);
	if(f_res)
		sd_stat = FAILED;

	return sd_stat;
}

#pragma optimize=none
static ReturnStatusEnum sdGetNodeRadDevAddrList(void)
{
	ReturnStatusEnum sd_stat;
	FRESULT f_res;	
	FIL file_obj;
	char *rtext_ptr = NULL;		/*buff to contain read text*/
	char *fname_ptr = (char *)SD_FNAME_NL_BNRDEVADDR;
	uint32_t i = 1;
    
    for(int x = 1; x <= NODE_COUNT; x++)
	{
		 DEVICE_INFO_LIST[x].rad_dev_add = 0xFFFF;
	}
	
	f_res = f_chdir(CFGFOLDER_NAMES[CFG_NODELIST]);
	
	if(!f_res)
		/*open file*/
		sd_stat = sdReadInit(fname_ptr, &file_obj);
	
	if(sd_stat)
	{
		rtext_ptr = (char *)memAlloc(SD_INIT_MALLOC_SZ);
        while(f_gets(rtext_ptr, SD_INIT_MALLOC_SZ, &file_obj) != 0)
        {
            DEVICE_INFO_LIST = (DeviceInfoStruct *)memReAlloc( DEVICE_INFO_LIST, 
                          sizeof(DeviceInfoStruct) * (i + 1) );
            if( isalnum(rtext_ptr[strlen(rtext_ptr) - 1]) == 0 )
                rtext_ptr[strlen(rtext_ptr) - 1] = '\0';
            
           DEVICE_INFO_LIST[i].rad_dev_add = (uint16_t)strtoul(rtext_ptr, NULL, 10);
            i++;
        }
	}
	
	f_res = f_close(&file_obj);
	if(f_res) sd_stat = FAILED;
    if(rtext_ptr) free(rtext_ptr);
    
	return sd_stat;
}

#pragma optimize=none
static ReturnStatusEnum sdGetSensList(void)
{
	ReturnStatusEnum sd_stat;
	FRESULT f_res;	
	FIL file_obj;
	char *rtext_ptr;		/*buff to contain read text*/
	char *fname_ptr = (char *)SD_FNAME_SENSLIST;
	uint32_t i = 0;
	char temp_str[SD_INIT_MALLOC_SZ];
	char *parsed_strings[2];
	SensorCodeStruct SENSOR_CODE_LIST_test[50];
	/*copy gate's info*/
	SENSOR_CODE_LIST =( SensorCodeStruct*)memAlloc(sizeof(SensorCodeStruct));
	
	f_res = f_chdir(CFGFOLDER_NAMES[CFG_CODEDLIST]);
	
	if(!f_res)
		/*open file*/
		sd_stat = sdReadInit(fname_ptr, &file_obj);
	
	if(sd_stat)
	{
		rtext_ptr = (char *)memAlloc(SD_INIT_MALLOC_SZ);

		while(f_gets(rtext_ptr, SD_INIT_MALLOC_SZ, &file_obj) != 0)
		{
			SENSOR_CODE_LIST = (SensorCodeStruct *)memReAlloc( SENSOR_CODE_LIST, 
						  sizeof(SensorCodeStruct) * (i + 1) );
			if( isalnum(rtext_ptr[strlen(rtext_ptr) - 1]) == 0 )
				rtext_ptr[strlen(rtext_ptr) - 1] = '\0';
			memcpy(temp_str, rtext_ptr, SD_INIT_MALLOC_SZ);
			eqStringSplit(parsed_strings,temp_str,",");
			SENSOR_CODE_LIST[i].sc_name = strdup(parsed_strings[1]);
			SENSOR_CODE_LIST[i].sc_code = (uint32_t)strtoul(parsed_strings[0],NULL,16);;
			memcpy(&SENSOR_CODE_LIST_test[i], &SENSOR_CODE_LIST[i], sizeof(SensorCodeStruct));
			eqStringSplitFree(parsed_strings, 2);
			i++;
		}
	}
	
	f_res = f_close(&file_obj);
	if(f_res)
		sd_stat = FAILED;

	return sd_stat;
}

#pragma optimize=none
static ReturnStatusEnum sdGetSensTypeList(void)
{
	ReturnStatusEnum sd_stat;
	FRESULT f_res;	
	FIL file_obj;
	char *rtext_ptr;		/*buff to contain read text*/
	char *fname_ptr = (char *)SD_FNAME_STYPELIST;
	uint32_t i = 0;
	char temp_str[SD_INIT_MALLOC_SZ];
	char *parsed_strings[2];
	SensorTypeStruct SENSOR_TYPE_LIST_test[50];
	
	SENSOR_TYPE_LIST =( SensorTypeStruct*)memAlloc(sizeof(SensorTypeStruct));
	
	f_res = f_chdir(CFGFOLDER_NAMES[CFG_CODEDLIST]);
	
	if(!f_res)
		/*open file*/
		sd_stat = sdReadInit(fname_ptr, &file_obj);
	
	if(sd_stat)
	{
		rtext_ptr = (char *)memAlloc(SD_INIT_MALLOC_SZ);

		while(f_gets(rtext_ptr, SD_INIT_MALLOC_SZ, &file_obj) != 0)
		{
			SENSOR_TYPE_LIST = (SensorTypeStruct *)memReAlloc( SENSOR_TYPE_LIST, 
						  sizeof(SensorTypeStruct) * (i + 1) );
			if( isalnum(rtext_ptr[strlen(rtext_ptr) - 1]) == 0 )
				rtext_ptr[strlen(rtext_ptr) - 1] = '\0';
			memcpy(temp_str, rtext_ptr, SD_INIT_MALLOC_SZ);
			eqStringSplit(parsed_strings,temp_str,",");
			SENSOR_TYPE_LIST[i].name = strdup(parsed_strings[1]);
			SENSOR_TYPE_LIST[i].hexcode = (uint32_t)strtoul(parsed_strings[0],NULL,16);;
			memcpy(&SENSOR_TYPE_LIST_test[i], &SENSOR_TYPE_LIST[i], sizeof(SensorCodeStruct));
			eqStringSplitFree(parsed_strings, 2);
			i++;
		}
	}
	
	f_res = f_close(&file_obj);
	if(f_res)
		sd_stat = FAILED;

	return sd_stat;
}

#pragma optimize=none
static ReturnStatusEnum sdGetSensUomList(void)
{
	ReturnStatusEnum sd_stat;
	FRESULT f_res;	
	FIL file_obj;
	char *rtext_ptr;		/*buff to contain read text*/
	char *fname_ptr = (char *)SD_FNAME_UOMLIST;
	uint32_t i = 0;
	char temp_str[SD_INIT_MALLOC_SZ];
	char *parsed_strings[2];
	SensorUomStruct SENSOR_UOM_LIST_test[50];
	
	SENSOR_UOM_LIST =( SensorUomStruct*)memAlloc(sizeof(SensorUomStruct));
	
	f_res = f_chdir(CFGFOLDER_NAMES[CFG_CODEDLIST]);
	
	if(!f_res)
		/*open file*/
		sd_stat = sdReadInit(fname_ptr, &file_obj);
	
	if(sd_stat)
	{
		rtext_ptr = (char *)memAlloc(SD_INIT_MALLOC_SZ);

		while(f_gets(rtext_ptr, SD_INIT_MALLOC_SZ, &file_obj) != 0)
		{
			SENSOR_UOM_LIST = (SensorUomStruct *)memReAlloc( SENSOR_UOM_LIST, 
						  sizeof(SensorUomStruct) * (i + 1) );
			if( isalnum(rtext_ptr[strlen(rtext_ptr) - 1]) == 0 )
				rtext_ptr[strlen(rtext_ptr) - 1] = '\0';
			memcpy(temp_str, rtext_ptr, SD_INIT_MALLOC_SZ);
			eqStringSplit(parsed_strings,temp_str,",");
			SENSOR_UOM_LIST[i].name = strdup(parsed_strings[1]);
			SENSOR_UOM_LIST[i].hexcode = (uint32_t)strtoul(parsed_strings[0],NULL,16);;
			memcpy(&SENSOR_UOM_LIST_test[i], &SENSOR_UOM_LIST[i], sizeof(SensorUomStruct));
			eqStringSplitFree(parsed_strings, 2);
			i++;
		}
	}
	
	f_res = f_close(&file_obj);
	if(f_res)
		sd_stat = FAILED;

	return sd_stat;
}

#pragma optimize=none
ReturnStatusEnum sdUpdateAcctID(uint32_t account_id)
{
  	ReturnStatusEnum sd_stat;
	FRESULT f_res;
	FILINFO f_info;
	char *fname_ptr = (char *)SD_FNAME_ACCNTID;
	FIL file_obj;
	const void *buff = "";
	
	f_res = f_chdir(CFGFOLDER_NAMES[CFG_ACCTDEV]);
	if(f_res == FR_OK)
		f_res = f_unlink(fname_ptr);
	
	if(f_res == FR_OK)
	{
	  	sd_stat = sdWriteInit(fname_ptr, &file_obj);
		if(sd_stat != SUCCESSFUL)
			sd_stat = sdWriteInit(fname_ptr, &file_obj);
		if(sd_stat == SUCCESSFUL)
		  	f_res = f_printf(&file_obj, "%d\n", account_id);
		
		if(f_res != FR_OK)
		{
		  	f_res = f_close(&file_obj);
			if(f_res != FR_OK)
				sd_stat = FAILED;
		}
		else
		  	sd_stat = FAILED;

	}
	sdCfgGetAcctDev();
	if(account_id != ACCOUNT_ID)
	  	assert(0);
	return sd_stat;
}
#endif

#pragma optimize=none
ReturnStatusEnum sdUpdateDeviceRadioModbusID(uint32_t modbus_id)
{
  	ReturnStatusEnum sd_stat;
	FRESULT f_res;
	FILINFO f_info;
	char *fname_ptr = (char *)SD_FNAME_RADMBID;
	FIL file_obj;
	const void *buff = "";
	
	f_res = f_chdir(CFGFOLDER_NAMES[CFG_ACCTDEV]);
	if(f_res == FR_OK)
		f_res = f_unlink(fname_ptr);
	
	if(f_res == FR_OK)
	{
	  	sd_stat = sdWriteInit(fname_ptr, &file_obj);
		if(sd_stat != SUCCESSFUL)
			sd_stat = sdWriteInit(fname_ptr, &file_obj);
		if(sd_stat == SUCCESSFUL)
		  	f_res = f_printf(&file_obj, "%d\n", modbus_id);
		
		f_res = f_close(&file_obj);
		if(f_res != FR_OK)
		{
			if(f_res != FR_OK)
				assert(0);
		}

	}
	sdCfgGetRadNetInfo();
	
	#if DEVICE_TYPE == DEVICETYPE_NODE
		if(modbus_id != DEVICE_INFO->rad_modbus_id)
			assert(0);
	#endif	
	return sd_stat;
}

#pragma optimize=none
ReturnStatusEnum sdCfgGetNetworkId(void)
{
	ReturnStatusEnum sd_stat;
	volatile FRESULT f_res;
	char *fname_ptr = (char *)SD_FNAME_NETID;
	FIL file_obj;
	char *netid_ptr;
	
	/*malloc poiner for network ID */
	netid_ptr = NULL; /*Null value initially*/
	
	f_res = f_chdir(CFGFOLDER_NAMES[CFG_ACCTDEV]);
	if(!f_res)
		/*open file*/
		sd_stat = sdReadInit(fname_ptr, &file_obj);

	if(sd_stat)
	{
		netid_ptr = (char *)memAlloc(SD_INIT_MALLOC_SZ);	
		f_gets(netid_ptr, SD_INIT_MALLOC_SZ, &file_obj);	
		netid_ptr = (char *)memReAlloc(netid_ptr , strlen(netid_ptr));
				
		if( isdigit(netid_ptr[strlen(netid_ptr) - 1]) == 0)
			netid_ptr[strlen(netid_ptr) - 1] = '\0';
        
        //set network id in global variable
        DEVICE_INFO->network_id = (uint32_t)strtoul(netid_ptr, NULL, 10);
	}
	else
	{
	  	f_res = f_close(&file_obj);
		
	  	sd_stat = sdWriteInit(fname_ptr, &file_obj);
		if(sd_stat != SUCCESSFUL)
			sd_stat = sdWriteInit(fname_ptr, &file_obj);
		if(sd_stat == SUCCESSFUL)
        {
            f_res = f_printf(&file_obj, "%d\n", DEFAULT_NETID);
            //set network id in global variable
            DEVICE_INFO->network_id = DEFAULT_NETID;
        }
		
		if(f_res != FR_OK)
		{
		  	f_res = f_close(&file_obj);
			if(f_res != FR_OK)
				sd_stat = FAILED;
		}
		else
		  	sd_stat = FAILED;
	}
	/*close file*/
	f_res = f_close(&file_obj);
	
	if(f_res)
		sd_stat = FAILED;
	
	if(netid_ptr)
	  	free(netid_ptr);
	return sd_stat;
}

#pragma optimize=none
ReturnStatusEnum sdUpdateNetworkId(uint32_t a_netid)
{
  	ReturnStatusEnum sd_stat;
	FRESULT f_res;
	FILINFO f_info;
	char *fname_ptr = (char *)SD_FNAME_NETID;
	FIL file_obj;
	const void *buff = "";
	
	f_res = f_chdir(CFGFOLDER_NAMES[CFG_ACCTDEV]);
	if(f_res == FR_OK)
		f_res = f_unlink(fname_ptr);
	
	if(f_res == FR_OK || FR_NO_FILE)
	{
	  	sd_stat = sdWriteInit(fname_ptr, &file_obj);
		if(sd_stat != SUCCESSFUL)
			sd_stat = sdWriteInit(fname_ptr, &file_obj);
		if(sd_stat == SUCCESSFUL)
		  	f_res = f_printf(&file_obj, "%d\n", a_netid);
		
		f_res = f_close(&file_obj);
	}
    
	sdCfgGetNetworkId();
	if(a_netid != DEVICE_INFO->network_id)
    {
        sd_stat = FAILED;
    }
    
	return sd_stat;
}


#if DEVICE_TYPE == DEVICETYPE_GATE
ReturnStatusEnum sdUpdateDeviceNodeList(char **nodelist, uint32_t count)
{
  	ReturnStatusEnum sd_stat;
	FRESULT f_res;
	FILINFO f_info;
	char *fname_ptr = (char *)SD_FNAME_NODELIST;
	FIL file_obj;
	const void *buff = "";
	
	f_res = f_chdir(CFGFOLDER_NAMES[CFG_NODELIST]);
	if(f_res == FR_OK)
		f_res = f_unlink(fname_ptr);
	
	if(f_res == FR_OK)
	{
	  	sd_stat = sdWriteInit(fname_ptr, &file_obj);
		if(sd_stat != SUCCESSFUL)
			sd_stat = sdWriteInit(fname_ptr, &file_obj);
		
		if(sd_stat == SUCCESSFUL)
		{
		  	for(uint8_t i = 0; i < count; i++)
			{
		  		f_res = f_printf(&file_obj, "%s\n", nodelist[i]);
			}
		}
		
		f_res = f_close(&file_obj);
		if(f_res != FR_OK)
		{
			if(f_res != FR_OK)
				assert(0);
		}

	}
	return sd_stat;
}
#endif
/*******************************************************************************
  * Revision History
  * @file		sdio.c
  ******************************************************************************
  * @version	v2.2.0	
  * @date		04/08/16
  * @author		M.I. Lorica
  * @changes     - created file
  ******************************************************************************
  */