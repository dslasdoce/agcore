/*******************************************************************************
  * @file      	sdMain.h
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

#ifndef __SDIOMAIN_H
#define __SDIOMAIN_H


/* Includes ==================================================================*/
#include "stm32f7xx_hal.h"
#include "productCfg.h"
#include "ff_gen_drv.h"
#include "agBoardInit.h"
#include "sd_diskio.h"
#include "dmMain.h"
#include "sdProcess.h"

/* Contants ==================================================================*/

/* Typedefs ==================================================================*/
typedef enum
{	
	SENSRDRVR,
	CODEDLIST,
	NODELIST,
	ACCTDEV,
	SENSREGN
} CfgTypNameEnum;

/* List of existing coded list text files, UPDATE if necessary*/
typedef enum
{	
	SD_SENSCODE,
	SD_SENSTYP,
	SD_PORT,
	SD_RAWUOM,
	SD_BASEUOM,
	SD_MISCUOM,
	SD_INTRFC,
	SD_POWSUP,
	SD_VLVTYP,
	SD_VLVCODE,
	SD_POWREQMT,
	SD_LOGTYP,
	SD_MODULE
} CfgCodedLstTypEnum;


typedef enum
{	
	SD_DEV_SN,
	SD_MODBUSID,
	SD_BNRRDSN,
	SD_XBRDSN
} CfgNodeLstTypEnum;

/* for Cfg files global sram storage */
typedef struct
{
	uint8_t strline_num;
	uint16_t *offset_ptr;	/*length per data string per line*/
	char *str_ptr;		/*string pointer*/
} CfgStringStruct;


/* Defines ===================================================================*/
#define SD_CFGTYP_CNT			((uint8_t) 4)
#define SD_CFGLISTTYP_CNT		((uint8_t) 2)	//for Coded List and Node List
/*modify if file count for coded list and/or node list file is updated*/
#define SD_CODEDLST_FILENUM		((uint8_t) 13)
#define SD_NODELST_FILENUM		((uint8_t) 2)
/*number of coded list text files*/
#define SD_LSTTYP_FILENUM(cfg_typ)		(cfg_typ == CODEDLIST) ? \
											SD_CODEDLST_FILENUM:SD_NODELST_FILENUM

/* External Declarations =====================================================*/
extern char *acct_id_ptr;
extern CfgStringStruct *cfg_liststr_ptr[SD_CFGLISTTYP_CNT];

/* Function Prototypes =======================================================*/
ReturnStatusEnum sdInit(void);
ReturnStatusEnum sdCfgCopy(void);
ReturnStatusEnum sdCfgTestCopySensDrvr(uint16_t sensor_code,  
										SensorRegistrationStruct *buff_ptr);
ReturnStatusEnum sdCfgMkDir(void);
ReturnStatusEnum sdCfgReadTxtDir(CfgTypNameEnum cfg_typ, uint64_t *bitmap_ptr,
								 uint16_t *content_numptr);
//ReturnStatusEnum sdUserMkDir(void);

ReturnStatusEnum sdReadInit(char *file_name_ptr, FIL *file_obj);
ReturnStatusEnum sdWriteInit(char *file_name_ptr, FIL *file_obj);
ReturnStatusEnum sdGetSensDrvrOrRegn(CfgTypNameEnum cfg_typ,
									 SensorRegistrationStruct *buff,
									 char *fname_ptr);

void sdDeallocHandle(void);
uint8_t sdGetCfgListVal(CfgTypNameEnum cfg_typ, 
						uint8_t list_typ,
						uint16_t hexcode, uint8_t *buff_ptr);
ReturnStatusEnum sdUpdateSensDrvr(SensorRegistrationStruct *buff_ptr, 
								  char *sencode_fname_ptr);

ReturnStatusEnum sdUpdateSensDrvrOrRegn(CfgTypNameEnum cfg_typ,
								  SensorRegistrationStruct *buff_ptr, 
								  char *fname_ptr);
ReturnStatusEnum sdCfgGetRadNetInfo(void);
ReturnStatusEnum sdCfgGetAcctDev(void);
ReturnStatusEnum sdGetNodeSNList(void);
ReturnStatusEnum sdUpdateDeviceRadioModbusID(uint32_t raddevice_num);
ReturnStatusEnum sdUpdateAcctID(uint32_t account_id);
ReturnStatusEnum sdUpdateDeviceNodeList(char **nodelist, uint32_t count);
ReturnStatusEnum sdCfgGetNetworkId(void);
ReturnStatusEnum sdUpdateNetworkId(uint32_t a_netid);

extern SD_HandleTypeDef uSdHandle;
extern SD_CardInfo uSdCardInfo;
#endif /* __SDIO_H */


/*******************************************************************************
  * Revision History
  * @file		sdMain.h
  ******************************************************************************
  * @version	v2.2.0	
  * @date		04/08/16
  * @author		M.I. Lorica
  * @changes     - created file
  ******************************************************************************
  */

