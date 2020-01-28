/********************************************************************************
  * @file      	sdio.c
  * @author		Hardware Team
  * @version	v2.0.2
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

/* Include ===================================================================*/
#include "ff.h"
#include "agBoardInit.h"
#include "sdMain.h"

/* Global Declarations =======================================================*/
char sd_path[4]; /* SD card logical drive path */
FATFS sd_fatfs;  /* File system object for SD card logical drive */
DMA_HandleTypeDef hdma_sdio_tx;
DMA_HandleTypeDef hdma_sdio_rx;

/* Functions =================================================================*/

/*==============================================================================
* @brief
* @param
* @retval
*=============================================================================*/
ReturnStatusEnum sdInit(void)
{
	ReturnStatusEnum sd_stat;
	FRESULT fr_stat;

	sd_stat = FAILED;
	sdInitHardware();
	if(!FATFS_LinkDriver(&SD_Driver, sd_path))
	{ 
		/*  Register the file system object to the FatFs module */
		fr_stat = f_mount(&sd_fatfs, (TCHAR const*)sd_path, 0);	
		if(!fr_stat)
		{
			sd_stat = sdCfgMkDir();
			if(sd_stat)
			{	
				/* temp: copy coded list and node list from sdcard*/
				sd_stat = sdCfgCopy();
			}			
		}
	}
	return sd_stat;
}

#pragma optimize=none
/*==============================================================================
* @brief	Initialize reading by opening corresponding file
* @param
* @retval
*=============================================================================*/
ReturnStatusEnum sdReadInit(char *file_name_ptr, FIL *file_obj)
{
	volatile FRESULT res;
	ReturnStatusEnum sd_stat;
	
	sd_stat = FAILED;
	
	/* Opens an existing file. If it does not exist, create a new file. */
	res = f_open(file_obj, file_name_ptr, FA_READ);
	if(!res)
		sd_stat = SUCCESSFUL;

	return sd_stat;
}

#pragma optimize=none
/*==============================================================================
* @brief	Initialize reading by opening corresponding file
* @param
* @retval
*=============================================================================*/
ReturnStatusEnum sdWriteInit(char *file_name_ptr, FIL *file_obj)
{
	volatile FRESULT res;
	ReturnStatusEnum sd_stat;
	
	sd_stat = FAILED;
	
	/* Opens an existing file. If it does not exist, create a new file. */
	res = f_open(file_obj, file_name_ptr, FA_WRITE);
	if(!res)
		sd_stat = SUCCESSFUL;
	else
	{
	  	res = f_open(file_obj, file_name_ptr, FA_CREATE_NEW | FA_WRITE);
		f_close(file_obj);
	}
	  
	return sd_stat;
}

/*==============================================================================
* @brief
* @param
* @retval
==============================================================================*/
void DMA2_Stream5_IRQHandler(void)
{
	HAL_DMA_IRQHandler(uSdHandle.hdmarx);
  //BSP_SD_DMA_Rx_IRQHandler();
}

/*==============================================================================
* @brief
* @param
* @retval
=============================================================================*/
void DMA2_Stream0_IRQHandler(void)
{
	HAL_DMA_IRQHandler(uSdHandle.hdmatx);
  //BSP_SD_DMA_Tx_IRQHandler();
}

/*==============================================================================
* @brief	This function handles SDIO interrupt request.
* @param
* @retval
==============================================================================*/
void SDMMC2_IRQHandler(void)
{
  BSP_SD_IRQHandler();
}


/*******************************************************************************
  * Revision History
  * @file
  ******************************************************************************
  * @version	v2.2.0	
  * @date		04/08/16
  * @author		M.I. Lorica
  * @changes     - created file
  ******************************************************************************
  */

