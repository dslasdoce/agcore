/********************************************************************************
  * @file      	intFlash.c
  * @author     Hardware Team
  * @version    v2.2.1
  * @date       
  * @brief		Manages internal flash access (program/erase/read).
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

/* Includes ==================================================================*/
#include <deployAssert.h>
#include "intFlash.h"

#include <stm32f7xx_hal_dma.h>
	  
/* Constants =================================================================*/

	  
/* External Declarations=====================================================*/
DMA_HandleTypeDef	dma_hdle;
FlBuffTrackerStruct fl_handle[REC_TYP_CNT_TO_LOG];
//uint32_t sensor_regn_fl_last_addr;

/* Defines ===================================================================*/
#define	INTFL_WORD_UNUSED			((uint32_t)0xFFFFFFFFFFFFFFFF)
#define FLASH_TIMEOUT_VALUE			((uint32_t)50000)/* 50 s */


/* Functions =================================================================*/

/*============================================================================
* @brief     
* @param
* @retval
*============================================================================*/
uint32_t intFlReadWord(uint32_t src_addr)
{
	__IO uint32_t 	data32;

	/*Copy record from flash to SRAM*/

	data32 = *(__IO uint32_t*)src_addr;
	src_addr = src_addr + sizeof(uint32_t);

	return data32;
}

/*============================================================================
 * @brief	
 * @param	
 * @retval  
============================================================================*/
static void intFlDmaTxComplete(DMA_HandleTypeDef *dma_handle_ptr)
{
	/*TODO: add implementation here*/
	return;
}

/*============================================================================
 * @brief	
 * @param	
 * @retval  
============================================================================*/
static void intFlDmaTxError(DMA_HandleTypeDef *dma_handle_ptr)
{
	/*TODO: add implementation here*/
	assert(0);
}

/*============================================================================
 * @brief  Configures the DMA Transfer
 * @retval HAL status
 * @retval  
============================================================================*/
static void intFlDmaConfig(void)
{
	/*## -1- Enable DMA2 clock #################################################*/
	__HAL_RCC_DMA2_CLK_ENABLE();

	/*##-2- Select the DMA functional Parameters ###############################*/
	dma_hdle.Init.Channel = DMA_CHANNEL_0;
	/* M2M transfer mode*/
	dma_hdle.Init.Direction = DMA_MEMORY_TO_MEMORY;
	/* Peripheral increment mode Enable*/
	dma_hdle.Init.PeriphInc = DMA_PINC_ENABLE;
	/* Memory increment mode Enable */
	dma_hdle.Init.MemInc = DMA_MINC_ENABLE;
	/* Peripheral data alignment : Word */
	dma_hdle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	/* memory data alignment : Word */
	dma_hdle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
	/* Normal DMA mode */
	dma_hdle.Init.Mode = DMA_NORMAL;
	/* priority level : high            */
	dma_hdle.Init.Priority = DMA_PRIORITY_HIGH;
	/* FIFO mode enabled */
	dma_hdle.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
	dma_hdle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	/* Memory burst */
	dma_hdle.Init.MemBurst = DMA_MBURST_SINGLE;
	/* Peripheral burst */
	dma_hdle.Init.PeriphBurst = DMA_PBURST_SINGLE;

	/*Select the DMA instance to be used for the transfer : DMA2_Stream0 #
	 * only DMA2 is able to do memory-to-memory transfer, in this mode, the circular
	 * and direct modes are not allowed*/
	dma_hdle.Instance = INTFL_DMA_STREAM;

	/*Callback functions called after Transfer complete and Transfer error */
	dma_hdle.XferCpltCallback  = intFlDmaTxComplete;
	dma_hdle.XferErrorCallback = intFlDmaTxError;
}

/*============================================================================
 * @brief	
 * @param
 * @retval
 *============================================================================*/
ReturnStatusEnum intFlashInit(void)
{
	uint8_t i;
	
	/* Configure DMA peripheral*/
	intFlDmaConfig();
	
	/*initialize internal flash tracker global variables*/
	/*for first time use, initialize flash setting after reset*/
	/*TODO: redesign this since it is possible that addr contains 
	  valid 0xFFFFFFFF*/
	
	if(intFlReadWord(FL_SETTINGS_START_ADDR) == INTFL_WORD_UNUSED)
	{
		for(i = 0; i < REC_TYP_CNT_TO_LOG; i++)
		{
			fl_handle[i].last_rec_addr = FL_START_TEMP_ADDR(i);
			fl_handle[i].last_rec_idx = 0;
			fl_handle[i].unsent_rec_count = 0;
			fl_handle[i].unsent_tracker_addr = fl_handle[i].last_rec_addr;
			fl_handle[i].unsent_rec_cnt_in_sr = 0;
			fl_handle[i].sent_last_rec_addr = FL_START_SENT_ADDR(i);
			fl_handle[i].sent_last_rec_idx = 0;
			fl_handle[i].sent_rec_count = 0;
		}
	}
	else
	{
		for(i = 0; i < REC_TYP_CNT_TO_LOG; i++)
		{
			if((intFlDmaExec(FL_SETTINGS_ADDR(i),
							(uint32_t)&fl_handle[i],
							REC_FL_SETTINGS_SZ_BYTES/sizeof(uint32_t))) != SUCCESSFUL)
				return FAILED;
		}
	}
	
	return SUCCESSFUL;
}
//:TODO
/*============================================================================
 * @brief	
 * @param	
 * @retval  
============================================================================*/
void INTFL_DMA_STREAM_IRQHANDLER(void)
{
	HAL_DMA_IRQHandler(&dma_hdle);
}

/*============================================================================
  * @brief  Executes the DMA Transfer from Flash to RAM (Read)
  * 		or RAM to Flash (Write),
  * @param  src_addr: The source memory Buffer address
  * @param  dest_addr: The destination memory Buffer address
  * @param  dma_size: The length of data to be transferred from source to destination
  * @retval HAL status
============================================================================*/
ReturnStatusEnum intFlDmaExec(uint32_t src_addr, uint32_t dest_addr, 
							  uint32_t dma_size)
{
	/*disable uart interrupt for the whole duration of DMA IRQ*/
//	__HAL_UART_DISABLE_IT(&UART4HandlerDef, UART_IT_RXNE);

    //SCB_CleanDCache();
    SCB_CleanInvalidateDCache();

	if(HAL_DMA_Init(&dma_hdle) != HAL_OK)
	{
		return FAILED;
	}

	HAL_NVIC_SetPriority(INTFL_DMA_STREAM_IRQ, 0, 0);
	HAL_NVIC_EnableIRQ(INTFL_DMA_STREAM_IRQ);
		
	if(HAL_DMA_Start_IT(&dma_hdle, src_addr, dest_addr, dma_size) != HAL_OK)
	{
			return FAILED;
	}
	/* Enable uart interrupt for the whole duration of DMA IRQ*/
//	__HAL_UART_ENABLE_IT(&UART4HandlerDef, UART_IT_RXNE);

	return SUCCESSFUL;
} 


/*============================================================================
* @brief     
* @param
* @retval
*============================================================================*/
ReturnStatusEnum intFlErase(uint8_t first_sector, uint8_t num_sec)
{
	ReturnStatusEnum	ret_stat;
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t sector_error = 0;
	

	ret_stat = FAILED;

	/*disable uart interrupt*/
//	__HAL_UART_DISABLE_IT(&UART4HandlerDef, UART_IT_RXNE);
	
	/* Unlock the Flash to enable the flash control register access */
	HAL_FLASH_Unlock();

	/* Erase the user Flash area
	 * (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) */
	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	/* Get the 1st sector to erase */
	EraseInitStruct.Sector = first_sector;
	/* Get the number of sector to erase from 1st sector*/
	EraseInitStruct.NbSectors = num_sec;

	if(HAL_FLASHEx_Erase(&EraseInitStruct, &sector_error) != HAL_OK)
	{
		/*TODO: add implementation if erasing fails*/
		ret_stat = FAILED;
	}
	else
		ret_stat = SUCCESSFUL;

	/* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
	you have to make sure that these data are rewritten before they are accessed during code
	execution. If this cannot be done safely, it is recommended to flush the caches by setting the
	DCRST and ICRST bits in the FLASH_CR register. */
	
        // abarbaso??? Commented out since FLASH_ACR register no longer have these fields
        
        //__HAL_FLASH_DATA_CACHE_DISABLE();
	//__HAL_FLASH_INSTRUCTION_CACHE_DISABLE();

	//__HAL_FLASH_DATA_CACHE_RESET();
	//__HAL_FLASH_INSTRUCTION_CACHE_RESET();

	//__HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
	//__HAL_FLASH_DATA_CACHE_ENABLE();

	HAL_FLASH_Lock();
	
	/*enable uart interrupt*/
//	__HAL_UART_ENABLE_IT(&UART4HandlerDef, UART_IT_RXNE);
	
	return ret_stat;
}

/*============================================================================
* @brief    called when flash temp_buff for either telem and sys stat records 
			is full
* @param
* @retval
*============================================================================*/
ReturnStatusEnum intFlRecEraseTempBuff(RecordTypEnum rec_typ,
									   uint8_t num_sec)
{
	ReturnStatusEnum ret_stat;
	
	ret_stat = FAILED;
	
	/* save both records to their allocated places in SDCard*/
	//no handling for sdcard transfer for now

	/* flash erase after saving to sdcard */
	if(intFlErase(BU_BUFF_SECNUM(rec_typ), num_sec) == SUCCESSFUL)
	{
		fl_handle[rec_typ].last_rec_addr = FL_START_TEMP_ADDR(rec_typ);
		fl_handle[rec_typ].last_rec_idx = 0;
		fl_handle[rec_typ].unsent_rec_count = 0;
		fl_handle[rec_typ].unsent_tracker_addr = fl_handle[rec_typ].last_rec_addr;		
	
	}
	return ret_stat;
}


/*============================================================================
* @brief     
* @param
* @retval
*============================================================================*/
ReturnStatusEnum intFlEraseSentBuff(void)
{
	ReturnStatusEnum ret_stat;
	uint8_t i;
	
	ret_stat = FAILED;
	
	/* save both records to their allocated places in SDCard*/
	//no handling for sdcard transfer for now

	/* flash erase after saving to sdcard */
	ret_stat = intFlErase(FL_SENTBUFF_SECNUM, 1);

	if(ret_stat == SUCCESSFUL)
	{
		for(i = 0; i < REC_TYP_CNT_TO_LOG; i++)
		{
			fl_handle[i].sent_last_rec_addr = FL_START_SENT_ADDR(i);
			fl_handle[i].sent_last_rec_idx = 0;
			fl_handle[i].sent_rec_count = 0;	
		}
	}

	return ret_stat;
}

/*============================================================================
* @brief     
* @param
* @retval
*============================================================================*/
ReturnStatusEnum intFlSaveSettings(void)
{
	uint8_t i;
	
	//TEMP
	//uint32_t sensor_regn_bu;

	/*copy latest sensor reg to SRAM*/
	//dmaExecute(sensor_regn_fl_last_addr, &sensor_regn_bu,
	//		   sizeof(sensor_regn_bu) / sizeof(uint32_t));

	/* erase sector 11*/
	if(intFlErase(FL_SETTINGS_SECNUM, SENT_SEC_COUNT) == SUCCESSFUL)
	{
		/* copy record settings/flash trackers to flash*/
		for(i = 0; i < REC_TYP_CNT_TO_LOG; i++)
		{	
			if(intFlDmaExec((uint32_t)&fl_handle[i],
						  	FL_SETTINGS_ADDR(i),
						  	(REC_FL_SETTINGS_SZ_BYTES / sizeof(uint32_t))) 
			   					!= SUCCESSFUL)
				return FAILED;
		}
		
		/* update address for latest sensor regn record */
		//sensor_regn_fl_last_addr
		//		= SYSSTAT_FL_TRACKERS_ADDR + REC_FL_SETTINGS_SZ_BYTES;
		/* copy addr of latest sensor regn record to flash */

		/* copy addr of latest sensor regn record to flash */
		//if((dmaExecute(sensor_regn_bu,
		//			  sensor_regn_fl_last_addr,
		//			  sizeof(sensor_regn_bu) / sizeof(uint32_t))) != SUCCESSFUL)
		//	return FAILED;

		//sensor_regn_fl_last_addr += sizeof(sensor_regn_fl_last_addr);
	}
	else
		return FAILED;

	return SUCCESSFUL;
}


/*============================================================================
 * @brief	updates the index and count based on record access
 * @param	rec_access:
 * @retval  none
 ============================================================================*/
void intFlUpdateTracker(IntFlTrackerEnum tracker,
							   		uint16_t record_num, 
							   		RecordTypEnum rec_typ)
{
	switch(tracker)
	{
		case TEMP_BUFF_TRACKER:
		{
		/*update tracker after DMAwrite of UNSENT records*/
			fl_handle[rec_typ].last_rec_addr += (record_num * REC_SZ(rec_typ));
			fl_handle[rec_typ].last_rec_idx += record_num;
			fl_handle[rec_typ].unsent_rec_count += record_num;
			fl_handle[rec_typ].unsent_tracker_addr = fl_handle[rec_typ].last_rec_addr;
			break;
		}
		case TEMP_UNSENT_BUFF_TRACKER:
		{
			fl_handle[rec_typ].unsent_rec_count -= record_num;
			fl_handle[rec_typ].unsent_rec_cnt_in_sr -= record_num;
			break;
		}
		/*update tracker after dma write of SENT rec*/
		case SENT_BUFF_TRACKER:
		{
			fl_handle[rec_typ].sent_last_rec_addr += (record_num * REC_SZ(rec_typ));
			fl_handle[rec_typ].sent_last_rec_idx += record_num;
			fl_handle[rec_typ].sent_rec_count += record_num;
			break;
		}

		default:
			break;
	}
	return;
}

uint32_t intFlRetLastAddr(RecordTypEnum rec_typ)
{

	return fl_handle[rec_typ].last_rec_addr;
}


/*********************************************************************************
  * Revision History
  * @file         
  ********************************************************************************
  * @version    v2.2.1
  * @date       
  * @author		M.I.Lorica
  * @changes
  ********************************************************************************
  * @version    v2.1.0
  * @date       12/23/2015
  * @author		M.I.Lorica
  * @changes
  ********************************************************************************
  * @version
  * @date
  * @author         
  * @changes     - created file
  ********************************************************************************
  */
