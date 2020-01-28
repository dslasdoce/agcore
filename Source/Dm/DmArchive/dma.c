/**
  ******************************************************************************
  * @file      	dma.c
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
#include <assert.h>

#include "dataCommon.h"
#include "stm32f7xx_hal.h"
#include "agBoardInit.h"
#include "dma.h"

/* Constants ===============================================================*/
#define FLASH_TIMEOUT_VALUE       ((uint32_t)50000)/* 50 s */


/* Global Definitions ======================================================*/
DMA_HandleTypeDef	dma_handle;


/* Functions ===============================================================*/

/*============================================================================
 * @brief	
 * @param	
 * @retval  
============================================================================*/
static void TransferComplete(DMA_HandleTypeDef *dma_handle_ptr)
{
	return;
}

/*============================================================================
 * @brief	
 * @param	
 * @retval  
============================================================================*/
static void TransferError(DMA_HandleTypeDef *dma_handle_ptr)
{
	assert(0);
}
/*============================================================================
 * @brief  Configures the DMA Transfer
 * @retval HAL status
 * @retval  
============================================================================*/
void dmaConfig(void)
{
	/*## -1- Enable DMA2 clock #################################################*/
	__HAL_RCC_DMA2_CLK_ENABLE();

	/*##-2- Select the DMA functional Parameters ###############################*/
	dma_handle.Init.Channel = DMA_CHANNEL_0;
	/* M2M transfer mode*/
	dma_handle.Init.Direction = DMA_MEMORY_TO_MEMORY;
	/* Peripheral increment mode Enable*/
	dma_handle.Init.PeriphInc = DMA_PINC_ENABLE;
	/* Memory increment mode Enable */
	dma_handle.Init.MemInc = DMA_MINC_ENABLE;
	/* Peripheral data alignment : Word */
	dma_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	/* memory data alignment : Word */
	dma_handle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
	/* Normal DMA mode */
	dma_handle.Init.Mode = DMA_NORMAL;
	/* priority level : high            */
	dma_handle.Init.Priority = DMA_PRIORITY_HIGH;//DMA_PRIORITY_HIGH;
	/* FIFO mode enabled */
	dma_handle.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
	dma_handle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	/* Memory burst */
	dma_handle.Init.MemBurst = DMA_MBURST_SINGLE;
	/* Peripheral burst */
	dma_handle.Init.PeriphBurst = DMA_PBURST_SINGLE;

	/*Select the DMA instance to be used for the transfer : DMA2_Stream0 #
	 * only DMA2 is able to do memory-to-memory transfer, in this mode, the circular
	 * and direct modes are not allowed*/
	dma_handle.Instance = DMA2_Stream0;

	/*Callback functions called after Transfer complete and Transfer error */
	dma_handle.XferCpltCallback  = TransferComplete;
	dma_handle.XferErrorCallback = TransferError;
}

/*============================================================================
  * @brief  Starts the DMA Transfer from Flash to RAM (Read)
  * 		or RAM to Flash (Write),
  * @param  src_addr: The source memory Buffer address
  * @param  dest_addr: The destination memory Buffer address
  * @param  dma_size: The length of data to be transferred from source to destination
  * @retval HAL status
============================================================================*/
ReturnStatusEnum dmaExecute(uint32_t src_addr, uint32_t dest_addr, uint16_t dma_size)
{
	uartSHOW((uint8_t*)"\n=====START DMA=====\n", 21);
	__HAL_UART_DISABLE_IT(&UART4HandlerDef, UART_IT_RXNE);
	if(HAL_DMA_Init(&dma_handle) != HAL_OK)
	{
		/* Turn LED3/LED4 on: in case of Initialization Error */
		return FAILED;
	}
	  HAL_NVIC_SetPriority(DMA_STREAM_IRQ, 0, 0);
	  HAL_NVIC_EnableIRQ(DMA_STREAM_IRQ);
	//start dma
	if(HAL_DMA_Start_IT(&dma_handle, src_addr, dest_addr, dma_size) != HAL_OK)
	{
			uartSHOW((uint8_t*)"=====DMA FAILED=====\n\n",21);
			return FAILED;
	}
	__HAL_UART_ENABLE_IT(&UART4HandlerDef, UART_IT_RXNE);
	uartSHOW((uint8_t*)"=====DMA OK=====\n\n",18);
	return SUCCESSFUL;
}

void DMA_STREAM_IRQHANDLER(void)
{
	HAL_DMA_IRQHandler(&dma_handle);
}

/*********************************************************************************
  * Revision History
  *	@file      	dma.c
  ********************************************************************************
  * @version	v.2.2.0
  * @date		04/08/16
  * @author		MI Lorica
  * @changes	changed file directory
  ******************************************************************************	
  * @version    v2.1.0
  * @date       12/23/2015
  * @author		M.I. Lorica
  * @changes	- added DMA IRQ handler
  * 			- simplified DMA read and write to DMA execute fxn
  ********************************************************************************
  * @version
  * @date
  * @author		Ma. Irene Lorica
  * @changes	- created file
  ********************************************************************************
  */
