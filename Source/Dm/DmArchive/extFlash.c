/**
  ****************************************************************************
  * @file     extFlash.c
  * @author	Hardware Team 
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

#include "delay.h"
#include "stm32f7xx_hal.h"
#include "extFlash.h"
#include "agBoardInit.h"
/* Defines ===================================================================*/

/*Page Size*/
#define PAGE_SZ				(256)
#define TIMER_FREQUENCY_HZ 	(1000u)


#define SPI_DELAY   (500)

#define SPI1_TIMEOUT_MAX		((uint32_t)0x100000)

/*m25 flash specific inctruction codes*/
#define  M25_WREN           0x06    /*  Write Enable instruction */
#define  M25_WRDI           0x04    /*  Write Disable instrction */
#define  M25_RDID           0x9F    /*  Read ID instruction */
#define  M25_RDSR           0x05    /*  Read Status Register instruction */
#define  M25_WRSR           0x01    /*  Write Status Register instruction */
#define  M25_READ           0x03    /*  Read Data Byte instruction */
#define  M25_FASTREAD       0x0B    /*  Read Data Byte at Higher Speed */
#define  M25_PP             0x02    /*  Page Program instruction */
#define  M25_SE             0xD8    /*  Sector Erase instruction */
#define  M25_BE             0xC7    /*  Bulk Erase instruction */
#define  M25_DP             0xB9    /*  Deep Power-down instruction */
#define  M25_RES            0xAB    /*  Release from Deep Power-down, and Read */
                                    /*  Electronic Signature */

/* Typedefs ==================================================================*/

typedef struct
{
	uint8_t		wren;
	uint8_t		wrdi;
	uint8_t		rdid;
	uint8_t		rdsr;
	uint8_t		wrsr;
	uint8_t		rd;
	uint8_t		rdfr;
	uint8_t		pp;
	uint8_t		se;
	uint8_t		be;
	uint8_t		dp;
	uint8_t		res;
} FlM25Cmd;

/* External Declarations======================================================*/
SPI_HandleTypeDef SPIFlashRTCHandle;
FlM25Cmd flm25_cmd;

/* Functions =================================================================*/

/*==============================================================================
* @brief
* @param
* @retval
==============================================================================*/
static void FlM25CmdInit(void)
{
	flm25_cmd.wren = M25_WREN;
	flm25_cmd.wrdi = M25_WRDI;
	flm25_cmd.rdid = M25_RDID;
	flm25_cmd.rdsr = M25_RDSR;
	flm25_cmd.wrsr = M25_WRSR;
	flm25_cmd.rd = M25_READ;
	flm25_cmd.rdfr = M25_FASTREAD;
	flm25_cmd.pp = M25_PP;
	flm25_cmd.se = M25_SE;
	flm25_cmd.be = M25_BE;
	flm25_cmd.dp = M25_DP;
	flm25_cmd.res = M25_RES;

	return;
}

/*==============================================================================
* @brief
* @param
* @retval
==============================================================================*/
//static ReturnStatusEnum extFlRdStatReg(void)
//{
//	uint8_t stat_reg;
//
//	HAL_GPIO_WritePin(GPIOA, FLM25_CS_PIN, GPIO_PIN_RESET);
//
//	if(HAL_SPI_Transmit(&SPIFlashRTCHandle, (uint8_t *)&flm25_cmd.rdsr,
//					sizeof(flm25_cmd.rdsr),
//					SPI1_TIMEOUT_MAX) != HAL_OK)
//		return FAILED;
//
//	if(HAL_SPI_Receive(&SPIFlashRTCHandle, (uint8_t *)&stat_reg,
//					sizeof(stat_reg),
//					SPI1_TIMEOUT_MAX) != HAL_OK)
//		return FAILED;
//
//	HAL_GPIO_WritePin(GPIOA, FLM25_CS_PIN, GPIO_PIN_SET);
//
//	if(stat_reg != FLASH_READY)
//		return FAILED;
//
//	return SUCCESSFUL;
//}

/*==============================================================================
* @brief	external flash initialization
* @param	
* @retval
==============================================================================*/
ReturnStatusEnum extFlashInit(void)
{
	/*Init m25 specific flash commands*/
	FlM25CmdInit();
	
	return SUCCESSFUL;
}

/*==============================================================================
* @brief        external flash read ID
* @param	
* @retval
==============================================================================*/
ReturnStatusEnum extFlReadId (uint8_t *buff_ptr,
                              uint16_t buff_size)
{
    /* A. Set CS pin to low */
    gpioDigitalWrite(&FLM25_CS_PIN,
                     LOW);
    // delayMicros(SPI_DELAY);
    delayMillis(1);

    /* B. Send read ID command */
    if (HAL_SPI_Transmit(&SPIFlashRTCHandle,
                         (uint8_t *)&flm25_cmd.rdid,
                         sizeof(flm25_cmd.rdid),
                         SPI1_TIMEOUT_MAX) != HAL_OK)
        return FAILED;

    /* C. Receive data */
    if (HAL_SPI_Receive(&SPIFlashRTCHandle,
                        (uint8_t *)buff_ptr,
                        buff_size,
                        SPI1_TIMEOUT_MAX) != HAL_OK)
        return FAILED;

    /* D. Set CS pin to high */
    gpioDigitalWrite(&FLM25_CS_PIN,
                     HIGH);

    return SUCCESSFUL;
}

/*==============================================================================
* @brief        external flash read status register
* @param	
* @retval
==============================================================================*/
ReturnStatusEnum extFlReadStatReg (uint8_t *buff_ptr,
                                   uint16_t buff_size)
{
    /* A. Set CS pin to low */
    gpioDigitalWrite(&FLM25_CS_PIN,
                     LOW);
    // delayMicros(SPI_DELAY);
    delayMillis(1);

    /* B. Send read status register command */
    if (HAL_SPI_Transmit(&SPIFlashRTCHandle,
                         (uint8_t *)&flm25_cmd.rdsr,
                         sizeof(flm25_cmd.rdsr),
                         SPI1_TIMEOUT_MAX) != HAL_OK)
        return FAILED;

    /* C. Receive data */
    if (HAL_SPI_Receive(&SPIFlashRTCHandle,
                        (uint8_t *)buff_ptr,
                        buff_size,
                        SPI1_TIMEOUT_MAX) != HAL_OK)
        return FAILED;

    /* D. Set CS pin to high */
    gpioDigitalWrite(&FLM25_CS_PIN,
                     HIGH);

    return SUCCESSFUL;
}

/*==============================================================================
* @brief
* @param
* @retval
==============================================================================*/
ReturnStatusEnum extFlWrEnable(void)
{
  	ReturnStatusEnum stat = SUCCESSFUL;
    gpioDigitalWrite(&FLM25_CS_PIN, LOW);
	if(HAL_SPI_Transmit(&SPIFlashRTCHandle, (uint8_t *)&flm25_cmd.wren,
						sizeof(flm25_cmd.wren),
						SPI1_TIMEOUT_MAX) != HAL_OK)
		stat = FAILED;

    gpioDigitalWrite(&FLM25_CS_PIN, HIGH);
	return stat;
}

///*==============================================================================
//* @brief
//* @param
//* @retval
//==============================================================================*/
//ReturnStatusEnum extFlBulkErase(uint8_t sec_cnt, uint32_t sec_addr)
//{
//	uint8_t i;
//	uint32_t addr_temp, addr_and_instcode;
//	
//	/*check sector arguments*/
// 	if(sec_addr > END_SEC_ADDR)
//		return FAILED;
//	
//	if(extFlWrEnable() != SUCCESSFUL)
//		return FAILED;
//
//	for(i = 0; i <= sec_cnt; i++) 
//	{
//		/* rearrange address to big endian */
//		addr_temp = (  ((sec_addr & 0xFF0000)  >> 16)
//					 + ((sec_addr & 0x00FF00))
//					 + ((sec_addr & 0x0000FF)  << 16));	
//		
//		/* reassemble to append read command code to the leftmost byte*/	
//		addr_and_instcode = ((addr_temp << 8) & 0xFFFFFF00) + flm25_cmd.se;
//				
//		HAL_GPIO_WritePin(GPIOA, FLM25_CS_PIN, GPIO_PIN_RESET);
//
//		if(HAL_SPI_Transmit(&SPIFlashRTCHandle, (uint8_t *)&addr_and_instcode,
//							sizeof(uint32_t), SPI1_TIMEOUT_MAX) != HAL_OK)
//			return FAILED;
//		HAL_GPIO_WritePin(GPIOA, FLM25_CS_PIN, GPIO_PIN_SET);
//	
//		sec_addr += EXT_FL_SEC_SZ;
//	}
//	
//	return SUCCESSFUL;
//}

/*==============================================================================
* @brief
* @param
* @retval
==============================================================================*/
ReturnStatusEnum extFlSectorErase(uint8_t sec_cnt, uint32_t sec_addr)
{
  	ReturnStatusEnum stat = SUCCESSFUL;
	uint8_t i;
	uint32_t addr_temp, addr_and_instcode;
	/*check sector arguments*/
 	if(sec_addr > END_SEC_ADDR)
		stat = FAILED;
	
	for(i = 0; i < sec_cnt; i++) 
	{
		if(extFlWrEnable() != SUCCESSFUL)
			stat = FAILED;

		/* rearrange address to big endian */
		addr_temp = (  ((sec_addr & 0xFF0000)  >> 16)
					 + ((sec_addr & 0x00FF00))
					 + ((sec_addr & 0x0000FF)  << 16));	
		
		/* reassemble to append read command code to the leftmost byte*/	
		addr_and_instcode = ((addr_temp << 8) & 0xFFFFFF00) + flm25_cmd.se;
				
        gpioDigitalWrite(&FLM25_CS_PIN, LOW);
		if(HAL_SPI_Transmit(&SPIFlashRTCHandle, (uint8_t *)&addr_and_instcode,
							sizeof(uint32_t), SPI1_TIMEOUT_MAX) != HAL_OK)
			stat = FAILED;
        gpioDigitalWrite(&FLM25_CS_PIN, HIGH);
		sec_addr += EXT_FL_SEC_SZ;
	
		delayMillis(800);
	}
	delayMillis(500);
	return stat;
}


/*==============================================================================
* @brief
* @param
* @retval
==============================================================================*/
ReturnStatusEnum  extFlWrite(int8_t *buff_ptr, uint32_t dest_addr, uint16_t buff_size)
{
  	ReturnStatusEnum stat = SUCCESSFUL;
	uint32_t addr_temp, addr_and_instcode;
	
	/* rearrange address to big endian */
	addr_temp = (  ((dest_addr & 0xFF0000)  >> 16)
				 + ((dest_addr & 0x00FF00))
				 + ((dest_addr & 0x0000FF)  << 16));
									
	/* reassemble to append read command code to the leftmost byte*/	
	addr_and_instcode = ((addr_temp << 8) & 0xFFFFFF00) + flm25_cmd.pp;
	
	/* write enable*/
	if(extFlWrEnable() != SUCCESSFUL)
		stat = FAILED;
	
	/* set CS pin to low */
    gpioDigitalWrite(&FLM25_CS_PIN, LOW);
    //delayMicros(SPI_DELAY);
	 delayMillis(1);
	/* Send write command */
	if(HAL_SPI_Transmit(&SPIFlashRTCHandle, (uint8_t *)&addr_and_instcode,
						sizeof(addr_and_instcode), SPI1_TIMEOUT_MAX) != HAL_OK)
		stat = FAILED;

	/* receive data*/
	if(HAL_SPI_Transmit(&SPIFlashRTCHandle, (uint8_t *)buff_ptr,
						buff_size, SPI1_TIMEOUT_MAX) != HAL_OK)
		stat = FAILED;

	/* set CS pin to high */
    gpioDigitalWrite(&FLM25_CS_PIN, HIGH);
	return stat;
}

/*==============================================================================
* @brief
* @param
* @retval
==============================================================================*/
ReturnStatusEnum  extFlRead(int8_t *buff_ptr, uint32_t src_addr, uint16_t buff_size)
{
  	ReturnStatusEnum stat = SUCCESSFUL;
	uint32_t addr_temp, addr_and_instcode;
	
	/* rearrange address to big endian */
	addr_temp = (  ((src_addr & 0xFF0000)  >> 16)
				 + ((src_addr & 0x00FF00))
				 + ((src_addr & 0x0000FF)  << 16));
									
	/* reassemble to append read command code to the leftmost byte*/	
	addr_and_instcode = ((addr_temp << 8) & 0xFFFFFF00) + flm25_cmd.rd;
	
	/* a. set CS pin to low */

    gpioDigitalWrite(&FLM25_CS_PIN, LOW);
    //delayMicros(SPI_DELAY);
	 delayMillis(1);
	/* b. Send read command */
	if(HAL_SPI_Transmit(&SPIFlashRTCHandle, (uint8_t *)&addr_and_instcode,
						sizeof(addr_and_instcode), SPI1_TIMEOUT_MAX) != HAL_OK)
		stat = FAILED;

	/* c. receive data*/
	if(HAL_SPI_Receive(&SPIFlashRTCHandle, (uint8_t *)buff_ptr,
						buff_size, SPI1_TIMEOUT_MAX) != HAL_OK)
		stat = FAILED;

	/* D. set CS pin to high */
    gpioDigitalWrite(&FLM25_CS_PIN, HIGH);
	return stat;
}

/******************************************************************************
  * Revision History
  *	@file      	extFlash.c
  *****************************************************************************
  * @version	v.2.2.0
  * @date		04/08/16
  * @author		MI Lorica
  * @changes	- created file
  ******************************************************************************
  */
