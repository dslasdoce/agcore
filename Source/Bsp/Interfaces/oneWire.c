/**
  ******************************************************************************
  * @file      	onewire.c
  * @author     Hardware Team
  * @version    v2.2.0
  * @date       04/08/16
  * @brief		Manages internal flash access (program/erase/read).
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
#include <stdint.h>
#include <string.h>

#include "agBoardInit.h"
#include "oneWire.h"
#include "gpio.h"
#include "delay.h"

/* Global Declarations =======================================================*/
unsigned char ROM_NO[8];
uint8_t LastDiscrepancy;
uint8_t LastFamilyDiscrepancy;
uint8_t LastDeviceFlag;

/* Functions =================================================================*/

/* ============================================================================
 * @brief	read single one wire pulse
 * @param	gpio_ptr: ptr to io to be used
 * ==========================================================================*/
static uint8_t oneWireReadBit(AgTechIOStruct *gpio_ptr)
{
	uint8_t r;
	gpioMode(gpio_ptr,MODE_OUTPUT);
	gpioDigitalWrite(gpio_ptr, LOW);
	delayMicros(3);
	gpioMode(gpio_ptr,MODE_INPUT);
	delayMicros(10);
	r = gpioDigitalRead(gpio_ptr);
	delayMicros(53);
	return r;
}

/* ============================================================================
 * @brief	read one byte
 * @param	gpio_ptr: ptr to io to be used
 * ==========================================================================*/
uint8_t oneWireRead(AgTechIOStruct *gpio_ptr)
{
    uint8_t bitMask;
    uint8_t r = 0;

    for (bitMask = 0x01; bitMask; bitMask <<= 1)
    {
		if ( oneWireReadBit(gpio_ptr))
			r |= bitMask;
    }
    return r;
}

/* ============================================================================
 * @brief	read multiple bytes
 * @param	gpio_ptr: ptr to io to be used
 * ==========================================================================*/
void oneWireReadBytes(AgTechIOStruct *gpio_ptr, uint8_t *buf, uint16_t count)
{
  for (uint16_t i = 0 ; i < count ; i++)
    buf[i] = oneWireRead(gpio_ptr);
}


/* ============================================================================
 * @brief	send single pulse to owire bus
 * @param	gpio_ptr: ptr to io to be used
 * @param	bit: pulse state to be sent(1 or 0)
 * ==========================================================================*/
static void oneWireWriteBit(AgTechIOStruct *gpio_ptr, uint8_t bit)
{
	if (bit)
	{
		gpioDigitalWrite(gpio_ptr, LOW);
		gpioMode(gpio_ptr,MODE_OUTPUT);
		delayMicros(10);
		gpioDigitalWrite(gpio_ptr, HIGH);
		delayMicros(55);
	}
	else
	{
		gpioDigitalWrite(gpio_ptr, LOW);
		gpioMode(gpio_ptr,MODE_OUTPUT);
		delayMicros(65);
		gpioDigitalWrite(gpio_ptr, HIGH);
		delayMicros(5);
	}
}

/* ============================================================================
 * @brief	send single byte to owire bus
 * @param	gpio_ptr: ptr to io to be used
 * @param	out_byte: 8bit data to be sent
 * @param	power: power state after transaction
 * ==========================================================================*/
void oneWireWrite(AgTechIOStruct *gpio_ptr, uint8_t out_byte, bool power)
{
	uint8_t mask;
	for(mask = 0x01; mask; mask<<=1)
	{
		oneWireWriteBit( gpio_ptr,(mask & out_byte)?1:0);
	}

	if (!power)
	{
		gpioMode(gpio_ptr,MODE_INPUT);
		gpioDigitalWrite(gpio_ptr, LOW);
    }
}

/* ============================================================================
 * @brief	send multiple bytes to owire bus
 * @param	gpio_ptr: ptr to io to be used
 * @param	buf: start address of data to be sent
 * @param	count: number of data bytes to be sent
 * @param	power: power state after transaction
 * ==========================================================================*/
void oneWireWriteBytes(AgTechIOStruct *gpio_ptr, const uint8_t *buf,
					   uint16_t count, bool power)
{
  uint16_t i;
  for (i = 0 ; i < count ; i++)
    oneWireWrite(gpio_ptr, buf[i], power);

  if (!power)
  {
  	gpioMode(gpio_ptr,MODE_INPUT);
    gpioDigitalWrite(gpio_ptr, LOW);
  }
}

/* ============================================================================
 * @brief	reset one wire bus
 * @param	gpio_ptr: ptr to io to be used
 * ==========================================================================*/
uint8_t oneWireReset(AgTechIOStruct *gpio_ptr)
{
	uint8_t r;
	uint8_t retries = 125;

	gpioMode(gpio_ptr,MODE_INPUT);
	/*wait until the wire is high... just in case*/
	do
	{
		if (--retries == 0) return 0;
			delayMicros(2);
	} while ( !gpioDigitalRead(gpio_ptr));

	gpioDigitalWrite(gpio_ptr, LOW);
	gpioMode(gpio_ptr, MODE_OUTPUT);
	delayMicros(480);
	gpioMode(gpio_ptr, MODE_INPUT);
	delayMicros(70);
	r = !gpioDigitalRead(gpio_ptr);;
	delayMicros(410);
	return r;
}

/* ============================================================================
 * @brief	reset search details
 * ==========================================================================*/
void reset_search()
{
  	// reset the search state
	LastDiscrepancy = 0;
	LastDeviceFlag = false;
	LastFamilyDiscrepancy = 0;
	for(int i = 7; ; i--)
	{
		ROM_NO[i] = 0;
		if ( i == 0)
			break;
	}
}

/* ============================================================================
 * @brief	search for active devices
 * @param	gpio_ptr: ptr to gpio to be used in reading
 * @param	newAddr: ptr to storage for new device found
 * @param	command: bytes to be sent to one wire bus
 * ==========================================================================*/
static bool oneWiresearch_internal(AgTechIOStruct *gpio_ptr,
										uint8_t *newAddr, uint8_t command)
{
	uint8_t id_bit_number;
	uint8_t last_zero, rom_byte_number;
	uint8_t id_bit, cmp_id_bit;
	bool search_result;
	unsigned char rom_byte_mask, search_direction;

	/*initialize for search*/
	id_bit_number = 1;
	last_zero = 0;
	rom_byte_number = 0;
	rom_byte_mask = 1;
	search_result = false;

    /*if the last call was not the last one*/
	if (!LastDeviceFlag)
    {
		/*1-Wire reset*/
		if (!oneWireReset(gpio_ptr))
      	{
         	/*reset the search*/
			LastDiscrepancy = 0;
			LastDeviceFlag = false;
			LastFamilyDiscrepancy = 0;
			return false;
      	}

      /*issue the search command*/
      oneWireWrite(gpio_ptr, command, OWIRE_PARASITIC_POWEROFF);

      /*loop to do the search*/
      do
      {
         /*read a bit and its complement*/
		 id_bit = oneWireReadBit(gpio_ptr);
		 cmp_id_bit = oneWireReadBit(gpio_ptr);

		 /*check for no devices on 1-wire*/
         if ((id_bit == 1) && (cmp_id_bit == 1))
            break;
         else
         {
            /*all devices coupled have 0 or 1*/
			if (id_bit != cmp_id_bit)
			   search_direction = id_bit;  // bit write value for search
			else
            {
				/*if this discrepancy if before the Last Discrepancy
				  on a previous next then pick the same as last time*/
				if (id_bit_number < LastDiscrepancy)
					search_direction = ((ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
				else
				  	/*if equal to last pick 1, if not then pick 0*/
					search_direction = (id_bit_number == LastDiscrepancy);

				/*if 0 was picked then record its position in LastZero*/
				if (search_direction == 0)
				{
					last_zero = id_bit_number;
				  /*check for Last discrepancy in family*/
				  if (last_zero < 9)
					 LastFamilyDiscrepancy = last_zero;
				}
            }

            /*set or clear the bit in the ROM byte rom_byte_number
              with mask rom_byte_mask*/
            if (search_direction == 1)
            	ROM_NO[rom_byte_number] |= rom_byte_mask;
            else
            	ROM_NO[rom_byte_number] &= ~rom_byte_mask;

            /*serial number search direction write bit*/
            oneWireWriteBit(gpio_ptr, search_direction);

            /*increment the byte counter id_bit_number
              and shift the mask rom_byte_mask*/
            id_bit_number++;
            rom_byte_mask <<= 1;

            /*if the mask is 0 then go to new SerialNum byte
              rom_byte_number and reset mask*/
            if (rom_byte_mask == 0)
            {
                rom_byte_number++;
                rom_byte_mask = 1;
            }
         }
      }
      while(rom_byte_number < 8);  /*loop until through all ROM bytes 0-7*/

      /*if the search was successful then*/
      if (!(id_bit_number < 65))
      {
         /*search successful so set LastDiscrepancy,
           LastDeviceFlag,search_result*/
         LastDiscrepancy = last_zero;

         /*check for last device*/
         if (LastDiscrepancy == 0)
            LastDeviceFlag = true;

         search_result = true;
      }
    }

    /*if no device found then reset counters so next 'search'
      will be like a first*/
	if (!search_result || !ROM_NO[0])
	{
		LastDiscrepancy = 0;
		LastDeviceFlag = false;
		LastFamilyDiscrepancy = 0;
		search_result = false;
	}
	for (int i = 0; i < 8; i++)
	{
	   newAddr[i] = ROM_NO[i];
	}
	return search_result;
}

/* ============================================================================
 * @brief	searcg for active devices
 * @param	gpio_ptr: ptr to gpio struct to be used
 * @param	newAddr: ptr to storage of new device address detected
 * ==========================================================================*/
uint8_t oneWireSearch(AgTechIOStruct *gpio_ptr, uint8_t *newAddr)
{
	if(!oneWiresearch_internal(gpio_ptr, newAddr, 0xF0))
		return oneWiresearch_internal(gpio_ptr, newAddr, 0xF0);
	return 1;
}

/* ============================================================================
 * @brief	get 8 bit crc
 * @param	addr: ptr to char array to be computed
 * @param	len: size of data to be computed in bytes
 * ==========================================================================*/
uint8_t crc8(const uint8_t *addr, uint8_t len)
{
	uint8_t crc = 0;

	while (len--)
	{
		uint8_t inbyte = *addr++;
		for (uint8_t i = 8; i; i--)
		{
			uint8_t mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix) crc ^= 0x8C;
			inbyte >>= 1;
		}
	}
	return crc;
}

/* ============================================================================
 * @brief	get 16 bit crc
 * @param	input: ptr to char array to be computed
 * @param	len: size of data to be computed in bytes
 * ==========================================================================*/
uint16_t crc16(const uint8_t* input, uint16_t len, uint16_t crc)
{
    static const uint8_t oddparity[16] =
        { 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0 };

    for (uint16_t i = 0 ; i < len ; i++) {
      /*Even though we're just copying a byte from the input,
        we'll be doing 16-bit computation with it.*/
      uint16_t cdata = input[i];
      cdata = (cdata ^ crc) & 0xff;
      crc >>= 8;

      if (oddparity[cdata & 0x0F] ^ oddparity[cdata >> 4])
          crc ^= 0xC001;

      cdata <<= 6;
      crc ^= cdata;
      cdata <<= 1;
      crc ^= cdata;
    }
    return crc;
}

/* ============================================================================
 * @brief	select one wire device
 * @param	addr: ptr to char array to be computed
 * @param	rom
 * ==========================================================================*/
void oneWireSelect(AgTechIOStruct *gpio_ptr, const uint8_t rom[8])
{
    uint8_t i;
    oneWireWrite(gpio_ptr ,0x55, OWIRE_PARASITIC_POWEROFF);
    for (i = 0; i < 8; i++)
    	oneWireWrite(gpio_ptr, rom[i], OWIRE_PARASITIC_POWEROFF);
}

/*********************************************************************************
  * Revision History
  * @file       oneWire.c
  ********************************************************************************
  * @version    v2.2.0
  * @date       04/08/16
  * @author     D.Lasdoce
  * @changes
  ********************************************************************************
  * @version    v2.1.0
  * @date       12/23/2015
  * @author     D.Lasdoce
  * @changes     - created file
  ********************************************************************************
  */