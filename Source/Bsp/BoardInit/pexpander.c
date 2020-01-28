///**
//  ******************************************************************************
//  * @file      	pexpander.c
//  * @author		Hardware Team
//  * @version	v2.2.0
//  * @date		04/08/16
//  * @brief
//  ******************************************************************************
//  * @attention
//  *
//  * COPYRIGHT(c) 2016 AgTech Labs, Inc.
//  *
//  * Redistribution and use in source and binary forms, with or without modification,
//  * are permitted provided that the following conditions are met:
//  *   1. Redistributions of source code must retain the above copyright notice,
//  *      this list of conditions and the following disclaimer.
//  *   2. Redistributions in binary form must reproduce the above copyright notice,
//  *      this list of conditions and the following disclaimer in the documentation
//  *      and/or other materials provided with the distribution.
//  *   3. Neither the name of AgTech Labs, Inc. nor the names of its contributors
//  *      may be used to endorse or promote products derived from this software
//  *      without specific prior written permission.
//  *
//  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
//  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//  *
//  ******************************************************************************
//  */
//
//
///* Includes ==================================================================*/
//#include <string.h>
//
//#include "agboardinit.h"
//#include "delay.h"
//#include "i2c.h"
//#include "pexpander.h"
//
///* Typedefs ==================================================================*/
//typedef struct
//{
//	const char *name;
//	int gpio;
//}pexpander_gpio;
//
///* Defines ===================================================================*/
//#define I2CREG_IOCON 0x05
//#define I2CREG_IODIRA 0x00
//#define I2CREG_IODIRB 0x01
//#define I2CREG_GPIOA 0x09
//#define I2CREG_GPIOB 0x13
//
///* Constants =================================================================*/
//static pexpander_gpio pins_t[] = {
//		{"GPA0",0x0},
//		{"GPA1",0x1},
//		{"GPA2",0x2},
//		{"GPA3",0x3},
//		{"GPA4",0x4},
//		{"GPA5",0x5},
//		{"GPA6",0x6},
//		{"GPA7",0x7},
//		{"GPB0",0x8},
//		{"GPB1",0x9},
//		{"GPB2",0xA},
//		{"GPB3",0xB},
//		{"GPB4",0xC},
//		{"GPB5",0xD},
//		{"GPB6",0xE},
//		{"GPB7",0xF},
//		{NULL,-1}
//		};
//
///* Global Declarations =======================================================*/
//char databuffer[100];
//
///* Functions =================================================================*/
//
///*==============================================================================
// * @brief
// * @param
// * @retval
//==============================================================================*/
//int pexpanderLookup(const char *gpio_name)
//{
//	pexpander_gpio *pin;
//	for(pin = pins_t;pin->name != NULL;++pin)
//	{
//		if (strcmp(pin->name,gpio_name) == 0)
//		{
//			return pin->gpio;
//		}
//	}
//	return -1;
//}
//
///*==============================================================================
// * @brief
// * @param
// * @retval
//==============================================================================*/
//bool pexpanderMode(const char *gpio_name, uint8_t dir, uint8_t i2c_add)
//{
//	int current_mode;
//	int gpioreg_oe;
//	uint8_t content[I2C_DATABUFFSIZE];
//	char cmd[2];
//	//char data[I2C_DATABUFFSIZE];
//	int gpio_num;
//	//extern int i2c_delay;
//	//i2c_delay = I2C_DELAY;
//	gpio_num = pexpanderLookup(gpio_name);	//get gpio number
//	int gpio_bitpos = 1<<(gpio_num%8);
//	////printf("%s:%d\n",gpio_name,gpio_num);
//	/* check gpio group and set gpio register value*/
//	if (gpio_num/8 == 0)
//		{
//		gpioreg_oe = I2CREG_IODIRA;
//		}	//group A
//	else
//		{
//			gpioreg_oe = I2CREG_IODIRB;
//		}					//group B
//
//	/*read current status of port expander*/
//	if ((i2c1Read(i2c_add, 0, content, I2C_DATABUFFSIZE)) != INTERFACE_OK)
//	{
//		delayMillis(10);
//		//printf("READ EXPANDER SETDIR \"%s\" ERROR!\n",gpio_name);
//		//exit(0);
//	}
//
//	//content = get_i2cbuff();
//	current_mode = *(content + gpioreg_oe);	//get the register value of IODIR
//
//	/*get value to be written to IODIR register*/
//	if (dir == 1)
//	{
//		current_mode |= gpio_bitpos;
//	}
//	else
//	{
//		current_mode &= ~gpio_bitpos;
//	}
//	cmd[0] = gpioreg_oe;	//address to be written(I2CREG_IODIR)
//	cmd[1] = current_mode;	//value to be written on address
//
//	/*write the new value to IODIR register*/
//	if ( (i2c1Write(i2c_add, cmd, sizeof(cmd)/sizeof(char))) != INTERFACE_OK )
//	{
//		////printf("WRITE EXPANDER SETDIR \"%s\" ERROR!\n",gpio_name);
//		//exit(0);
//		return false;
//	}
//	return true;
//}
//
///*==============================================================================
// * @brief
// * @param
// * @retval
//==============================================================================*/
//bool pexpanderWrite(const char *gpio_name,uint8_t state, uint8_t i2c_add)
//{
//	//int fd;
//	int current_state;
//	int gpioreg_state;
//	int gpioreg_oe;
//	int current_mode;
//	uint8_t content[I2C_DATABUFFSIZE];
//	int gpio_num;
//	char cmd[I2C_CMDBUFFSIZE];
//	//extern int i2c_delay;
//	//i2c_delay = I2C_DELAY;
//	//char data[256];
//	//****************************
//	//double timenow,time_diff;
//	//timenow = get_time("START",0);
//
//	gpio_num = pexpanderLookup(gpio_name);	//get gpio number
//
//	//****************************
//	//timenow = get_time("LOOKUP",timenow);
//
//	int gpio_bitpos = 1<<(gpio_num%8);
//	/* check gpio group and set gpio register value*/
//	if (gpio_num/8 == 0)
//		{
//			gpioreg_state = I2CREG_GPIOA;
//			gpioreg_oe = I2CREG_IODIRA;
//		}
//	else
//		{
//			gpioreg_state = I2CREG_GPIOB;
//			gpioreg_oe = I2CREG_IODIRB;
//		}
//
//	/*read current status of port expander*/
//	if ( (i2c1Read(i2c_add,0,content,I2C_DATABUFFSIZE)) != INTERFACE_OK)
//	{
//		//printf("READ EXPANDER SETSTATE \"%s\" ERROR\n",gpio_name);
//		return false;
//	}
//	//content = get_i2cbuff();
//	current_mode = *(content + gpioreg_oe);
//	current_state = *(content + gpioreg_state);
//
//	////printf("VERIFYING\n");
//	if ( (current_mode & gpio_bitpos) )
//	{
//		////printf("Expander 0x%x %s Not Configured as OUTPUT\n",i2c_add,gpio_name);
//		pexpanderMode(gpio_name, MODE_OUTPUT,i2c_add);
//	}
//
//	//****************************
//	//timenow = get_time("READ",timenow);
//
//	/*get value to be written to GPIO register*/
//	if (state == 1)
//	{
//		current_state |= gpio_bitpos;
//	}
//	else
//	{
//		current_state &= ~gpio_bitpos;
//	}
//	cmd[0] = gpioreg_state;
//	cmd[1] = current_state;
//	if (!(i2c1Write(i2c_add, cmd, sizeof(cmd))))
//	{
//		//printf("WRITE EXPANDER SETSTATE \"%s\" ERROR!\n",gpio_name);
//		//exit(0);
//		return false;
//	}
//
//	//****************************
//	//timenow = get_time("WRITE",timenow);
//
//	return true;
//}
//
///*==============================================================================
// * @brief
// * @param
// * @retval
//==============================================================================*/
//void pexpanderToggle(const char *gpio_name)
//{
//	pexpanderMode(gpio_name,0,I2CADD_PEXPANDER1);
//	pexpanderMode(gpio_name,0,I2CADD_PEXPANDER1);
//	while (1)
//	{
//		pexpanderWrite(gpio_name,HIGH,I2CADD_PEXPANDER1);
//		pexpanderWrite(gpio_name,LOW,I2CADD_PEXPANDER1);
//	}
//
//}
//
///*******************************************************************************
//  * Revision History
//  *	@file       pexpander.c
//  ******************************************************************************
//  * @version    v2.2.0
//  * @date       04/08/16
//  * @author     D.Lasdoce
//  * @changes	created file
//  ******************************************************************************
//  */