
#include <deployAssert.h>

#include "extRTC.h"
#include "agBoardInit.h"

#define		RTCCMD_READ			(0x00)
#define		RTCCMD_WRITE		(0x80)

#define		RTCADD_MILLIS		(0x00)
#define		RTCADD_ST_SEC		(0x01)
#define		RTCADD_MIN			(0x02)
#define		RTCADD_HOUR			(0x03)
#define		RTCADD_WEEKDAY		(0x04)
#define		RTCADD_MONTHDAY		(0x05)
#define		RTCADD_MONTH		(0x06)
#define		RTCADD_YEAR			(0x07)
#define		RTCADD_HT_ALH		(0x0C)

#define		RTCBITPOS_ST		(7)
#define		RTCSTATE_ST_SET		(0x01)
#define		RTCSTATE_ST_RESET	(0x00)

#define		RTCBITPOS_HT		(6)
#define		RTCSTATE_HT_SET		(0x01)
#define		RTCSTATE_HT_RESET	(0x00)

#define		RTCBM_ST			(0x80)
#define		RTCBM_CB			(0xC0)
#define		RTCBM_MNTH			(0xE0)

#define		RTCBM_100MIL		(0xF0)
#define		RTCBM_10MIL			(0x0F)

//#define		RTCBM_10SEC			(0x70)
#define		RTCBM_SEC			(0x7F)

//#define		RTCBM_10MIN			(0x70)
#define		RTCBM_MIN			(0x7F)

//#define		RTCBM_10HOUR		(0x30)
#define		RTCBM_HOUR			(0x3F)

#define		RTCBM_WEEKDAY		(0x07)

//#define		RTCBM_10MONTHDAY	(0x10)
#define		RTCBM_MONTHDAY		(0x3F)

//#define		RTCBM_10MONTH		(0x10)
#define		RTCBM_MONTH			(0x1F)

//#define		RTCBM_10YEAR		(0xF0)
#define		RTCBM_YEAR			(0xFF)
#define		RTCBM_MSB4				(0xF0)
#define		RTCBM_LSB4				(0x0F)
#define		RTCBM_16BIT				(0xFF)
#define 	RTCBM_10BASE			(10)
#define		RTCBM_HALFBYTE			(4)
#define		RTC_TIME_BUFSIZE		(8)
#define		RTC_SPI_TO				(1000)

static uint8_t extRTCBcdToDec(uint8_t bcd_val)
{
	return ( (( (bcd_val & RTCBM_MSB4) >> RTCBM_HALFBYTE) * 10) + (bcd_val & RTCBM_LSB4) );
}

static uint8_t extRTCDecToBcd(uint32_t dec_val)
{
	dec_val = dec_val & RTCBM_16BIT;
	return (dec_val/RTCBM_10BASE)<<RTCBM_HALFBYTE | (dec_val%RTCBM_10BASE);
}

void exRTCWriteTime(RTCDateTimeStruct *date_time)
{
	uint8_t rtc_cmd = RTCCMD_WRITE | RTCADD_ST_SEC;
	uint8_t rtc_sec = extRTCDecToBcd(date_time->Seconds) & (~RTCBM_ST);
	uint8_t rtc_min = extRTCDecToBcd(date_time->Minutes) & (~RTCBM_ST);
	uint8_t rtc_hour = extRTCDecToBcd(date_time->Hours) & (~RTCBM_CB);
	uint8_t rtc_weekday = 1;
	uint8_t rtc_date = extRTCDecToBcd(date_time->Date) & (~RTCBM_CB);
	uint8_t rtc_month = extRTCDecToBcd(date_time->Month) & (~RTCBM_MNTH);
	uint8_t rtc_year = extRTCDecToBcd(date_time->Year);
	
	uint8_t rtc_databuff[8] = {rtc_cmd, rtc_sec, rtc_min, rtc_hour, rtc_weekday 
							  ,rtc_date, rtc_month, rtc_year};
	gpioDigitalWrite(&RTC_ENB, LOW);
	HAL_Delay(10);
	if(HAL_SPI_Transmit(&SPIFlashRTCHandle, (uint8_t *)&rtc_databuff,	RTC_TIME_BUFSIZE, RTC_SPI_TO) != HAL_OK)
		 assert(0);
	gpioDigitalWrite(&RTC_ENB, HIGH);
}

void exRTCReadTime(RTCDateTimeStruct *date_time)
{
	uint8_t rtcdata_buff[8];
	uint8_t rtc_cmd = RTCCMD_READ;
	
	HAL_Delay(100);
	gpioDigitalWrite(&RTC_ENB, LOW);
	HAL_Delay(10);
	
	if(HAL_SPI_Transmit(&SPIFlashRTCHandle, (uint8_t *)&rtc_cmd,
						1, RTC_SPI_TO) != HAL_OK)
		 assert(0);
	if(HAL_SPI_Receive(&SPIFlashRTCHandle, (uint8_t *)rtcdata_buff,
						8, RTC_SPI_TO) != HAL_OK)
	  	assert(0);
	gpioDigitalWrite(&RTC_ENB, HIGH);
	
	date_time->Millis =  ((rtcdata_buff[RTCADD_MILLIS] & RTCBM_100MIL) >>4 )*100
						+ (rtcdata_buff[RTCADD_MILLIS] & RTCBM_10MIL)*10;
	
	date_time->Seconds = extRTCBcdToDec(rtcdata_buff[RTCADD_ST_SEC] & RTCBM_SEC);
	
	date_time->Minutes = extRTCBcdToDec(rtcdata_buff[RTCADD_MIN] & RTCBM_MIN);
	
	date_time->Hours = extRTCBcdToDec(rtcdata_buff[RTCADD_HOUR] & RTCBM_HOUR);
	
	date_time->Weekday = extRTCBcdToDec(rtcdata_buff[RTCADD_WEEKDAY] & RTCBM_WEEKDAY);
		
	date_time->Date = extRTCBcdToDec(rtcdata_buff[RTCADD_MONTHDAY] & RTCBM_MONTHDAY);
	
	date_time->Month = extRTCBcdToDec(rtcdata_buff[RTCADD_MONTH] & RTCBM_MONTH);
		
	date_time->Year = extRTCBcdToDec(rtcdata_buff[RTCADD_YEAR] & RTCBM_YEAR);

	gpioDigitalWrite(&RTC_ENB, HIGH);
}


void extRTCInit(void)
{
	uint8_t rtc_cmd;		//MSB = R/w, Remaining Bits = address
	uint8_t rtc_data;
	
	gpioDigitalWrite(&FLM25_CS_PIN, HIGH);
	gpioDigitalWrite(&RTC_ENB, HIGH);
		
	/*SET Stop Bit to 1*/
	rtc_cmd = RTCCMD_WRITE | RTCADD_ST_SEC;
	rtc_data = RTCSTATE_ST_SET << RTCBITPOS_ST;
	gpioDigitalWrite(&RTC_ENB, LOW);
	HAL_Delay(100);
	if(HAL_SPI_Transmit(&SPIFlashRTCHandle, (uint8_t *)&rtc_cmd, 1, RTC_SPI_TO) != HAL_OK)
			 assert(0);
	if(HAL_SPI_Transmit(&SPIFlashRTCHandle, (uint8_t *)&rtc_data, 1, RTC_SPI_TO) != HAL_OK)
			 assert(0);
    gpioDigitalWrite(&RTC_ENB, HIGH);
	HAL_Delay(100);
	
	/*SET Stop Bit to 0*/
	rtc_cmd = RTCCMD_WRITE | RTCADD_ST_SEC;
	rtc_data = RTCSTATE_ST_RESET << RTCBITPOS_ST;;
	gpioDigitalWrite(&RTC_ENB, LOW);
	HAL_Delay(10);
	if(HAL_SPI_Transmit(&SPIFlashRTCHandle, (uint8_t *)&rtc_cmd,	1, RTC_SPI_TO) != HAL_OK)
		 assert(0);
	if(HAL_SPI_Transmit(&SPIFlashRTCHandle, (uint8_t *)&rtc_data, 1, RTC_SPI_TO) != HAL_OK)
		 assert(0);
    gpioDigitalWrite(&RTC_ENB, HIGH);
	HAL_Delay(100);
		
	rtc_cmd = RTCCMD_WRITE | RTCADD_HT_ALH;
	rtc_data = RTCSTATE_HT_RESET << RTCBITPOS_HT;
	gpioDigitalWrite(&RTC_ENB, LOW);
	HAL_Delay(10);
	if(HAL_SPI_Transmit(&SPIFlashRTCHandle, (uint8_t *)&rtc_cmd, 1, RTC_SPI_TO) != HAL_OK)
			 assert(0);
	if(HAL_SPI_Transmit(&SPIFlashRTCHandle, (uint8_t *)&rtc_data, 1, RTC_SPI_TO) != HAL_OK)
			 assert(0);
    gpioDigitalWrite(&RTC_ENB, HIGH);
	HAL_Delay(100);
}



void exRTCSample(void)
{
	uint8_t cmd = 0;
	uint8_t buff_ptr[32];
	char rtc_buffshow[20];
	//while(1)
	//{
		HAL_Delay(100);
		gpioDigitalWrite(&RTC_ENB, LOW);
		HAL_Delay(10);
		if(HAL_SPI_Transmit(&SPIFlashRTCHandle, (uint8_t *)&cmd,
							1, RTC_SPI_TO) != HAL_OK)
			 assert(0);
		HAL_SPI_Receive(&SPIFlashRTCHandle, (uint8_t *)buff_ptr,
							32, RTC_SPI_TO);
    	gpioDigitalWrite(&RTC_ENB, HIGH);
		
		sprintf(rtc_buffshow, "SR = 0x%02X\n", buff_ptr[1]);
	  uartSHOW(rtc_buffshow, strlen(rtc_buffshow));
	//}
}