/*
 * tempUnitList.h
 *
 *  Created on: Jan 27, 2016
 *      Author: jamesdanf
 */

#ifndef TEMPUNITLIST_H_
#define TEMPUNITLIST_H_

#include <stdint.h>
#include "radio.h"

typedef enum
{
	UNIT_STAT_NULL,
	UNIT_STAT_INACTIVE,
	UNIT_STAT_ACTIVE,
	UNIT_STAT_ERROR
} UnitStatEnum;

typedef struct
{
	uint8_t unit_id;
	char unit_sn[14];
	char bnrrd_sn[6];
	char xbrd_sn[16];
	RdTypeEnum rd_type;
	UnitStatEnum status;
} UnitListStruct;

#define UNIT_CNT ((uint32_t)5)

UnitListStruct getUnitInfo(uint8_t arg_id);

#endif /* TEMPUNITLIST_H_ */
