/*
 * tempNodeList.c
 *
 *  Created on: Jan 27, 2016
 *      Author: jamesdanf
 */

#include <string.h>
#include <stdio.h>
#include "utilMem.h"
#include "tempUnitList.h"
#include "radio.h"

const uint8_t ID[] = {
		0x01,
		0x25,
		0x0D,
		0x0E,
		0x0F,
        0x27
		
};
const char *SN[] = {
		"PUN2016021900",
		"PUP2016021901",
		"PUP2016021902",
		"PUP2016021903",
		"PUP2016021904",
        "PUP2016021905"
};
const char *BNRRD_SN[] = {
		"145536",
		"145537",
		"145538",
		"145539",
		NULL,
        "145540",
};
const char *XBRD_SN[] = {
		NULL,
		NULL,
		NULL,
		NULL,
		"0013A20040C2C4A9",
        NULL
};
const uint8_t RD_TYP[] = {
		RDTYPE_BANNER,
		RDTYPE_BANNER,
		RDTYPE_BANNER,
		RDTYPE_BANNER,
		RDTYPE_XBEE,
        RDTYPE_BANNER,
		RDTYPE_BANNER
};
const uint8_t STAT[] = {
		UNIT_STAT_ACTIVE,
		UNIT_STAT_ACTIVE,
		UNIT_STAT_ACTIVE,
		UNIT_STAT_INACTIVE,
		UNIT_STAT_ACTIVE,
        UNIT_STAT_ACTIVE,
 		UNIT_STAT_ACTIVE
};

//UnitListStruct getUnitInfo(uint8_t arg_id)
//{
//	UnitListStruct unitlist[UNIT_CNT];
//
//	CLEAR(unitlist[arg_id]);
//	unitlist[arg_id].unit_id = arg_id;
//	memcpy(unitlist[arg_id].unit_sn, SN[arg_id], sizeof(SN[arg_id]));
//	memcpy(unitlist[arg_id].bnrrd_sn, BNRRD_SN[arg_id], sizeof(BNRRD_SN[arg_id]));
//	memcpy(unitlist[arg_id].xbrd_sn, XBRD_SN[arg_id], sizeof(XBRD_SN[arg_id]));
//	unitlist[arg_id].status = STAT[arg_id];
//
//	return unitlist[arg_id];
////	return(sizeof(unitlist) / sizeof(UnitListStruct));
//}
UnitListStruct getUnitInfo(uint8_t arg_id)
{
	UnitListStruct unitlist;

	CLEAR(unitlist);
	unitlist.unit_id = ID[arg_id];
	memcpy(unitlist.unit_sn, SN[arg_id], sizeof(unitlist.unit_sn));
	if(BNRRD_SN[arg_id] != NULL)
	{
		memcpy(unitlist.bnrrd_sn, BNRRD_SN[arg_id], sizeof(unitlist.bnrrd_sn));
	}
	if(XBRD_SN[arg_id] != NULL)
	{
		memcpy(unitlist.xbrd_sn, XBRD_SN[arg_id], sizeof(unitlist.xbrd_sn));
	}
	unitlist.rd_type = RD_TYP[arg_id];
	unitlist.status = STAT[arg_id];

	return unitlist;
}
