/*
 * tempApiCodedList.h
 *
 *  Created on: Jan 27, 2016
 *      Author: jamesdanf
 */

#ifndef TEMPAPICODEDLIST_H_
#define TEMPAPICODEDLIST_H_

/* External Declarations ====================================================*/
	

#define		SC_DVIS_SR			((SensorCodeStruct){0x02, "SR-Davis-6450"})
#define		SC_AQCHK24			((SensorCodeStruct){0x1A, "SMST-Aqchk-24in"})
#define		SC_DVIS_7911		((SensorCodeStruct){0x1F, "WSWD-Davis-7911"})
#define		SC_DVIS_7852		((SensorCodeStruct){0x20, "RG-Davis-7852"})
#define		SC_DVIS_6470		((SensorCodeStruct){0x2A, "AT-Davis-6470"})
#define		SC_DVIS_6475		((SensorCodeStruct){0x2B, "AT-Davis-6475"})
#define		SC_DVIS_6477		((SensorCodeStruct){0x2C, "AT-Davis-6477"})
#define		SC_DVIS_6830		((SensorCodeStruct){0x2D, "TPRH-Davis-6830"})
#define		SC_DVIS_6832		((SensorCodeStruct){0x2E, "TPRH-Davis-6832"})
#define		SC_AQCHK32			((SensorCodeStruct){0x24, "SMST-Aqchk-32in"})
#define		SC_AQCHK48			((SensorCodeStruct){0x26, "SMST-Aqchk-48in"})
#define		SC_AQCHK56			((SensorCodeStruct){0x27, "SMST-Aqchk-56in"})
#define		SC_TRDCR50			((SensorCodeStruct){0x34, "WP-Trdcr-td1000050"})
#define		SC_TRDCR200			((SensorCodeStruct){0x35, "WP-Trdcr-td1000200"})
#define		SC_DAVIS_6152		((SensorCodeStruct){0x38, "RH-Davis-6152"})
#define		SC_HSTI24			((SensorCodeStruct){0x39, "SMST-HSTI-HS24"})
#define		SC_HSTI32			((SensorCodeStruct){0x3A, "SMST-HSTI-HS32"})
#define		SC_HSTI48			((SensorCodeStruct){0x3B, "SMST-HSTI-HS48"})
#define		SC_HSTI60			((SensorCodeStruct){0x3C, "SMST-HSTI-HS60"})


static const char* DEVICE_COORDINATES[] = {"[-121.130013, 36.180870]", "[-121.13713305, 36.17788233]",
		"[-121.12779666666667, 36.18546166666667]", "[-121.11659833333333, 36.17108833333333]",
		"[-121.13054333333334, 36.17615333333333]", "[ -121.10608, 36.16688833333333]",
		"[-121.14045166666666, 36.194785]", "[-121.13708333333334, 36.186688333333336]",
		"[-121.12451, 36.182793333333336]"};

//SensorCodeStruct *SENSOR_CODE[] = {&SC_DVIS_SR, &SC_AQCHK24, &SC_DVIS_7911, 
//										  &SC_DVIS_7852, &SC_DVIS_6470, &SC_DVIS_6475, 
//										  &SC_DVIS_6477, &SC_DVIS_6830, &SC_DVIS_6832,
//										  &SC_AQCHK32, &SC_AQCHK48, &SC_AQCHK56,
//										  &SC_TRDCR50, &SC_TRDCR200, &SC_DAVIS_6152, 
//										  &SC_HSTI24, &SC_HSTI32, &SC_HSTI48, 
//										  &SC_HSTI60, NULL};
//static const char *SENSOR_CODE[] = {
//    /*0x00*/"WF-Smtrc-AG2000",
//    /*0x01*/"WF-Smtrc-AG2000",
//    /*0x02*/"SR-Davis-6450",
//    /*0x03*/"SMST-Gropo-GPMS55mA",
//    /*0x04*/"SMST-Dcgn-5TM",
//	/*0x05*/"SMST-Dcgn-MAS1",
//	/*0x06*/"SM-Irmtr-SRwEguage",
//	/*0x07*/"WP-Trdcr-TDH30",
//	/*0x08*/"WP-Trdcr-TDI85",
//	/*0x09*/"SMST-Sntek-EazyAG50ES",
//	/*0x0A*/"SMST-Dcgn-EC5",
//	/*0x0B*/"SMST-Dcgn-5TE",
//	/*0x0C*/"WP-Trdcr-td1000",
//	/*0x0D*/"WP-Trdcr-TDWLB",
//	/*0x0E*/"TL-Flwln-LU29",
//	/*0x0F*/"LW-Dcgn-LWS",
//	/*0x10*/"WF-Netfm-WST",
//	/*0x11*/"SF-Dynmx-SGEX925",
//	/*0x12*/"HM-Adcon-HM2014",
//	/*0x13*/"SMST-Gropo-GPMS420mA",
//	/*0x14*/"SMST-Sntek-AG60TS",
//	/*0x15*/"SMST-Sntek-AG60ES",
//	/*0x16*/"SMST-Sntek-EazyAG30ES",
//	/*0x17*/"SMST-Sntek-EazyAG30TS",
//	/*0x18*/"SMST-Gropo-GPMS025V",
//	/*0x19*/"SMST-Gropo-GPLP2",
//	/*0x1A*/"SMST-Aqchk-24in",
//	/*0x1B*/"SMST-Adcon-SM1",
//	/*0x1C*/"SMST-Adcon-HydraProbe2",
//	/*0x1D*/"SMST-Dcgn-10HS",
//	/*0x1E*/"SMST-Dcgn-GS3",
//	/*0x1F*/"WSWD-Davis-7911",
//	/*0x20*/"RG-Davis-7852",
//	/*0x21*/"SM-Irmtr-200SS",
//	/*0x22*/"AT-Irmtr-2014",
//	/*0x23*/"LW-Davis-6420",
//	/*0x24*/"SMST-Aqchk-32in",
//	/*0x25*/"SMST-Aqchk-40in",
//	/*0x26*/"SMST-Aqchk-48in",
//	/*0x27*/"SMST-Aqchk-60in",
//	/*0x28*/"SMST-Sntek-1.5M",
//	/*0x29*/"SMST-Sntek-1.5M TS",
//	/*0x2A*/"AT-Davis-6470",
//	/*0x2B*/"AT-Davis-6475",
//	/*0x2C*/"AT-Davis-6477",
//	/*0x2D*/"EN-Davis-ENVOY8X-6318",
//	/*0x2E*/"EN-Davis-ENVOY-6316",
//	/*0x2F*/"WS-Davis-7911",
//	/*0x30*/"WD-Davis-7911",
//	/*0x31*/"TPRH-CDWST-2500LF",
//	/*0x32*/"RH-CDWST-2500LF",
//	/*0x33*/"SMST-HSTI-HS40",
//	/*0x34*/"WP-Trdcr-td1000050",
//	/*0x35*/"WP-Trdcr-td1000200",
//	/*0x36*/"SM-Dcgn-10HS",
//	/*0x37*/"UV-Davis-6490",
//	/*0x38*/"RH-Davis-6152"
//};

//static const char *SENSOR_TYPE[] = {
//	/*0x00*/"SM", 
//	/*0x01*/"SM", 
//	/*0x02*/"WP", 
//	/*0x03*/"WF", 
//	/*0x04*/"AT", 
//	/*0x05*/"RH", 
//	/*0x06*/"WD", 
//	/*0x07*/"WS", 
//	/*0x08*/"RG", 
//	/*0x09*/"LW", 
//	/*0x0A*/"SF", 
//	/*0x0B*/"SR", 
//	/*0x0C*/"LT",
//	/*0x0D*/"WM", 
//	/*0x0E*/"IM", 
//	/*0x0F*/"ST", 
//	/*0x10*/"AT", 
//	/*0x11*/"UV", 
//	/*0x12*/"EC",
//	/*0x13*/"SBEC",
//	/*0x14*/"SPEC"
//};

static const uint16_t PORT[] = {
	0, 1, 2, 3, 4, 0x7631, 0x7632
};

//static const char *UOM[] = {
//	/*0x00*/"V", 
//	/*0x01*/"V", 
//	/*0x02*/"mA", 
//	/*0x03*/"psi", 
//	/*0x04*/"kPa", 
//	/*0x05*/"VWC", 
//	/*0x06*/"C", 
//	/*0x07*/"F", 
//	/*0x08*/"e", 
//	/*0x09*/"pulse", 
//	/*0x0A*/"%", 
//	/*0x0B*/"mph", 
//	/*0x0C*/"mm",
//	/*0x0D*/"CFM", 
//	/*0x0E*/"W/m2", 
//	/*0x0F*/"in",
//	/*0x10*/"ft", 
//	/*0x11*/"m", 
//	/*0x12*/"cm", 
//	/*0x13*/"ohm",
//	/*0x14*/"deg",
//	/*0x15*/"in/hr", 
//	/*0x16*/"MEDs", 
//	/*0x17*/"ds/m", 
//	/*0x18*/"uS/cm",
//	/*0x19*/"uS/cm"
//};

static const char *INTERFACE[] = {
	"Analog", "SDI-12", "Pulse Counter", "Digital Counter", "Davis ISS",
	"ModBus", "BLE", "WiFi"
};

static const float POWER_SUPPLY[] = {
	3.3, 5, 12
};

#endif /* TEMPAPICODEDLIST_H_ */
