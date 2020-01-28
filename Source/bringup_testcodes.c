#include "productCfg.h"
#include "agBoardInit.h"
#include "dmMain.h"
#include "delay.h"
#include "rtc.h"
#include "intFlash.h"
#include "rdBnrMain.h"
#include "port.h"
#include "daqProcess.h"
#include "stm32f7xx_hal.h"
#include "cell.h"
#include "radio.h"
#include "extFlash.h"
#include "bsp_pwm.h"
#include "gpio.h"
#include "extRTC.h"
#include "agps.h"
#include "daqAdcSpi.h"
#include "delay.h"
#include "utilMem.h"
#include "utilCalc.h"
#include "sdMain.h"
#include "tempApiCodedList.h"
#include "port.h"
#include "utilPwrMonitor.h"
#include "daqSensInterface.h"
#include "isrMain.h"
#include "daqDavis.h"
#include "i2c.h"
#include "equations.h"
#include "cloud.h"
#include "mspSpi.h"
#include "bringup_testcodes.h"

#pragma optimize=none
void brngup_RCVtest(void)
{
  	setPowerRails(true);
  	periphEnable();
	gpioDigitalWrite(&VCON15_EN, HIGH);
	
	gpioDigitalWrite(&SIG1_SEL, HIGH);
	gpioDigitalWrite(&SIG1_SEL0, HIGH);
	gpioDigitalWrite(&SIG1_SEL1, LOW);
	
	gpioDigitalWrite(&SIG2_SEL, HIGH);
	gpioDigitalWrite(&SIG2_SEL0, HIGH);
	gpioDigitalWrite(&SIG2_SEL1, LOW);
	
	gpioDigitalWrite(&SIG3_SEL, HIGH);
	gpioDigitalWrite(&SIG3_SEL0, HIGH);
	gpioDigitalWrite(&SIG3_SEL1, LOW);
	
	gpioDigitalWrite(&SIG4_SEL, HIGH);
	gpioDigitalWrite(&SIG4_SEL0, HIGH);
	gpioDigitalWrite(&SIG4_SEL1, LOW);
	volatile float adc_val = 0;
	while(1)
	{
	  	adc_val = daqSensAnalogI(1)*291/191;
	}
}
//#pragma optimize=none
 float supply_phil;
void brngup_mspSensorTest(void)
{
 //phil 	mspPortConfig(1,MSPCMD3_MUX_10K_UP);
  
//phil  #define		MSPCMD3_MUX_3K_UP				(0x00)
//phil#define		MSPCMD3_MUX_10K_UP				(0x01)
//phil#define		MSPCMD3_MUX_10K_DOWN			(0x02)
//phil#define		MSPCMD3_MUX_3KUP_3V				(0x03)
  
 	
  
  	while(1)
	{
          for (uint8_t phil_i=1; phil_i<5; phil_i++) { //phil
                supply_phil = 0.0;
                mspSetPortPower(phil_i, supply_phil);
                supply_phil = 3.3;
                mspSetPortPower(phil_i, supply_phil);
                supply_phil = 5.0;
                mspSetPortPower(phil_i, supply_phil);
                supply_phil = 9.0;
                mspSetPortPower(phil_i, supply_phil);
                supply_phil = 12.0;
                mspSetPortPower(phil_i, supply_phil);
                supply_phil = 15.0;                
                mspSetPortPower(phil_i, supply_phil);
                mspPortConfig(phil_i,MSPCMD3_MUX_3K_UP);
                mspPortConfig(phil_i,MSPCMD3_MUX_10K_UP);
                mspPortConfig(phil_i,MSPCMD3_MUX_10K_DOWN);
                mspPortConfig(phil_i,MSPCMD3_MUX_3KUP_3V);                
  		mspStartSensorRead(phil_i,2500);//phil
		delayMillis(3000);
		mspSendSensorData(phil_i);//phil
                delayMillis(3000);
                mspClearSensorData(phil_i);
                  
          }//phil
	}
}


void brngup_autoassocTest(void)
{
  	uint32_t default_net_id = 0;
	uint32_t test_net_id = 1;
	while(1)
	{
		#if ENABLE_AUTONETID
			rdBnrMainSetNetId(test_net_id);

			rdBnrMainGetNetId(&default_net_id);
			if(default_net_id != test_net_id)
				assert(0);
			
			delayMillis(500);
			gpioDigitalWrite(&BANNER_EN, LOW);
			delayMillis(2000);
			gpioDigitalWrite(&BANNER_EN, HIGH);
		#endif
		test_net_id++;
		
		delayMillis(10000);
		delayMillis(10000);
		delayMillis(10000);
		delayMillis(10000);
		delayMillis(10000);
		delayMillis(10000);
		
		
	}
}

void brngup_mspspitest(void)
{
  
	//gpioMode(&VCON3, MODE_INPUT);
	mspSetPortPower(3,5);
	gpioDigitalWrite(&VCON3_EN, HIGH);
	delayMillis(1000);
	gpioDigitalWrite(&PWR3_A_EN, HIGH); 	
	//			mspSetPortPower(sensordetails_ptr->port,sensordetails_ptr->power_supply);
  	
  	while(1)
	{
//	  	for(uint8_t i = 0; i < 4; i++)
//		{
//			mspPortConfig(4, i);
//			//delayMillis(3000);
//			gpioDigitalWrite(&LD_USER1, HIGH);
//			gpioDigitalWrite(&LD_USER2, HIGH);
//			delayMillis(200);
//			gpioDigitalWrite(&LD_USER1, LOW);
//			gpioDigitalWrite(&LD_USER2, LOW);
//			delayMillis(200);
//		}
	  	gpioDigitalWrite(&VCON15_EN, LOW);
	  	mspSetPowerRails(MSPCMD2_PR_ALL, false);
//		mspSetPowerRails(MSPCMD2_PR_ALL, true);
		
		mspSetPowerRails(MSPCMD2_PR_3_5, true);
		delayMillis(100);
		mspSetPowerRails(MSPCMD2_PR_3_5, false);
		delayMillis(100);
	  	
		mspSetPowerRails(MSPCMD2_PR_1V8, true);
		delayMillis(100);
		mspSetPowerRails(MSPCMD2_PR_1V8, false);
		delayMillis(100);
		
		mspSetPowerRails(MSPCMD2_PR_3V3, true);
		delayMillis(100);
		mspSetPowerRails(MSPCMD2_PR_3V3, false);
		delayMillis(100);
		
		mspSetPowerRails(MSPCMD2_PR_4V, true);
		delayMillis(100);
		mspSetPowerRails(MSPCMD2_PR_4V, false);
		delayMillis(100);
		
		mspSetPowerRails(MSPCMD2_PR_5V, true);
		delayMillis(100);
		mspSetPowerRails(MSPCMD2_PR_5V, false);
		delayMillis(100);
		
		mspSetPowerRails(MSPCMD2_PR_420, true);
		delayMillis(100);
		mspSetPowerRails(MSPCMD2_PR_420, false);
		delayMillis(100);
		
		mspSetPowerRails(MSPCMD2_PR_2947, true);
		delayMillis(100);
		mspSetPowerRails(MSPCMD2_PR_2947, false);
		delayMillis(100);
	}
}


void brngup_mspsleeptest(void)
{
 	while(1)
	{
          //Phil  delay a bit so we can read the power meter and see how much MSP430 sleep saves
       		delayMillis(10000);

		mspSleep();
	}
}


//#define		MSPCMD3_PR_OFF					(0x00)
//#define		MSPCMD3_PR_ON					(0x01)

// LED's
//#define LED1_STAT_RED	    (uint8_t) 0x01	  // Red LED
//#define LED2_STAT_GREEN	    (uint8_t) 0x02	  // Green LED
//#define BOTH_LEDS	    (uint8_t) 0x03	  // Both LED's
//#define IDLE_TASK_LED_TEST  (uint8_t) 0x04	

// LED's -
//#define DIAGNOSTIC_PATTERN_1	(uint8_t) 0x02		// diagnostic pattern
//                                                  LED RED - toggle red
//                                                  LED GREEN - toggle green
//                                                  LED BOTH  - toggle both leds
//#define DIAGNOSTIC_PATTERN_2	(uint8_t) 0x03
//                                                  LED BOTH  - toggle red, green off
//#define DIAGNOSTIC_PATTERN_3	(uint8_t) 0x04      
//                                                  LED BOTH  - toggel green , red off
//#define DIAGNOSTIC_PATTERN_4	(uint8_t) 0x05		// not used 

void brngup_mspledtest(void)
{
  	while(1)
	{
		mspLedDiagnostic(LED1_STAT_RED,MSPCMD3_PR_OFF);
		delayMillis(4000);
                mspLedDiagnostic(LED1_STAT_RED,MSPCMD3_PR_ON);
		delayMillis(4000);
                mspLedDiagnostic(LED1_STAT_RED,DIAGNOSTIC_PATTERN_1);
		delayMillis(6000);                

                
                mspLedDiagnostic(LED2_STAT_GREEN,MSPCMD3_PR_OFF);
		delayMillis(4000);
                mspLedDiagnostic(LED2_STAT_GREEN,MSPCMD3_PR_ON);
		delayMillis(4000);
                mspLedDiagnostic(LED2_STAT_GREEN,DIAGNOSTIC_PATTERN_1);
		delayMillis(6000);                

                
                mspLedDiagnostic(BOTH_LEDS,MSPCMD3_PR_OFF);
		delayMillis(4000);
                mspLedDiagnostic(BOTH_LEDS,MSPCMD3_PR_ON);
		delayMillis(4000);
                mspLedDiagnostic(BOTH_LEDS,DIAGNOSTIC_PATTERN_1);
		delayMillis(6000);                
                mspLedDiagnostic(BOTH_LEDS,DIAGNOSTIC_PATTERN_2);
		delayMillis(6000);                
                mspLedDiagnostic(BOTH_LEDS,DIAGNOSTIC_PATTERN_3);
		delayMillis(6000); 
                mspLedDiagnostic(BOTH_LEDS,MSPCMD3_PR_OFF);
		delayMillis(4000);
                
                mspLedDiagnostic(IDLE_TASK_LED_TEST,MSPCMD3_PR_ON);
		delayMillis(10000);
                mspLedDiagnostic(IDLE_TASK_LED_TEST,MSPCMD3_PR_OFF);
		delayMillis(1000);
                mspLedDiagnostic(LED2_STAT_GREEN,MSPCMD3_PR_OFF);
		delayMillis(4000);

	}
}

char VersionNumberString[12];

void brngup_mspsoftwareVersionNumbertest(void)
{
        mspSoftwareVersion(&VersionNumberString[0]);
	delayMillis(2000);
        mspLedDiagnostic(LED1_STAT_RED,MSPCMD3_PR_ON);
  	while(1)
	{
		
	}
}


#if 0
// the watchdog command is no longer supported
// on 5/25/2017 Bob told me to let the MSP handle the watchdog timer. Phil
void brngup_mspwatchdogtest(void)
{
  	while(1)
	{
		mspwatchdogkick();
		delayMillis(2000);
	}
}
#endif




// check the power on Port 4 for testing. 
int checkPowerPortNumber = 4;
float voltage_out_port_power_check=0.0;
void brngup_mspCheckpwmVoltages( void)
{
         AgTechIOStruct *cs[5] = {NULL, &PM_SIG1, &PM_SIG2, &PM_SIG3, &PM_SIG4};

         mspSetPortPower(checkPowerPortNumber, 5); // send message to the MSP to turn on the PWM pulses 
		
         utilPwrModeCont(cs[checkPowerPortNumber], true);//turn on utility meter
         delayMillis(1000);  //phil time for the MSP to PWM the proper port

          // read the voltage on the port from the utility meter
         while (1)
         {
         voltage_out_port_power_check = utilPwrGetVoltage(cs[checkPowerPortNumber]);
         delayMillis(10000);  

         }


        
}

// check the power on Port 3  for testing. Turn off and off the MSP Pwm power supply
int zero_volts_PortNumber = 3;
float voltage_out_port_power_zero=0.0;
void brngup_mspCheckZeroPower( void)
{
         AgTechIOStruct *cs[5] = {NULL, &PM_SIG1, &PM_SIG2, &PM_SIG3, &PM_SIG4};
         
         // turn on the 15 volt supply.
         
          mspSetPortPower(zero_volts_PortNumber, 15.0); // send message to the MSP to turn OFF the PWM pulses 
        
          delayMillis(10000);  //phil  leave it on for 10 seconds
          
          // turn off the 15 volts supply.
       
         mspSetPortPower(zero_volts_PortNumber, 0.0); // send message to the MSP to turn OFF the PWM pulses 
		
         utilPwrModeCont(cs[zero_volts_PortNumber], true);//turn on utility meter
         delayMillis(1000);  //phil time for the MSP to PWM the proper port

          // read the voltage on the port from the utility meter
         while (1)
         {
         voltage_out_port_power_zero = utilPwrGetVoltage(cs[zero_volts_PortNumber]);
         delayMillis(10000);  

         }


        
}


// check the power on Port 3  for testing. Turn off and off the MSP Pwm power supply and turn on
// the ARM pwm supply to make sure the ARM can run the PWM after the MSP PWMs have been turned off.
//
int Arm_volts_PortNumber = 3;
float voltage_out_port_power_Arm=0.0;
void brngup_mspCheckArmPower( void)
{
         AgTechIOStruct *cs[5] = {NULL, &PM_SIG1, &PM_SIG2, &PM_SIG3, &PM_SIG4};
         
         // turn on the 15 volt supply.
         
          mspSetPortPower(Arm_volts_PortNumber, 15.0); // send message to the MSP to turn OFF the PWM pulses 
        
          delayMillis(10000);  //phil  leave it on for 10 seconds
          
          // turn off the 15 volts supply.
       
         mspSetPortPower(Arm_volts_PortNumber, 0.0); // send message to the MSP to turn OFF the PWM pulses 
		
            delayMillis(5000);  //phil  leave it off for 5 seconds 
            
         // now ask the ARM to turn on the port3 PWM at 5 volts
         
         portPowerSet(Arm_volts_PortNumber, 5.0);//phil ARM sets the port power
       
         delayMillis(5000);  //phil  leave it on for 5 seconds 

         // now ask the ARM to turn off the port3 PWM 
         
         portPowerReset(Arm_volts_PortNumber);
         
         delayMillis(10000);  //phil  leave it on for 10 seconds

        // turn on the 9 volt supply.
         
          mspSetPortPower(Arm_volts_PortNumber, 9.0); // send message to the MSP to turn ON the PWM pulses 
        
          delayMillis(10000);  //phil  leave it on for 10 seconds
          
          // turn off the 15 volts supply.
       
         mspSetPortPower(Arm_volts_PortNumber, 0.0); // send message to the MSP to turn OFF the PWM pulses 
		
         delayMillis(5000);  //phil  leave it off for 5 seconds                     
         utilPwrModeCont(cs[Arm_volts_PortNumber], true);//turn on utility meter
         delayMillis(1000);  //phil time for the MSP to PWM the proper port

          // read the voltage on the port from the utility meter
         while (1)
         {
         Arm_volts_PortNumber = utilPwrGetVoltage(cs[Arm_volts_PortNumber]);
         delayMillis(10000);  

         }


        
}
int WeatherStationPort =2;
int counter_value_phil =0;
int maxcounter_value_phil =0;
int mincounter_value_phil =0;
int intervalcounter_value_phil =0;
int sumcounter_value_phil =0;
float average_value=0;

void brngup_mspweatherstationtest( void)
{
 
         

        // mspSetPortPower(WeatherStationPort, 5); // send message to the MSP to turn on the PWM pulses 

         // ask the ARM to make port 3 power while I am testing MSP with the debugger.
         // if you reset the MSP while it is making port power the ARM will have Vconx_EN high
         // and the PWM output from the MSP will be low and somehow I think you might damage the 
         // buck converter.  5/24/2017  Phil
         portPowerSet(WeatherStationPort, 5.0);//phil ARM sets the port power

         delayMillis(10000);  //phil time for the weather station to power up

           //phil     
             
 
                
         // the Davis anemometer needs a 10k pullup to +5
         mspPortConfig(WeatherStationPort,MSPCMD3_MUX_10K_UP); 
         
         mspLedDiagnostic(LED2_STAT_GREEN,MSPCMD3_PR_ON);
         delayMillis(1000);
//         mspStartSensorRead(WeatherStationPort,250);//phil - sample every 2.5 seconds
         mspStartSensorRead(WeatherStationPort,1000);//phil - sample every 10.0 seconds

         delayMillis(30000);

	 counter_value_phil = mspSendSensorData(WeatherStationPort);//phil
         delayMillis(3000);
        
         maxcounter_value_phil = mspSendSensorMaxCountData(WeatherStationPort);//phil
         delayMillis(3000);
         
         mincounter_value_phil = mspSendSensorMinCountData(WeatherStationPort);//phil
         delayMillis(3000);
         intervalcounter_value_phil = mspSendSensorIntervalCountData(WeatherStationPort);//phil
         delayMillis(3000);
                  
         sumcounter_value_phil = mspSendSensorRunningSumData(WeatherStationPort);//phil
         delayMillis(3000);
         average_value = sumcounter_value_phil/intervalcounter_value_phil;

 //        mspResetSensorData(WeatherStationPort);
         delayMillis(3000);

 //        mspClearSensorData(WeatherStationPort);      
         delayMillis(1000);//phil
        
                 // turn on the red LED so we know we are done
         mspLedDiagnostic(LED1_STAT_RED,MSPCMD3_PR_ON);//phil
         delayMillis(1000);//phil
         while(1)//phil
            {

	delayMillis(100);

            }

        
}

void mspSetPortPower(uint8_t port, float supply);
int phil_test_port_number=1;
void brngup_mspspecial5voltpower( void)
{
 
         
//      gpioDigitalWrite(&PWR1_A_EN, HIGH); // enable port power
//	gpioDigitalWrite(&PWR2_A_EN, HIGH);
//	gpioDigitalWrite(&PWR3_A_EN, HIGH);
//	gpioDigitalWrite(&PWR4_A_EN, HIGH);
         
//phil ARM sets the port power        portPowerSet(1, 12);
         delayMillis(10000);
//phil ARM turns off the port power         portPowerReset(1);
                
            
         
         // now turn on the 5 volt supply for 10 seconds on port 1
         supply_phil = 5.0;
         phil_test_port_number=1;
         mspSetPortPower(phil_test_port_number, supply_phil);
        delayMillis(10000);
        // now turn off the 5 volts supply and wait 30 seconds on port 1
         supply_phil = 0.0;
         phil_test_port_number=1;
         mspSetPortPower(phil_test_port_number, supply_phil);
	delayMillis(30000);
        // now turn it back on again.
         supply_phil = 5.0;
         phil_test_port_number=1;
         mspSetPortPower(phil_test_port_number, supply_phil);
        delayMillis(1000);
        // turn on the red LED so we know we are done
         mspLedDiagnostic(LED1_STAT_RED,MSPCMD3_PR_ON);//phil
         delayMillis(1000);//phil
    
         while(1)//phil
            {
            }

        
}

void brngup_mspspecial4volttest( void)
{
  
 

         // now turn on the 4volt supply for 10 seconds
  	mspSetPowerRails(MSPCMD2_PR_4V, true);
        delayMillis(10000);

        // now turn off the 4 volts supply and wait 30 seconds
	mspSetPowerRails(MSPCMD2_PR_4V, false);
	delayMillis(30000);
        // now turn it back on again.
        mspSetPowerRails(MSPCMD2_PR_4V, true);
        delayMillis(1000);
        // turn on the red LED so we know we are done
         mspLedDiagnostic(LED1_STAT_RED,MSPCMD3_PR_ON);//phil
         delayMillis(1000);//phil
        
         while(1)//phil
            {
            }

        
}
 void brngup_mspsinglecommandtest(void)  //phil
{   	 // special code to send lots of commands and verify that all the bytes are received
         // properly by using the CCS debugger to look at special arrays in the MSP code.
  
         int flag =0;//phil
  

   	 delayMillis(1000);//phil
         for (flag=0; flag <20; flag++)//phil
           {
              delayMillis(1000);//phil
              mspLedDiagnostic(LED2_STAT_GREEN,MSPCMD3_PR_ON);//phil
              delayMillis(1000);//phil
              mspLedDiagnostic(LED2_STAT_GREEN,MSPCMD3_PR_OFF);//phil
              delayMillis(1000);//phil
              mspLedDiagnostic(LED1_STAT_RED,MSPCMD3_PR_ON);//phil
              delayMillis(1000);//phil
              mspLedDiagnostic(LED1_STAT_RED,MSPCMD3_PR_OFF);//phil
              delayMillis(1000);//phil
              mspSetPowerRails(MSPCMD2_PR_2947, true);
	      delayMillis(1000);
	      mspSetPowerRails(MSPCMD2_PR_2947, false);
	      delayMillis(1000);		

           }
         
         // these commands are sent out of sequence and should generate 3 faults
         
         delayMillis(1000);//phil
         mspLedDiagnostic(LED1_STAT_RED,MSPCMD3_PR_ON);//phil
         delayMillis(1000);//phil
         mspLedDiagnostic(LED2_STAT_GREEN,MSPCMD3_PR_OFF);//phil
	 delayMillis(1000);
         mspSetPowerRails(MSPCMD2_PR_2947, true);

         while(1)//phil
            {
            }

}

/********************************SDI TEST**************************************/
void brngup_sditest(void)
{

  	periphEnable();
	gpioDigitalWrite(&SIG1_SEL, HIGH);
	gpioDigitalWrite(&SIG1_SEL0, LOW);
	gpioDigitalWrite(&SIG1_SEL1, LOW);
	
	gpioDigitalWrite(&SIG2_SEL, HIGH);
	gpioDigitalWrite(&SIG2_SEL0, LOW);
	gpioDigitalWrite(&SIG2_SEL1, LOW);
	
	gpioDigitalWrite(&SIG3_SEL, HIGH);
	gpioDigitalWrite(&SIG3_SEL0, LOW);
	gpioDigitalWrite(&SIG3_SEL1, LOW);
	
	gpioDigitalWrite(&SIG4_SEL, HIGH);
	gpioDigitalWrite(&SIG4_SEL0, LOW);
	gpioDigitalWrite(&SIG4_SEL1, LOW);
	
	float tmp_values[20];
	AgTechIOStruct *test = &DIG2;
	//gpioMode(&XBEE_RX, MODE_OUTPUT);
	//pwmStartPwr(2, DUTYCYC_12V);
	portPowerSet(2, 12);
	while(1)
	{
//		gpioDigitalWrite(&DIG3, LOW);
//		delayMillis(1);
//		gpioDigitalWrite(&DIG3, HIGH);
//		delayMillis(1);
		sdiMain(test, "?!",tmp_values);
		sdiMain(test, "0M!",tmp_values);
	}
}

int calcrc(char *ptr, int count)
{
    int  crc;
    char i;
    crc = 0;
    while (--count >= 0)
    {
        crc = crc ^ (int) *ptr++ << 8;
        i = 8;
        do
        {
            if (crc & 0x8000)
                crc = crc << 1 ^ 0x1021;
            else
                crc = crc << 1;
        } while(--i);
    }
    return (crc);
}

/*******************************RS232 TEST*************************************/
void brngup_rs232test(void)
{
  	periphEnable();
	portPowerSet(1,5);
	gpioDigitalWrite(&RSx_A_EN, HIGH);
	gpioDigitalWrite(&RSx_A0_SEL, LOW);
	gpioDigitalWrite(&RSx_A1_SEL, LOW);
	
	gpioDigitalWrite(&RSx_B_EN, HIGH);
	gpioDigitalWrite(&RSx_B0_SEL, LOW);
	gpioDigitalWrite(&RSx_B1_SEL, LOW);
	
	gpioDigitalWrite(&RS485_RES_EN, LOW);
	gpioDigitalWrite(&RS232_ENB, LOW);
	gpioDigitalWrite(&RS485_DXEN, LOW);
	gpioDigitalWrite(&RS485_RXENB, LOW);
	gpioDigitalWrite(&RS485_FEN, LOW);
	char *test = "B";
	char test_rx = 0;
	while(1)
	{
		gpioDigitalWrite(&RS485_DXEN, HIGH);
		gpioDigitalWrite(&RS485_RXENB, HIGH);
	  	if (uartTxRS485(test, 1) != HAL_OK)
		  	assert(0);
		gpioDigitalWrite(&RS485_DXEN, LOW);
		gpioDigitalWrite(&RS485_RXENB, LOW);
		if (uartRxRS485(&test_rx, 1, 1000) == HAL_OK)
		  	uartSHOW(&test_rx, 1);
		
		
	}
}
//#pragma optimize=none
/*******************************RS485 TEST*************************************/
void brngup_rs485test(void)
{
  	setPowerRails(true);
  	periphEnable();
	portPowerSet(1,5);
	gpioDigitalWrite(&RSx_A_EN, HIGH);
	gpioDigitalWrite(&RSx_A0_SEL, LOW);
	gpioDigitalWrite(&RSx_A1_SEL, LOW);
	
	gpioDigitalWrite(&RSx_B_EN, HIGH);
	gpioDigitalWrite(&RSx_B0_SEL, LOW);
	gpioDigitalWrite(&RSx_B1_SEL, LOW);
	
	gpioDigitalWrite(&RS485_RES_EN, LOW);
	gpioDigitalWrite(&RS232_ENB, HIGH);
	gpioDigitalWrite(&RS485_DXEN, LOW);
	gpioDigitalWrite(&RS485_RXENB, LOW);
	gpioDigitalWrite(&RS485_FEN, LOW);
	uint16_t crc_rx = 0;
	uint16_t crc_cl = 0;
	//uint8_t test[9];
	uint8_t frame[8] = {'N', 'I', 'C', '1', '2', '3', '4',  '\n'};
	uint8_t frame2[8] = {0x00, 0x05, 0x00, 0x01, 0x00, 0x00};
	uint8_t frame3[8] = {0x00, 0x01, 0x00, 0x05, 0x00, 0x02};
	uartRS485ITEnable(rxbuff_rs485);
	uint8_t frame_rx[10];
	memset(frame_rx, 0, 10);
	uint32_t temp = 0;
	char *test = "B";
	crc_cl = calcrc((char*)frame, 6);
	while(1)
	{
	  
	  		/*******************************/
//	  		gpioDigitalWrite(&RS485_DXEN, HIGH);
//			gpioDigitalWrite(&RS485_RXENB, HIGH);
//	 	  	if (uartTxRS485(frame, 8) != HAL_OK)
//		  		assert(0);
//			delayMillis(500);
			/*******************************/
			
			
//			crc_cl = calcrc((char*)frame2, 6);
//			frame2[6] = crc_cl>>8;
//			frame2[7] = crc_cl & 0xFF;
//			if (uartTxRS485(frame2, 8) != HAL_OK)
//		  		assert(0);
//			//delayMillis(200);
//			
//			crc_cl = calcrc((char*)frame3, 6);
//			frame3[6] = crc_cl>>8;
//			frame3[7] = crc_cl & 0xFF;
//			if (uartTxRS485(frame3, 8) != HAL_OK)
//		  		assert(0);
			
			/*******************************/
//			if (uartRxRS485(frame_rx, 6, 1000) == HAL_OK)
//				uartSHOW(frame_rx, 6);
//			else
//			{
//			  	HAL_UART_DeInit(&UART8HandlerDef);
//				HAL_UART_Init(&UART8HandlerDef);
//			}
			/*******************************/
	
//	  if( uartRxRS485(rxbuff_rs485, 8, 5000) == HAL_OK)
//	  {
//			//memcpy(rxbuff_rs485, &test[1], 8);
//			crc_rx = (rxbuff_rs485[6]<<8) | rxbuff_rs485[7];
//			crc_cl = calcrc((char*)rxbuff_rs485, 6);
//			if(crc_rx == crc_cl)
//			{
//				davis((DavisMainStruct*)rxbuff_rs485, 1);
//			}
//			memset(rxbuff_rs485, 0, 8);
//	  }
//	  else
//	  {
//			UARTRESET(&UART8HandlerDef);
//			UART8HandlerDef.Instance->RQR = UART8HandlerDef.Instance->RQR | 0x08;
//			
//	  }
		
	}
}

/**********************************MUX Test************************************/
void brngup_muxtest(void)
{	
	/*PORT 1*/
	gpioDigitalWrite(&SIG1_SEL, HIGH);
	gpioDigitalWrite(&SIG1_SEL0, LOW);
	gpioDigitalWrite(&SIG1_SEL1, LOW);
	
	gpioDigitalWrite(&SIG1_SEL0, HIGH);
	gpioDigitalWrite(&SIG1_SEL1, LOW);
	
	gpioDigitalWrite(&SIG1_SEL0, LOW);
	gpioDigitalWrite(&SIG1_SEL1, HIGH);
	
	gpioDigitalWrite(&SIG1_SEL0, HIGH);
	gpioDigitalWrite(&SIG1_SEL1, HIGH);
	
	/*PORT 2*/
	gpioDigitalWrite(&SIG2_SEL, HIGH);
	gpioDigitalWrite(&SIG2_SEL0, LOW);
	gpioDigitalWrite(&SIG2_SEL1, LOW);
	
	gpioDigitalWrite(&SIG2_SEL0, HIGH);
	gpioDigitalWrite(&SIG2_SEL1, LOW);
	
	gpioDigitalWrite(&SIG2_SEL0, LOW);
	gpioDigitalWrite(&SIG2_SEL1, HIGH);
	
	gpioDigitalWrite(&SIG2_SEL0, HIGH);
	gpioDigitalWrite(&SIG2_SEL1, HIGH);
	
	/*PORT 3*/
	gpioDigitalWrite(&SIG3_SEL, HIGH);
	gpioDigitalWrite(&SIG3_SEL0, LOW);
	gpioDigitalWrite(&SIG3_SEL1, LOW);
	
	gpioDigitalWrite(&SIG3_SEL0, HIGH);
	gpioDigitalWrite(&SIG3_SEL1, LOW);
	
	gpioDigitalWrite(&SIG3_SEL0, LOW);
	gpioDigitalWrite(&SIG3_SEL1, HIGH);
	
	gpioDigitalWrite(&SIG3_SEL0, HIGH);
	gpioDigitalWrite(&SIG3_SEL1, HIGH);
	
	/*PORT 4*/
	gpioDigitalWrite(&SIG4_SEL, HIGH);
	gpioDigitalWrite(&SIG4_SEL0, LOW);
	gpioDigitalWrite(&SIG4_SEL1, LOW);
	
	gpioDigitalWrite(&SIG4_SEL0, HIGH);
	gpioDigitalWrite(&SIG4_SEL1, LOW);
	
	gpioDigitalWrite(&SIG4_SEL0, LOW);
	gpioDigitalWrite(&SIG4_SEL1, HIGH);
	
	gpioDigitalWrite(&SIG4_SEL0, HIGH);
	gpioDigitalWrite(&SIG4_SEL1, HIGH);
	
	gpioMode(&DIG1, MODE_INPUT);
	gpioMode(&DIG3, MODE_INPUT);
	gpioMode(&DIG4, MODE_INPUT);
//	gpioDigitalWrite(&SIG2_SEL0, HIGH);
//	gpioDigitalWrite(&SIG2_SEL1, LOW);
//	
//	gpioDigitalWrite(&SIG2_SEL0, LOW);
//	gpioDigitalWrite(&SIG2_SEL1, HIGH);
//	
//	gpioDigitalWrite(&SIG2_SEL0, HIGH);
//	gpioDigitalWrite(&SIG2_SEL1, HIGH);
}

void fcctest(void)
{

	//buzzEnable(1);
	
	const char *fcc_test = "TEST:     ON\n";
	const char *fcc_test_o = "TEST:    OFF\n";
	while(1)
	{		
		/*Turn ON Periph*/
	  	buzzWaked();
		pwmStartPwr(1, DUTYCYC_3V);
		periphEnable();
		gpioDigitalWrite(&TP38, HIGH);
		cellON_OFF(true);
		//gpioDigitalWrite(&CELL_ON_OFF, HIGH);
		gpioMode(&XBEE_RX, MODE_OUTPUT);
		gpioDigitalWrite(&XBEE_RX, LOW);
		uartSHOW((uint8_t*)fcc_test, strlen(fcc_test));
		
		delayMillis(10000);
		delayMillis(10000);
		
		/*Turn OFF Periph*/
		periphDisable();
		gpioDigitalWrite(&TP38, LOW);
		//cellON_OFF(false);
		gpioDigitalWrite(&BANNER_EN, LOW);
		gpioDigitalWrite(&WIFI_SLEEP, LOW);
		gpioMode(&XBEE_RX, MODE_INPUT);
		gpioDigitalWrite(&XBEE_RX, HIGH);
		pwmPwrOFF(1);
		buzzEnable(1);
		uartSHOW((uint8_t*)fcc_test_o, strlen(fcc_test_o));
		
		rtcSetWakeupTimer(10);
		//HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
		HAL_PWR_EnterSTANDBYMode();
		//HAL_Delay(10000);
	}
}

/********************************************SDCARD TEST***********************/
void brngup_sdtest(void)
{
	dmInitialize();
	sdCfgGetAcctDev();
	sdtest();
	portInit();
}

/*********************************PWM TEST*************************************/
void brngup_pwmtest(void)
{
	gpioDigitalWrite(&VCON1_EN, HIGH);
	gpioDigitalWrite(&VCON2_EN, HIGH);
	gpioDigitalWrite(&VCON3_EN, HIGH);
	gpioDigitalWrite(&VCON4_EN, HIGH);
	gpioDigitalWrite(&PWR1_A_EN, HIGH);
	gpioDigitalWrite(&PWR2_A_EN, HIGH);
	gpioDigitalWrite(&PWR3_A_EN, HIGH);
	gpioDigitalWrite(&PWR4_A_EN, HIGH);
	periphEnable();
	float xtest = 0.52;
	uint32_t ptest = 3;
	while(1){
		for(uint8_t i = 1; i < 5; i++)
		{
			portPowerSet(i, ptest);
			delayMillis(1000);
			portPowerReset(i);
			delayMillis(1000);
			
			mspSetPortPower(i, ptest);
			delayMillis(1000);
			mspSetPortPower(i, 0);
			delayMillis(1000);
		}
	}
//	for(xtest = 0.1; xtest < 0.7; xtest+=0.05)
//	{
//		pwmStartPwr(4, DUTYCYC_12V);
//		pwmPwrOFF(4);
//	}
//	pwmStartPwr(ptest, xtest);
	
}
#pragma optimize=none
/***********************************************RTC TEST***********************/
void brngup_rtctest(void)
{
	extRTCInit();
	RTCDateTimeStruct rtctest;
	rtctest.Millis = 0;
	rtctest.Seconds = 25;
	rtctest.Minutes = 5;
	rtctest.Hours = 17;
	rtctest.Weekday = 1;
	rtctest.Date = 18;
	rtctest.Month = 7;
	rtctest.Year = 16;
	exRTCWriteTime(&rtctest);
	while(1)
	{
		//exRTCSample();
		exRTCReadTime(&rtctest);
		delayMillis(100);
	}
}

/*****************************************ACCE TEST****************************/
void brngup_accetest(void)
{
	gpioDigitalWrite(&ACCEL_ENB, LOW);
	while(1)
	sysAzimuth(NULL);
}

/*****************************************OWIRE TEST***************************/
void brngup_owiretest(void)
{
	uint16_t id_test = 0x50;
	AgTechIOStruct *test = &SIG4_ID;
	while(1)
	{
	  	portWriteID(id_test + 1, test);
		delayMillis(200);
		portReadID(&id_test, test);
		portReadID(&id_test, test);
		id_test = id_test + 1;
	}
}

/***********************************CELL TEST**********************************/
void brngup_celltest(void)
{
 	#if DEVICE_TYPE == DEVICETYPE_GATE
		#if ENABLE_POSTING
			cellForcedShutDown();
			cellON_OFF(true);
			cellSetupParameters();
			#if CELLTYPE == CELLTYPE_CDMA
				cellProvision();
			#endif
				cellSetDateTime();
			cellON_OFF(false);
		#endif
 	#endif
}

/************************************GPS TEST**********************************/
void brngup_gpstest(void)
{
	//gpioDigitalWrite(&GPS_ON_OFF, HIGH);
	//gpioDigitalWrite(&GPS_SYSON, HIGH);
	gpioDigitalWrite(&GPS_RESET, HIGH);
	uint8_t gpsrx[2];// = malloc(1);
	//delayMillis(5000);
	gpsExec(0, 100000);
	gpioMode(&GPS_ON_OFF, MODE_OUTPUT);
	gpioDigitalWrite(&GPS_ON_OFF, HIGH);
	//delayMillis(1000);
//	/* open drain as GPS on-off pin logic low */
	gpioDigitalWrite(&GPS_ON_OFF, LOW);
	//delayMillis(REST_PULSE_DLY);
	//HAL_Delay(1000);
	HAL_StatusTypeDef current_stat = HAL_OK;
	while(1)
	{	
//	  	current_stat = HAL_UART_Receive(&UART4HandlerDef, gpsrx, 1, (uint32_t)100);
//	  	if (current_stat == HAL_OK)
//	  		uartSHOW((uint8_t*)gpsrx, 1);
//		else
//		{	
//			HAL_UART_DeInit(&UART4HandlerDef);
//			HAL_UART_Init(&UART4HandlerDef);
//		}
	}
}

void brngup_pwrmntrtest(void)
{
	float vcc = 0;
	utilPwrModeCont(&PM_SIG1, true);
	utilPwrModeCont(&PM_SIG2, true);
	utilPwrModeCont(&PM_SIG3, true);
	utilPwrModeCont(&PM_SIG4, true);
	while(1)
	{
		utilPwrModeCont(&PM_SIG1, true);
		utilPwrModeCont(&PM_SIG2, true);
		utilPwrModeCont(&PM_SIG3, true);
		utilPwrModeCont(&PM_SIG4, true);
		
		vcc = utilPwrGetVoltage(&PM_SIG1);
		delayMicros(1);
		vcc = utilPwrGetVoltage(&PM_SIG2);
		delayMicros(1);
		vcc = utilPwrGetVoltage(&PM_SIG3);
		delayMicros(1);
		vcc = utilPwrGetVoltage(&PM_SIG4);
		if(vcc > 5)
		  delayMillis(5);
		utilPwrModeCont(&PM_MAIN, false);
		utilPwrModeCont(&PM_SIG1, false);
		utilPwrModeCont(&PM_SIG2, false);
		utilPwrModeCont(&PM_SIG3, false);
		utilPwrModeCont(&PM_SIG4, false);
	}
}

/**************************************ADC TEST********************************/
void brngup_adctest(void)
{
	float adc_val = 0;
	gpioDigitalWrite(&SIG1_SEL, HIGH);
	gpioDigitalWrite(&SIG1_SEL0, LOW);
	gpioDigitalWrite(&SIG1_SEL1, HIGH);
	
	gpioDigitalWrite(&SIG2_SEL, HIGH);
	gpioDigitalWrite(&SIG2_SEL0, LOW);
	gpioDigitalWrite(&SIG2_SEL1, HIGH);
	
	gpioDigitalWrite(&SIG3_SEL, HIGH);
	gpioDigitalWrite(&SIG3_SEL0, LOW);
	gpioDigitalWrite(&SIG3_SEL1, HIGH);
	
	gpioDigitalWrite(&SIG4_SEL, HIGH);
	gpioDigitalWrite(&SIG4_SEL0, LOW);
	gpioDigitalWrite(&SIG4_SEL1, HIGH);
	
	AgTechIOStruct *cs = &AD_CS;
	uint32_t orig_code = 0;
	uint32_t code_nosign = 0;
	gpioDigitalWrite(&AD_CONV, LOW);

	
	//adcSpiReadNone(&adc_val, &SIG1_AD_EN);
	//adcSpiReadNone(&adc_val, &SIG1_AD_EN);
	while(1)
	{
	  if(adcSpiRead(&adc_val, cs, 0) != SUCCESSFUL)
		 assert(0);
	  daqPrint(adc_val, 0, 0, 0, 0, 0, 1, 0);
	  
	  
	  if(adcSpiRead(&adc_val, cs, 1) != SUCCESSFUL)
		 assert(0);
	  daqPrint(adc_val, 0, 0, 0, 0, 0, 2, 0);
	  
	  
	  if(adcSpiRead(&adc_val, cs, 2) != SUCCESSFUL)
		 assert(0);
	  daqPrint(adc_val, 0, 0, 0, 0, 0, 3, 0);
	  
	  
	  if(adcSpiRead(&adc_val, cs, 3) != SUCCESSFUL)
		 assert(0);
	  daqPrint(adc_val, 0, 0, 0, 0, 0, 4, 0);
	  
	  uartSHOW("\n\n", 2);
	  delayMillis(1000);
	}	
}

/**************************************ADC TEST********************************/
void brngup_thermtest(void)
{
	float adc_val = 0;
	gpioDigitalWrite(&SIG1_SEL, HIGH);
	gpioDigitalWrite(&SIG1_SEL0, HIGH);
	gpioDigitalWrite(&SIG1_SEL1, HIGH);
	
	gpioDigitalWrite(&SIG2_SEL, HIGH);
	gpioDigitalWrite(&SIG2_SEL0, HIGH);
	gpioDigitalWrite(&SIG2_SEL1, HIGH);
	
	gpioDigitalWrite(&SIG3_SEL, HIGH);
	gpioDigitalWrite(&SIG3_SEL0, HIGH);
	gpioDigitalWrite(&SIG3_SEL1, HIGH);
	
	gpioDigitalWrite(&SIG4_SEL, HIGH);
	gpioDigitalWrite(&SIG4_SEL0, HIGH);
	gpioDigitalWrite(&SIG4_SEL1, HIGH);
	
	AgTechAdcStruct *adc = &THERM1_ADC;
	EquationParameterStruct params_ptr;
	while(1)
	{
	  	adc_val = gpioAnalogRead(adc);
		params_ptr.raw_value = adc_val;
		eqThermistor(&params_ptr);
	}
}
	
/**********************SPI EXT FL TEST*****************************************/
void brngup_extflashtest(void)
{
	extFlashInit();
	const char *buffer = "THIS IS A TEST";
	const char *buffer2 = "THIS IS ANOTHER TEST";
	char rx_buf_test[128];
	extFlSectorErase(2, 0);
	extFlWrite((int8_t*)buffer, 0x010000, 128);
	delayMillis(100);
	extFlRead((int8_t*)rx_buf_test, 0x010000, 128);
	uartSHOW((uint8_t*)rx_buf_test, 14);
	
	while(1)
	{
		extFlSectorErase(2, 0);
		delayMillis(100);
		extFlWrite((int8_t*)buffer2, 0x010000, 128);
		delayMillis(100);
		extFlRead((int8_t*)rx_buf_test, 0x010000, 128);
		uartSHOW((uint8_t*)rx_buf_test, 14);
	}
}

void brngup_valvetest(void)
{
  		gpioDigitalWrite(&VCON15_EN, HIGH);
	while(1)
	{
	  	valveSetState(1, VALVE_OPEN, REVERSE_POLARITY);
		delayMillis(1000);
		valveSetState(1, VALVE_CLOSE, REVERSE_POLARITY);
		delayMillis(1000);
		
		valveSetState(2, VALVE_OPEN, REVERSE_POLARITY);
		delayMillis(1000);
		valveSetState(2, VALVE_CLOSE, REVERSE_POLARITY);
		delayMillis(1000);
	}
}

#pragma optimize=none
void brngup_dthtest(void)
{	
	AgTechIOStruct *test = &SIG2_ID;
	io_dth_sck = &DIG2;
	float temp_val = 0;
	float rh_val = 0;
	uint16_t id_test = 0;
	uint8_t port = 2;
	AgTechIOStruct *sigx_sel_ptr[] = {&SIG1_SEL, &SIG2_SEL, &SIG3_SEL ,&SIG4_SEL};
    AgTechIOStruct *sigx_sel0_ptr[] = {&SIG1_SEL0, &SIG2_SEL0, &SIG3_SEL0 ,&SIG4_SEL0};
    AgTechIOStruct *sigx_sel1_ptr[] = {&SIG1_SEL1, &SIG2_SEL1, &SIG3_SEL1 ,&SIG4_SEL1};
	gpioDigitalWrite(sigx_sel_ptr[port - 1], true);
	gpioDigitalWrite(sigx_sel0_ptr[port - 1], 0);
	gpioDigitalWrite(sigx_sel1_ptr[port - 1], 0);
	gpioDettachInt(&BLE_IRQ);

	gpioDigitalWrite(&DLS_OE, LOW);
	gpioDigitalWrite(&DLS_DIR, HIGH);
	gpioMode(io_dth_sck, MODE_OUTPUT);
	portPowerSet(port, 3);
	delayMillis(5000);
  	while(1)
	{
//		portWriteID(0x50,&SIG2_ID);
//		delayMillis(200);
//		portReadID(&id_test, &SIG2_ID);
//		id_test = id_test + 1;
		if (davisDTHReadTemp(&temp_val, test, io_dth_sck)  == SUCCESSFUL)
		{
		  	temp_val = temp_val*9/5 + 32;
			temp_val = (temp_val - 32)*5/9;
		}
		else
		  delayMillis(5000);
		
		if (davisDTHReadRelHum(&rh_val, test, io_dth_sck)  == SUCCESSFUL)
		{
		  	rh_val = rh_val + 10;
			rh_val = rh_val - 10;
		}
		else
		  delayMillis(5000);
	}
}

void brngup_eepromtest(void)
{
	InterfaceStatusEnum i2c_stat = 0;
	uint8_t cmd_write[] = {0,0,0x40};
	uint8_t databuff[2];
	
	i2c_stat = (i2c1Write((uint8_t)I2CADD_EEPROM,(uint8_t *)cmd_write,3));
	
	if (i2c_stat != INTERFACE_OK)
		 assert(0);
	delayMillis(100);
	
	i2c_stat = (i2c1Write((uint8_t)I2CADD_EEPROM,(uint8_t *)cmd_write,2));
	delayMillis(100);
	
	if ( i2c1Readx((uint8_t)I2CADD_EEPROM, 0, databuff, 1)
					== INTERFACE_ERROR)
			 assert(0);
	uartSHOW(databuff,1);
}

void brngup_delaytest(void)
{
  	gpioMode(&XBEE_RX, MODE_OUTPUT);
	
	while(1)
	{
		gpioDigitalWrite(&XBEE_RX, HIGH);
	 	delayMicros(1);
		gpioDigitalWrite(&XBEE_RX, LOW);
	 	delayMicros(1);
	}
}