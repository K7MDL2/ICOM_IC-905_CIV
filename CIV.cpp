///////////////////////////spin the encoder win a frequency!!////////////////////////////
//
//		CIV.cpp
//
//   Process CIV messages sent and received
//

#include "ICOM_IC-905_CIV.h"
#include "RadioConfig.h"
#include "CIV.h"

extern Metro CAT_Poll;          //   = Metro();  // Throttle the servicing for CAT comms
extern Metro CAT_Log_Clear;   //     = Metro();   // Clear the CIV log buffer
extern Metro CAT_Freq_Check;//       = Metro();   // Clear the CIV log buffer

extern uint8_t     curr_band;  // global tracks our current band setting.  
extern uint64_t    VFOA;        // 0 value should never be used more than 1st boot before EEPROM since init should read last used from table.
extern uint64_t    VFOB;

uint64_t        freq        = 0;
radioOnOff_t  	radioS      = RADIO_NDEF;
radioOnOff_t  	localS      = RADIO_NDEF;
uint64_t 	      radioF      = 0;
uint64_t 	      localF      = 0;
uint8_t       	radioM      = MOD_NDEF;
uint8_t       	localM      = MOD_NDEF;
uint8_t       	radioFil    = FIL_NDEF;
uint8_t       	localFil    = FIL_NDEF;
// ********************************************Loop ******************************************
bool    freqReceived = false; // initially, no frequency info has been received from the radio
bool    modeReceived = false;
uint8_t freqPoll     = 0;     // number of initial frequency queries in addition to the broadcast info

//-------------------------------------------------------------------------------
// create the civ object

CIV         civ;  // create the CIV-Interface object
ICradio ICxxxx(TypeIC905,CIV_ADDR_905);
//-------------------------------------------------------------------------------

CIVresult_t CIVresultL;

void civ_setup(void)
{
    civ.setupp(true, false, "");      // initialize the civ object/module
                                      // and the ICradio objects

    civ.registerAddr(CIV_ADDR_905);    // tell civ, that this is a valid address to be used
}

//---------------------------------------------------------------------------------------------
// get the radio data and print them via the USB COM-port into the serial monitor

void	getradioInfo() {

    radioS = ICxxxx.getAvailability();
    if (radioS != localS) {               // if there is a change in ON/OFF state -> print it out!
      PC_Debug_port.print ("radioState: "); PC_Debug_port.println (radioOnOffStr[radioS]);
      localS = radioS;
    }

    if (radioS==RADIO_ON) {               // only in case, the radio is switched on 
                                          // and the connection is up and running

      radioF = ICxxxx.getFrequency();
      if (localF != radioF) {             // if there is a frequency change -> print it out!
        PC_Debug_port.print    ("Freq[Hz]: "); PC_Debug_port.println  (radioF);
        localF = radioF;
      }

      radioM    = ICxxxx.getModMode();
      radioFil  = ICxxxx.getRxFilter();
      if ((localM != radioM) || (localFil != radioFil)) { // if there is a change -> print it out!
        PC_Debug_port.print ("Mod:  "); PC_Debug_port.print   (modModeStr[radioM]);
        PC_Debug_port.print (" Fil: "); PC_Debug_port.println (FilStr[radioFil]);
        localM = radioM; localFil = radioFil;
      }
	}
}

void check_CIV(void)
{     
  CIVresultL = civ.readMsg(CIV_ADDR_905);
  if (CIVresultL.retVal<=CIV_NOK) {               // valid answer received !           
      #ifdef debug
        //DPRINTF('.');
      #endif
      if (CIVresultL.retVal==CIV_OK_DAV) {          // Data available
          // Frequency
          if ((CIVresultL.cmd[1]==CIV_C_F_SEND[1]) ||  // command CIV_C_F_SEND received
              (CIVresultL.cmd[1]==CIV_C_F_READ[1])) {  // command CIV_C_F_READ received
              freqReceived = true;
              DPRINTF("CI-V Returned Frequency: "); DPRINTLN(CIVresultL.value);
              freq = (uint64_t) CIVresultL.value;
              VFOA = freq;
              formatVFO(VFOA);
              if (!find_new_band(VFOA, curr_band))  // find band index for VFOA frequency, don't change bands is VFO is in the current band
                  changeBands(0);
              displayFreq();                        
          } // command CIV_C_F_SEND or CIV_C_F_READ received  
          // Test for MODE change
          else if ((CIVresultL.cmd[1]==CIV_C_MOD_SEND[1]) ||  // command CIV_C_MODE_SEND received
              (CIVresultL.cmd[1]==CIV_C_MOD_READ[1])) {  // command CIV_C_MODE_READ received
              modeReceived = true;
              DPRINTF("Mode Changed: ");
              DPRINTLN((uint8_t) CIVresultL.value);
              selectMode((uint8_t) CIVresultL.value); // Select the mode for the Active VFO
          }
      } // Data available
  } // valid answer received           

  // ----------------------------------  do a query for frequency, if necessary
  // poll every 500 * 10ms = 5sec until a valid frequency has been received
  if ( (freqReceived == false) && (CAT_Freq_Check.check() == 1) ) { 
      civ.writeMsg (CIV_ADDR_905, CIV_C_F_READ, CIV_D_NIX, CIV_wChk);
      //if (CIVresultL.retVal<=CIV_NOK)
      DPRINTF("Poll for RADIO Frequency Status: "); DPRINTLN(CIVresultL.retVal);
      DPRINTF("Poll for RADIO Frequency Return Value: "); DPRINTLN((uint8_t) CIVresultL.value);
  }        
} // if BASELOOP_TICK


void SendCIVmsg(void)
{
    uint8_t str[20] = "\xFE\xFE\xE0\x05\x00\x00\x20\x44\x01\xFD\x00";
    if (CAT_Poll.check() == 1) {
    CIVresultL = civ.writeMsg (CIV_ADDR_905, CIV_C_MOD1_SEND, str, CIV_wChk);
    //CIVresultL = civ.writeMsg (CIV_ADDR_905, CIV_C_F_READ, CIV_D_NIX, CIV_wChk);
    PC_Debug_port.print("retVal of writeMsg: "); PC_Debug_port.println(retValStr[CIVresultL.retVal]);
    
    }
    //give the radio some time to answer version 1:
    //delay(20);
    //CIVresultL = civ.readMsg(CIV_ADDR_905);
    //PC_Debug_port.print("retVal: ");      Serial.print(CIVresultL.retVal);
    //PC_Debug_port.print(" Frequency: "); Serial.println(CIVresultL.value);
}

void RcvCIVmsg(void)
{
    // give the radio some time to answer - version 2:  
    // do a cyclic polling until data is available
    //if (CAT_Poll.check() == 1) {
    CIVresultL = civ.readMsg(CIV_ADDR_905);
    if (CIVresultL.retVal<=CIV_NOK) {  // valid answer received !
        //PC_Debug_port.print("retVal: ");      PC_Debug_port.print(retValStr[CIVresultL.retVal]);
        if (CIVresultL.retVal==CIV_OK_DAV) // Data available
        {
            DPRINTF("RcvCIVMsg: Frequency: "); DPRINTLN(CIVresultL.value);
            freq = (uint64_t) CIVresultL.value;
            VFOA = freq;
            formatVFO(VFOA);
            if (!find_new_band(VFOA, curr_band))  // find band index for VFOA frequency, don't change bands if freq in current band
                changeBands(0);
            //bandSET();
            displayFreq();
        }
    }

    // use "l",if "#define log_CIV" in file civ.h is active
    //if (keyCmd==KEY_LOG_PRESSED) civ.logDisplay();
}

void FrequencyRequest(){
  #if defined(REQUEST)
  if(REQUEST > 0 && (millis() - RequestTimeout[0] > RequestTimeout[1])){
    CIVresultL = civ.writeMsg (CIV_ADDR_905, CIV_C_F_READ, CIV_D_NIX, CIV_wChk);
    PC_Debug_port.print("retVal of writeMsg: "); PC_Debug_port.println(retValStr[CIVresultL.retVal]);
    RequestTimeout[0]=millis();
  }
  #endif
}


#ifdef GPS
void pass_GPS(void)
{
    civ.readGPS();   // read USB serial ch 'B' for GPS NMEA data strings
    // ToDo: extract grid and time info and display
}
#endif

void pass_CAT_msgs_to_RADIO(void)
{
  civ.pass_CAT_msg_to_RADIO();
}


//void pass_CAT_msg_to_PC(void)
//{
    //civ.pass_CAT_msg_to_PC();   // civ.readmsg() always does this.
//}

    //if (CAT_Poll.check() == 1) 
        //civ.logDisplay();  // show messages accumulated until cleared.

    //if (CAT_Log_Clear.check() == 1)      // Clear the CIV log buffer, jsu show last 2 seconds
        //civ.logClear();