///////////////////////////spin the encoder win a frequency!!////////////////////////////
//
//		CIV.cpp
//
//   Process CIV messages sent and received
//

#include "ICOM_IC-905_CIV.h"
#include "RadioConfig.h"
#include "CIV.h"

extern Metro CAT_Poll;        // Throttle the servicing for CAT comms
extern Metro CAT_Log_Clear;   // Clear the CIV log buffer
extern Metro CAT_Freq_Check;  // Clear the CIV log buffer

extern uint8_t curr_band;  // global tracks our current band setting.
extern uint64_t VFOA;      // 0 value should never be used more than 1st boot before EEPROM since init should read last used from table.
extern uint64_t VFOB;
extern uint8_t radio_mode;         // mode from radio messages
extern uint8_t radio_filter;       // filter from radio messages
extern uint8_t radio_data;       // filter from radio messages
extern struct Band_Memory bandmem[];
extern struct Modes_List modeList[];
extern struct Filter_Settings filter[];
extern unsigned int hexToDec(String hexString);

uint64_t freq = 0;
radioOnOff_t radioS = RADIO_NDEF;
radioOnOff_t localS = RADIO_NDEF;
uint64_t radioF = 0;
uint64_t localF = 0;
uint8_t radioM = MOD_NDEF;
uint8_t localM = MOD_NDEF;
uint8_t radioFil = FIL_NDEF;
uint8_t localFil = FIL_NDEF;

// ********************************************Loop ******************************************
bool freqReceived = false;  // initially, no frequency info has been received from the radio
//bool modeReceived = false;
uint8_t freqPoll = 0;  // number of initial frequency queries in addition to the broadcast info

//-------------------------------------------------------------------------------
// create the civ object
CIV civ;  // create the CIV-Interface object
ICradio ICxxxx(TypeIC905, CIV_ADDR_905);
//-------------------------------------------------------------------------------

CIVresult_t CIVresultL;

void civ_905_setup(void) {
  civ.setupp(true, false, "");     // initialize the civ object/module
                                   // and the ICradio objects
  civ.registerAddr(CIV_ADDR_905);  // tell civ, that this is a valid address to be used
  ICxxxx.setupp(millis());
}

//---------------------------------------------------------------------------------------------
// get the radio data and print them via the USB COM-port into the serial monitor

void getradioInfo() {
  ICxxxx.loopp(millis());
  radioS = ICxxxx.getAvailability();
  if (radioS != localS) {  // if there is a change in ON/OFF state -> print it out!
    PC_Debug_port.print("radioState: ");
    PC_Debug_port.println(radioOnOffStr[radioS]);
    localS = radioS;
  }

  if (radioS == RADIO_ON) {  // only in case, the radio is switched on
                             // and the connection is up and running

    radioF = ICxxxx.getFrequency();
    if (localF != radioF) {  // if there is a frequency change -> print it out!
      PC_Debug_port.print("Freq[Hz]: ");
      PC_Debug_port.println(radioF);
      localF = radioF;
    }

    radioM = ICxxxx.getModMode();
    radioFil = ICxxxx.getRxFilter();
    if ((localM != radioM) || (localFil != radioFil)) {  // if there is a change -> print it out!
      PC_Debug_port.print("Mod:  ");
      PC_Debug_port.print(modModeStr[radioM]);
      PC_Debug_port.print(" Fil: ");
      PC_Debug_port.println(FilStr[radioFil]);
      localM = radioM;
      localFil = radioFil;
    }
  }
}

//***************************************************************************
//                check_CIV
//
// Polls for messages in queue from radio (CIV_Transceive must be on)
// returns
// 0 = nothing received
// 1 frequency received
// 2 mode received
// xxx others TBD
//
//***************************************************************************
//
uint8_t check_CIV(uint32_t time_current_baseloop) {
  	uint8_t msg_type;

  	msg_type = 0;
  	CIVresultL = civ.readMsg(CIV_ADDR_905);

  	//reqReceived = false;
  	if (CIVresultL.retVal <= CIV_NOK) {  // valid answer received !
    if (CIVresultL.retVal == CIV_OK_DAV) {  // Data available
		// Check for Frequency message type
		if ((CIVresultL.cmd[1] == CIV_C_F_READ[1]) ||  // command CIV_C_F_READ received
			(CIVresultL.cmd[1] == CIV_C_F_SEND[1])) 
		{  // command CIV_C_F_SEND received
			DPRINTF("check_CIV: CI-V Returned Frequency: "); DPRINTLN(CIVresultL.value);
			VFOA = (uint64_t)CIVresultL.value;
			msg_type = 1;
			freqReceived = true;
		}

		// Test for MODE change
		if ((CIVresultL.cmd[1] == CIV_C_MOD_READ[1]) ||  // command CIV_C_MODE_SEND received
			(CIVresultL.cmd[1] == CIV_C_MOD_SEND[1]) ) 
		{  // command CIV_C_MODE_READ received
			radio_mode = hexToDec(CIVresultL.value/100);
			radio_filter = CIVresultL.value - ((CIVresultL.value/100)*100);
			get_Mode_from_Radio();
			DPRINTF("check_CIV: CI-V Returned Mode: "); DPRINT(modeList[radio_mode].mode_label);  DPRINTF("  Filter: "); DPRINTLN(filter[radio_filter].Filter_name);    
			msg_type = 2;
			freqReceived = false;
		}  // Mode changed

		// Test for extended MODE change
		if ((CIVresultL.cmd[1] == CIV_C_F26_READ[1]) ||
			(CIVresultL.cmd[1] == CIV_C_F26_SEND[1]) ) 
		{
			// [0]=x is length, [1]== 0 is selected VFO
			radio_mode   = CIVresultL.datafield[2];  // mode
			radio_data   = bandmem[curr_band].data_A   = CIVresultL.datafield[3];  // data on/off
			radio_filter = bandmem[curr_band].filter_A = CIVresultL.datafield[4];  // filter setting

			for (uint8_t i = 0; i< MODES_NUM; i++)
			{
				if (modeList[i].mode_num == radio_mode && modeList[i].data == radio_data)
					radio_mode = bandmem[curr_band].mode_A = i;   // convert to our own mode list to show -D (or not)
			}
			modeList[radio_mode].Width = radio_filter;  // store filter in mode table using the extended mode value (-D or no -D)
			DPRINTF("check_CIV: CI-V Returned Extended Mode: "); DPRINT(modeList[radio_mode].mode_label); DPRINT("  Filter: "); DPRINT(filter[radio_filter].Filter_name); DPRINT("  Data: "); DPRINTLN(radio_data);  
			msg_type = 4;
			freqReceived = false;
		}  // Mode changed
	
		// Test for Bstack message
		if (CIVresultL.cmd[1] == CIV_C_BSTACK[1]) 
		{
			// returned mnessage is 1A 01 band code, reg code, freq, mode, data on/off
			// receive fields 6 to 52 on p18 of prog guide
			// 6-10 freq or 6-12 for 10GHz+
			// +2 Mode
			// +1 Data mode on/off
			// can ignore the rest - appies to Rptrs and DigV modes
			uint16_t bstack_band  = CIVresultL.datafield[1];  // byte 0 is the datafield length
			uint16_t bstack_reg   = CIVresultL.datafield[2];

			DPRINTF("check_CIV: CI-V Returned Bstack Band: "); DPRINT(bstack_band); DPRINTF("  Register: "); DPRINT(bstack_reg);
				
			uint8_t F_len = 6;	// 6 bytes for IC905
			uint8_t idx = 0;
			uint8_t DstartIdx = 3;  // start of freq for 12 bytes
			uint8_t DstopIdx = DstartIdx + F_len;  // start of mode, filter data on/off will be 1-3 bytes after
			uint8_t rxBuffer[F_len];  // hold 6 bytes of frequency
			uint64_t mul = 1;
			uint64_t bstack_freq = 0;
			uint8_t band = 0;  // temp storage for radio bstack band code to remote bandmem table band index

			for (idx = DstartIdx; idx < DstopIdx; idx++)           // pull out frequency from datafield result
				rxBuffer[idx-DstartIdx] = CIVresultL.datafield[idx];

			// 6 byte data -> first byte is of lowest order - for 905 10G and up bands
			for (idx = 0; idx < F_len; idx++) {
				bstack_freq += (rxBuffer[idx] & 0x0f) * mul; mul *= 10;
				bstack_freq += (rxBuffer[idx] >> 4) * mul; mul *= 10;
			}
			DPRINTF("  Frequency: "); DPRINT(bstack_freq);
			
			radio_mode = CIVresultL.datafield[DstopIdx];  // modulation mode
			radio_filter = CIVresultL.datafield[DstopIdx+1];  // filter 
			radio_data = CIVresultL.datafield[DstopIdx+2];  // data mode on or off
			DPRINTF("  Mode: "); DPRINT(radio_mode, HEX); DPRINT("  Filter: ");DPRINT(radio_filter, HEX);  DPRINT("  Data: ");DPRINTLN(radio_data, HEX);   

			// convert radio bstack band code to remote bandmem table band index
			switch (bstack_band)
			{
				case 1: band = BAND144; break;
				case 2: band = BAND432; break;
				case 3: band = BAND1296; break;
				case 4: band = BAND2400; break;
				case 5: band = BAND5760; break;
				case 6: band = BAND10G; break;
				default: band = BAND144; break;
			}

			switch (bstack_reg)
			{
				case 1: bandmem[band].vfo_A_last   = bstack_freq; 
						bandmem[band].mode_A = radio_mode;
						bandmem[band].filter_A = radio_filter;
						bandmem[band].data_A = radio_data; 		// LSB, USB, AM, FM modes can have DATa mode on or off.  All other radio modes data is NA.
						break;
				case 2: bandmem[band].vfo_A_last_1 = bstack_freq; 
						bandmem[band].mode_A_1 = radio_mode; 
						bandmem[band].filter_A_1 = radio_filter;
						bandmem[band].data_A_2 = radio_data;
						break;
				case 3: bandmem[band].vfo_A_last_2 = bstack_freq; 
						bandmem[band].mode_A_2 = radio_mode;
						bandmem[band].filter_A_2 = radio_filter; 
						bandmem[band].data_A_2 = radio_data;
						break;
			}

          	msg_type = 3;
          	freqReceived = false;
      	}
		return msg_type;
    }  // Data available

    // ----------------------------------  do a query for frequency, if necessary
    // poll every 500 * 10ms = 5sec until a valid frequency has been received
    if ((freqReceived == false) && (CAT_Freq_Check.check() == 1)) {
      CIVresultL = civ.writeMsg(CIV_ADDR_905, CIV_C_F_SEND, CIV_D_NIX, CIV_wChk);
      if (CIVresultL.retVal<=CIV_NOK)
      {
        DPRINTF("check_CIV: Poll for RADIO Frequency Status: "); DPRINT(CIVresultL.retVal);
        DPRINTF("  Poll for RADIO Frequency Return Value: "); DPRINTLN((uint8_t)CIVresultL.value);
        msg_type = 1;
      }
    }
  }  // valid answer received
  return msg_type;
}  // if BASELOOP_TICK

// Early test function, not used for now
void SendCIVmsg(void) {
  //uint8_t str[20] = "\xFE\xFE\xE0\x05\x00\x00\x20\x44\x01\xFD\x00";  // Just a test string
  uint8_t str[20] = "\x00\x01\x01\x02";  // Just a test string
  //  writeMsg format is address, command byte(s), data bytes (length byte plus actual data bytes), type of response expected
  if (CAT_Poll.check() == 1) {
    CIVresultL = civ.writeMsg(CIV_ADDR_905, CIV_C_F26_READ, str, CIV_wChk);
    //CIVresultL = civ.writeMsg (CIV_ADDR_905, CIV_C_F_READ, CIV_D_NIX, CIV_wChk);
    PC_Debug_port.print("retVal of writeMsg: ");
    PC_Debug_port.println(retValStr[CIVresultL.retVal]);
  }
  //give the radio some time to answer version 1:
  //delay(20);
  //CIVresultL = civ.readMsg(CIV_ADDR_905);
  //PC_Debug_port.print("retVal: ");      Serial.print(CIVresultL.retVal);
  //PC_Debug_port.print(" Frequency: "); Serial.println(CIVresultL.value);
}

// Early test function - not used for now.
void RcvCIVmsg(void) {
  // give the radio some time to answer - version 2:
  // do a cyclic polling until data is available
  //if (CAT_Poll.check() == 1) {
  CIVresultL = civ.readMsg(CIV_ADDR_905);
  if (CIVresultL.retVal <= CIV_NOK) {  // valid answer received !
    //PC_Debug_port.print("retVal: ");      PC_Debug_port.print(retValStr[CIVresultL.retVal]);
    if (CIVresultL.retVal == CIV_OK_DAV)  // Data available
    {
      DPRINTF("RcvCIVMsg: Frequency: ");
      DPRINTLN(CIVresultL.value);
      freq = (uint64_t)CIVresultL.value;
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

uint64_t FrequencyRequest() {
  return ICxxxx.getFrequency();  // frequency in Hz

#if defined(REQUEST)
  if (REQUEST > 0 && (millis() - RequestTimeout[0] > RequestTimeout[1])) {
    CIVresultL = civ.writeMsg(CIV_ADDR_905, CIV_C_F_READ, CIV_D_NIX, CIV_wChk);
    PC_Debug_port.print("retVal of writeMsg: ");
    PC_Debug_port.println(retValStr[CIVresultL.retVal]);
    RequestTimeout[0] = millis();
  }
#endif
}

radioModMode_t getModMode(void) {
  return ICxxxx.getModMode();
}

#ifdef GPS
void pass_GPS(void) {
  civ.readGPS();  // read USB serial ch 'B' for GPS NMEA data strings
                  // ToDo: extract grid and time info and display
}
#endif

void pass_CAT_msgs_to_RADIO(void) {
  civ.pass_CAT_msg_to_RADIO();
}

//void pass_CAT_msg_to_PC(void)
//{
//civ.pass_CAT_msg_to_PC();   // civ.readmsg() always does this.
//}

// If you want to see the hex message contest turn on logging in the library file.  This will display the log.
void show_CIV_log(void) {
  if (CAT_Poll.check() == 1)
    civ.logDisplay();  // show messages accumulated until cleared.

  // can clear the log periodically here based on timer
  if (CAT_Log_Clear.check() == 1)  // Clear the CIV log buffer, jsu show last 2 seconds
    civ.logClear();
}

uint8_t getByteResponse(const uint8_t m_Counter, const uint8_t offset, const uint8_t buffer[])
{
  if (m_Counter < offset + 3)
        return 0;
	uint8_t ret = bcdByte(buffer[offset]) * 100;
    ret += bcdByte(buffer[offset+1]);
    return ret;
}


// This is not working yet
uint8_t getRadioMode(void)
{
  uint8_t md = 0;
  //uint8_t ret_val = 0;

  //FrequencyRequest();
  // ask for the ModMode and Filterinfo
  //CIVresultL = civ.writeMsg(CIV_ADDR_905, CIV_C_MOD_READ, CIV_D_NIX, CIV_wChk);
  //CIVresultL = civ.writeMsg(CIV_ADDR_905, CIV_C_MOD1_SEND, CIV_D_NIX, CIV_wFast);
  SendCIVmsg();
  if (CIVresultL.retVal <= CIV_NOK) {
      DPRINT("getRadioMode: Mode request got OK back: "); DPRINTLN(CIVresultL.value);
  }
  //md = check_CIV(millis());
  //civ.readMsg(CIV_ADDR_905);
  //if (CIVresultL.retVal <= CIV_NOK) {
      //DPRINT("getRadioMode: readMsg request got OK back: "); DPRINTLN(CIVresultL.value);
  //}
  //if (ret_val == 2)  // got modulation mode
  //    {
  //      DPRINT("getRadioMode: Mode = "); DPRINT(radio_mode); DPRINT(" Filter = "); DPRINTLN(radio_filter);  
  //      //DPRINTLN(getModMode());
  //      selectMode(radio_mode);
  //    }

  //ICxxxx.loopp(millis());
  //md = ICxxxx.getModMode();
  DPRINT("getRadioMode: Getting mode from radio: ");DPRINTLN(md);
  return md;
}