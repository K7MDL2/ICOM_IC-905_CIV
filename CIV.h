///////////////////////////spin the encoder win a frequency!!////////////////////////////
//
//	 CIV.h
//
//   Process CIV messages sent and received
//

#ifndef _CIV_H_
#define _CIV_H_

#include <Arduino.h>
#include "ICOM_IC-905_CIV.h"
#include "RadioConfig.h"
#include <CIVcmds.h>                    // ICm CIV library https://github.com/WillyIoBrok/CIVmasterLib
#include <CIVmaster.h>                  // CIVcmds.h is automatically included in addition
#include <ICradio.h>                    // this would include CIVcmds.h and CIVmaster.h, if not included before !

void getradioInfo(void);
void check_CIV(void);
void FrequencyRequest(void);
void RcvCIVmsg(void);
void SendCIVmsg(void);
void civ_setup(void);
void pass_CAT_msgs_to_RADIO(void);
//void pass_CAT_msg_to_PC(void);

#ifdef GPS
  void pass_GPS(void);
#endif

const String retValStr[7] = {
  "CIV_OK",
  "CIV_OK_DAV",
  "CIV_NOK",
  "CIV_HW_FAULT",
  "CIV_BUS_BUSY",
  "CIV_BUS_CONFLICT",
  "CIV_NO_MSG"
};

#endif //_CIV_H_