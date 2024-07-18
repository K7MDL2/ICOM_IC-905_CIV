//
//  Vfo.cpp
//
//
#include "ICOM_IC-905_CIV.h"
#include "RadioConfig.h"
#include "Vfo.h"
#include <CIVmaster.h>

extern uint64_t VFOA;  // 0 value should never be used more than 1st boot before EEPROM since init should read last used from table.
extern int64_t Fc;     // Fc, Filter offsets, XIT and RIT offsets should all be taken into account for the value of Freq
extern  CIV     civ;
extern  CIVresult_t writeMsg (const uint8_t deviceAddr, const uint8_t cmd_body[], const uint8_t cmd_data[],writeMode_t mode);
CIVresult_t CIVresultL_vfo;
void formatFreq(uint64_t vfo);
uint8_t vfo_dec[7] = {};  // hold 6 or 7 bytes (length + 5 or 6 for frequency, bcd encoded bytes)
const String retValStr[7] = {
    "CIV_OK",
    "CIV_OK_DAV",
    "CIV_NOK",
    "CIV_HW_FAULT",
    "CIV_BUS_BUSY",
    "CIV_BUS_CONFLICT",
    "CIV_NO_MSG"
};

//////////////////////////Initialize VFO/DDS//////////////////////////////////////////////////////
COLD void initVfo(void)
{
    // placeholder
}

COLD void SetFreq(uint64_t Freq)
{ 
    formatFreq(Freq);  // Convert to BCD string
    //PC_Debug_port.printf("VFO: hex in SetFreq: %02X %02X %02X %02X %02X %02X %02X\n", vfo_dec[0], vfo_dec[1], vfo_dec[2], vfo_dec[3], vfo_dec[4], vfo_dec[5], vfo_dec[6]);
    CIVresultL_vfo = civ.writeMsg(CIV_ADDR_905, CIV_C_F1_SEND, vfo_dec, CIV_wFast);
    //PC_Debug_port.print("VFO: retVal of writeMsg: "); PC_Debug_port.println(retValStr[CIVresultL_vfo.retVal]);
}

extern unsigned int hexToDec(String hexString);

// Length is 5 or 6 depending if < 10GHz band  folowded by 5 or 6 BCD encoded frequency bytes
// vfo_dec[] holds the frequency result to send out
void formatFreq(uint64_t vfo)
{   
    if (vfo < 10000000000LL)
    {
      vfo_dec[0] = (uint8_t) 0x05;
      vfo_dec[6] = (uint8_t) 0x00;  // set to 0, unused < 10Ghz
      for (uint8_t i = 0; i < 5; ++i)
      {
        uint64_t x = vfo % 100;
        vfo_dec[1 + i] = bcdByteEncode(static_cast<uint8_t>(x));
        vfo = vfo / 100;
      }
      PC_Debug_port.printf(" VFO: < 10G Bands = Reversed hex to DEC byte %02X %02X %02X %02X %02X %02X\n", vfo_dec[0], vfo_dec[1], vfo_dec[2], vfo_dec[3], vfo_dec[4], vfo_dec[5]);
    }
    else
    {
      vfo_dec[0] = (uint8_t) 0x06;
      for (uint8_t i = 0; i < 6; ++i)
      {
        uint64_t x = vfo % 100;
        vfo_dec[1 + i] = bcdByteEncode(static_cast<uint8_t>(x));
        vfo = vfo / 100;
      }
      PC_Debug_port.printf(" VFO: > 10G Bands = Reversed hex to DEC byte %02X %02X %02X %02X %02X %02X %02X\n", vfo_dec[0], vfo_dec[1], vfo_dec[2], vfo_dec[3], vfo_dec[4], vfo_dec[5], vfo_dec[6]);
    }
}
