//
//  Vfo.cpp
//

// ToDO:  Fix up Quadrature Ouput config (VFO_MULT==1) to work with OCXO_10MHz preoperly).  Currently code is either/or.

//
#include "ICOM_IC-905_CIV.h"
#include "Vfo.h"
#include "RadioConfig.h"
#include <CIVmaster.h>

extern uint64_t VFOA;  // 0 value should never be used more than 1st boot before EEPROM since init should read last used from table.
extern int64_t Fc;

#if (VFO_MULT == 1)
static int currentEvenDivisor = 0;
int calcDivisor(uint32_t freq)
{
  int evenDivisor = 2;
  if (freq < 6850000) evenDivisor = 126;
  else if (freq < 9500000) evenDivisor = 88;
  else if (freq < 13600000)evenDivisor = 64;
  else if (freq < 17500000) evenDivisor = 44;
  else if (freq < 25000000) evenDivisor = 34;
  else if  (freq < 36000000) evenDivisor = 24;
  else if (freq < 45000000) evenDivisor = 18;
  else if (freq < 60000000) evenDivisor = 14;
  else if (freq < 80000000) evenDivisor = 10;
  else if (freq < 100000000) evenDivisor = 8;
  else if (freq < 146600000) evenDivisor = 6;
  else if (freq < 220000000) evenDivisor = 4;
  return evenDivisor;
}
#endif // VFO_MULT == 1

//////////////////////////Initialize VFO/DDS//////////////////////////////////////////////////////
COLD void initVfo(void)
{
    // placeholder
}

// Fc, Filter offsets, XIT and RIT offsets should all be taken into account for the value of Freq
extern  CIV     civ;
CIVresult_t CIVresultL_vfo;
extern  CIVresult_t writeMsg (const uint8_t deviceAddr, const uint8_t cmd_body[], const uint8_t cmd_data[],writeMode_t mode);
extern void SendCIVmsg(void);
void formatFreq(uint64_t vfo);
uint32_t MHz;
uint16_t Hz;
uint16_t KHz;
uint8_t vfo_dec[7] = {};

COLD void SetFreq(uint64_t Freq)
{ 
    const String retValStr[7] = {
        "CIV_OK",
        "CIV_OK_DAV",
        "CIV_NOK",
        "CIV_HW_FAULT",
        "CIV_BUS_BUSY",
        "CIV_BUS_CONFLICT",
        "CIV_NO_MSG"
    };

    formatFreq(Freq);  // Convert
    //PC_Debug_port.printf("VFO: hex in SetFreq: %02X %02X %02X %02X %02X %02X %02X\n", vfo_dec[0], vfo_dec[1], vfo_dec[2], vfo_dec[3], vfo_dec[4], vfo_dec[5], vfo_dec[6]);
    CIVresultL_vfo = civ.writeMsg(CIV_ADDR_905, CIV_C_F1_SEND, vfo_dec, CIV_wFast);
    //PC_Debug_port.print("VFO: retVal of writeMsg: "); PC_Debug_port.println(retValStr[CIVresultL_vfo.retVal]);
}

extern unsigned int hexToDec(String hexString);

void formatFreq(uint64_t vfo)
{
    char vfo_str[15]  = {};
    char vfo_str1[15] = {};
    char f100GHz[3] = {};  // for 10Ghz + bands on IC905
    char f1GHz[3] = {};
    char f10MHz[3] = {};
    char f100KHz[3] = {};
    char f1KHz[3] = {} ;
    char f10Hz[3] = {};

    //PC_Debug_port.printf("VFO: raw vfo uint64 value: %llu\n", vfo);

    lltoa(vfo,vfo_str,DEC);

    if (vfo < 10000000000LL)
    {
        sprintf(vfo_str1, "%010s", vfo_str);
        //PC_Debug_port.printf("VFO: after lltoa conversion: %s\n",vfo_str1);
        memcpy(f1GHz,   &vfo_str1[0], 2);
        memcpy(f10MHz,  &vfo_str1[2], 2);
        memcpy(f100KHz, &vfo_str1[4], 2);
        memcpy(f1KHz,   &vfo_str1[6], 2);
        memcpy(f10Hz,   &vfo_str1[8], 2);
        //PC_Debug_port.printf("VFO: memcpy = 05 %02s %02s %02s %02s %02s\n",f10Hz, f1KHz, f100KHz, f10MHz, f1GHz);
        PC_Debug_port.print("VFO: < 10GHz band - 05");  PC_Debug_port.print(f10Hz);  PC_Debug_port.print(f1KHz);  PC_Debug_port.print(f100KHz);  PC_Debug_port.print(f10MHz);  PC_Debug_port.print(f1GHz); PC_Debug_port.println();
        
        vfo_dec[0] = (uint8_t) 0x05;
        vfo_dec[1] = (uint8_t) hexToDec(f10Hz);
        vfo_dec[2] = (uint8_t) hexToDec(f1KHz);
        vfo_dec[3] = (uint8_t) hexToDec(f100KHz);
        vfo_dec[4] = (uint8_t) hexToDec(f10MHz);
        vfo_dec[5] = (uint8_t) hexToDec(f1GHz);
        //PC_Debug_port.printf("VFO: < 10Ghz Bands - Reversed hex to DEC byte %02X %02X %02X %02X %02X %02X\n", vfo_dec[0], vfo_dec[1], vfo_dec[2], vfo_dec[3], vfo_dec[4], vfo_dec[5]);
    }
    else  // Must be 10Gh or higher band
    {
        sprintf(vfo_str1, "%012s", vfo_str);
        //PC_Debug_port.printf("VFO: after lltoa conversion: %s\n",vfo_str1);
        memcpy(f100GHz, &vfo_str1[0], 2);
        memcpy(f1GHz,   &vfo_str1[2], 2);
        memcpy(f10MHz,  &vfo_str1[4], 2);
        memcpy(f100KHz, &vfo_str1[6], 2);
        memcpy(f1KHz,   &vfo_str1[8], 2);
        memcpy(f10Hz,   &vfo_str1[10], 2);
        //PC_Debug_port.printf("VFO: 10GHz+ memcpy = 06 %02s %02s %02s %02s %02s %02s\n",f10Hz, f1KHz, f100KHz, f10MHz, f1GHz, f100GHz);
        PC_Debug_port.print("VFO: > 10GHz band - 06");  PC_Debug_port.print(f10Hz);  PC_Debug_port.print(f1KHz);  PC_Debug_port.print(f100KHz);  PC_Debug_port.print(f10MHz);  PC_Debug_port.print(f1GHz); PC_Debug_port.print(f100GHz); PC_Debug_port.println();
        
        //vfo_dec[6] = 0U;
        vfo_dec[0] = (uint8_t) 0x06;
        vfo_dec[1] = (uint8_t) hexToDec(f10Hz);
        vfo_dec[2] = (uint8_t) hexToDec(f1KHz);
        vfo_dec[3] = (uint8_t) hexToDec(f100KHz);
        vfo_dec[4] = (uint8_t) hexToDec(f10MHz);
        vfo_dec[5] = (uint8_t) hexToDec(f1GHz);
        vfo_dec[6] = (uint8_t) hexToDec(f100GHz);
        //PC_Debug_port.printf(" VFO: > 10G Bands = Reversed hex to DEC byte %02X %02X %02X %02X %02X %02X %02X\n", vfo_dec[0], vfo_dec[1], vfo_dec[2], vfo_dec[3], vfo_dec[4], vfo_dec[5], vfo_dec[6]);
    }
}
