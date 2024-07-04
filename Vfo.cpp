//
//  Vfo.cpp
//

// ToDO:  Fix up Quadrature Ouput config (VFO_MULT==1) to work with OCXO_10MHz preoperly).  Currently code is either/or.

//
#include "ICOM_IC-905_CIV.h"
#include "Vfo.h"
#include "RadioConfig.h"

#ifndef USE_RS_HFIQ
    #if defined(OCXO_10MHZ) || (VFO_MULT == 1)
        //#include "si5351.h"
        //extern Si5351 si5351;
    #else
        //#include "si5351mcu.h" 
        //extern Si5351mcu si5351;
    #endif
#endif

extern uint64_t VFOA;  // 0 value should never be used more than 1st boot before EEPROM since init should read last used from table.
extern int64_t Fc;

//#define VFO_MULT    2   // 4x for QRP-Labs RX, 2x for NT7V QSE/QSD board, defined in RadioCfg.h
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
COLD void SetFreq(uint64_t Freq)
{ 
    // placeholder
}
