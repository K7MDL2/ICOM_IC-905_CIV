#ifndef _VFO_H_
#define _VFO_H_

// VFO.h
#include <Arduino.h>

void initVfo(void);
void SetFreq(uint64_t Freq);

// used by getFreq()
// used by getFreq()
//inline uint8_t bcdByte(const uint8_t x) const { return  (((x & 0xf0) >> 4) * 10) + (x & 0x0f); }
inline uint8_t bcdByte(const uint8_t x) { return  (((x & 0xf0) >> 4) * 10) + (x & 0x0f); }
// used by setFreq()
// input is between 0 and 99 decimal.  output 0 to 0x99
//inline uint8_t bcdByteEncode(const uint8_t x) const { return ((x / 10) << 4) + (x % 10); }
inline uint8_t bcdByteEncode(const uint8_t x) { return ((x / 10) << 4) + (x % 10); }

#endif //_VFO_H_