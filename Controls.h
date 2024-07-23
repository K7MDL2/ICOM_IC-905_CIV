#ifndef _CONTROLS_H_
#define _CONTROLS_H_
//
//    Controls.h
//
//    Core functions that control things.  The controller may be a touch event from a button or label object
//      or from a mechanical device or another function in a chain of interdepedencies.
//
//    Many controls have multipe possible controllers so have to adopt an Controller->control->display model
//      allowing parallel control requests.  
//      The control object changes (or requests to change) states, the display only scans and reports the state.
//      It becomes importan to pass through this as remote control and monitoring get built.
#include <Arduino.h>

void Set_Spectrum_Scale(int8_t zoom_dir);
void Set_Spectrum_RefLvl(int8_t zoom_dir);
void changeBands(int8_t direction);
void Mute();
void Menu();
void Display();
void Band(uint8_t new_band);
void BandDn();
void BandUp();
void Notch();
void Spot();
void Enet();
void setNR();
void setNB(int8_t toggle);
void Xmit(uint8_t state);
void Ant();
void Fine();
void Rate(int8_t dir);
void setMode(int8_t dir);
void AGC(int8_t dir);
void Filter(int8_t dir);
void ATU(uint8_t state);
void Split(uint8_t state);
void setXIT(int8_t toggle);
void XIT(int8_t delta);
void setRIT(int8_t toggle);
void RIT(int8_t delta);
void Preamp(int8_t toggle);
void setAttn(int8_t toggle);
void VFO_AB();
void Attn(int8_t delta);
void setAFgain(int8_t toggle);
void AFgain(int8_t delta);
void setRFgain(int8_t toggle);
void RFgain(int8_t delta);
void setRefLevel(int8_t toggle);
void NBLevel(int8_t delta);
void RefLevel(int8_t newval);
void TouchTune(int16_t touch_Freq);
void selectStep(uint8_t fndx);
void selectAgc(uint8_t andx);
void Zoom(int8_t dir);
void setZoom(int8_t toggle);
void setPAN(int8_t toggle);
void PAN(int8_t delta);
void digital_step_attenuator_PE4305(int16_t _attn);   // Takes a 0 to 100 input, converts to the appropriate hardware steps such as 0-31dB in 1 dB steps
uint64_t find_new_band(uint64_t new_frequency, uint8_t &_curr_band);
void clearMeter(void);
void send_Mode_to_Radio(uint8_t mndx);
void set_Mode_from_Radio(uint8_t mndx);
uint8_t get_Mode_from_Radio(void);
uint8_t read_BSTACK_from_Radio(uint8_t band, uint8_t reg);   // Ask the radio for band and register contents 
uint8_t get_MY_POSITION_from_Radio(void);
uint8_t get_RXTX_from_Radio(void);
uint8_t get_Preamp_from_Radio(void);
uint8_t get_Attn_from_Radio(void);
uint8_t get_AGC_from_Radio(void);
uint8_t send_AGC_to_Radio(void);
uint64_t get_Freq_from_Radio(void);
void Band_Decode_Output(uint8_t band);
void GPIO_Out(uint8_t pattern);
void Decoder_GPIO_Pin_Setup(void);

#endif  // _CONTROLS_H_
