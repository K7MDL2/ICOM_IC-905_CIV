//***************************************************************************************************
//
//    ICOM_IC-905_CIV.INO
//
//    USB Band Decoder for IC-905
//    Original June 23, 2023 by K7MDL
//    Updated July 2024 to display on a touchscreen radio frequency.
//      Eventuallly control the radio over CIV and to extract GPS time and location data for display.
// 
//  Connect IC-905 to Teensy Host port.
//  Connect PC to Teensy USB
//  Connect Teensy IO pins to buffers and devices for band specific control
//  3 USB Serial ports used.  1 = Debug, 2= CAT, 3 = GPS data
//
//  Includes modified code from RemoteTH.com - see below
//
//  Modified 4/2021 for SDR/Panadapter usage by K7MDL
//  Modified heavily 6/2023 to specialize as a IC-905 USB band decoder
//  Passes through 2 serial interfaces and taps into the CI-V frequency value to operate band decode outputs on GPIO pins.
//  IC-905 connects to Teensy host port which today cannot handle audio, so only teh serial ports are dealt with. 
//  Can use the IC-905 LAN connection to a PC for full audio anbd serial control, LAN operates in parallel with the USB OK.
//  Could generate PTT (aka SEND) from the CI-V messages and combine in software for per-band PTT outputs.  
//  I have seen some say they identified and broke out the 'SEND" or PTT signal on the RF unit cable but still band specific filtering.
//  Ideally the RF Unit would have band decode IO signals available using some of its spare accessory connector pins.  These pins are "Do Not Connect" 
//     suggesting they may be connected to something inside for future use.
//
//***************************************************************************************************

#include <Arduino.h>                    // from Arduino
#include <avr/pgmspace.h>               // from Arduino
//#include "USBHost_t36.h"
#include <Metro.h>
#include <SD.h>
#include <SerialFlash.h>
#include <SPI.h>                        // from Arduino
#include <Wire.h>                       // from Arduino
#include <TimeLib.h>                    // from Arduino
#include <CIVcmds.h>                    // ICm CIV library https://github.com/WillyIoBrok/CIVmasterLib
#include <CIVmaster.h> // CIVcmds.h is automatically included in addition
#include <ICradio.h>    // this would include CIVcmds.h and CIVmaster.h, if not included before !

// External libraries - These are not all of them as some appear with #ifdef blocks based on feature selection - See Github Wiki "Libraries" page for a full listing.
#define  ENCODER_OPTIMIZE_INTERRUPTS    // leave this one here.  Not normally user changed
#include "Encoder.h"                    // Internal Teensy library and at C:\Program Files (x86)\Arduino\hardware\teensy\avr\libraries
#include "Metro.h"                      // GitHub https://github.com/nusolar/Metro
#include "InternalTemperature.h"        // V2.1.0 @ Github https://github.com/LAtimes2/InternalTemperature

#define BYPASS_SPECTRUM_MODULE   // debugging temp 

// Below are local project files
#include "RadioConfig.h"        // Our main configuration file
#include "ICOM_IC-905_CIV.h"
#include "SDR_Data.h"
#include "SDR_I2C_Encoder.h"    // See RadioConfig.h for more config including assigning an INT pin.                                          
                                // Hardware verson 2.1, Arduino library version 1.40.      

int   counter = 0;

#ifdef I2C_ENCODERS // This turns on support for DuPPa.net I2C encoder with RGB LED integrated. 
    //  This is a basic example for using the I2C Encoder V2
    //  The counter is set to work between +10 to -10, at every encoder click the counter value is printed on the terminal.
    //  It's also printed when the push button is released.
    //  When the encoder is turned the led turn green
    //  When the encoder reach the max or min the led turn red
    //  When the encoder is pushed, the led turn blue

    //  Connections with Teensy 4.1:
    //  - -> GND
    //  + -> 3.3V
    //  SDA -> 18
    //  SCL -> 19
    //  INT -> 29 - Dependent on particular board pin assignments
    
    //Class initialization with the I2C addresses
    #ifdef I2C_ENC1_ADDR
        extern i2cEncoderLibV2 I2C_ENC1;  // Address 0x61 only - Jumpers A0, A5 and A6 are soldered.//
        extern uint8_t _e1;
    #endif
    #ifdef I2C_ENC2_ADDR
        extern i2cEncoderLibV2 I2C_ENC2;  // Address 0x62 only - Jumpers A1, A5 and A6 are soldered.//  
        extern uint8_t _e2;
    #endif
    #ifdef I2C_ENC3_ADDR
        extern i2cEncoderLibV2 I2C_ENC3;  // Address 0x63 only - Jumpers A1, A5 and A6 are soldered.// 
        extern uint8_t _e3;
    #endif
    #ifdef I2C_ENC4_ADDR
        extern i2cEncoderLibV2 I2C_ENC4;  // Address 0x64 only - Jumpers A0, A1, A5 and A6 are soldered.
        extern uint8_t _e4;
    #endif
    #ifdef I2C_ENC5_ADDR
        extern i2cEncoderLibV2 I2C_ENC5;  // Address 0x65 only - Jumpers A0, A1, A5 and A6 are soldered.
        extern uint8_t _e5;
    #endif
    #ifdef I2C_ENC6_ADDR
        extern i2cEncoderLibV2 I2C_ENC6;  // Address 0x66 only - Jumpers A0, A1, A5 and A6 are soldered.
        extern uint8_t _e6;
    #endif  
#endif // I2C_ENCODER
 
#ifdef GPIO_ENCODERS   // if you have both i2c and mechanical encoders, assignment get tricky.  Default is only i2c OR mechanical
    #include "SDR_I2C_Encoder.h"              // See RadioConfig.h for more config including assigning an INT pin.                                          
    // Hardware verson 2.1, Arduino library version 1.40.
    // On the Teensy motherboards, ENC1 is the VFO.  This is GPIO_ENC2 and GPIO_ENC3 jacks.
    // Using dummy i2c objects to inteface the gpio enmcoders through to support the same timer (tap/press), and role/switch features
    #if (GPIO_ENC2_ENABLE > 0)
        extern i2cEncoderLibV2 GPIO_ENC2;
        Encoder GPIO_Encoder2(GPIO_ENC2_PIN_A, GPIO_ENC2_PIN_B);
    #endif
    #if (GPIO_ENC3_ENABLE > 0)  
        extern i2cEncoderLibV2 GPIO_ENC3;
        Encoder GPIO_Encoder3(GPIO_ENC3_PIN_A, GPIO_ENC3_PIN_B);
    #endif
#endif   //GPIO_ENCODERS


uint8_t enc_ppr_response = VFO_PPR; // for VFO A/B Tuning encoder. This scales the PPR to account for high vs low PPR encoders.
                                    // 600ppr is very fast at 1Hz steps, worse at 10Khz!

// Set this to be the default MF knob function when it does not have settings focus from a button touch.
// Choose any from the MF Knob aware list below.
uint8_t MF_client         = MFTUNE; // Flag for current owner of MF knob services
bool MF_default_is_active = true;

COLD time_t getTeensy3Time();
COLD void I2C_Scanner(void);
HOT  void Check_Encoders(void);
HOT  void Check_GPIO_Switches(void);
COLD void init_band_map(void);
HOT  bool GPIO_Sw_read(bool sw_pushed, uint8_t sw_pin, uint8_t slot);
//COLD void Change_FFT_Size(uint16_t new_size, float new_sample_rate_Hz);
//COLD void digitalClockDisplay(void); 
COLD void MF_Service(int8_t counts, int8_t knob);
COLD void Change_FFT_Size(uint16_t new_size, float new_sample_rate_Hz);

#ifndef PANADAPTER
    #ifdef USE_ENET_PROFILE
        uint8_t     user_Profile = 0;   // Profile 0 has enet enabled, 1 and 2 do not.
    #else  // USE_ENET_PROFILE
        uint8_t     user_Profile = 1;   // Profile 0 has enet enabled, 1 and 2 do not.
    #endif  // USE_ENET_PROFILE
#else  // PANADAPTER
    uint8_t     user_Profile = 2;   // Profile 2 is optimized for Panadapter usage
#endif  // PANADAPTER

// Choose your actual pin assignments for any you may have.
Encoder VFO(GPIO_VFO_PIN_A, GPIO_VFO_PIN_B); // pins defined in RadioConfig.h - mapped to ENC1 on the PCB

#ifdef I2C_LCD
    #include <LiquidCrystal_I2C.h>
    LiquidCrystal_I2C lcd(LCD_ADR,LCD_COL, LCD_LINES);  // set the LCD address to 0x27 for a 16 chars and 2 line display
#endif

#ifdef USE_RA8875
    RA8875 tft    = RA8875(RA8875_CS,RA8875_RESET); //initialize the display object
#else
    RA8876_t3 tft = RA8876_t3(RA8876_CS,RA8876_RESET); //initiate the display object
    FT5206 cts    = FT5206(CTP_INT);    // Be sure to set the motherboard version used to get the correct Touch INT!                                    // Set in 2 places, the spectrum_RA887x library and in RadioConfig.h
#endif

#ifdef ENET
    extern uint8_t enet_ready;
    extern unsigned long enet_start_fail_time;
    extern uint8_t rx_count;
#endif

// #include <Metro.h>
//  Most of our timers are here.  Spectrum waterfall is in the spectrum settings section of that file
Metro touch                = Metro(50);     // used to check for touch events
Metro tuner                = Metro(700);    // used to dump unused encoder counts for high PPR encoders when counts is < enc_ppr_response for X time.
Metro meter                = Metro(400);    // used to update the meters
Metro popup_timer          = Metro(500);    // used to check for popup screen request
Metro NTP_updateTx         = Metro(10000);  // NTP Request Time interval
Metro NTP_updateRx         = Metro(65000);  // Initial NTP timer reply timeout. Program will shorten this after each request.
Metro MF_Timeout           = Metro(1800);   // MultiFunction Knob and Switch
Metro touchBeep_timer      = Metro(80);     // Feedback beep for button touches
Metro gpio_ENC2_Read_timer = Metro(700);    // time allowed to accumulate counts for slow moving detented encoders
Metro gpio_ENC3_Read_timer = Metro(700);    // time allowed to accumulate counts for slow moving detented encoders
Metro TX_Timeout           = Metro(180000); // 180000 is 3 minutes for RunawayTX timeout
Metro CAT_Serial_Check     = Metro(20);     // Throttle the servicing for CAT comms
Metro CAT_Poll             = Metro(1000);  // Throttle the servicing for CAT comms
Metro CAT_Log_Clear        = Metro(3000);   // Clear the CIV log buffer
uint64_t    xvtr_offset             = 0;
int16_t     rit_offset              = 0;    // global RIT offset value in Hz. -9999Hz to +9999H
int16_t     xit_offset              = 0;    // global XIT offset value in Hz. -9999Hz to +9999H
int16_t     rit_offset_last         = 0;    // track last used value when turning the RIT on and off. 
int16_t     xit_offset_last         = 0;    // track last used value when turning the RIT on and off. 
uint8_t     clipping                = 0;    // track state of clipping (primarily RS-HFIQ but could be applied to any RF hardware that has such indications)
int16_t     fft_size            = FFT_SIZE;       // This value will be passed to the init function.
                                                  // Ensure the matching FFT resources are enabled in the lib .h file!                            
int16_t     fft_bins            = (int16_t) fft_size;       // Number of FFT bins which is FFT_SIZE/2 for real version or FFT_SIZE for iq version
float       sample_rate_Hz      = 44100.0f;  //43Hz /bin  12.5K spectrum
float       fft_bin_size        = sample_rate_Hz/(fft_size*2);   // Size of FFT bin in HZ.  From sample_rate_Hz/FFT_SIZE for iq

//
//----------------------------------------------------------------------------------------------------------------------------
//
// These should be saved in EEPROM periodically along with several other parameters
uint8_t     curr_band   = BAND80M;  // global tracks our current band setting.  
uint64_t    VFOA        = 0;        // 0 value should never be used more than 1st boot before EEPROM since init should read last used from table.
uint64_t    VFOB        = 0;
int64_t     Fc          = 0;        //(sample_rate_Hz/4);  // Center Frequency - Offset from DC to see band up and down from cener of BPF.   Adjust Displayed RX freq and Tx carrier accordingly
int32_t     ModeOffset  = 0;        // Holds offset based on CW mode pitch
uint8_t     default_MF_slot         = 0;    // default MF client assignment slot 
float       S_Meter_Peak_Avg;              // For RF AGC Limiter
bool        MeterInUse;  // S-meter flag to block updates while the MF knob has control
int32_t     Freq_Peak   = 0;
uint8_t     display_state;   // something to hold the button state for the display pop-up window later.
bool        touchBeep_flag          = false;
uint8_t     popup       = 0;   // experimental flag for pop up windows

int BAND = 0;
int previousBAND = -1;
uint64_t freq = 0;
bool PTT = false;
uint32_t PttTiming[2]={0, 10};            // debouncing time and also minimal PTT on time in ms
float DCinVoltage;
#if defined(DISABLE_DIVIDER)
  float ResistorCoeficient = 1.0;
#else
  float ResistorCoeficient = 6.0;
#endif

//-------------------------------------------------------------------------------
// create the civ object

    CIV     civ;  // create the CIV-Interface object
    ICradio ICxxxx(TypeIC905,CIV_ADDR_905);
//-------------------------------------------------------------------------------

uint8_t lpCnt = 0;
CIVresult_t CIVresultL;

const String retValStr[7] = {
  "CIV_OK",
  "CIV_OK_DAV",
  "CIV_NOK",
  "CIV_HW_FAULT",
  "CIV_BUS_BUSY",
  "CIV_BUS_CONFLICT",
  "CIV_NO_MSG"
};

radioOnOff_t  	radioS      = RADIO_NDEF;
radioOnOff_t  	localS      = RADIO_NDEF;
uint64_t 	    radioF      = 0;
uint64_t 	    localF      = 0;
uint8_t       	radioM      = MOD_NDEF;
uint8_t       	localM      = MOD_NDEF;
uint8_t       	radioFil    = FIL_NDEF;
uint8_t       	localFil    = FIL_NDEF;

uint32_t VoltageRefresh[2] = {0, 3000};   // refresh in ms
float ArefVoltage = 4.303;            // Measure on Aref pin 20 for calibrate
float Divider = 1;

/*
// Modified 4/2021 for SDR/Panadapter usage by K7MDL
// Modified heavily 6/2023 to specialize as a IC-905 USB band decoder
// Passes through 2 serial interfaces and tap into the CI-V frequency value to operate band decode outputs
// The audio interface of the 905 USB cannot be passed thoriugh due to limitation of the Teensy USB Host library today.  
// Can use LAN as a workaround in parallel.

  Band decoder MK2 with TRX control output for Arduino
-----------------------------------------------------------
  https://remoteqth.com/wiki/index.php?page=Band+decoder+MK2

  ___               _        ___ _____ _  _
 | _ \___ _ __  ___| |_ ___ / _ \_   _| || |  __ ___ _ __
 |   / -_) '  \/ _ \  _/ -_) (_) || | | __ |_/ _/ _ \ '  \
 |_|_\___|_|_|_\___/\__\___|\__\_\|_| |_||_(_)__\___/_|_|_|


    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 2 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

Features:
Support inputs
  * PTT detector - if PTT on, outputs not change
  * SERIAL terminal (ASCII)
  * ICOM CI-V
  * KENWOOD - CAT
  * FLEX 6000 CAT series
  * YAESU / ELECRAFT BCD
  * ICOM ACC voltage
  * YAESU CAT - TRX since 2008 ascii format
  * YAESU CAT old binary format (tested on FT-817)
  * IP relay with automatic pair by rotary encoder ID https://remoteqth.com/wiki/index.php?page=IP+Switch+with+ESP32-GATEWAY

Outputs
  * 14 local relay
  * 14 remote relay
  * Yaesu BCD
  * Serial echo
  * Icom CIV
  * Kenwood CAT
  * YAESU CAT - TRX since 2008 ascii format
  * IP relay with automatic pair by rotary encoder ID
  * PTT by band - distributed PTT to output, dependency by frequency - TNX Tim W4YN
  * Analog (PWM) output by  preset table

  Major changes
  -------------
  - LCD support
  - Icom with request mode
  - PTT input block changes during transmit
  - own board with all smd parts including arduino nano module
  - without relays, only control driver outputs
  - optional ethernet module

  Changelog
  ---------
  2021-02 add 23cm (IC9700) Icom state machine
  2020-10 PWM (analog) output
  2020-07 PTT by band
  2020-02 fix out set
  2019-11 YAESU / ELECRAFT input BCD format
  2018-03 manual switch between four output on same band by BCD input - TNX ZS1LS behind inspiration
  2018-12 PTT bug fix
          LCD PCF8574 support
          copy YAESU CAT from old code
          added YAESU FT-100 support
  2018-11 support FLEX 6000 CAT series
  2018-06 support IP relay
  2018-05 mk2 initial release

*/
//=====[ Inputs ]=============================================================================================

// #define INPUT_BCD            // TTL BCD in A
//int BcdInputFormat = 0;         // if enable INPUT_BCD, set format 0 - YAESU, 1 ELECRAFT
// #define ICOM_ACC             // voltage 0-8V on pin4 ACC(2) connector - need calibrate table
// #define INPUT_SERIAL         // telnet ascii input - cvs format [band],[freq]\n
#define ICOM_CIV             // read frequency from CIV
// #define MULTI_OUTPUT_BY_BCD  // manual switch between four output on same band by BCD input
                                // - INPUT_BCD input must be disable
                                // - BCD output will be disble
                                // - it must always be select (grounded) one of BCD input
//=====[ Outputs ]============================================================================================
//   If enable:
// - baudrate is same as selected Inputs
// - Inputs work only in 'sniff mode'
// - for operation must disable REQUEST
//
#define ICOM_CIV_OUT       // send frequency to CIV ** you must set TRX CIV_ADRESS **
// #define SERIAL_echo        // Feedback on serial line in same baudrate, CVS format <[band],[freq]>\n
// #define PTT_BY_BAND        // distributed PTT dependency to band (disable band decoder outputs) TNX Tim W4YN for idea
// #define PWM_OUT               // PWM on D5 with rc filter (10k/10u) represent analog output J2.12

//=====[ Hardware ]=============================================================================================

//=====[ Settings ]===========================================================================================
#define BANDSET
#define SERBAUD     115200     // [baud] Serial port in/out baudrate
#define WATCHDOG        20     // [sec] determines the time, after which the all relay OFF, if missed next input data - uncomment for the enabled
#define REQUEST        500    // [ms] use TXD output for sending frequency request
#define CIV_ADRESS    0xAC    // CIV input HEX Icom adress (0x is prefix)
#define CIV_ADR_OUT   0xAC    // CIV output HEX Icom adress (0x is prefix)
// #define DISABLE_DIVIDER    // for lowest voltage D-SUB pin 13 inputs up to 5V only - need open JP9
//#define DEBUG              // enable some debugging
//=====[ FREQUENCY RULES ]===========================================================================================

const uint64_t Freq2Band[18][2] = {/*
Freq Hz from       to   Band number
*/   {1810000,   2000000},  // #1 [160m]
     {3500000,   3800000},  // #2  [80m]
     {5298000,   5403000},  // #3  [60m]
     {7000000,   7200000},  // #4  [40m]
    {10100000,  10150000},  // #5  [30m]
    {14000000,  14350000},  // #6  [20m]
    {18068000,  18168000},  // #7  [17m]
    {21000000,  21450000},  // #8  [15m]
    {24890000,  24990000},  // #9  [12m]
    {28000000,  29700000},  // #10  [10m]
    {50000000,  52000000},  // #11  [6m]
    {70000000,  72000000},  // #12  [4m]
   {144000000, 148000000},  // #13  [2m]
   {430000000, 450000000},  // #14  [70cm]
   {1240000000UL, 1300000000UL},  // #15  [23cm]
   {2300000000UL, 2450000000UL},  // #16  [13cm]
   //{3300000000, 3500000000},  // #17  [9cm]
   {5650000000UL, 5999000000UL},  // #17  [6cm]
   {10000000000UL, 10200000000UL}  // #18  [3cm]
};

//=====[ Sets band -->  to output in MATRIX table ]===========================================================

        const byte matrix[17][40] = { /* band out

        If enable #define MULTI_OUTPUT_BY_BCD
        you can select outputs manually to ground one from BCD inputs
        to select between four output on same band
        represent by bit in this table
        0x01 = B00000001 = bit 1
        0x02 = B00000010 = bit 2
        0x04 = B00000100 = bit 3
        0x08 = B00001000 = bit 4
        For example record
        Band 1 -->  {      0x01,       0x02,       0x04,  0,       0x08,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0 },
        is the same as record
        Band 1 -->  { B00000001,  B00000010,  B00000100,  0,  B00001000,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0 },

        0x0F = this output enable for any BCD input (enable all bit)

        Band 0 --> */ { 0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0 }, /* first eight shift register board
\       Band 1 --> */ { 0x0F,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0 }, /*
 \      Band 2 --> */ { 0,  0x0F,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0 }, /*
  \     Band 3 --> */ { 0,  0,  0x0F,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0 }, /*
   \    Band 4 --> */ { 0,  0,  0,  0x0F,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0 }, /*
    \   Band 5 --> */ { 0,  0,  0,  0,  0x0F,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0 }, /*
     \  Band 6 --> */ { 0,  0,  0,  0,  0,  0x0F,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0 }, /*
IN    ) Band 7 --> */ { 0,  0,  0,  0,  0,  0,  0x0F,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0 }, /*
     /  Band 8 --> */ { 0,  0,  0,  0,  0,  0,  0,  0x0F,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0 }, /*

    /   Band 9 --> */ { 0,  0,  0,  0,  0,  0,  0,  0,    0x0F,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0 }, /* second eight shift register board
   /    Band 10 -> */ { 0,  0,  0,  0,  0,  0,  0,  0,    0,  0x0F,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0 }, /* (optional)
  /     Band 11 -> */ { 0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0x0F,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0 }, /*
 /      Band 12 -> */ { 0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0x0F,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0 }, /*
/       Band 13 -> */ { 0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0x0F,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0 }, /*
        Band 14 -> */ { 0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0x0F,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0 }, /*
        Band 15 -> */ { 0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0x0F,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0 }, /*
        Band 16 -> */ { 0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0x0F,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0 }, /*
                        |   |   |   |   |   |   |   |     |   |   |   |   |   |   |   |
                        V   V   V   V   V   V   V   V     V   V   V   V   V   V   V   V
                     ----------------------------------  ---------------------------------
                     |  1   2   3   4   5   6   7   8     9  10  11  12  13  14  15  16  |
                     ----------------------------------  ---------------------------------
                                                   OUTPUTS
                                    (for second eight need aditional board)*/
        };
        const int NumberOfBoards = 1;    // number of eight byte shift register 0-x

//=====[ BCD OUT ]===========================================================================================

        const boolean BCDmatrixOUT[4][16] = { /*
        --------------------------------------------------------------------
        Band # to output relay   0   1   2   3   4   5   6   7   8   9  10
        (Yaesu BCD)                 160 80  40  30  20  17  15  12  10  6m
        --------------------------------------------------------------------
                                 |   |   |   |   |   |   |   |   |   |   |
                                 V   V   V   V   V   V   V   V   V   V   V
                            */ { 0,  1,  0,  1,  0,  1,  0,  1,  0,  1,  0, 1, 0, 1, 0, 1 }, /* --> DB25 Pin 11
                            */ { 0,  0,  1,  1,  0,  0,  1,  1,  0,  0,  1, 1, 0, 0, 1, 1 }, /* --> DB25 Pin 24
                            */ { 0,  0,  0,  0,  1,  1,  1,  1,  0,  0,  0, 0, 1, 1, 1, 1 }, /* --> DB25 Pin 12
                            */ { 0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1, 1, 1, 1, 1, 1 }, /* --> DB25 Pin 25
        */};
//=====[ PWM OUT ]===========================================================================================
#if defined(PWM_OUT)
  const uint32_t PwmByBand[17] = {/*
  PWM 0-255
  */ 0,    // #0  OUT of band
     11,   // #1  [160m] 0,23V
     24,   // #2  [80m] 0,46V
     36,   // #3  [60m] 0,69V
     49,   // #4  [40m] 0,92V
     62,   // #5  [30m] 1,15V
     74,   // #6  [20m] 1,38V
     87,   // #7  [17m] 1,61V
     99,  // #8  [15m] 1,84V
     111,  // #9  [12m] 2,07V
     124,  // #10 [10m] 2,3V
     137,  // #11 [6m]  2,53V
     180,  // #12 [4m]
     195,  // #13 [2m]
     210,  // #14 [70cm]
     225,  // #15 [23cm]
     240,  // #16 [13cm]
  };
#endif
//============================================================================================================


//byte ShiftByte[NumberOfBoards];
byte ShiftByte[5];

// int SelectOut = 0;
// int x;
  uint32_t RequestTimeout[2]={0,
    #if defined(REQUEST)
      REQUEST
    #else
      0
    #endif
  };

int watchdog2 = 1000;     // REQUEST refresh time [ms]
int previous2;
int timeout2;

#if defined(WATCHDOG)
    int previous;
    int timeout;
    uint32_t WatchdogTimeout[2] = {0, WATCHDOG*1000};  // {-WATCHDOG*1000, WATCHDOG*1000};
#endif
#if defined(ICOM_ACC)
    float AccVoltage = 0;
    float prevAccVoltage=0;
    int band = 0;
    int counter = 0;
#endif

// ************************************************* Setup *****************************************
//
// *************************************************************************************************

tmElements_t tm;
time_t prevDisplay = 0; // When the digital clock was displayed
/* timer  variables */
unsigned long time_current_baseloop;       // temporary time of the baseloop entry for calculations
unsigned long time_last_baseloop;          // will be updated at the end of every baseloop run
#define BASELOOP_TICK 10 


void setup()
{
    DSERIALBEGIN(115200);
    delay(1000);
    if (CrashReport) Serial.print(CrashReport);

    DPRINTLNF("Initializing SDR_RA887x Program");
    DPRINTF("FFT Size is ");
    DPRINTLN(fft_size);
    DPRINTLNF("**** Running I2C Scanner ****");

    civ.setupp(true, false, "");      // initialize the civ object/module
                                      // and the ICradio objects

    civ.registerAddr(CIV_ADDR_905);    // tell civ, that this is a valid address to be used


    // ---------------- Setup our basic display and comms ---------------------------
    Wire.begin();
    Wire.setClock(100000UL); // Keep at 100K I2C bus transfer data rate for I2C Encoders to work right
    I2C_Scanner();
    #ifdef  I2C_ENCODERS  
        set_I2CEncoders();   // Serasch through encoder_list table and identify active i2c encoder roles and slot assignments.
    #endif // I2C_ENCODERS

    #ifdef GPIO_ENCODERS
        if (GPIO_ENC2_ENABLE) pinMode(GPIO_ENC2_PIN_SW, INPUT_PULLUP);   // Pullups for GPIO Enc2 and 3 switches
        if (GPIO_ENC3_ENABLE) pinMode(GPIO_ENC3_PIN_SW, INPUT_PULLUP);
    #endif
    if (GPIO_SW1_ENABLE)  pinMode(GPIO_SW1_PIN,  INPUT_PULLUP);
    if (GPIO_SW2_ENABLE)  pinMode(GPIO_SW2_PIN,  INPUT_PULLUP);
    if (GPIO_SW3_ENABLE)  pinMode(GPIO_SW3_PIN,  INPUT_PULLUP);
    if (GPIO_SW4_ENABLE)  pinMode(GPIO_SW4_PIN,  INPUT_PULLUP);
    if (GPIO_SW5_ENABLE)  pinMode(GPIO_SW5_PIN,  INPUT_PULLUP);
    if (GPIO_SW6_ENABLE)  pinMode(GPIO_SW6_PIN,  INPUT_PULLUP);   // By default conifg this is assigned 'disabled' to the GPIO header pin 8 can be an output for GPIO_ANT_PIN

    // Use for ANT switch
    if (GPIO_ANT_ENABLE) pinMode(GPIO_ANT_PIN, OUTPUT); // Took over SW6 default input pin to make this an output (by config)

    // Serach for a default_MF_client tag and save it in a global var
    for (default_MF_slot = 0; default_MF_slot < NUM_AUX_ENCODERS; default_MF_slot++)
    {
        if (encoder_list[default_MF_slot].default_MF_client && encoder_list[default_MF_slot].enabled) // set back to designated default control role
        {
            break; // find first control with a match
        }
        else
            default_MF_slot = 0;
    } // got the slot number of our control

#ifdef USE_RA8875
        DPRINTLN(F("Initializing RA8875 Display"));
        tft.begin(RA8875_800x480);
        tft.setRotation(SCREEN_ROTATION); // 0 is normal, 1 is 90, 2 is 180, 3 is 270 degrees
        delay(20);
        #ifdef USE_FT5206_TOUCH
            tft.useCapINT(RA8875_INT);
            tft.setTouchLimit(MAXTOUCHLIMIT);
            tft.enableCapISR(true);
            tft.setTextColor(RA8875_WHITE, RA8875_BLACK);
        #else
            #ifdef USE_RA8875
                //tft.print("you should open RA8875UserSettings.h file and uncomment USE_FT5206_TOUCH!");
            #endif  // USE_RA8875
        #endif // USE_FT5206_TOUCH
    #else 
        DPRINTLN(F("Initializing RA8876 Display"));   
        tft.begin(50000000UL);  // 10 is very slow, 30, much better, 40-50 seem to be were perf gain flattens off.  
        // Works up to 70Mhz but so little perf gain above 40Mhz thaqt 50Mhz seems a solid compromise.
        cts.begin();
        cts.setTouchLimit(MAXTOUCHLIMIT);
        tft.touchEnable(false);   // Ensure the resitive controller, if any is off
        tft.displayImageStartAddress(PAGE1_START_ADDR); 
        tft.displayImageWidth(SCREEN_WIDTH);
        tft.displayWindowStartXY(0,0);
        // specify the page 2 for the current canvas
        tft.canvasImageStartAddress(PAGE2_START_ADDR);
        // specify the page 1 for the current canvas
        tft.canvasImageStartAddress(PAGE1_START_ADDR);
        tft.canvasImageWidth(SCREEN_WIDTH);
        //tft.activeWindowXY(0,0);
        //tft.activeWindowWH(SCREEN_WIDTH,SCREEN_HEIGHT);

        //#ifndef BYPASS_SPECTRUM_MODULE        
            setActiveWindow_default();
        //#endif 

        tft.graphicMode(true);
        tft.clearActiveScreen();
        tft.selectScreen(0);  // Select screen page 0
        tft.fillScreen(BLACK);
        tft.setBackGroundColor(BLACK);
        tft.setTextColor(WHITE, BLACK);
        tft.backlight(true);
        tft.displayOn(true);
        tft.setRotation(SCREEN_ROTATION); // 0 is normal, 1 is 90, 2 is 180, 3 is 270 degrees.  
                        // RA8876 touch controller is upside down compared to the RA8875 so correcting for it there.
    #endif

    // Display Startup Banner
    tft.setFont(Arial_28_Bold);
    tft.setTextColor(BLUE);
    tft.setCursor(70, 100);
    tft.print(BANNER); // Customize the Startup Banner Text
    tft.setCursor(70, 200);
    tft.setFont(Arial_28_Bold);
    tft.setTextColor(WHITE);
    tft.print(CALLSIGN); // Put your callsign here
 
    // -------------------- Setup Ethernet and NTP Time and Clock button  --------------------------------
    #ifdef ENET
    if (user_settings[user_Profile].enet_enabled)
    {
        struct Standard_Button *t_ptr = &std_btn[UTCTIME_BTN];

        tft.fillRect(t_ptr->bx, t_ptr->by, t_ptr->bw, t_ptr->bh, RA8875_BLACK);
        tft.setFont(Arial_14);
        tft.setTextColor(RA8875_BLUE);
        tft.setCursor(t_ptr->bx+10, t_ptr->by+10);
        tft.print(F("Starting Network"));
        enet_start();
        if (!enet_ready)
        {
            enet_start_fail_time = millis(); // set timer for 10 minute self recovery in main loop
            DPRINTLNF("Ethernet System Startup Failed, setting retry timer (10 minutes)");
        }
        DPRINTLNF("Ethernet System Startup Completed");
        // setSyncProvider(getNtpTime);
    }
    #endif

    // Update time on startup from RTC. If a USB connection is up, get the time from a PC.
    // Later if enet is up, get time from NTP periodically.
    setSyncProvider(getTeensy3Time); // the function to get the time from the RTC
    if (timeStatus() != timeSet)     // try this other way
        DPRINTLNF("Unable to sync with the RTC");
    else
        DPRINTLNF("RTC has set the system time");
    
    init_band_map();

    #ifdef I2C_LCD    // initialize the I2C LCD
        lcd.init(); 
        lcd.backlight();
        lcd.print("MyCall SDR"); // Edit this to what you like to see on your display
    #endif

    while (!Serial && (millis() < 5000)) ; // wait for Arduino Serial Monitor
    PC_Debug_port.println("\n\nUSB Host Testing - Serial V0.2");
    
    counter = 0;
    //disp_Menu();

    PC_Debug_port.println("End of Setup");

    //FrequencyRequest();

     // -------- Read SD Card data----------------------------------------------------------
    // To use the audio card SD card Reader instead of the Teensy 4.1 onboard Card Reader
    // UNCOMMENT THESE TWO LINES FOR TEENSY AUDIO BOARD   ***IF*** they are not used for something else:
    //SPI.setMOSI(7);  // Audio shield has MOSI on pin 7
    //SPI.setSCK(14);  // Audio shield has SCK on pin 14

    // see if the card is present and can be initialized:
    //SD_CardInfo();
    // open or create our config file.  Filenames follow DOS 8.3 format rules
    //Open_SD_cfgfile();
    // test our file
    // make a string for assembling the data to log:
    //write_db_tables();
    //read_db_tables();              // Read in stored values to memory
    //write_radiocfg_h();         // write out the #define to a file on the SD card.
                                    // This could be used by the PC during compile to override the RadioConfig.h

    // -------- Setup our radio settings and UI layout --------------------------------

    curr_band = user_settings[user_Profile].last_band; // get last band used from user profile.
    PAN(0);

    //==================================== Frequency Set ==========================================
    #ifdef PANADAPTER
        VFOA = PANADAPTER_LO;
        VFOB = PANADAPTER_LO;
    #else
        VFOA = bandmem[curr_band].vfo_A_last;
        VFOB = user_settings[user_Profile].sub_VFO;
    #endif
    DPRINT("Setup: VFOA = "); DPRINTLN(VFOA);
    
    // Calculate frequency difference between the designated xvtr IF band's lower edge and the current VFO band's lower edge (the LO frequency).
    find_new_band(VFOA, curr_band);  // find band index for VFOA frequency
    if (bandmem[curr_band].xvtr_IF)
        xvtr_offset = bandmem[curr_band].edge_lower - bandmem[bandmem[curr_band].xvtr_IF].edge_lower; // if band is 144 then PLL will be set to VFOA-xvtr_offset
    else
        xvtr_offset = 0;
    DPRINTF("Setup: xvr_offset = "); DPRINTLN(xvtr_offset);

    #if defined USE_RS_HFIQ // if RS-HFIQ is used, then send the active VFO frequency and receive the (possibly) updated VFO
        DPRINTLNF("Initializing RS-HFIQ Radio via USB Host port");
        #ifndef NO_RSHFIQ_BLOCKING
            RS_HFIQ.setup_RSHFIQ(1, VFOA - xvtr_offset); // 0 is non blocking wait, 1 is blocking wait.  Pass active VFO frequency
        #else
            RS_HFIQ.setup_RSHFIQ(0, VFOA - xvtr_offset); // 0 is non blocking wait, 1 is blocking wait.  Pass active VFO frequency
        #endif
        DPRINTF("Setup: Post RS-HFIQ VFOA = "); DPRINTLN(VFOA);
    #elif defined USE_CAT_SER
        //CAT_Serial.setup_CAT_Serial();   
        //delay(1000); // Give time to see the slash screen
    #endif

    #ifdef USE_RA8875
        tft.clearScreen();
    #else
        tft.clearActiveScreen();
    #endif

    DPRINTF("\nInitial Dial Frequency is "); DPRINT(formatVFO(VFOA)); DPRINTLNF("MHz");

    //--------------------------   Setup our Audio System -------------------------------------
    initVfo(); // initialize the si5351 vfo
    delay(10);
    //initDSP();
    // RFgain(0);
    changeBands(0); // Sets the VFOs to last used frequencies, sets preselector, active VFO, other last-used settings per band.
                    // Call changeBands() here after volume to get proper startup volume
    
    InternalTemperature.begin(TEMPERATURE_NO_ADC_SETTING_CHANGES);

    update_icon_outline(); // update any icons related to active encoders functions  This also calls displayRefresh.
    // displayRefresh();
}

// ********************************************Loop ******************************************
static uint32_t delta = 0;
bool    freqReceived = false; // initially, no frequency info has been received from the radio
bool    modeReceived = false;
uint8_t freqPoll     = 0;     // number of initial frequency querys in addtion to the broadcast info

void loop()
{  
    static int64_t newFreq   = 0;
    static uint32_t time_old = 0;
    static uint32_t time_n   = 0;
    #ifdef DEBUG
        static uint32_t time_sp;
    #endif

    #ifdef DEBUG_LOOP
        // JH
        static uint16_t loopcount = 0;
        static uint32_t jhTime    = millis();
        loopcount++;
        if (loopcount > 10)
        {
            uint32_t jhElapsed = millis() - jhTime;
            jhTime             = millis();
            loopcount          = 0;
            tft.fillRect(234, 5, 25, 25, BLACK);
            tft.setFont(Arial_12);
            tft.setCursor(236, 9, false);
            tft.setTextColor(DARKGREY);
            tft.print(jhElapsed / 10);
        }
    #endif

    time_current_baseloop = millis();
    //if ((time_current_baseloop - time_last_baseloop) > BASELOOP_TICK) {
        // ----------------------------------  check, whether there is something new from the radio
        //if (time_current_baseloop>t_RadioCheck) 
        
        CIVresultL = civ.readMsg(CIV_ADDR_905);
        if (CIVresultL.retVal<=CIV_NOK) {               // valid answer received !           
            #ifdef debug
              //Serial.print('.');
            #endif
            if (CIVresultL.retVal==CIV_OK_DAV) {          // Data available
                // Frequency
                if ((CIVresultL.cmd[1]==CIV_C_F_SEND[1]) ||  // command CIV_C_F_SEND received
                    (CIVresultL.cmd[1]==CIV_C_F_READ[1])) {  // command CIV_C_F_READ received
                    freqReceived = true;
                    PC_Debug_port.print(" Frequency: "); PC_Debug_port.println(CIVresultL.value);
                    freq = (uint64_t) CIVresultL.value;
                    VFOA = freq;
                    formatVFO(VFOA);
                    FreqToBandRules();
                    //bandSET();
                    displayFreq();                        
                } // command CIV_C_F_SEND or CIV_C_F_READ received  
                else
                // Mode
                if ((CIVresultL.cmd[1]==CIV_C_MOD_SEND[1]) ||  // command CIV_C_MODE_SEND received
                    (CIVresultL.cmd[1]==CIV_C_MOD_READ[1])) {  // command CIV_C_MODE_READ received
                    modeReceived = true;
                    PC_Debug_port.print("Mode Changed: ");
                    PC_Debug_port.println((uint8_t) CIVresultL.value);
                    selectMode((uint8_t) CIVresultL.value); // Select the mode for the Active VFO
                }
            } // Data available
        } // valid answer received           

        // ----------------------------------  do a query for frequency, if necessary
        // poll every 500 * 10ms = 5sec until a valid frequency has been received
        if ( (freqReceived == false) && ((lpCnt%500)==0) ) { 
            civ.writeMsg (CIV_ADDR_905, CIV_C_F_READ, CIV_D_NIX, CIV_wChk);
            freqPoll++;
            Serial.println("P ");
        }
        
        lpCnt++;
        time_last_baseloop = time_current_baseloop;
	//} // if BASELOOP_TICK

    #ifdef GPS
        civ.readGPS();   // read USB serial ch 'B' for GPS NMEA data strings
        // ToDo: extract grid and time info and display
    #endif

    //civ.pass_CAT_msg_to_PC();   // civ.readmsg() always does this.
    civ.pass_CAT_msg_to_RADIO();
  
    //if (CAT_Poll.check() == 1) 
        //civ.logDisplay();  // show messages accumulated until cleared.

    //if (CAT_Log_Clear.check() == 1)      // Clear the CIV log buffer, jsu show last 2 seconds
        //civ.logClear();
  
    //FrequencyRequest();   // Normally the frequency will be polled by external computer but if a local display is used, then poll it here
    #ifdef DEBUG
        time_sp = millis();
    #endif

    time_n = millis() - time_old;

    if (time_n > delta) // Main loop performance timer probe
    {
        delta = time_n;
        DPRINTF("Loop T=");
        DPRINT(delta);
        DPRINTF("  Spectrum T=");
        DPRINTLN(millis() - time_sp);
    }
    time_old = millis();

    // if (touch.check() == 1)
    //{
    Touch(); // touch points and gestures
    //}

    if (!popup && tuner.check() == 1 && newFreq < enc_ppr_response) // dump counts accumulated over time but < minimum for a step to count.
    {
        VFO.readAndReset();
        // VFO.read();
        newFreq = 0;
    }

    if (!popup)
        newFreq += VFO.read();      // faster to poll for change since last read
                                    // accumulate counts until we have enough to act on for scaling factor to work right.
    if (!popup && newFreq != 0 && abs(newFreq) > enc_ppr_response) // newFreq is a positive or negative number of counts since last read.
    {
        newFreq /= enc_ppr_response; // adjust for high vs low PPR encoders.  600ppr is too fast!
        selectFrequency(newFreq);
        VFO.readAndReset();
        // VFO.read();             // zero out counter for next read.
        newFreq = 0;
    }

    #if defined I2C_ENCODERS || defined MECH_ENCODERS
        Check_Encoders();
    #endif

    Check_GPIO_Switches();

    if (MF_Timeout.check() == 1)
    {
        MeterInUse = false;
        if (!MF_default_is_active)
        {
            MeterInUse = false;
            // DPRINTF("Switching to Default MF knob assignment, current owner is = "); DPRINTLN(MF_client);
            set_MF_Service(encoder_list[default_MF_slot].default_MF_client); // will turn off the button, if any, and set the default as new owner.
            MF_default_is_active = true;
            DPRINTF("Switching to Default MF knob assignment = ");
            DPRINTLN(MF_client);
        }
    }

    if (popup_timer.check() == 1 && popup) // stop spectrum updates, clear the screen and post up a keyboard or something
    {
        // timeout the active window
        pop_win_down(BAND_MENU);
        Band(255);
    }

    // The timer and flag are set by the rogerBeep() function
    if (touchBeep_flag && touchBeep_timer.check() == 1)
    {
        //touchBeep(false);
    }

    #ifdef ENET // Don't compile this code if no ethernet usage intended

    if (user_settings[user_Profile].enet_enabled) // only process enet if enabled.
    {
        if (!enet_ready)
            if ((millis() - enet_start_fail_time) > 600000) // check every 10 minutes (600K ms) and attempt a restart.
                enet_start();
        enet_read(); // Check for Control head commands
        if (rx_count != 0)
        {
        } // get_remote_cmd();       // scan buffer for command strings

        if (NTP_updateTx.check() == 1)
        {
            // while (Udp_NTP.parsePacket() > 0)
            //{};  // discard any previously received packets
            sendNTPpacket(timeServer);  // send an NTP packet to a time server
            NTP_updateRx.interval(100); // Start a timer to check RX reply
        }
        if (NTP_updateRx.check() == 1) // Time to check for a reply
        {
            if (getNtpTime())
                ;                         // Get our reply
            NTP_updateRx.interval(65000); // set it long until we need it again later
            Ethernet.maintain();          // keep our connection fresh
        }
    }
    #endif // End of Ethernet related functions here

    // Check if the time has updated (1 second) and update the clock display
    if (timeStatus() != timeNotSet) // && enet_ready) // Only display if ethernet is active and have a valid time source
    {
        if (now() != prevDisplay)
        {
            // update the display only if time has changed
            prevDisplay = now();
            // if(!bandmem[curr_band].XIT_en)
            displayTime();
        }
    }
}

void SendCIVmsg(void)
{
    uint8_t str[20] = "\xFE\xFE\xE0\x05\x00\x00\x20\x44\x01\xFD\x00";
    if (CAT_Poll.check() == 1) {
    CIVresultL = civ.writeMsg (CIV_ADDR_905, CIV_C_F_SEND, str, CIV_wChk);
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
            PC_Debug_port.print(" Frequency: "); PC_Debug_port.println(CIVresultL.value);
            freq = (uint64_t) CIVresultL.value;
            VFOA = freq;
            formatVFO(VFOA);
            FreqToBandRules();
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
    //CIVresultL = civ.writeMsg (CIV_ADDR_905, CIV_C_F_READ, CIV_D_NIX, CIV_wChk);
    PC_Debug_port.print("retVal of writeMsg: "); PC_Debug_port.println(retValStr[CIVresultL.retVal]);
    RequestTimeout[0]=millis();
  }
  #endif
}

//---------------------------------------------------------------------------------------------------------

unsigned int hexToDec(String hexString) {
    hexString.reserve(2);
    unsigned int decValue = 0;
    unsigned int nextInt;
    for (unsigned int i = 0; i < hexString.length(); i++) {
        nextInt = int(hexString.charAt(i));
        if (nextInt >= 48 && nextInt <= 57) nextInt = map(nextInt, 48, 57, 0, 9);
        if (nextInt >= 65 && nextInt <= 70) nextInt = map(nextInt, 65, 70, 10, 15);
        if (nextInt >= 97 && nextInt <= 102) nextInt = map(nextInt, 97, 102, 10, 15);
        nextInt = constrain((signed)nextInt, 0, 15);
        decValue = (decValue * 16) + nextInt;
    }
    return decValue;
}
//---------------------------------------------------------------------------------------------------------


void FreqToBandRules()
{
return;
         if (freq >=Freq2Band[0][0] && freq <=Freq2Band[0][1] )  {BAND=1;}  // 160m
    else if (freq >=Freq2Band[1][0] && freq <=Freq2Band[1][1] )  {BAND=2;}  //  80m
    else if (freq >=Freq2Band[2][0] && freq <=Freq2Band[2][1] )  {BAND=3;}  //  40m
    else if (freq >=Freq2Band[3][0] && freq <=Freq2Band[3][1] )  {BAND=4;}  //  30m
    else if (freq >=Freq2Band[4][0] && freq <=Freq2Band[4][1] )  {BAND=5;}  //  20m
    else if (freq >=Freq2Band[5][0] && freq <=Freq2Band[5][1] )  {BAND=6;}  //  17m
    else if (freq >=Freq2Band[6][0] && freq <=Freq2Band[6][1] )  {BAND=7;}  //  15m
    else if (freq >=Freq2Band[7][0] && freq <=Freq2Band[7][1] )  {BAND=8;}  //  12m
    else if (freq >=Freq2Band[8][0] && freq <=Freq2Band[8][1] )  {BAND=9;}  //  10m
    else if (freq >=Freq2Band[9][0] && freq <=Freq2Band[9][1] ) {BAND=10;}  //   6m
    else if (freq >=Freq2Band[10][0] && freq <=Freq2Band[10][1] ) {BAND=11;}  // 2m
    else if (freq >=Freq2Band[11][0] && freq <=Freq2Band[11][1] ) {BAND=12;}  // 70cm
    else if (freq >=Freq2Band[12][0] && freq <=Freq2Band[12][1] ) {BAND=13;}  // 1296
    else if (freq >=Freq2Band[13][0] && freq <=Freq2Band[13][1] ) {BAND=14;}  // 2.3G
    else if (freq >=Freq2Band[14][0] && freq <=Freq2Band[14][1] ) {BAND=15;}  // 5.7G
    else if (freq >=Freq2Band[15][0] && freq <=Freq2Band[15][1] ) {BAND=16;}  // 10G
    else {BAND=0;}   // out of range
}

bool CompareStrings(const char *sz1, const char *sz2) {
  while (*sz2 != 0) {
    if (toupper(*sz1) != toupper(*sz2)) 
      return false;
    sz1++;
    sz2++;
  }
  return true; // end of string so show as match
}

/*
char * convert_freq_to_Str(uint32_t freq)
{
  //sprintf(freq_str, "%lu", freq);
  //send_fixed_cmd_to_RSHFIQ(freq_str);
  //return freq_str;
}
*/

#ifdef BANDSET
void bandSET() {                                               // set outputs by BAND variable

  if(BAND==0 && previousBAND != 0){    // deactivate PTT
    //digitalWrite(PttOffPin, HIGH);
    PTT = true;
    #if defined(LCD)
      LcdNeedRefresh = true;
    #endif
  }else if(BAND!=0 && previousBAND == 0){    // deactivate PTT
    //digitalWrite(PttOffPin, LOW);
  }

  #if !defined(PTT_BY_BAND)
  if((PTT==false && previousBAND != 0 ) || (PTT==true && previousBAND == 0)){
  #endif
    for (int i = 0; i < NumberOfBoards; i++) {
      ShiftByte[i] = B00000000;
    }

    #if defined(MULTI_OUTPUT_BY_BCD)
      for (int i = 0; i < 17; i++) {   // outputs 1-8
        for (int y = 0; y < 4; y++) { // bcd bit
          if(bitRead(SelectBank,y)==1 && bitRead(matrix[BAND][i],y)==1){
            if(i<8){
              bitSet(ShiftByte[0], i);
            }else{
              bitSet(ShiftByte[1], i-8);
            }
          }
        }
      }
    #else
      for (int i = 0; i < 8; i++) {   // outputs 1-8
        if(matrix[BAND][i]>0){
          ShiftByte[0] = ShiftByte[0] | (1<<i);
        }
      }
      for (int i = 8; i < 16; i++) {   // outputs 9-16
        if(matrix[BAND][i]>0){
          ShiftByte[1] = ShiftByte[1] | (1<<(i-8));
        }
      }
      for (int i = 16; i < 24; i++) {   // outputs 17-24
        if(matrix[BAND][i]>0){
          ShiftByte[2] = ShiftByte[2] | (1<<(i-16));
        }
      }
      for (int i = 24; i < 32; i++) {   // outputs 25-32
        if(matrix[BAND][i]>0){
          ShiftByte[3] = ShiftByte[3] | (1<<(i-24));
        }
      }
      for (int i = 32; i < 40; i++) {   // outputs 33-40
        if(matrix[BAND][i]>0){
          ShiftByte[4] = ShiftByte[4] | (1<<(i-32));
        }
      }
    #endif

    //digitalWrite(ShiftOutLatchPin, LOW);    // ready for receive data
      for (int i = NumberOfBoards; i >0; i--) {   // outputs 9-16
        //shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftByte[i-1]);
      }
    ////////////////////////////////////////////////         digitalWrite(ShiftOutLatchPin, HIGH);    // switch to output pin

    #if !defined(INPUT_BCD) && !defined(MULTI_OUTPUT_BY_BCD) && !defined(PWM_OUT)
        //bcdOut();
    #endif

    #if defined(EthModule)
      TxUDP(ThisDevice, RemoteDevice, ShiftByte[1], ShiftByte[0], 0x00);
    #endif

    #if defined(LCD)
      LcdNeedRefresh = true;
    #endif
  #if !defined(PTT_BY_BAND)
  }
  #endif

  #if defined(PWM_OUT)
    analogWrite(PwmOutPin, PwmByBand[BAND]);
  #endif

  // #if defined(EthModule)
  //   if(DetectedRemoteSw[NET_ID][4]==0 || RemoteSwLatencyAnsw==0){
  //     //  && millis() < RemoteSwLatency[0]+RemoteSwLatency[1]*5) ){
  //     digitalWrite(PttOffPin, HIGH);
  //     #if defined(UdpBroadcastDebug_debug)
  //       TxBroadcastUdp("BandSet-" + String(DetectedRemoteSw[NET_ID][4]) + "-" + String(RemoteSwLatencyAnsw) );
  //     #endif
  //     PTT = true;
  //     #if defined(LCD)
  //     LcdNeedRefresh = true;
  //     #endif
  //   }
  // #endif
  #if defined(EthModule_XXX)
    #if defined(BcdIpRelay)
      TxUDP(ThisDevice, RemoteDevice, BAND, 0x00, 0x00);
    #else
      byte A = 0x00;
      for (int i = 0; i < 8; i++) {   // outputs 1-8
        if(matrix[BAND][i]==1){
          A = A | (1<<i); // set n-th bit
        }
      }
      byte B = 0x00;
      for (int i = 8; i < 16; i++) {   // outputs 8-16
        if(matrix[BAND][i]==1){
          B = B | (1<<i-8); // set n-th bit
        }
      }
      TxUDP(ThisDevice, RemoteDevice, A, B, 0x00);
    #endif
  #endif

  previousBAND = BAND;
  #if defined(WATCHDOG)
    WatchdogTimeout[0] = millis();                   // set time mark
  #endif
}
#endif
//----------

#ifndef USE_RA8875
    int16_t _activeWindowXL = 0;
    int16_t _activeWindowXR = SCREEN_WIDTH;
    int16_t _activeWindowYT = 0;
    int16_t _activeWindowYB = SCREEN_HEIGHT;

/**************************************************************************/
void setActiveWindow(int16_t XL,int16_t XR ,int16_t YT ,int16_t YB)
{
	//if (_portrait){ swapvals(XL,YT); swapvals(XR,YB);}

//	if (XR >= SCREEN_WIDTH) XR = SCREEN_WIDTH;
//	if (YB >= SCREEN_HEIGHT) YB = SCREEN_HEIGHT;
	_activeWindowXL = XL; _activeWindowXR = XR;
	_activeWindowYT = YT; _activeWindowYB = YB;
	updateActiveWindow(false);
}

/**************************************************************************/
/*!		
		Set the Active Window as FULL SCREEN
*/
/**************************************************************************/
void setActiveWindow_default(void)
{
	_activeWindowXL = 0; _activeWindowXR = SCREEN_WIDTH;
	_activeWindowYT = 0; _activeWindowYB = SCREEN_HEIGHT;
	//if (_portrait){swapvals(_activeWindowXL,_activeWindowYT); swapvals(_activeWindowXR,_activeWindowYB);}
	updateActiveWindow(true);
}

/**************************************************************************/
/*!
		this update the RA8875 Active Window registers
		[private]
*/
/**************************************************************************/
void updateActiveWindow(bool full)
{ 
	if (full){
		// X
		tft.activeWindowXY(0, 0);
		tft.activeWindowWH(SCREEN_WIDTH, SCREEN_HEIGHT);;
	} else {
		tft.activeWindowXY(_activeWindowXL, _activeWindowYT);
		tft.activeWindowWH(_activeWindowXR-_activeWindowXL, _activeWindowYB-_activeWindowYT);		
	}
}
#endif

//
//  Scans for any I2C connected devices and reports them to the serial terminal.  Usually done early in startup.
//
COLD void I2C_Scanner(void)
{
    byte error, address; // variable for error and I2C address
    int nDevices;

    // uncomment these to use alternate pins
    // WIRE.setSCL(37);  // On montherboard V2 PCBs these pins are for the VFO encoder
    // WIRE.setSDA(36);
    // WIRE.begin();

    DPRINTLNF("Scanning...");

    nDevices = 0;
    for (address = 1; address < 127; address++)
    {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        // Wire1.beginTransmission(address);   // Wire1 is the name for 2nd set of i2c port pins.
        error = Wire.endTransmission();
        // error = Wire1.endTransmission();

        if (error == 0)
        {
            DPRINTF("I2C device found at address 0x");
            if (address < 16)
                DPRINT("0");
            DPRINT(address, HEX);
            DPRINT("  (");
            printKnownChips(address);
            DPRINTLN(")");
            // DPRINTLN("  !");
            nDevices++;
        }
        else if (error == 4)
        {
            DPRINTF("Unknown error at address 0x");
            if (address < 16)
                DPRINT("0");
            DPRINTLN(address, HEX);
        }
    }
    if (nDevices == 0)
        DPRINTLNF("No I2C devices found\n");
    else
        DPRINTLNF("done\n");

    // delay(500); // wait 5 seconds for the next I2C scan
}

// Turns on or off a tone injected in the RX_summer block.
// Function that calls for a rogerBeep sets a global flag.
// The main loop starts a timer for a short beep an calls this to turn the tone on or off.
COLD void touchBeep(bool enable)
{
    if (enable)
    {
        touchBeep_flag = true;
        touchBeep_timer.reset();
        //Beep_Tone.amplitude(user_settings[user_Profile].rogerBeep_Vol);
        //Beep_Tone.frequency((float)user_settings[user_Profile].pitch); //    Alert tones
    }
    else
    {
        // if (rogerBeep_timer.check() == 1)   // make sure another event does not cut it off early
        //{
        touchBeep_flag = false;
        //Beep_Tone.amplitude(0.0);
        //}
    }
}

COLD void printDigits(int digits)
{
    // utility function for digital clock display: prints preceding colon and leading 0
    DPRINT(":");
    if (digits < 10)
        DPRINT('0');
    DPRINT(digits);
}

COLD time_t getTeensy3Time()
{
    return Teensy3Clock.get();
}

/*  code to process time sync messages from the serial port   */
#define TIME_HEADER "T" // Header tag for serial time sync message

COLD unsigned long processSyncMessage()
{
    unsigned long pctime             = 0L;
    const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

    if (Serial.find(TIME_HEADER)) // Search for the 'T' char in incoming serial stream of chars
    {
        pctime = Serial.parseInt(); // following the 'T' get the digits and convert to an int
        // return pctime;
        // DPRINTLN(pctime);
        if (pctime < DEFAULT_TIME)
        {                // check the value is a valid time (greater than Jan 1 2013)
            pctime = 0L; // return 0 to indicate that the time is not valid
        }
    }
    return pctime; // return will be seconds since jan 1 1970.
}

COLD void digitalClockDisplay() {
  // digital clock display of the time
  DPRINT(hour());
  printDigits(minute());
  printDigits(second());
  DPRINT(" ");
  DPRINT(day());
  DPRINT(" ");
  DPRINT(month());
  DPRINT(" ");
  DPRINT(year()); 
  DPRINTLN(); 
}

#if defined I2C_ENCODERS || defined MECH_ENCODERS
void Check_Encoders(void)
{   
    #if defined I2C_ENCODERS

        // Watch for the I2C Encoder INT pin to go low  (these I2C encoders are typically daisy-chained)
        if (digitalRead(I2C_INT_PIN) == LOW) 
        {
            //DPRINTLNF("I2C encoder Event");
            // We now have a valid slot determined suring progranm setup init (slot_e1, slot_e2, etc)
            #ifdef I2C_ENC1_ADDR
            // Check the status of the encoder (if enabled) and call the callback  
            if(I2C_ENC1.updateStatus() && encoder_list[_e1].role_A && encoder_list[_e1].enabled)
            {                           
                uint8_t mfg = I2C_ENC1.readStatus();
                if (mfg) {}
                //if (mfg) {DPRINTF("****Checked MF_Enc status = "); DPRINTLN(mfg); }
            }
            #endif
            #ifdef I2C_ENC2_ADDR
            if(I2C_ENC2.updateStatus() && encoder_list[_e2].role_A && encoder_list[_e2].enabled)
            {
                uint8_t mfg = I2C_ENC2.readStatus();
                if (mfg) {}
                //if (mfg) {DPRINTF("****Checked Encoder #2 status = "); DPRINTLN(mfg); }
            }
            #endif
            #ifdef I2C_ENC3_ADDR
            if(I2C_ENC3.updateStatus() && encoder_list[_e3].role_A && encoder_list[_e3].enabled)
            {
                uint8_t mfg = I2C_ENC3.readStatus();
                if (mfg) {}
                //if (mfg) {DPRINTF("****Checked Encoder #3 status = "); DPRINTLN(mfg); }
            }
            #endif
            #ifdef I2C_ENC4_ADDR
            if(I2C_ENC4.updateStatus() && encoder_list[_e4].role_A && encoder_list[_e4].enabled)
            {
                uint8_t mfg = I2C_ENC4.readStatus();
                if (mfg) {}
                //if (mfg) {DPRINTF("****Checked Encoder #4 status = "); DPRINTLN(mfg); }
            }
            #endif
            #ifdef I2C_ENC5_ADDR
            if(I2C_ENC5.updateStatus() && encoder_list[_e5].role_A && encoder_list[_e5].enabled)
            {
                uint8_t mfg = I2C_ENC5.readStatus();
                if (mfg) {}
                //if (mfg) {DPRINTF("****Checked Encoder #5 status = "); DPRINTLN(mfg); }
            }
            #endif
            #ifdef I2C_ENC6_ADDR
            if(I2C_ENC6.updateStatus() && encoder_list[_e6].role_A && encoder_list[_e6].enabled)
            {
                uint8_t mfg = I2C_ENC6.readStatus();
                if (mfg) {}
                //if (mfg) {DPRINTF("****Checked Encoder #6 status = "); DPRINTLN(mfg); }
            }
            #endif
        }
    #endif

    #ifdef GPIO_ENCODERS
        // Use an encoder on the GPIO pins, if any.
        //if (MF_client)  // skip if no one is listening.MF_Service();  // check the Multi-Function encoder and pass results to the current owner, of any.
        //{
        
        #if (GPIO_ENC2_ENABLE > 0)
            static int32_t gpio_enc2_counts = 0;
            int32_t enc2_ppr_response = 8;

            gpio_enc2_counts += GPIO_Encoder2.read();

            if (gpio_ENC2_Read_timer.check() == 1 && gpio_enc2_counts < enc2_ppr_response) // dump counts accumulated over time but < minimum for a step to count.
            {
                GPIO_Encoder2.readAndReset();
                gpio_enc2_counts = 0;
                gpio_ENC2_Read_timer.reset();
            }
        
            if (gpio_enc2_counts != 0 && abs(gpio_enc2_counts) > enc2_ppr_response)
            {
                gpio_enc2_counts /= enc2_ppr_response;
                DPRINT(F("GPIO 2 Encoder Calling I2C lib - Count = ")); DPRINTLN(gpio_enc2_counts);
                gpio_encoder_rotated(&GPIO_ENC2, gpio_enc2_counts);
                GPIO_Encoder2.readAndReset();
                gpio_enc2_counts = 0;
            }
        #endif

        #if (GPIO_ENC3_ENABLE > 0) 
            static int32_t gpio_enc3_counts = 0;
            int32_t enc3_ppr_response = 8;

            gpio_enc3_counts += GPIO_Encoder3.read();
            
            if (gpio_ENC3_Read_timer.check() == 1 && gpio_enc3_counts < enc3_ppr_response) // dump counts accumulated over time but < minimum for a step to count.
            {
                GPIO_Encoder3.readAndReset();
                gpio_enc3_counts = 0;
                gpio_ENC3_Read_timer.reset();
            }

            if (gpio_enc3_counts != 0 && abs(gpio_enc3_counts) > enc3_ppr_response)
            {
                gpio_enc3_counts /= enc3_ppr_response;
                DPRINT(F("GPIO 3 Encoder Calling I2C lib - Count = ")); DPRINTLN(gpio_enc3_counts);
                gpio_encoder_rotated(&GPIO_ENC3, gpio_enc3_counts);
                GPIO_Encoder3.readAndReset();
                gpio_enc3_counts = 0;
            }
        #endif

        // Switches associated with encoders.
        static uint8_t ENC2_sw_pushed = 0;
        static uint8_t ENC3_sw_pushed = 0;
        uint8_t slot = 0;
        

        if (GPIO_ENC2_ENABLE > 0)
        {
           	for (slot = 1; slot< NUM_AUX_ENCODERS; slot++)
            {
                if ((GPIO_ENC2_ENABLE == encoder_list[slot].id) && encoder_list[slot].enabled)
                {
                    //DPRINT(F("GPIO Slot # = ")); DPRINTLN(slot);
                    //DPRINT(F("id from slot # = ")); DPRINTLN(encoder_list[slot].id);
                    break;   // Got slot number for a valid GPIO_ENC encoder
                }
            }

            if (!digitalRead(GPIO_ENC2_PIN_SW) && !ENC2_sw_pushed )
            {
                ENC2_sw_pushed = 1;    //   Start button timer to test if this is going to be a tap or press
                gpio_switch_timer_start(encoder_list[slot].id);
            }       
            else if (digitalRead(GPIO_ENC2_PIN_SW) && ENC2_sw_pushed)
            {
                ENC2_sw_pushed = 0;
                gpio_switch_click(encoder_list[slot].id);   // switch released, process action, tap and press
            }
        }        
            
        if (GPIO_ENC3_ENABLE > 0)
        {
           	for (slot = 1; slot< NUM_AUX_ENCODERS; slot++)
            {
                if ((GPIO_ENC3_ENABLE == encoder_list[slot].id) && encoder_list[slot].enabled)
                {
                    //DPRINT(F("GPIO ENC3 Slot # = ")); DPRINTLN(slot);
                    //DPRINT(F("id from slot # = ")); DPRINTLN(encoder_list[slot].id);
                    break;  // Got a slot number for a valid GPIO_ENC encoder
                }
            }

            if (!digitalRead(GPIO_ENC3_PIN_SW) && !ENC3_sw_pushed)
            {
                ENC3_sw_pushed = 1;   //   Start button timer to test if this is going to be a tap or press
                gpio_switch_timer_start(encoder_list[slot].id);
            }
            else  if (digitalRead(GPIO_ENC3_PIN_SW) && ENC3_sw_pushed)
            {
                ENC3_sw_pushed = 0;  
                gpio_switch_click(encoder_list[slot].id);   // switch released, prcess action, tap and press
            }             
        }
    #endif
}
#endif 

// Scan the GPIO switches. Scan for press and release and set timers
void Check_GPIO_Switches(void)
{
    uint8_t slot;

    // Track switch timing for TAP vs PRESS
    static bool GPIO_sw1_pushed = 0;
    static bool GPIO_sw2_pushed = 0;
    static bool GPIO_sw3_pushed = 0;
    static bool GPIO_sw4_pushed = 0;
    static bool GPIO_sw5_pushed = 0;
    static bool GPIO_sw6_pushed = 0;

    for (slot = 1; slot < NUM_AUX_ENCODERS; slot++) // NUM_AUX ENCODER defines the size of the encoder/switch table.
    {
        if ((GPIO_SW1_ENABLE == encoder_list[slot].id) && encoder_list[slot].enabled)
        {
            GPIO_sw1_pushed = GPIO_Sw_read(GPIO_sw1_pushed, GPIO_SW1_PIN, slot);
            break;
        }
    }
    for (slot = 1; slot < NUM_AUX_ENCODERS; slot++) // NUM_AUX ENCODER defines the size of the encoder/switch table.
    {
        if ((GPIO_SW2_ENABLE == encoder_list[slot].id) && encoder_list[slot].enabled)
        {
            GPIO_sw2_pushed = GPIO_Sw_read(GPIO_sw2_pushed, GPIO_SW2_PIN, slot);
            break;
        }
    }
    for (slot = 1; slot < NUM_AUX_ENCODERS; slot++) // NUM_AUX ENCODER defines the size of the encoder/switch table.
    {
        if ((GPIO_SW3_ENABLE == encoder_list[slot].id) && encoder_list[slot].enabled)
        {
            GPIO_sw3_pushed = GPIO_Sw_read(GPIO_sw3_pushed, GPIO_SW3_PIN, slot);
            break;
        }
    }
    for (slot = 1; slot < NUM_AUX_ENCODERS; slot++) // NUM_AUX ENCODER defines the size of the encoder/switch table.
    {
        if ((GPIO_SW4_ENABLE == encoder_list[slot].id) && encoder_list[slot].enabled)
        {
            GPIO_sw4_pushed = GPIO_Sw_read(GPIO_sw4_pushed, GPIO_SW4_PIN, slot);
            break;
        }
    }
    for (slot = 1; slot < NUM_AUX_ENCODERS; slot++) // NUM_AUX ENCODER defines the size of the encoder/switch table.
    {
        if ((GPIO_SW5_ENABLE == encoder_list[slot].id) && encoder_list[slot].enabled)
        {
            GPIO_sw5_pushed = GPIO_Sw_read(GPIO_sw5_pushed, GPIO_SW5_PIN, slot);
            break;
        }
    }
    for (slot = 1; slot < NUM_AUX_ENCODERS; slot++) // NUM_AUX ENCODER defines the size of the encoder/switch table.
    {
        if ((GPIO_SW6_ENABLE == encoder_list[slot].id) && encoder_list[slot].enabled)
        {
            GPIO_sw6_pushed = GPIO_Sw_read(GPIO_sw6_pushed, GPIO_SW6_PIN, slot);
            break;
        }
    }
}

// Generic GPIO switch pin read, when pushed start a timer.
// If a switch is released after timer is expired then it is a PRESS(long), else it is a TAP (short).
// returns timer status so it can be tracked per switch]
bool GPIO_Sw_read(bool sw_pushed, uint8_t sw_pin, uint8_t slot)
{
    // DPRINTF("GPIO SW Pin # = "); DPRINTLN(sw_pin);
    // DPRINTF("GPIO SW Pushed # = "); DPRINTLN(sw_pushed);
    // DPRINTF("Encoder List Slot # = "); DPRINTLN(slot);
    if (!digitalRead(sw_pin) && !sw_pushed)
    {
        DPRINTF("Checking GPIO Switch - Start Timer - Slot = ");
        DPRINTLN(slot);
        sw_pushed = true; //   Start button timer to test if this is going to be a tap or press
        gpio_switch_timer_start(encoder_list[slot].id);
    }
    else if (digitalRead(sw_pin) && sw_pushed)
    {
        DPRINTF("Checking GPIO Switch - End Timer - Slot = ");
        DPRINTLN(slot);
        sw_pushed = false;
        gpio_switch_click(encoder_list[slot].id); // switch released, process action, tap and press
    }
    return sw_pushed;
}

void init_band_map(void)
{
    // Initialize Band Map.  255 means band is inactive.
    // Overwrites Panel_Pos default values in std_btn table band rows.
    (ENABLE_160M_BAND == 1)? enable_band(BS_160M, 1): enable_band(BS_160M, 0);
    (ENABLE_80M_BAND  == 1)? enable_band(BS_80M,  1): enable_band(BS_80M,  0);
    (ENABLE_60M_BAND  == 1)? enable_band(BS_60M,  1): enable_band(BS_60M,  0);
    (ENABLE_40M_BAND  == 1)? enable_band(BS_40M,  1): enable_band(BS_40M,  0);
    (ENABLE_30M_BAND  == 1)? enable_band(BS_30M,  1): enable_band(BS_30M,  0);
    (ENABLE_20M_BAND  == 1)? enable_band(BS_20M,  1): enable_band(BS_20M,  0);
    (ENABLE_17M_BAND  == 1)? enable_band(BS_17M,  1): enable_band(BS_17M,  0);
    (ENABLE_15M_BAND  == 1)? enable_band(BS_15M,  1): enable_band(BS_15M,  0);
    (ENABLE_12M_BAND  == 1)? enable_band(BS_12M,  1): enable_band(BS_12M,  0);
    (ENABLE_10M_BAND  == 1)? enable_band(BS_10M,  1): enable_band(BS_10M,  0);
    (ENABLE_6M_BAND   == 1)? enable_band(BS_6M,   1): enable_band(BS_6M,   0);
    (ENABLE_144_BAND  == 1)? enable_band(BS_144,  1): enable_band(BS_144,  0);
    (ENABLE_222_BAND  == 1)? enable_band(BS_222,  1): enable_band(BS_222,  0);
    (ENABLE_432_BAND  == 1)? enable_band(BS_432,  1): enable_band(BS_432,  0);
    (ENABLE_902_BAND  == 1)? enable_band(BS_902,  1): enable_band(BS_902,  0);
    (ENABLE_1296_BAND == 1)? enable_band(BS_1296, 1): enable_band(BS_1296, 0);
    (ENABLE_2304_BAND == 1)? enable_band(BS_2304, 1): enable_band(BS_2304, 0);
    (ENABLE_2400_BAND == 1)? enable_band(BS_2400, 1): enable_band(BS_2400, 0);
    (ENABLE_3400_BAND == 1)? enable_band(BS_3400, 1): enable_band(BS_3400, 0);
    (ENABLE_5760_BAND == 1)? enable_band(BS_5760, 1): enable_band(BS_5760, 0);
    (ENABLE_10G_BAND  == 1)? enable_band(BS_10G,  1): enable_band(BS_10G,  0);
    (ENABLE_24G_BAND  == 1)? enable_band(BS_24G,  1): enable_band(BS_24G,  0);
    (ENABLE_47G_BAND  == 1)? enable_band(BS_47G,  1): enable_band(BS_47G,  0);
    (ENABLE_76G_BAND  == 1)? enable_band(BS_76G,  1): enable_band(BS_76G,  0);
    (ENABLE_122G_BAND == 1)? enable_band(BS_122G, 1): enable_band(BS_122G, 0);
}

void enable_band(uint8_t _band, uint8_t _enable)
{
    struct Standard_Button* ptr = std_btn;

    if (_enable) 
    {
        (ptr + _band)->Panelpos =  _band - BS_160M;  // set panel position to position in band window (ordered from lowest band to highest band) any that are enabled.
        bandmem[_band - BS_160M].bandmap_en = ON; // set bandmap to enabled
    }
    else 
    {
        (ptr + _band)->Panelpos = 255;  // 255 disables showing a button
        bandmem[_band - BS_160M].bandmap_en = OFF;   // diable the band in the band map
    }
}

// prints a list of known i2C devices that match a discovered address
COLD void printKnownChips(byte address)
{
  // Is this list missing part numbers for chips you use?
  // Please suggest additions here:
  // https://github.com/PaulStoffregen/Wire/issues/new
  switch (address) {
    case 0x00: DPRINT(F("AS3935")); break;
    case 0x01: DPRINT(F("AS3935")); break;
    case 0x02: DPRINT(F("AS3935")); break;
    case 0x03: DPRINT(F("AS3935")); break;
    case 0x0A: DPRINT(F("SGTL5000")); break; // MCLK required
    case 0x0B: DPRINT(F("SMBusBattery?")); break;
    case 0x0C: DPRINT(F("AK8963")); break;
    case 0x10: DPRINT(F("CS4272")); break;
    case 0x11: DPRINT(F("Si4713")); break;
    case 0x13: DPRINT(F("VCNL4000,AK4558")); break;
    case 0x18: DPRINT(F("LIS331DLH")); break;
    case 0x19: DPRINT(F("LSM303,LIS331DLH")); break;
    case 0x1A: DPRINT(F("WM8731, WM8960")); break;
    case 0x1C: DPRINT(F("LIS3MDL")); break;
    case 0x1D: DPRINT(F("LSM303D,LSM9DS0,ADXL345,MMA7455L,LSM9DS1,LIS3DSH")); break;
    case 0x1E: DPRINT(F("LSM303D,HMC5883L,FXOS8700,LIS3DSH")); break;
    case 0x20: DPRINT(F("MCP23017,MCP23008,PCF8574,FXAS21002,SoilMoisture")); break;
    case 0x21: DPRINT(F("MCP23017,MCP23008,PCF8574")); break;
    case 0x22: DPRINT(F("MCP23017,MCP23008,PCF8574")); break;
    case 0x23: DPRINT(F("MCP23017,MCP23008,PCF8574")); break;
    case 0x24: DPRINT(F("MCP23017,MCP23008,PCF8574")); break;
    case 0x25: DPRINT(F("MCP23017,MCP23008,PCF8574")); break;
    case 0x26: DPRINT(F("MCP23017,MCP23008,PCF8574")); break;
    case 0x27: DPRINT(F("MCP23017,MCP23008,PCF8574,LCD16x2,DigoleDisplay")); break;
    case 0x28: DPRINT(F("BNO055,EM7180,CAP1188")); break;
    case 0x29: DPRINT(F("TSL2561,VL6180,TSL2561,TSL2591,BNO055,CAP1188")); break;
    case 0x2A: DPRINT(F("SGTL5000,CAP1188")); break;
    case 0x2B: DPRINT(F("CAP1188")); break;
    case 0x2C: DPRINT(F("MCP44XX ePot")); break;
    case 0x2D: DPRINT(F("MCP44XX ePot")); break;
    case 0x2E: DPRINT(F("MCP44XX ePot")); break;
    case 0x2F: DPRINT(F("MCP44XX ePot")); break;
    case 0x33: DPRINT(F("MAX11614,MAX11615")); break;
    case 0x34: DPRINT(F("MAX11612,MAX11613")); break;
    case 0x35: DPRINT(F("MAX11616,MAX11617")); break;
    case 0x38: DPRINT(F("RA8875,FT6206")); break;
    case 0x39: DPRINT(F("TSL2561, APDS9960")); break;
    case 0x3C: DPRINT(F("SSD1306,DigisparkOLED")); break;
    case 0x3D: DPRINT(F("SSD1306")); break;
    case 0x40: DPRINT(F("PCA9685,Si7021")); break;
    case 0x41: DPRINT(F("STMPE610,PCA9685")); break;
    case 0x42: DPRINT(F("PCA9685")); break;
    case 0x43: DPRINT(F("PCA9685")); break;
    case 0x44: DPRINT(F("PCA9685, SHT3X")); break;
    case 0x45: DPRINT(F("PCA9685, SHT3X")); break;
    case 0x46: DPRINT(F("PCA9685")); break;
    case 0x47: DPRINT(F("PCA9685")); break;
    case 0x48: DPRINT(F("ADS1115,PN532,TMP102,PCF8591")); break;
    case 0x49: DPRINT(F("ADS1115,TSL2561,PCF8591")); break;
    case 0x4A: DPRINT(F("ADS1115")); break;
    case 0x4B: DPRINT(F("ADS1115,TMP102")); break;
    case 0x50: DPRINT(F("EEPROM")); break;
    case 0x51: DPRINT(F("EEPROM")); break;
    case 0x52: DPRINT(F("Nunchuk,EEPROM")); break;
    case 0x53: DPRINT(F("ADXL345,EEPROM")); break;
    case 0x54: DPRINT(F("EEPROM")); break;
    case 0x55: DPRINT(F("EEPROM")); break;
    case 0x56: DPRINT(F("EEPROM")); break;
    case 0x57: DPRINT(F("EEPROM")); break;
    case 0x58: DPRINT(F("TPA2016,MAX21100")); break;
    case 0x5A: DPRINT(F("MPR121")); break;
    case 0x60: DPRINT(F("MPL3115,MCP4725,MCP4728,TEA5767,Si5351")); break;
    case 0x61: DPRINT(F("MCP4725,AtlasEzoDO,DuPPaEncoder")); break;
    case 0x62: DPRINT(F("LidarLite,MCP4725,AtlasEzoORP,DuPPaEncoder")); break;
    case 0x63: DPRINT(F("MCP4725,AtlasEzoPH,DuPPaEncoder")); break;
    case 0x64: DPRINT(F("AtlasEzoEC,DuPPaEncoder")); break;
    case 0x66: DPRINT(F("AtlasEzoRTD,DuPPaEncoder")); break;
    case 0x67: DPRINT(F("DuPPaEncoder")); break;
    case 0x68: DPRINT(F("DS1307,DS3231,MPU6050,MPU9050,MPU9250,ITG3200,ITG3701,LSM9DS0,L3G4200D,DuPPaEncoder")); break;
    case 0x69: DPRINT(F("MPU6050,MPU9050,MPU9250,ITG3701,L3G4200D")); break;
    case 0x6A: DPRINT(F("LSM9DS1")); break;
    case 0x6B: DPRINT(F("LSM9DS0")); break;
    case 0x70: DPRINT(F("HT16K33")); break;
    case 0x71: DPRINT(F("SFE7SEG,HT16K33")); break;
    case 0x72: DPRINT(F("HT16K33")); break;
    case 0x73: DPRINT(F("HT16K33")); break;
    case 0x76: DPRINT(F("MS5607,MS5611,MS5637,BMP280")); break;
    case 0x77: DPRINT(F("BMP085,BMA180,BMP280,MS5611")); break;
    default: DPRINT(F("unknown chip"));
  }
}

// ------------------------------------ MF_Service --------------------------------------
//
//  Called in the main loop to look for an encoder event and if found, call the registered function
//  All encoder rotation events pass through here in case it is a MF knob, otherwise the counts are passed on to the control function
//
static uint16_t old_ts;
COLD void MF_Service(int8_t counts, uint8_t knob)
{
    if (counts == 0) // no knob movement, nothing to do.
        return;

    if (knob == MF_client)
        MF_Timeout.reset(); // if it is the MF_Client then reset (extend) timeout timer as long as there is activity.
                            // When it expires it will be switched to default

    // DPRINTF("MF Knob Client ID is "); DPRINTLN(MF_client);

    switch (knob) // Give this owner control until timeout
    {
        case MFNONE:                                break; // no current owner, return
        case RFGAIN_BTN:    RFgain(counts);         break;
        case AFGAIN_BTN:    AFgain(counts);         break;
        case REFLVL_BTN:    RefLevel(counts*-1);    break;
        case PAN_BTN:       PAN(counts);            break;
        case ATTEN_BTN:     Atten(counts);          break;  // set attenuator level to value in database for this band
        case NB_BTN:        NBLevel(counts);        break;
        case ZOOM_BTN:      if (counts > 0) counts =  1;
                            if (counts < 0) counts = -1;
                            Zoom(counts);           break;
        case FILTER_BTN:    if (counts > 0) counts =  1;         // The controls here and below are not yet MF aware
                            if (counts < 0) counts = -1;
                            if (VARIABLE_FILTER) 
                                {Variable_Filter(counts);break;}
                            else 
                                {Filter(counts);    break;}
		case RATE_BTN:      if (counts > 0) counts =  1;
                            if (counts < 0) counts = -1;
                            Rate(counts);           break;
		case MODE_BTN:      if (counts > 0) counts =  1;
                            if (counts < 0) counts = -1;
                            setMode(counts);        break;
    	case AGC_BTN:       if (counts > 0) counts =  1;
                            if (counts < 0) counts = -1;
                            AGC(counts);            break;
        case ATU_BTN:       if (counts > 0) counts =  1;
                            if (counts < 0) counts = -1;
                            ATU(2);                 break;
        case ANT_BTN:       Ant();                  break;
        case BANDDN_BTN:    BandUp();               break;
        case BANDUP_BTN:    BandDn();               break;
        case BAND_BTN:      if (counts > 0) BandUp();
                            if (counts < 0) BandDn(); break;
        case RIT_BTN:       if (counts > 0) counts =  1;
                            if (counts < 0) counts = -1;
                            RIT(counts);   break;
        case XIT_BTN:       if (counts > 0) counts =  1;
                            if (counts < 0) counts = -1;
                            XIT(counts);   break;
        case MFTUNE :       old_ts = bandmem[curr_band].tune_step;   // Use MFTune as coarse Tune
                            #if defined GPIO_ENCODERS || defined I2C_ENCODERS   // skip when the touchscreen is the only tuning device
                                bandmem[curr_band].tune_step = old_ts+1;
                            #endif
                            selectFrequency(counts);
                            bandmem[curr_band].tune_step = old_ts;
                            break;     
        default:            break;   
    };
}

// Deregister the MF_client
COLD void unset_MF_Service(uint8_t old_client_name) // clear the old owner button status
{
    if (old_client_name == MF_client) // nothing needed if this is called from the button itself to deregister
    {
        // MF_client = user_settings[user_Profile].default_MF_client;    // assign new owner to default for now.
        // return;
    }

    // This must be from a timeout or a new button before timeout
    // Turn off button of the previous owner, if any, using the MF knob at change of owner or timeout
    // Some buttons can be left on such as Atten or other non-button MF users.  Just leave them off this list.
    switch (old_client_name)
    {
        case NONE:                              break;  // no current owner, return
        case RFGAIN_BTN:    setRFgain(-1);      break;  // since it was active toggle the output off
        case AFGAIN_BTN:    setAFgain(-1);      break;
        case REFLVL_BTN:    setRefLevel(-1);    break;
        case PAN_BTN:       setPAN(-1);         break;
        case ZOOM_BTN:      setZoom(-1);        break;
        case ATTEN_BTN:     setAtten(-1);       break;
        case NB_BTN:        setNB(-1);          break;
        case RIT_BTN:       setRIT(-1);         break;
        case XIT_BTN:       setXIT(-1);         break;    
        case MFTUNE:
        default:    //MF_client = encoder_list[0].default_MF_client;
                    //DPRINTF("unset_MF_Service: set default to "); DPRINTLN(MF_client);
                    break;  // No button for VFO tune, atten button stays on
    }
}

// ---------------------------------- set_MF_Service -----------------------------------
// Register the owner for the MF services.  Called by a client like RF Gain button.  Last caller wins.
// Clears the MF knob count and sets the flag for the new owner
// On any knob event the owner will get called with the MF counter value or switch action

// Potential owners can query the MF_client variable to see who owns the MF knob.
// Can take ownership by calling this fucntion and passing the enum ID for it's service function
COLD void set_MF_Service(uint8_t new_client_name) // this will be the new owner after we clear the old one
{
    unset_MF_Service(MF_client); //  turn off current button if on
    MF_client = new_client_name; // Now assign new owner
    MF_Timeout.reset();          // reset (extend) timeout timer as long as there is activity.
                                 // When it expires it will be switched to default
    // DPRINTF("New MF Knob Client ID is "); DPRINTLN(MF_client);
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
