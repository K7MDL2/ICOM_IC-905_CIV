//***************************************************************************************************
//
//    ICOM_IC-905_CIV.INO
//
//    USB Band Decoder for IC-905
//    Original June 23, 2023 by K7MDL
// 
//  Connect IC-905 to Teensy Host port.
//  Connect PC to Teensy USB
//  Connect Teensy IO pins to buffers and devices for band specific control
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
//     suggesing they may be connected to something inside for future use.
/*
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
*/
//
//***************************************************************************************************
#include "USBHost_t36.h"
#include <Metro.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

//#define USE_USB_AUDIO
#ifdef USE_USB_AUDIO
  #include <Audio.h>
  AudioInputUSB            usb_in;           //xy=200,69  (must set Tools > USB Type to Audio)
  AudioOutputUSB           usb_out;           //xy=200,69  (must set Tools > USB Type to Audio)
  AudioOutputI2S           i2s1;           //xy=365,94
  AudioConnection          patchCord_usb_li(usb_in, 0, i2s1, 0);
  AudioConnection          patchCord_usb_ri(usb_in, 1, i2s1, 1);
  //AudioConnection          patchCord1(i2s1, 0, usb_out, 0);
  //AudioConnection          patchCord2(i2s1, 1, usb_out, 1);
  AudioConnection          patchCord_usb_lo(usb_in, 0, usb_out, 0);
  AudioConnection          patchCord_usb_ro(usb_in, 1, usb_out, 1);
  AudioControlSGTL5000     sgtl5000_1;     //xy=302,184
#endif

#define USBBAUD 115200   // RS-HFIQ uses 57600 baud
uint32_t baud = USBBAUD;
uint32_t format = USBHOST_SERIAL_8N1;
USBHost myusb;
USBHub hub1(myusb);
USBHub hub2(myusb);
USBHIDParser hid1(myusb);
USBHIDParser hid2(myusb);
//USBHIDParser hid3(myusb);

// There are now two versions of the USBSerial class, that are both derived from a common Base class
// The difference is on how large of transfers that it can handle.  This is controlled by
// the device descriptor, where up to now we handled those up to 64 byte USB transfers.
// But there are now new devices that support larger transfer like 512 bytes.  This for example
// includes the Teensy 4.x boards.  For these we need the big buffer version. 
// uncomment one of the following defines for userial
//USBSerial userial(myusb);  // works only for those Serial devices who transfer <=64 bytes (like T3.x, FTDI...)
//USBSerial userial1(myusb);  // works only for those Serial devices who transfer <=64 bytes (like T3.x, FTDI...)
USBSerial_BigBuffer userial(myusb, 1); // Handles anything up to 512 bytes
USBSerial_BigBuffer userial1(myusb, 1); // Handles anything up to 512 bytes
//USBSerial_BigBuffer userial(myusb); // Handles up to 512 but by default only for those > 64 bytes
//USBSerial_BigBuffer userial1(myusb); // Handles up to 512 but by default only for those > 64 bytes

USBDriver *drivers[] = {&hub1, &hub2, &hid1, &hid2, &userial, &userial1};
#define CNT_DEVICES (sizeof(drivers)/sizeof(drivers[0]))
const char * driver_names[CNT_DEVICES] = {"Hub1", "Hub2",  "HID1", "HID2", "USERIAL1", "USERIAL2" };
bool driver_active[CNT_DEVICES] = {false, false, false, false};

Metro CAT_Poll = Metro(1000);     // Throttle the servicing for CAT comms
bool Proceed  = false; 
int   counter = 0;

#define DISP_FREQ
#define GPS
#define PC_CAT_port Serial
//#define PC_CAT_port SerialUSB1
#define PC_GPS_port SerialUSB1
#define PC_Debug_port SerialUSB2

int fromAdress = 0xE0;              // 0E
byte rdI[12];   //read data icom
String rdIS;    //read data icom string
uint64_t freqPrev1;
byte incomingByte = 0;
int state = 1;  // state machine
bool StateMachineEnd = false;
int icomSM(byte b);
int txCIV(int commandCIV, uint32_t dataCIVtx, int toAddress);
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

uint32_t VoltageRefresh[2] = {0, 3000};   // refresh in ms
float ArefVoltage = 4.303;            // Measure on Aref pin 20 for calibrate
float Divider = 1;

/*
// Modified 4/2021 for SDR/Panadapter usage by K7MDL
// Modified heavily 6/2023 to specialize as a IC-905 USB band decoder
 - pass through 2 serial interfaces and 1 audio interface, tap into the CI-V frequency value to operate band decode outputs

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
void setup()
{
    //int blocking = 1;
    PC_CAT_port.begin(115200);
    #ifdef GPS
      PC_GPS_port.begin(9600);
    #endif

    while (!Serial && (millis() < 5000)) ; // wait for Arduino Serial Monitor
    PC_Debug_port.println("\n\nUSB Host Testing - Serial V0.2");
    myusb.begin();
    delay(50);
    PC_Debug_port.println("Waiting for USB device to register on USB Host port");
    while (!Proceed)  // observed about 500ms required.
    {
        refresh_myUSB();   // wait until we have a valid USB 
        //PC_Debug_port.print("Retry (500ms) = "); PC_Debug_port.println(counter++);
        delay (500);
    }
    delay(1);  // about 1-2 seconds needed before RS-HFIQ ready to receive commands over USB
    PC_Debug_port.println("Start of USB Host port Setup");

    counter = 0;
    //disp_Menu();

    PC_Debug_port.println("End of Setup");
    //FrequencyRequest();
    PC_CAT_port.flush();
    #ifdef GPS
      PC_GPS_port.flush();
    #endif

  #ifdef USE_USB_AUDIO    
    AudioMemory(12);
    sgtl5000_1.enable();
    sgtl5000_1.lineInLevel(15); // Set to minimum, maybe prevent false Auto-iI2S detection
    sgtl5000_1.volume(0.6);
  #endif
}

// ********************************************Loop ******************************************

void loop()
{  
  cmd_Console();
  
  //if (CAT_Poll.check() == 1) 
    //FrequencyRequest();   // Normall the frequency wil be polled by external computer but if a local display is used, then poll it here

}

void cmd_Console(void)
{
    byte incomingByte = 0;
    byte outgoingByte = 0;

  #ifdef GPS
    byte incomingByte_1 = 0;
    // read the 905 GPS data on the 2nd virtual serial USB Host interface and pass it through to the PC at 9600baud
    while (userial1.available() > 0) 
    {
      incomingByte_1 = userial1.read();   // GPS 
      PC_GPS_port.write(incomingByte_1);
      //PC_Debug_port.print("GPS: ");
      //PC_Debug_port.printf("%c",incomingByte_1);
    }
  #endif

  // Pass though CI-V data FROM PC to RADIO
  if (PC_CAT_port.available() > 0) 
  {
    outgoingByte = PC_CAT_port.read();
    userial.write(outgoingByte);
  }

  // CI-V CAT port/  Pass through CI-V CAT data FROM radio to PC
  if (userial.available() > 0) 
  {
    incomingByte = userial.read();    // Read radio CI-V output
    PC_CAT_port.write(incomingByte);  // pass through to the PC

    //#if defined(DEBUG)
      //PC_Debug_port.print(incomingByte);
      //PC_Debug_port.print(F("|"));
      //PC_Debug_port.print(incomingByte, HEX);
      //PC_Debug_port.print(" ");
    //#endif
    
    icomSM(incomingByte);
    
    rdIS="";
    // if(rdI[10]==0xFD){    // state machine end// state machine end
    if(StateMachineEnd == true)  // state machine end
    {
      StateMachineEnd = false;
      for (int i=10; i>=5; i-- )  //format frequency  - byte 11 is FD EOM  byte 5 is 1 and 10Hz.
      {
        if (rdI[i] < 10)            // leading zero
        { 
          rdIS = rdIS + 0;
        }
        rdIS = rdIS + String(rdI[i], HEX);  // append BCD digit from HEX variable to string
      }
      //PC_Debug_port.print(rdIS);
      freq = strtoll(rdIS.c_str(), NULL, 10);

      #ifdef DISP_FREQ
        PC_Debug_port.print("-");
        PC_Debug_port.print(freq);
        PC_Debug_port.print("-");
      #endif
      
      formatVFO(freq);
      FreqToBandRules();
      //bandSET();

      #if defined(SERIAL_echo)
          serialEcho();
      #endif
      RequestTimeout[0]=millis();
    }
  }
}
 
int icomSM(byte b)
{      
   // state machine
    // This filter solves read from 0x00 0x05 0x03 commands and 00 E0 F1 address used by software
    static bool Band100GHz = false;
    /* 
    PC_Debug_port.print(b, HEX);
    PC_Debug_port.print("|");
    PC_Debug_port.print(state);
    PC_Debug_port.print("|");
    PC_Debug_port.print(Band100GHz);
    PC_Debug_port.print(" | ");
    */
    switch (state) 
    {
        case 1: if( b == 0xFE ) 
                { state = 2; rdI[0]=b; }  
                for (int j=1; j<12; j++)
                  rdI[j]=0x00;
                break;

        case 2: if( b == 0xFE ) 
                { state = 3; rdI[1]=b;} 
                else
                { state = 1;}; 
                break;

        // addresses that use different software 00-trx, e0-pc-ale, winlinkRMS, f1-winlink trimode
        case 3: if( b == 0x00 || b == 0xE0 || b == 0x0E || b == 0xF1 ){ state = 4; rdI[2]=b;}                       // choose command $03
                else if( b == CIV_ADRESS )
                { state = 6; rdI[2]=b;}
                else if( b == 0xFE )
                { state = 3; rdI[1]=b;}      // FE (3x reduce to 2x)
                else 
                { state = 1;}
                break;                       // or $05

        case 4: if( b == CIV_ADRESS )
                { state = 5; rdI[3]=b;}
                else
                { state = 1;}
                break;                      // select command $03

        case 5: if( b == 0x00 || b == 0x03 )
                {state = 8; rdI[4]=b;}  // freq
                else if( b == 0x04 )
                {state = 15; rdI[4]=b;}        // mode
                else if( b == 0xFE )
                { state = 2; rdI[0]=b;}        // FE
                else
                { state = 1;}
                break;

        case 6: if( b == 0x00 || b == 0xE0 || b == 0xF1 )
                { state = 7; rdI[3]=b;} 
                else
                { state = 1;}
                break;  // select command $05

        case 7: if( b == 0x00 || b == 0x05 )
                { state = 8; rdI[4]=b;}
                else
                { state = 1;}
                break;

        case 8: if( b <= 0x99 )
                {state = 9; rdI[5]=b;}             // 10Hz 1Hz
                else if( b == 0xFE )
                { state = 2; rdI[0]=b;}      // FE
                else
                {state = 1;} 
                break;

        case 9: if( b <= 0x99 )
                {state = 10; rdI[6]=b;}            // 1kHz 100Hz
                else if( b == 0xFE ) 
                { state = 2; rdI[0]=b;}      // FE
                else
                {state = 1;}
                break;
        
        case 10: if( b <= 0x99 )
                {state = 11; rdI[7]=b;}            // 100kHz 10kHz
                else if( b == 0xFE )
                { state = 2; rdI[0]=b;}      // FE
                else
                {state = 1;}
                break;
        
        case 11: if( b <= 0x99)
                { 
                  state = 12; rdI[8]=b;            // 10MHz 1Mhz
                  Band100GHz=false;
                }
                else if( b == 0xFE )
                { state = 2; rdI[0]=b;}      // FE
                else
                { state = 1;} 
                break;
        
        case 12: if( b <= 0x99)
                {
                  state = 13; rdI[9]=b;            // 1GHz 100MHz
                  Band100GHz=true;
                }
                else if( b == 0xFE )
                { state = 2; rdI[0]=b;}      // FE
                else
                {state = 1;}
                break;
        
        // For the IC-905, the FD will be after 5 bytes for frequencies < 6GHz, 6 bytes for 10GB
        case 13: //PC_Debug_port.print("Test 100GHz|"); 
                if( b == 0x01 && Band100GHz==true)
                {state = 14; rdI[10]=b;}   // 100GHz 10GHz  <-- 100GHz limit
                else if( b == 0xFD )
                {state = 1; rdI[11]=b; StateMachineEnd = true;}  // end of freq report
                else if( b == 0xFE )
                {state = 2; rdI[0]=b;}      // FE
                else{state = 1;}
                break;
        
        case 14: //PC_Debug_port.println("Test FD"); 
                if( b == 0xFD )
                {state = 1; rdI[11]=b; StateMachineEnd = true;}   // end of freq report
                else if( b == 0xFE )
                { state = 2; rdI[0]=b;}      // FE
                else
                {state = 1; rdI[10] = 0x00;}
                break;

        case 15: if( b <= 0x12 )
                {state = 16; rdI[5]=b;}
                else if( b == 0xFE )
                { state = 2; rdI[0]=b;}      // FE
                else
                {state = 1;}
                break;   // Mode
        
        case 16: if( b <= 0x03 )
                {state = 17; rdI[6]=b;}
                else if( b == 0xFE )
                {state = 2; rdI[0]=b;}      // FE
                else
                {state = 1;}
                break;   // Filter
        
        case 17: if( b == 0xFD )
                {state = 1; rdI[7]=b;}
                else if( b == 0xFE )
                {state = 2; rdI[0]=b;}      // FE
                else
                {state = 1; rdI[7] = 0;}
                break;
    }

    // will process all 6 bytes of frequency
    //PC_Debug_port.printf("%02X|%02X|%02X|%02X|%02X|%02X|%02X|%02X|%02X|%02X|%02X|%02X\n",rdI[0],rdI[1],rdI[2],rdI[3],rdI[4],rdI[5],rdI[6],rdI[7],rdI[8],rdI[9],rdI[10],rdI[11]);
    return 0;
}

void FrequencyRequest(){
  #if defined(REQUEST)
  if(REQUEST > 0 && (millis() - RequestTimeout[0] > RequestTimeout[1])){

    #if defined(ICOM_CIV)
      txCIV(3, 0, CIV_ADRESS);  // ([command], [freq]) 3=read
    #endif

    RequestTimeout[0]=millis();
  }
  #endif
}

int txCIV(int commandCIV, uint32_t dataCIVtx, int toAddress) 
{
        //CAT_port.flush();
        userial.write(254);                                    // FE
        userial.write(254);                                    // FE
        userial.write(toAddress);                              // to adress
        userial.write(fromAdress);                             // from OE
        userial.write(commandCIV);                             // data
        if (dataCIVtx != 0){
            String freqCIVtx = String(dataCIVtx);             // to string
            freqCIVtx.reserve(11);
            String freqCIVtxPart;
            freqCIVtxPart.reserve(11);
            while (freqCIVtx.length() < 10) {                 // leding zeros
                freqCIVtx = 0 + freqCIVtx;
            }
            for (int x=8; x>=0; x=x-2){                       // loop for 5x2 char [xx xx xx xx xx]
                freqCIVtxPart = freqCIVtx.substring(x,x+2);   // cut freq to five part
                    userial.write(hexToDec(freqCIVtxPart));    // HEX to DEC, because write as DEC format from HEX variable
            }
        }
        userial.write(253);                                    // FD
        // CAT_port.flush();
        while(userial.available()){        // clear buffer
          userial.read();
        }
        return 0;
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

void write_RSHFIQ(int ch)
{   
    userial.write(ch);
} 

int read_RSHFIQ(void)
{
    while (userial.available()) 
    {
        //PC_Debug_port.println("USerial Available");
        return userial.read();
    }
    return 0;
}
*/

void refresh_myUSB(void)
{
    myusb.Task();
    // Print out information about different devices.
    for (uint8_t i = 0; i < CNT_DEVICES; i++) 
    {
        if (*drivers[i] != driver_active[i]) 
        {
            if (driver_active[i]) 
            {
                PC_Debug_port.printf("*** Device %s - disconnected ***\n", driver_names[i]);
                driver_active[i] = false;
                Proceed = false;
            } 
            else 
            {
                PC_Debug_port.printf("*** Device %s %x:%x - connected ***\n", driver_names[i], drivers[i]->idVendor(), drivers[i]->idProduct());
                driver_active[i] = true;
                Proceed = true;

                const uint8_t *psz = drivers[i]->manufacturer();
                if (psz && *psz) PC_Debug_port.printf("  manufacturer: %s\n", psz);
                psz = drivers[i]->product();
                if (psz && *psz) PC_Debug_port.printf("  product: %s\n", psz);
                psz = drivers[i]->serialNumber();
                if (psz && *psz) PC_Debug_port.printf("  Serial: %s\n", psz);

                // If this is a new Serial device.
                if (drivers[i] == &userial1) 
                {
                    // Lets try first outputting something to our USerial to see if it will go out...
                    userial.begin(baud);
                }
                if (drivers[i] == &userial1) 
                {
                    // Lets try first outputting something to our USerial to see if it will go out...
                    userial1.begin(9600);
                }
            }
        }
    }
}

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

//
//    formatVFO()
//
const char* formatVFO(uint64_t vfo)
{
	static char vfo_str[20] = {""};
	//if (ModeOffset < -1 || ModeOffset > 1)
		//vfo += ModeOffset;  // Account for pitch offset when in CW mode, not others
	
	uint32_t MHz = (vfo/1000000 % 1000000);
	uint16_t Hz  = (vfo % 1000);
	uint16_t KHz = ((vfo % 1000000) - Hz)/1000;
	sprintf(vfo_str, "%lu.%03u.%03u", MHz, KHz, Hz);
	
  #ifdef DISP_FREQ
    PC_Debug_port.printf(" %sMHz\n", vfo_str);
  #endif
  
	return vfo_str;
}