#ifndef _IC_905_CIV_H_
#define _IC_905_CIV_H_

//  ICOM_IC-905.h
//
//  Header File for the Main Arduino program file
//
#include <Arduino.h>                    // from Arduino
#include <avr/pgmspace.h>               // from Arduino
#include <SPI.h>                        // from Arduino
#include <Wire.h>                       // from Arduino
#include <TimeLib.h>                    // from Arduino

// External libraries - These are not all of them as some appear with #ifdef blocks based on feature selection - See Github Wiki "Libraries" page for a full listing.
#define  ENCODER_OPTIMIZE_INTERRUPTS    // leave this one here.  Not normally user changed
#include "Encoder.h"                    // Internal Teensy library and at C:\Program Files (x86)\Arduino\hardware\teensy\avr\libraries
#include "Metro.h"                      // GitHub https://github.com/nusolar/Metro
//#include "OpenAudio_ArduinoLibrary.h"   // F32 library located on GitHub. https://github.com/chipaudette/OpenAudio_ArduinoLibrary
#include "InternalTemperature.h"        // V2.1.0 @ Github https://github.com/LAtimes2/InternalTemperature

#define BYPASS_SPECTRUM_MODULE   // debugging temp 

#define DISP_FREQ
#define GPS
#define PC_CAT_port SerialUSB1
//#define PC_CAT_port SerialUSB1
#define PC_GPS_port SerialUSB2
#define PC_Debug_port Serial

#define DEBUG  //set for debug output

#ifdef  DEBUG
#define DEBUG_ERROR true
#define DEBUG_ERROR_SERIAL if(DEBUG_ERROR)PC_Debug_port

#define DEBUG_WARNING true
#define DEBUG_WARNING_SERIAL if(DEBUG_WARNING)PC_Debug_port

#define DEBUG_INFORMATION true
#define DEBUG_INFORMATION_SERIAL if(DEBUG_INFORMATION)PC_Debug_port
#define DSERIALBEGIN(...)   PC_Debug_port.begin(__VA_ARGS__)
#define DPRINTLN(...)       PC_Debug_port.println(__VA_ARGS__)
#define DPRINT(...)         PC_Debug_port.print(__VA_ARGS__)
#define DPRINTF(...)        PC_Debug_port.print(F(__VA_ARGS__))
#define DPRINTLNF(...)      PC_Debug_port.println(F(__VA_ARGS__)) //printing text using the F macro
#define DELAY(...)          delay(__VA_ARGS__)
#define PINMODE(...)        pinMode(__VA_ARGS__)
#define TOGGLEd13           PINB = 0x20                    //UNO's pin D13
#define DEBUG_PRINT(...)    PC_Debug_port.print(F(#__VA_ARGS__" = ")); PC_Debug_port.print(__VA_ARGS__); PC_Debug_port.print(F(" ")) 
#define DEBUG_PRINTLN(...)  DEBUG_PRINT(__VA_ARGS__); Serial.println()
#define DEBUG_PRINTF(...)   PC_Debug_port.printf(__VA_ARGS__)
#else
#define DSERIALBEGIN(...)
#define DPRINTLN(...)
#define DPRINT(...)
#define DPRINTF(...)      
#define DPRINTLNF(...)    
#define DELAY(...)        
#define PINMODE(...)      
#define TOGGLEd13      
#define DEBUG_PRINT(...)    
#define DEBUG_PRINTLN(...)
#define DEBUG_PRINTF(...)
#endif

// Below are local project files
#include "RadioConfig.h"        // Our main configuration file
//#include "SDR_Network.h"        // for ethernet UDP remote control and monitoring
#include "Vfo.h"
#include "Display.h"
#include "Tuner.h"
//#include "Smeter.h"
#include "Controls.h"
#include "UserInput.h"          // include after Spectrum_RA8875.h and Display.h
//#include "Bandwidth2.h"
#include "SD_Card.h"

#define useUSBHostSerial_A      // set for Teensy USB Serial CAT port ch 'A'
#define TEENSY4                 // tell CIV lib to use Teensy USB Host

// Simple ways to designate functions to run out of fast or slower memory to help save RAM
#define HOT     FASTRUN     __attribute__((hot))
#define COLD    FLASHMEM    __attribute__((cold))

//
//--------------------------------- RA8875 LCD TOUCH DISPLAY INIT & PINS --------------------------
//
//  Usually defined in main program header file such as RadioConfig.h for SDR_887x program
    #ifdef USE_RA8875
        #define  SCREEN_WIDTH       800 
        #define  SCREEN_HEIGHT      480
        #if defined SMALL_PCB_V1
            #define  RA8875_INT        28   //for John's small V1 motherboard
        #elif defined V1_4_3_PCB || defined V2_4_3_PCB || defined V21_7_PCB || defined V22_7_PCB
            #define  RA8875_INT        27   //27 for John's larger 4.3" motherboard
        #else
            #define  RA8875_INT        14   //14 for K7MDL old prototype board
        #endif    
        #define  RA8875_CS         10       //any digital pin
        #define  RA8875_RESET      9        //any pin or nothing!
        #define  MAXTOUCHLIMIT     3        //1...5  using 3 for 3 finger swipes, otherwise 2 for pinches or just 1 for touch
        #include <SPI.h>                    // included with Arduino
        #include <RA8875.h>                 // internal Teensy library with ft5206 cap touch enabled in user_setting.h
        #include <ili9488_t3_font_Arial.h>      // https://github.com/PaulStoffregen/ILI9341_t3
        #include <ili9488_t3_font_ArialBold.h>  // https://github.com/PaulStoffregen/ILI9341_t3
    #else 
        #define  USE_RA8876_t3
        #define  SCREEN_WIDTH      1024 
        #define  SCREEN_HEIGHT     600
        #include <RA8876_t3.h>                  // https://github.com/wwatson4506/Ra8876LiteTeensy
        #include <ili9488_t3_font_Arial.h>      // https://github.com/PaulStoffregen/ILI9341_t3
        #include <ili9488_t3_font_ArialBold.h>  // https://github.com/PaulStoffregen/ILI9341_t3
        #include <FT5206.h>
        #if defined SMALL_PCB_V1
            #define  CTP_INT        28  //for John's small V1 motherboard
        #elif definedV1_4_3_PCB || defined V2_4_3_PCB || defined V21_7_PCB || defined V22_7_PCB
            #define  CTP_INT        27  // for John's larger 4.3" motherboard
        #else
            #define  CTP_INT        14  //14 for K7MDL old prototype board
        #endif
        #define  RA8876_CS         10   //any digital pin
        #define  RA8876_RESET       9   //any pin or nothing!
        #define  MAXTOUCHLIMIT      3   //1...5  using 3 for 3 finger swipes, otherwise 2 for pinches or just 1 for touch              
    #endif // USE_RA8876_t3

// From RA8876_t3/RA8876Registers.h
#define BLACK		  0x0000
#define WHITE		  0xffff
#define RED		  	  0xf800
#define LIGHTRED	  0xfc10
#define CRIMSON		  0x8000
#define GREEN		  0x07e0
#define PALEGREEN	  0x87f0
#define DARKGREEN	  0x0400
#define BLUE		  0x001f
#define LIGHTBLUE	  0x051f
#define SKYBLUE		  0x841f
#define DARKBLUE	  0x0010
#define YELLOW		  0xffe0
#define LIGHTYELLOW	  0xfff0
#define DARKYELLOW	  0x8400 // mustard
#define CYAN		  0x07ff
#define LIGHTCYAN	  0x87ff
#define DARKCYAN	  0x0410
#define MAGENTA		  0xf81f
#define VIOLET		  0xfc1f
#define BLUEVIOLET	  0x8010
#define ORCHID		  0xA145 
// Other sources of RGB color definitions
#define NAVY          0x000F
#define MAROON        0x7800
#define PURPLE        0x780F
#define OLIVE         0x7BE0
#define LIGHTGREY     0xC618
#define DARKGREY      0x7BEF
#define ORANGE        0xFD20
#define GREENYELLOW   0xAFE5
#define PINK          0xF81F
#define LIGHTORANGE   0xFC80 // the experimentalist

// Some custom color mixes   RGB565 
// Can use the RGB565 Color PIcker tool  at https://www.barth-dev.de/online/rgb565-color-picker/
#define myDARKGREY    31727u
#define myVDARKGREY   0x4A49  //0x632C //0x5AEC 0x2945  0x4A49  0x4208
//#define myVDARKGREEN  0x12C3 // very dark green
const uint16_t myVDARKGREEN = 0x12C3; // very dark green  spectrum function wants this form.
#define myDARKGREEN   0x02C0 // dark green
#define myDARKBLUE    0x02B0
#define myVDARKBLUE   0x01B0 // very dark blue
#define myVDKORANGE   0xCB21 // very dark orange, maybe burnt orange
#define myDKYELLOW    0x9422 // very dark yellow
#define myDKPINK      0xA1F3 // dark 
#define myDARKRED     0xC8C9 // dark red, lighter than Maroon
#define myMIDGREEN    0x1688 // green , not bright

// Convert 24bit RGB to RGB565 using  http://www.drakker.org/convert_rgb565.html
#define myBRIGHTPINK  0xF8F1
#define myMIDGREEN2   0x03E0
#define myBRIGHTGREEN 0x6646
#define myVIOILET     0xA01F
#define myLAVBLUE     0x631F
#define myGREY        0xBDF7
//Conversion formula is RGB565 = (((RGB888&0xf80000)>>8) + ((RGB888&0xfc00)>>5) + ((RGB888&0xf8)>>3));


///////////////////////Set up global variables for Frequency, mode, bandwidth, step
// Index to bandmem table rows.  Use BANDX since rows can be in any order
                    // Band slot index into data table
#define BAND160M      0 // 160M 
#define BAND80M       1 // 80M
#define BAND60M       2 // 60M
#define BAND40M       3 // 40M
#define BAND30M       4 // 30M
#define BAND20M       5 // 20M
#define BAND17M       6 // 17M      
#define BAND15M       7 // 15M
#define BAND12M       8 // 12M
#define BAND10M       9 // 10M
#define BAND6M       10 // 6M
#define BAND144      11 // 2M
#define BAND222      12 // 222
#define BAND432      13 // 432
#define BAND902      14 // 902
#define BAND1296     15 // 1296
#define BAND2400     16 // 2400
#define BAND3400     17 // 3400
#define BAND5760     18 // 5760M
#define BAND10G      19 // 10.368.1G
#define BAND24G      20 // 24.192G
#define BAND47G      21 // 47.1G
#define BAND76G      22 // 76.1G
#define BAND122G     23 // 122G
#define PAN_ADAPT    24 // Panadapter IF band

// Zoom level for UI control
#define ZOOMx1      0       // Zoom out the most (fft1024 @ 48K) (aka OFF)  x1 reference
#define ZOOMx2      1       // in between (fft2048)  is x2 of 1024
#define ZOOMx4      2       // Zoom in the most (fft4096 at 96K)   is x4 of 1024
#define ZOOM_NUM    3       // Number of zoom level choiced for menu system

// ------------------------  OPERATIONAL PARAMETER STORAGE --------------------------------------
//
//  Most users will not normally mess around in this section but you can edit some of the table data to refine the default to your liking.  
//  Contained here are per-band settings, User Profile settings (globals), Font type and sizes
//  buttons and indicator colors, button an label locations and more. 
//  Pretty much every global variable that controls a setting is here in a table of some sort. 
//
#define CW          3
#define CW_R        7
#define USB         1
#define LSB         0     
#define RTTY        4     
#define RTTY_R      8     
#define AM          2
#define FM          5
#define DV          23
#define DD          34
#define ATV         35

#define DATA_OFF    0
#define DATA_ON     1

#define ON          1
#define OFF         0
#define XVTR        0
#define ANT1        1
#define ANT2        2
#define ANT3        3
#define ATTN_OFF   0       // Bypass
#define ATTN_ON    1       // Turn relay on
#define PREAMP_OFF  0       // Bypass
#define PREAMP_ON   1       // Switch relay on
#define XVTR1       0       // Transverter band Slot ID
#define XVTR2       1
#define XVTR3       2
#define XVTR4       3
#define XVTR5       4
#define XVTR6       5
#define XVTR7       6
#define XVTR8       7
#define XVTR9       8
#define XVTR10      9
#define XVTR11      10
#define XVTR12      11
#define XVTR13      12
#define XVTR14      13
#define XVTR15      14
#define AGC_OFF     0       // Index to AGC Settings table
#define AGC_SLOW    3
#define AGC_MID     2
#define AGC_FAST    1
#define MIC_OFF     0
#define MIC_ON      1
#define BW0_25      0
#define BW0_5       1
#define BW0_7       2
#define BW1_0       3
#define BW1_8       4
#define BW2_3       5
#define BW2_8       6
#define BW3_2       7
#define BW4_0       8
#define BW6_0       9
#define FILT1       1
#define FILT2       2
#define FILT3       3
#define VFO_A       1
#define VFO_B       0

#define NBOFF       0
#define NB1         1
#define NB2         2
#define NB3         3
#define NB4         4
#define NB5         4
#define NB6         4

#define NROFF       0
#define NR1         1
#define NR2         2
#define NR3         3
#define NR4         4
#define NTCHOFF     0
#define NTCH1       1
#define NTCH2       2

// This group defines the number of records in each structure
#define MODES_NUM   16
#define FREQ_DISP_NUM  4
#define BANDS       25
#define XVTRS       15
#define TS_STEPS    6
#define FILTER      4
#define AGC_SET_NUM 4
#define NB_SET_NUM  7
#define USER_SETTINGS_NUM 3
#define LABEL_NUM   24      // number of labels in the table
#define NUM_CNTL_ACTIVE 25
#define NUM_AUX_ENCODERS 12
#define GPIO_ENC    1
#define I2C_ENC     2
#define GPIO_SW     3
#define NONE        0
#ifdef USE_RA8875
#define STD_BTN_NUM 59      // number of rows in the buttons table
#else
#define STD_BTN_NUM 59      // number of rows in the buttons table
#endif
// Alternative to #define XXX_BTN is use "const int XXX_BTN" or enum to create index names to the table.
// enum Button_List {FN_BTN, MODE_BTN, FILTER_BTN, ATTN_BTN, PREAMP_BTN, };
// using #define method as it is easiet to relate the purpose and more obvious which row it is mapped to.
#define PANEL_ROWS  7       // Set Number of panels + 2.  0 is disable, 1 is not used.
                            // numbers 2 and up are the panel index number (panel number -2) for the panel to display.
//  There are 6 100px wide buttons that can swap places, enabled/dispable by the function button for a row
//Anchor buttons normally stay put
//Panel 1  Fn is an anchor, the rest swap out
#define FN_BTN      0       // Swaps out buttons in bottom row.  This stays for each row.
#define MODE_BTN    1       // index to button
#define FILTER_BTN  2       // will display the current FILTER value in the button
#define RATE_BTN    3       // will display the RATE (step) value in the button
#define ATTN_BTN   4       // invoke a pop up (ON) for atte slider or MF knob adjustment
#define PREAMP_BTN  5       // On off
#define BAND_BTN    6       // Pops up Band Window.  Touch or MF can change bands
//Panel 2  These swap out
#define NB_BTN      7       // not implemented yet
#define NR_BTN      8       // will display the NR value in the button
#define NOTCH_BTN   9       // will display the NR value in the button
#define AGC_BTN     10      // will display the AGC value in the button
#define ZOOM_BTN    11      // Sets Zoom level
#define PAN_BTN     12      // Enables PAN spectrum shift
//Panel 3
#define MENU_BTN    13      // invoke a pop up (ON) for attn slider or MF knob adjustment
#define ANT_BTN     14      // Antenna switch
#define ATU_BTN     15      // not implemented yet
#define XMIT_BTN    16      // not implemented yet
#define BANDDN_BTN  17      // this will change to a copmplex button soon
#define BANDUP_BTN  18      // this will change to a copmplex button soon
//Panel 4
#define RIT_BTN     19      // not implemented yet 
#define XIT_BTN     20      // not implemented yet 
#define FINE_BTN    21      // Fine and coarse rate
#define SPLIT_BTN   22      // ON/OFF
#define DISPLAY_BTN 23      // Invoke PopUp (On), close popup (Off)
#define VFO_AB_BTN  24      // ON/(A)/OFF(B)   Will go away soon. 
//Panel 5
#define ENET_BTN    25      // turn on and off the enet data output (does not enable/disable the enet hardware) 
#define XVTR_BTN    26      // not implemented yet
#define RFGAIN_BTN  27      // Sets digital RF level
#define REFLVL_BTN  28      // Sets the Spectrum Noise floor.
#define AFGAIN_BTN  29      // Sets digital AF level
#define MUTE_BTN    30      // Audio headphone jack output on/off

// Not in a Panel
#define UTCTIME_BTN 31      // NTP UTC time when ethernet (and internet) is available 
#define SMETER_BTN  32      // Box for the Smeter.  Can be a meter for any use.  Can touch the meter to configure maybe
#define SPECTUNE_BTN 33     // Converts a touch in the spectrum window to a frequency to tune too.
#define BAND_MENU   34      // Band selection window 
#define SW1_BTN    35      // Encoder push switch entry function (Can change modes of encoder like filer mode vs rate mode on rotation, or call a function direct like change filter)
#define SW2_BTN    36
#define SW3_BTN    37
#define SW4_BTN    38
#define SW5_BTN    39
#define SW6_BTN    40
#define TOGGLE     41

//#define SPOT_BTN          // will display the NR value in the button - Not used today but a function exists.

// Band select Window buttons
#define BS_160M     35
#define BS_80M      36
#define BS_60M      37
#define BS_40M      38
#define BS_30M      39
#define BS_20M      40
#define BS_17M      41
#define BS_15M      42
#define BS_12M      43
#define BS_10M      44
#define BS_6M       45
#define BS_144      46
#define BS_222      47 // 222
#define BS_432      48 // 432
#define BS_902      49 // 902
#define BS_1296     50 // 1296
#define BS_2400     51 // 2400
#define BS_3400     52 // 3400
#define BS_5760     53 // 5760M
#define BS_10G      54 // 10.368.1G
#define BS_24G      55 // 24.192G
#define BS_47G      56 // 47.1G
#define BS_76G      57 // 76.1G
#define BS_122G     58 // 122G


// The #define button numbers act as the ID of possible owners of MF knob services
#define MFTUNE      50      // Fake button so the MF knob can tune the VFO since there is no button
                            // Make sure this does have the same value as any buttom #defne value. 50 is safe.
#define MFNONE      0       // No active MF knob client.  0 is safe.

// Our Database of settings. This is the "factory default".  A user copy will be stored in EEPROM with user changes
struct Band_Memory {
    char        band_name[20];  // Freindly name or label.  Default here but can be changed by user.
    uint64_t    edge_lower;     // band edge limits for TX and for when to change to next band when tuning up or down.
    uint64_t    edge_upper;
    uint64_t    vfo_A_last;     // remember last VFO dial setting in this band
    uint8_t     mode_A;         // CW, LSB, USB, etc
    uint8_t     filter_A;       // 1, 2 or 3 for Icoms
    uint8_t     data_A;         // LSB, USB, AM, FM modes can have DATA mode on or off.  All other radio modes data is NA.
    uint64_t    vfo_A_last_1;   // remember previous VFO dial setting in this band (bandstack)
    uint8_t     mode_A_1;       // CW, LSB, USB, etc
    uint8_t     filter_A_1;     // 1, 2 or 3 for Icoms  
    uint8_t     data_A_1;       // LSB, USB, AM, FM modes can have DATA mode on or off.  All other radio modes data is NA.
    uint64_t    vfo_A_last_2;   // remember previous VFO dial setting in this band (bandstack)
    uint8_t     mode_A_2;       // CW, LSB, USB, etc
    uint8_t     filter_A_2;     // 1, 2 or 3 for Icoms
    uint8_t     data_A_2;       // LSB, USB, AM, FM modes can have DATA mode on or off.  All other radio modes data is NA.
    uint64_t    vfo_B_last;     // moving away from per-band VFOB recall, moving to use a global sub-VFO which can be on any band.
    uint8_t     mode_B;         // CW, LSB, USB, DATA. - Depricating in favor is global Sub_VFO mode in user_settings 
    uint8_t     filter;         // index to Bandwidth selection for this band. (depricating for 905)
    uint16_t    var_filter;     // Continuously variable filter setting in Hz.
    uint8_t     band_num;       // generally the same as the index but could be used to sort bands differently and skip bands
    uint8_t     tune_step;      // last step rate on this band.  Index to Tune step table 
    uint8_t     agc_mode;       // index to group of AGC settings in another table
    uint8_t     split;          // split active or not. 0 is off
    uint8_t     RIT_en;         // RIT active. 
    uint8_t     XIT_en;         // XIT active
    uint8_t     ATU;            // enable ATU or not.
    uint8_t     ant_sw;         // antenna selector switch.
    uint8_t     preselector;    // preselector band set value.
    uint8_t     attenuator;     // 0 = button ON/OFF
    uint8_t     attenuator_byp; // 0 = bypass, 1 is attenuation hardware active
    uint8_t     attenuator_dB;  // 0 is attenuation value to set.
    uint8_t     preamp;         // 0-off, 1 is level 2, level 2
    int16_t     sp_ref_lvl;     // per band spectrum reference level.  Overides the spectrum module level by copying this value into it.
    uint8_t     bandmap_en;     // Enable in Bandmap.  Skip band if 0, include if !0.
    uint8_t     xvtr_num;       // index to Transverter Table.
    uint8_t     xvtr_IF;        // index to actual radio RF frequency to be used  (ex: BAND10M )
    uint8_t     xvtr_Dirty;     // =1 when this IF band was used. When changing to a new non-xvtr band, 
                                //    search for an IF band that matches the target band.  If "dirty" send down stored values to reset the normal band.
    uint16_t    xvtr_PwrSet;    // last used xvtr power level  
    int16_t     DialCal;        // Calibration offset correction to apply to main frequency (VFOA) - most useful for unlocked LO transverters
    uint16_t    bandDecode;     // Output pattern for band decoder per-band. 
};

struct Standard_Button {
    uint8_t  enabled;       // ON - enabled. Enable or disable this button. UserInput() will look for matched coordinate and skip if disabled.                            
    uint8_t  show;          // ON= Show key. 0 = Hide key. Used to Hide a button without disabling it. Useful for swapping panels of buttons.
    uint8_t  Panelnum;      // Panel to display this button in, if any
    uint8_t  Panelpos;      // Display order for a panel starting with 0 for the left most position.
    uint16_t bx;            // coordinates used by both touch and display systems
	uint16_t by;
	uint16_t bw;
	uint16_t bh;            // height 
	uint16_t br;            // radius of roundRect corners
    uint16_t outline_color; // used for button outline color
    uint16_t txtclr;        // used for button text color
    uint16_t on_color;      // fill color when button is ON state
    uint16_t off_color;     // fill color when button is OFF state
    uint16_t padx;          // # of pixels to pad label text horizontally shifting right to center the text 
    uint16_t pady;          // # of pixels to pad label text vertically shifting text down to center the text 
    uint8_t  label_idx;     // index to matching function label structure if there is one.  255 if not.
    char     label[20];     // Text to display for the button label. Use padx and pady to center..
};

 //UserInput() will look for a matching touch event coordinate if SHOW = ON. Will skip if SHOW of OFF and keep looking for coordinate matches.  
 //DisplayXXX() also looks at SHOW to decide whether to draw something. 
 //
struct Label {
    uint8_t  enabled;       // Not used for Labels today. Can be used for state tracking. 
    uint8_t  show;          // ON= Show key. 0 = Hide key. Used to Hide a label without disabling it.
    uint16_t x;             // coordinates used by both touch and display systems
	uint16_t y;
	uint16_t w;
	uint16_t h;             // height 
	uint16_t r;             // radius of roundRect corners (if used)
    uint16_t outline_color; // used for label outline color
    uint16_t on_txtclr;     // used for label text color
    uint16_t on_color;      // fill color when label is ON state
    uint16_t off_txtclr;    // used for label text color
    uint16_t off_color;     // fill color when label is OFF state
    uint16_t padx;          // # of pixels to pad label text horizontally shifting right to center the text 
    uint16_t pady;          // # of pixels to pad label text vertically shifting text down to center the text 
    char     label[20];     // Text to display for the label. Use padx and pady to center
};

struct User_Settings {
    char        configset_name[20]; // friendly anme for this record
    uint16_t    sp_preset;          // Sets the Spectrum module layout preset
    uint8_t     main_page;          // stores index to page settings table
    uint64_t    sub_VFO;            // Global sub-VFO (B).  Not per band like VFOA is.
    uint8_t     sub_VFO_mode;       // Mode for the Sub_VFO. Moved from per-band.
    uint8_t     usrcfgpage_1;       // index to user configuration page layout
    uint8_t     usrcfgpage_2;       // index to user configuration page layout
    uint8_t     usrcfgpage_3;       // index to user configuration page layout
    uint8_t     last_band;          // index into band memory table to recall last settings  - this might get moved out later
    uint8_t     mute;               // Current status of Mute 
    uint8_t     mic_input_en;       // mic on or off
    float       mic_Gain_level;     // 0-100 Mic gain level in percent. Applied to LineOut_TX level.
    uint8_t     lineIn_level;       // codec line in max level. Range 0 to 15.  0 => 3.12Vp-p, 15 => 0.24Vp-p sensitivity. Set this to max allowed.
    uint8_t     rfGain_en;          // Shows when RF gain setting value is displayed or not on meter face
    uint8_t     rfGain;             // 0-100 (% of lineIn level). Last used line-in setting on this band.
    uint8_t     spkr_en;            // 0 is disable or mute. 1= mono, 2= stereo. 3= sound effect 1 and so on. 255 is ignore and use current setting
    uint8_t     afGain_en;          // Shows when AF gain setting value is displayed or not on meter face
    uint8_t     afGain;             // 0-100 Last setting for unmute or power on (When we store in EEPROM). scaled to 0.0 to 1.0.
    uint8_t     lineOut_RX;         // Output Level when in Receive. (Line out to headphone jack) off = 0. Level = Range 13 to 31.  13 => 3.16Vp-p, 31=> 1.16Vp-p
    uint8_t     lineOut_TX;         // Output level when in Transmit (LineOut to TX board)
    uint8_t     enet_enabled;       // Allow initialization and operation of ethernet hardware
    uint8_t     enet_output;        // Allow ethernet data to flow (if enet is enabled)
    uint8_t     nb_en;              // Noise Blanker mode.  0 is off.  1+ is mode
    uint8_t     nb_level;           // 0 to NB_SET_NUM records in the table
    uint8_t     nr_en;              // Noise Reduction.  0 is off.  1+ is mode
    uint8_t     spot;               // Spot  0 is off.  1+ is mode
    float       rogerBeep_Vol;      // feedback beeps level  Range 0.0 to 1.0. 
    uint16_t    pitch;              // Pitch  0 is off.  1+ is mode
    uint8_t     notch;              // Notch mode.  0 is off.  1+ is mode
    uint8_t     xmit;               // xmit state.  0 is off.  1+ is mode
    uint8_t     fine;               // Fine tune state.  0 is off.  1+ is mode
    uint8_t     VFO_last;           // Track the last known state of the VFO A/B feature - either on A or B
    uint8_t     zoom_level;         // 0 - 2.  Zoom level memory.  x1, x2, x4 
    uint8_t     pan_state;          // 0 = OFF, 1 = ON
    uint8_t     pan_level;          // 0-100 converts to pan range of -0.50 to 0.50 for the pan memory.  0  is centered.
    uint8_t     RIT_ts;             // Last used Tune step size for RIT. Uses tstep table like regular VFO does.
    uint8_t     XIT_ts;             // Last used Tune step size for XIT. Uses tstep table like regular VFO does.
};

struct Frequency_Display {
    uint16_t bx;        // X - upper left corner anchor point
    uint16_t by;        // Y - upper left corner anchor point
    uint16_t bw;        // width of whole box
    uint16_t bh;        // height of whole box
    uint16_t br;        // radius of corners
    uint16_t bs;        // spacing between the VFO numerals
    uint16_t bm;        // marker spacing between VFO letter and digits  (A: 123.456.789)
    uint16_t ol_clr;    // VFO box outline color
    uint16_t box_clr;   // Color of outline around entire VFO region
    uint16_t bg_clr;    // background color
    uint16_t txt_clr;   // color of Active VFO numbers
    const ILI9341_t3_font_t txt_Font;    // size of Active VFO Label text
    uint16_t TX_clr;    // Color when active VFO is in transmit
    uint16_t padx;      // horizonal padding from left side of box
    uint16_t pady;      // vertical padding form top of box
};

struct AGC {
    char        agc_name[10];
    uint8_t     agc_maxGain;
    uint8_t     agc_response;
    uint8_t     agc_hardlimit;    // autoVolumeControl(maxGain, response, hardLimit, threshold, attack, decay);
    float       agc_threshold;
    float       agc_attack;
    float       agc_decay;
};

// Noise Blanker Settings
struct NB {
    char            nb_name[10];        // A friendly name for display
    float           nb_threshold;       // threshold recommended to be between 1.5 and 20, closer to 3 maybe best.
    uint16_t        nb_nAnticipation;   // nAnticipation is 1 to 125
    uint16_t        nb_decay;           // Decay is 1 to 10.
};

// Zoom Level Info table
struct Zoom_Lvl {
    char            zoom_name[10];        // A friendly name for display  x1, x2 or x4, etc
    uint8_t         zoom_factor;         // Factor equates to 1, 2 ,or 4
};

struct Filter_Settings {
    char        Filter_name[12];   // display name for UI
};

struct TuneSteps {
    char        ts_name[12];    // display name for UI
    char        ts_units[4];    // units for display HZ or KHz
    uint16_t    step;           // step size in HZ
    uint8_t     pref_mode;      // preferred mode when enabled (future use)
};

struct Modes_List {
    uint8_t     mode_num;
    char        mode_label[8];
    uint8_t     Width;             // bandwidth in HZ - look up matching width in Filter table when changing modes
    uint8_t     data;
};

enum Label_List {BAND_LBL, MODE_LBL, FILTER_LBL, RATE_LBL, AGC_LBL, ANT_LBL, ATTN_LBL, PREAMP_LBL, ATU_LBL, RIT_LBL, XIT_LBL, FINE_LBL, NB_LBL, NR_LBL, NOTCH_LBL, SPLIT_LBL, MUTE_LBL, XMIT_LBL, XVTR_LBL, REFLVL_LBL, SPOT_LBL, ZOOM_LBL, PAN_LBL, DATA_LBL};
/*
#define BAND_LBL    0       // Band label (if used)
#define MODE_LBL    1       // index to button
#define FILTER_LBL  2       // will display the current FILTER value in the button
#define RATE_LBL    3       // will display the RATE (step) value in the button
#define AGC_LBL     4       // will display the AGC value in the button
#define ANT_LBL     5       // Antenna switch
#define ATTN_LBL   6       // invoke a pop up (ON) for attn slider or MF knob adjustment
#define PREAMP_LBL  7       // On off
#define ATU_LBL     8       // not implemented yet
#define RIT_LBL     9       // not implemented yet 
#define XIT_LBL     10      // not implemented yet 
#define FINE_LBL    11      // Fine and coarse rate
#define NB_LBL      12      // not implemented yet
#define NR_LBL      13      // will display the NR value in the button
#define NOTCH_LBL   14      // will display the NR value in the button
#define SPLIT_LBL   15      // ON/OFF
#define MUTE_LBL    16      // On off
#define XMIT_LBL    17      // not implemented yet
#define XVTR_LBL    18      // not implemented yet
#define REFLVL_LBL  19      // not implemented yet
#define SPOT_LBL    20      // not implemented yet
#define ZOOM_LBL    21      // not implemented yet
#define PAN_LBL     22      // not implemented yet
#define DATA_LBL    23      // Radio current mode has DATA ON or OFF
*/

// This stores type and status info for auxillary encoders (VFO is separate).  
// This contains the mapping read in from config to assign differnt types of encoders to 'control slots' for aux encoders
// Each record is for 1 encoder.  Today there is support for 6 aux encoders (plus 1 VFO) so 6 records.
// Can be any mix of type.  
struct EncoderList {
    uint8_t     type;       // encoder type, 0 = i2c connected , 1= GPIO connected
    uint8_t     id;         // unique of encoder. Set value equal to the encoder_list row number (0-7 typical)
    uint8_t     enabled;    // encoder is enabled
    uint8_t     default_MF_client;   // The default "client" assignment for the the MF Knob.  0 = none, 1 = MFClient assign function    
    uint8_t     role_A;     // The "client" action for one of the encoder knobs - Set to 0 if no encoder is wired up   
    uint8_t     a_active;   // track if role A is active
    uint8_t     role_B;     // The "client" alternate action for one of the encoder knobs - Set to 0 if no encoder is wired up   
    uint8_t     tap;        // The "client" action for one of the encoder switches when tapped- Set to 0 if no encoder is wired up
    uint8_t     press;      // The "client" action for one of the encoder switches when pressed (longer push) - Set to 0 if no encoder is wired up
};   

/* 
ICradio_test - a_Defines.h
*/

// Global compile switches ===================================================================================

//#define VERSION_STRING "CIVbusLib ICradio_selRadioTest V0_1 July 2024"

// common switches -----------------------------------

// if defined, debug messages on the serial line will be generated
#define debug 

// some general defines ----------------------------------

enum onOff_t:uint8_t {
	OFF1 = 0,
	ON1  = 1,
    UNDEF
};

enum keyPressed_t:uint8_t {
    NO_KEY_PRESSED 		= 0,
    KEY_EIN_PRESSED,
    KEY_AUS_PRESSED,
    KEY_VOICE_PRESSED,
    KEY_DATA_PRESSED,
    KEY_MODE_PRESSED,
    KEY_FREQ_PRESSED,
    KEY_MODMODE_PRESSED,
    KEY_HELP_PRESSED,
    KEY_LOG_PRESSED
};

// For CIV commands

// translation of the radio's general mode
const String ModeStr[3] = {
  "MODE_NDEF",
  "MODE_VOICE",
  "MODE_DATA"
};

// states of radio's DC-Power (on/Off State)
const String radioOnOffStr[6] = {
  "RADIO_OFF",
  "RADIO_ON",
  "RADIO_OFF_TR",     // transit from OFF to ON
  "RADIO_ON_TR",      // transit from ON to OFF
  "RADIO_NDEF",       // don't know
  "RADIO_TOGGLE"
};

// clear test translation of the modulation modes
const String modModeStr[MODES_NUM+1] = {
  "LSB   ", // 00 (00 .. 08 is according to ICOM's documentation) 
  "USB   ", // 01
  "AM    ", // 02
  "CW    ", // 03
  "RTTY  ", // 04
  "FM    ", // 05
  //"WFM   ", // 06
  "CW-R  ", // 07
  "RTTY-R", // 08
  "DV    ", // 09 (Note: on the ICOM CIV bus, this is coded as 17 in BCD-code, i.e. 0x17)
  "DD    ",
  "ATV   ",
  "LSB-D ",
  "USB-D ",
  "AM-D  ",
  "FM-D  ",
  "NDEF  "  // 10
};

// clear text translation of the Filter setting
const String FilStr[4] = {
  "NDEF",
  "FIL1",   // 1 (1 .. 3 is according to ICOM's documentation)
  "FIL2",
  "FIL3"
};

// repeat time of the baseloop actions in ms
#define BASELOOP_TICK 10 

// Debugging ...

#ifdef debug
  #define SET_TIME_MARKER1 G_timemarker1 = micros();
  #define EVAL_TIME_MARKER1 G_timemarker1a = micros();Serial.print("t1:  ");Serial.println(G_timemarker1a-G_timemarker1);
#endif

#ifndef debug
  #define SET_TIME_MARKER1
  #define EVAL_TIME_MARKER1
#endif

#endif //_IC_905_H_
