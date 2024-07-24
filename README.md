### USB Band Decoder for Icom IC-905

Description:

Project:

This project is a CI-V decoder and remote control UI for the IC-905 control head.  A Teensy 4.0 or 4.1 USB host port connectes with the radio's USB-C serial port CI-V bus to extract frequency values to operate band decode outputs on GPIO pins - aka PTT breakout box and band decoder. 

It can run as a minimal headless package on a Teensy 4.0 for the purpose of a band decoder to operate antenna relays and power amps. Fit it with a RA8875 or RA8876 touchscreen display and optional encoder and buttons, you have a physical remote control head with a 4.3" (800x480px) or a 7" touchscreen (1024x600px). It can also act as a Transverter frequency and band select display with custom IF offset and dial calibration correction.  This is likely more useful for radios like the IC-705 or 7300, 9100 that are often used as 28MHz or 144 and 432Mhz IF rigs but in theory the IC-905 can be a very stable and capable IF for 6M and band > 10Ghz also.  A PC can still be connected to the radio through this decoder for both CI-V and GPS NMEA data. Useful for running WSJT-X.

The leveraged Teensy SDR project is at https://github.com/K7MDL2/KEITHSDR
The RF Wattmeter and Band decoder project is at https://github.com/K7MDL2/RF-Power-Meter-V1

Here is a compact 4.3" SDR chassis with touch screen and 2 encoders that can double as a Remote controller and band decoder box.  A 7" is seen down the page.

![Compact SDR chassis](https://github.com/K7MDL2/ICOM_IC-905_CIV/blob/main/Pictures/Compact%204.3inch%20Teensy%20SDR%20chassis.png)


Why would some combo of knobs, buttons or touchscreen be useful you ask?

Due to the limited front panel size and few buttons, several features require navigating multiple menu levels or at least 2-3 button pushes. Some examples of what a custom conmtrol head can do:

- Put all combos of Mode (with data and without as appropriate) into a single list, scrollable with a quick encoder twist or button touches.  Same for filters, AGC, preamp, ATTN, tune rate.
- Favorite memory buttons on front panel or in a 1 touch window
- CW message buttons on front panel or in a 1 touch window
- Band selection on front display or in a 1 touch window 
- Transverter band selection and display, tied into the band selection.  PC connec

Some things like full date and time, GPS grid square (calculated from the radio CI-V bus location+time+date message on the IC-905) an be isplayed on teh front screen in full.  

Most radio operational controls state are indicated if not controlled, on the front screen.

If you built a K7MDL Teensy SDR, you can reuse the same hardware, just connect the radio to the USB Host port like the RS-HFIQ is.

The IC-905 connects to the Teensy host port which today cannot handle audio, so only the serial ports are dealt with.  Both serial ports are passed on to an optional PC USB connection.

For full audio and serial control you can use the IC-905 LAN connection to a PC.  The LAN operates in parallel with the USB OK.


Challenge:

The IC-905 does not have hardware outputs to select or key band specific antenna relays or amps.  There is only 1 PTT (aka SEND) port supporting up to 6 bands.  In the radio's config screens you can enable SEND for each band but on the wire there is no way to differentiate which band is active on the SEND port.  Further, 144, 432, and 1296 bands are combined on 1 N connector.  The 2.3, 5.7, and 10G SMA connectors are separate.  Keying 1 of 6 possible band amplifiers, or routing coax to a particular amp is a challenge.  While a PC could be used, many field operations do not use a PC so a small low power standalone solution is desired.

Solution:

A small CPU connects to the IC-905 USB port to control IO pins.  This Arduino code runs on a on small Teensy 4.0 or 4.1 CPU module. The Teensy 4.X CPUs have a USB2 host port which connects to the radio. It can have multiple serial ports but cannot handle audio today.

The IC-905 USB port has 2 serial ports and USB audio on it.  One is CI-V for CAT control, the 2nd is configurable, usually set to the GPS ASCII data output useful to set time on a PC. This project connects to the 2 serial ports and examine only the CI-V serial data for any frequency messages. The frequency info is then translated to a pattern on the GPIO pins to control equipment. You can set configuration at compile time to set the patterns you need for your equipment. I expect top add a minimal config and monitor UI for this later. Be sure to consider the limits of the 3.3V IO pins and use driver interfaces as required.

As an option, a PC can be connected to the Teensy main USB port. The 2 radio serial ports are passed on to the PC. The PC connection is not required, this will operate standalone. The T4 CPU can be packaged in a very small box with a short USB cable to the radio. Inside the box would be suitable buffers added to control some combo of relays and amps. Long wires may then be run to the equipment including all the way up to the antenna, wherever that may be.

Can generate PTT (aka SEND) from the CI-V messages and combine in software for per-band PTT outputs. I have seen some say they identified and broke out the 'SEND" or PTT signal on the RF unit cable but still need band specific filtering.  Somehow the RF Unit needs to know what band to operate on so some sort of data exchange is likely over the cable from the control head. I have never looked at it. Ideally the RF Unit would have band decode IO signals available using some of its spare accessory connector pins. The unused RF Unit acc pins are noted as "Do Not Connect" and are connected to internal coms and power for the 10Ghz and future Icom transveters.  

I am using a heavily modified fork from Icom CIV library https://github.com/WillyIoBrok/CIVmasterLib. My updated fork is at https://github.com/K7MDL2/ICOM_IC-905_CIV. I added support for the IC-905 (bands > 2.4GHz), removed high level functions (they seemed too slow) and converted it to use a lookup table approach that is easy to add, remove, or change any command message. The library of interface fucntions to connect to my Teensy SDR control and display system is being developed and most of the normal used functions are working now. Work needs to be done to make it work with other CI-V radios in an wasy to configure way. The IC-905 uses a longer frequency message to support 10Ghz and the frequency variables needs to change form 32 to 64bit. For basioc frequency messages the message length changes between 5 and 6 bytes, 6 for bands 10G and higher.

Usage:

Plug the radio USB cable into the T4 USB host port. Connect your amps and relays via suitable driver interfaces. Configure IO (Work in queue) to create the correct output IO pin pattern you need for your relays and transverters.

If using digital modes on a PC and audio is desired, use the LAN connection. It will operate at the same time as the USB port providing full control with serial, spectrum and audio. The USB connected box will then only handle the band decoding tasks. I use wfView at https://wfview.org/.

This is in active devlopment as of July 2024 and tested on my bench with a IC-905 with the 10Ghz transverter. Feel free to open Issues on the GitHub repository. I hope to soon borrow a IC-705 and use teh BT serial connection. The driving demand for the 705 is to use it as an IF rig, often mixed with its native bands so full direct and transverted frequency display and easy transverter band select is desired.

------------------------------------------------------------------------------------------------------------------

Dev update details:   This is work in progress, changes can happen fast!   You can scan the details below for the major work items checked in.

July 1 2024 formal start

Pulled in my Teensy SDR code and stubbed out audio and spectrum and PLL related code. Running the code on one of my Teensy SDR chassis (7" for now) since it has all the hardware needed.  I copy the extracted CI-V bus radio frequency into VFOA on the SDR chassis for display.  The encoders and switches change things on the display thinking it is still an SDR, but they have no effect today. Next steps are to extract other radio settings of interest and pull in the Band decoder project code. Key features I want first up and band menu select, VFO A and B display and control with the VFO knobs, display mode, filter, and then add ability to control the radio more fully. Buffered IO hardware will be added to control antennas, transverters and amps.

In the case of at least the IC-905, can extract time from the USB ch 'B' NMEA data strings and update clock and Grid square on my display.  Currently time is form PC over USB when programmed or when ENET is enabled, NTP.  The GPS will work offline so is desireable to use.  Possible the grid square is already in the CI-V but since they show it on the 905 info screens.

Showing stripped down SDR screen with live VFOA from CI-V bus (USB ch'A')

![Showing stripped down SDR screen with live VFOA from CI-V bus (USB ch'A')](https://github.com/K7MDL2/ICOM_IC-905_CIV/blob/main/Pictures/20240703_234200.jpg)

Band Select screen configured for IC-905 supported bands. Can be configured for 160M through 122Ghz for other radios with or without transverters

![Band Select screen configured for IC-905 supported bands. Can be configured for 160M through 122Ghz for other radios with or without transverters](https://github.com/K7MDL2/ICOM_IC-905_CIV/blob/main/Pictures/20240703_234354.jpg)

GPS NMEA strings from USB ch 'B'

![GPS NMEA strings from USB ch 'B'](https://github.com/K7MDL2/ICOM_IC-905_CIV/blob/main/Pictures/905%20NMEA%20data.jpg)


July 5, 2024

Significant rework done though still quite messy from pulling in differnt project's code.  Lots to weed out and clean up.

You can now use touch or an encoder to control VFOA and change bands. So VFOA is now 2 way.  Change the radio dial and this box's VFOA will stay synced.  Mode and Filters are received but not yet dealt with fully.

This code update replaces the previous CI-V library with CIVMasterLib library found at https://github.com/WillyIoBrok/CIVmasterLib. It was not that bad to swap out but getting the send VFO frequency code to work was a challenge.  It wants const uint8_t values in the writemsg fucntion and bytes in reverse order with a length byte at the front. Changes had to be made to the library and my frequency formatting code to account for the 905 using a 6 byte frequency message on bands 10GHz and higher, 5 bytes on lower bands.  Also replaced several unsigned longs with uint64_t to handle frequencies above 2.4Ghz.  Some of the higher level library fucntion I tried were causing slow perf so for now I am sticking with building my own as I need them.  The basics stuff seems to work fast enough. 

Code is in place to pass theough both direction CI-V data so you can run WSJT-X.  If you you use the encoder to change frequency it causes WSJT-X to panic and offer a retry. The long term answer is probably to have this box be a separate CI_V address but more study is required.  The library does have some code to handle multiple radios on the bus.

In SDR_Data.h I edited the Bandmem table defaults to turn off transverter support on IC-905 native bands.  I had to disable the SD Card coifig as it was overriding the changes. Need to sort that out.

Other than the removed spectrum and audio related functions, the screen is not updated but should work well as-is until time for a new layout.  The database allows for multiple layouts so just takes time to figure new xy cordinates and sizes.

Still have to cut in band decoder logic and UI though I may do a simpler version instead.

Hooking this box up is simple.  Plug a USB Type-A cable into the Teensy 4.X host board. The other Type-C end goes into the IC-905. The IC-705, IC-9700, and maybe other radios should work by changing the CI-V addresses in the code.  Will extract that to a config file later.  Optionally you can have a PC connected to the main Teensy USB port (micro connector), used for programming and debug but there are 3 USB serial ports active.  The 2nd is for CAT to programs like WSJT-X.  The 3rd is the IC-905 GPS NMEA data streaming.  I use NMEATime2 to use this data to compute grid square and keep the PC time sync'd.

In theory it can work without any screen or encoders and act just as a band decoder once that code is added.  A read-only Screen at minimum, could be an OLED, or character LCD even. I think the touch screen, switches, and encoders will offer good value for things like CW macros, quick bands changes, more accurate touch tuning, less menu hunting.  Things like external amp temp and power can be read and displayed, perhaps over hardware serial or i2C bus connections.

On my GitHub is a fork of the CIVMasterLib repo @ https://github.com/K7MDL2/CIVmasterLib with my modifications for the 905 and bands > 2GHz.  You can copy them to your normal Arduino libraries folder.  I expect they will need further udpates as I go, especially as I add higher level command support.

7/8/2024 : Made several fixes.  Can now bidirectionally tune from either end on all bands including 10Ghz, followsws the configured band map.  The SD card config read at startup is bypassed until dev changes are stabilized more.  Next up are getting mode and filter to align.  There is a long list of settings and status stuff that I could sync with the radio (both directions) given time to code it up, mostly now a matter of priority order vs time vs usefulness.

7/11/2024 : Moved CIV function calls into CIV.cpp/CIV.h.  Can extend comms with the radio in the CIV file and unclutter the main .ino file some.
Now decoding radio and filter setting messages from the radio. The mode is now displayed on touchscreen.  Will do the same for filter and be send the mode and filter to the radio.

7/18/2024 : Mode now stays in sync between radio and remote, either side can initiate change and it is remembered. ThE remote stores the last used mode and frequency and other parameters on each band change to the SD card. it is read upon startup and each band change will set the radio. If you just send a new band frewuency to the radio the radio will not change the mode so theremtoe must remember it. Using the bandstack value for each band is probably a way to make the radio's last used mode and filter apply, let the remote follow. For noW the radio will follow the remote. If you change bands, freq, or mode (or any combo like on band change) from the radio side, the remote will follow. Next up is to get the filters working the same. One can go on and get every radio parameter but I think I will focus on the most used stuff, maybe some memories.  But first need to set up the band decoder GPIO outputs per band for relays, PTT breakout.

7/20/2024 : Mode and Filter control sync to radio both directions.  Filters changed to FIL1, FIL2, FIL3. Mode list on remote now has XXX-D for appropropriate modes (LSB-D, USB-D, AM-D, FM-D). Just spin the encoder to set the radio to the last known mode and filter and data. DATA mode status icon updates anytime a -D mode is active. Modified the bandmem table to record filter anbd data for each VFO and mode (3 per band). Also for the current band the modelist table stores the current filters associated with each mode. The current mode/filter/data is stored per band. When possible the radio mode/filt/data is retrieved first. At startup I read all 3 band stack filters for each of the radios bands. Any transverter bands will use the table defaults to start with. This helps populate the per band settings to be closer to the radio stored values. This will need to be extended for IC705.  Other things like the bandstack message frequency digits (5 vs 6 bytes long) for will change as well.

7/23/2024 : Major fixes for ATTN, Preamp operation and their buttons and icons. Turning on one turns off the other. Much work was put into tracking down CI-V bus message errors. Some required delays, most are because they are not applicable in a certain scenario of band and/or mode. On bands > 1296 Preamp and ATTN are not used. For modes some filters or AGC settings are not allowed. Changing modes, filters, or AGC now updates relative to each other and for band. Basically you have a set of allowed values per mode, per band. Almost all settings come from the radio now, not the local dB. They are still stored on the SD card (if available or enabled). At startup the major parameters are collected from the radio. Some radio side changes like AGC, Preamp, and ATTN do not send out a CI=V message on change, you have to query for it. I currently only check on band change. Ideally periodic queries for these type settings would be sent. The Clock can display at local or UTC time. Time and offset comes from a CI-V query along with Lat and Long. Time displayed at UTC or Local is configurable in RadioConfig.h with #define UTC. Can calculate grid square and display it somewhere.   

Added GPIO band decoder output on 8 pins following the Elecraft BCD pattern on the lower 4 bit (0-3). Bit 4 signals HF or VHF bit group (0-3) since some are the same between them.  Configure these pins to be any pattern you want in new RadioConfig.h section with #defines like this:

    #define DECODE_BAND160M     (0x01)   //160M 
    #define DECODE_BAND80M      (0x02)    //80M
    #define DECODE_BAND60M      (0x00)    //60M

and so on to 122G band.

The GPIO pins are also defined in RadioConfig.h Look for a section that looks like this:

    #define BAND_DECODE_OUTPUT_PIN_0    GPIO_SW4_PIN     // bit 0
    #define BAND_DECODE_OUTPUT_PIN_1    GPIO_SW5_PIN     // bit 1
    #define BAND_DECODE_OUTPUT_PIN_2    GPIO_SW6_PIN     // bit 2

The IO pins number themselves are assigned in the motherboard definitions also in RadioConfig.h - Here is a small bit of it:

	#define PTT_OUT1      		   41   	// buffered GPIO digital output pin number for external PTT.  Typically LO (GND) = TX, HI = RX.
	#define GPIO_SW1_PIN            3   	// pin assignment for external switches. When enabled, these will be scanned and software debounced
	#define GPIO_SW2_PIN            4   	// Rev 2 PCBs have an 8x2 header U7 that has Teensy GPIO pins 0-7 on it.  
	#define GPIO_SW3_PIN            5		  // Pins 0 and 1 I try to reserve for hardware serial port duties so assigning pins 2 through 7.

In my shack I can wire this directly to my existing opto-coupler shack band decoder and program that do do about anything with over 20 opto-coupler outputs. For simpler tripod microwave outings with the IC-905 I can package a headless T4.0 in a snmall case and add a suitable interface transistors to drive antenna relays and switch PTT to the right amp.

Tested the Band enable setting in RadioConfig.h to set 6M on.  Here is a section whre I enabled 6M traneveter band.  No need to change the band map value in the bandmem table.  You still need to set the transverter IFband field however (for now).

    #define ENABLE_12M_BAND   0
    #define ENABLE_10M_BAND   0
    // These are transverter bands common to all RF hardware that covers HF bands to 30MHz.
    // The default IF is 10M band defined in the bandmem table in SDR_DATA.h
    #define ENABLE_6M_BAND    1  // if you hardware does 6M then edit the bandmem table in SDR_DATA.h
    #define ENABLE_144_BAND   1
    #define ENABLE_222_BAND   0
    #define ENABLE_432_BAND   1

In SDR_Data.h I changed the IF band from NONE to BAND1296 which the software treats as a transverter band and tune the radio on 1296 but display and reports at 50MMhz. 6M shows in the Band Select and appears properly when cycling up and down bands. VFO is stored separately from the direct 1296 band. So this seems to do exactly as desired.  In the SDR_Data.h bandmem table, besides the xvtr_IF value (BAND1296 here) there is also a dialcal value where you can set any small corrections due to repeatable transverter crystal inaccuracy.  There is a power setting also which is not implemented yet.  Some handy items to finish volume, RF Gain, and RF power for each band.  The last field in each row is a band decode output pattern. The RadioConfig.h pattern described above is written into that field thus stored on SD card also. 
                                                                              
    { "10M",    28000000,    29600000,    28074000, USB, FILT1, DATA_OFF,     28200000, USB, FILT1, DATA_OFF,     29400000,  USB, FILT2, DATA_OFF,     28200000, USB, BW4_0, 4000,  BAND10M,  1, AGC_SLOW,OFF,OFF,OFF,OFF,ANT1, 9,  ATTN_OFF,  0,    0,  PREAMP_ON,   5,  OFF,  NONE,    NONE,    100,   -0,  0xFFFF},
    {  "6M",    50000000,    54000000,    50125000, USB, FILT1, DATA_OFF,     50313000, USB, FILT1, DATA_OFF,     50100000,  CW,  FILT2, DATA_OFF,     50313000, USB, BW3_2, 3200,  BAND6M,   1, AGC_SLOW,OFF,OFF,OFF,OFF,ANT1,10,  ATTN_OFF,  0,    0,  PREAMP_ON,   5,  OFF,  XVTR1,   BAND1296, 30,   -0,  0x0001},
    { "144",   144000000,   148000000,   144200000, USB, FILT2, DATA_OFF,    144200000, USB, FILT1, DATA_OFF,    144200000,  CW,  FILT1, DATA_OFF,    144200000, USB, BW3_2, 3200,  BAND144,  1, AGC_SLOW,OFF,OFF,OFF,OFF,ANT1,10,  ATTN_ON,   1,    3,  PREAMP_ON,   5,  ON,   NONE,    NONE,     10,   -0,  0x0002},
    { "222",   222000000,   225000000,   222100000, USB, FILT2, DATA_OFF,    222100000, USB, FILT1, DATA_OFF,    222100000,  CW,  FILT1, DATA_OFF,    222100000, USB, BW3_2, 3200,  BAND222,  1, AGC_SLOW,OFF,OFF,OFF,OFF,ANT1,10,  ATTN_OFF,  0,    0,  PREAMP_OFF,  5,  OFF,  XVTR3,   BAND10M,  10,  -10,  0x0004},
    

When changing to an IF band the database values are sent. Today this is mode, filter, data, agc, preamp, attn and vfo A.  Apart from VFO, as a consequence of making the radio the master source, these parameters are normally read from the radio on band change ignoring the database copy. When a Xvtr band is active is marked as "Dirty". When the same band as the IF band is later called up, the database is searched to see if that band is an IF band, and if so, is the Dirty value set. If true, then then stored values are sent to radio to reset the band values which will still have the Xvtr values.  Ths works when teh radio beahves the same but 2 diffent time I have seen teh mod mode not change with band changes.  So to keep things straight, the remote is the master. At startup the Band Stack values are collected and they can be leveraged by the remote Band Stack. In general, the stored values now rule for all bands Xvtr or not, major paramters are always sent to the radio.  This seems to work pretty good, fast and no bus errors so far now that th einvalid settings combos have been sorted.

 
