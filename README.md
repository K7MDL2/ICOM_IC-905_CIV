### USB Band Decoder for Icom IC-905

Description:

Project:

This project is a CI-V decoder and remote control UI for the IC-905 and IC-705 control head.  A Teensy 4.0 or 4.1 USB host port connects with the radio's USB serial port CI-V bus channel 'A' to extract frequency data to operate band decode outputs on GPIO pins for antenna, amp, and transverter selection, and pass through PTT to designated GPIO pin(s) per band.  One way to describe this is a "PTT breakout box and band decoder".   I have only tested this with teIC-905 and IC-705 so far.  It should work with other modern CI-V radios like the IC-7300, IC-9100, and IC-9700.  Some minor features like GPS/Time function may need tweaking for older models.

Transverter bands are supported with predefined ham bands from 160M to 122Ghz. with custom IF offset and dial calibration correction.  Equipped with a display and optional switches and/or encoders, it can act as a Transverter frequency and band select display.  An upcoming feature is to translate the radio frequency on the CI-V CAT bus to send the transverter frequency for logging and digital mode programs. This is likely more useful for radios like the IC-705 or 7300, 9100 that are often used as 28MHz or 144 and 432Mhz IF rigs but in theory the IC-905 can be a very stable and capable IF for 6M and band > 10Ghz also.  A PC can still be connected to the radio through this decoder for both CI-V and GPS NMEA data. Useful for running WSJT-X.

It can run as a minimal headless package on a Teensy 4.0 for the purpose of a band decoder to operate antenna relays and power amps. Fit it with a RA8875 or RA8876 touchscreen display and optional encoder and buttons, you have a physical remote control head with a 4.3" (800x480px) or a 7" touchscreen (1024x600px).  A Teensy 4.1 is recommended for easier access to more GPIO pins and the USB host port pins without need for a T4.0 breakout board.  To try this out, cut a USB2 extension cable and crimp on a 5 pin flat connector on the USB2 female side cable and plug it into a T4.1.  

I leveraged my Teensy SDR project at https://github.com/K7MDL2/KEITHSDR sicne the hardware is the same for the display versoin usage. The UI is the same minus the spectrum window.  I will also be leveraging code from my RF Wattmeter and Band decoder project is at https://github.com/K7MDL2/RF-Power-Meter-V1.  This will permit configuring complex Band decoder IO using a Python desktop app over ethernet or USB connection.

Here is a compact 4.3" SDR chassis with touch screen and 2 encoders that can double as a Remote controller and band decoder box.  A 7" is seen down the page.

![Compact SDR chassis](https://github.com/K7MDL2/ICOM_IC-905_CIV/blob/main/Pictures/Compact%204.3inch%20Teensy%20SDR%20chassis.png)


Why would some combo of knobs, buttons or touchscreen be useful you ask?

Due to the limited front panel size and few buttons, several features require navigating multiple menu levels or at least 2-3 button pushes. Some examples of what a custom conmtrol head can do:

- Put all combos of Mode (with data and without as appropriate) into a single list, scrollable with a quick encoder twist or button touches.  Same for filters, AGC, preamp, ATTN, tune rate.
- Favorite memory buttons on front panel or in a 1 touch window
- CW message buttons on front panel or in a 1 touch window
- Band selection on front display or in a 1 touch window 
- Transverter band selection and display, tied into the band selection.  PC connec

Some things like full date and time, GPS grid square (calculated from the radio CI-V bus location+time+date message on the IC-905) an be displayed on the front screen in full.  

Most radio operational control state are indicated, if not controlled, on the front screen.

If you built a K7MDL Teensy SDR, you can reuse the same hardware, just connect the radio to the USB Host port like the RS-HFIQ is.

The IC-905/705/XXXX model radio connects to the Teensy host port which today cannot handle audio, so only the serial ports are dealt with.  Both serial ports are passed on to an optional PC USB connection.

For full audio and serial control you can use the IC-905 LAN connection to a PC.  The LAN operates in parallel with the USB OK.  Same should apply for the 705 WiFi.


Challenge:

The IC-905 and other recent models do not have hardware outputs to select or key band specific antenna relays or amps.  There is only 1 PTT (aka SEND) port supporting many bands.  In the radio's config screens you can enable SEND for each band but on the wire there is no way to differentiate which band is active on the SEND port.  Further on some models like teh 905 and 705 many bands, like 144, 432, and 1296 on teh 905, are combined on 1 RF connector.  For the 905 the 2.3, 5.7, and 10G SMA connectors are separate.  Keying 1 of many possible amplifiers, or routing coax and band-specific PTT to a particular amp is a challenge.  While a PC could be used, many field operations do not use a PC so a small low power standalone solution is desired.

Solution:

A small CPU connects to the USB port to control IO pins.  This Arduino code runs on a on small Teensy 4.0 or 4.1 CPU module. The Teensy 4.X CPUs have a USB2 host port which connects to the radio. It can have multiple serial ports but cannot handle audio today.

The USB port has 2 serial ports and USB audio on it.  One is CI-V for CAT control, the 2nd is configurable, usually set to the GPS ASCII data output useful to set time on a PC. This project connects to the 2 serial ports and for band decoding, looks at the CI-V serial data for any frequency messages. The frequency info is used to determine the band and associated custom pin patterns to set on GPIO pins to control equipment.

You can set configuration at compile time to set the patterns you need for your equipment. I expect to add a minimal GPIO config and monitor UI for this later or use my existing Python desktop app to config the pins like I do on my big band decoder project. For now it is configured with several #defines. Be sure to consider the limits of the 3.3V IO pins and use driver interfaces as required.  

PTT state from radio side operation (pressing the mic key for example) is tracked via CI-V polling. The decdoer can send PTT to the radio using the USB Host port serial DTR line. PTT is routed to configured GPIO pins to act as a PTT breakout box. External PTT can come from the UI, or switch.

As an option, a PC can be connected to the Teensy main USB port. The 2 radio serial ports are passed on to the PC. The PC connection is not required, this will operate standalone. The T4 CPU can be packaged in a very small box with a short USB cable to the radio. Inside the box would be suitable buffers added to control some combo of relays and amps. Long decoder output wires may then be run to the equipment including all the way up to the antenna, wherever that may be.  In my shack I have a full-featured band decoder located remotely near the antennas monitored by ethernet and controlled by radio's  band decoder output pins (and PTT).  Here the CI-V breakout will provide the same 4 wire BCD (or any custom pattern) signals as many Yaesu, Kewnwood, and Elecraft radios do. 

Ideally the RF Unit would have band decode IO signals available using some of its spare accessory connector pins. The unused RF Unit acc pins are noted as "Do Not Connect" and are connected to internal comms and power for the 10Ghz and future Icom transverters.  If one knew the 10GHz transverter comm protocol it might have the band info and combined with SEND line state have the needed band decoder input for PTT breakout per band, at the RF unit.

I am using a heavily modified fork from Icom CIV library https://github.com/WillyIoBrok/CIVmasterLib. My updated fork is at  https://github.com/K7MDL2/CIVmasterLib. I added support for the IC-905 (bands > 2.4GHz), removed high level functions (they seemed too slow) and converted it to use a lookup table approach that is easy to add, remove, or change any command message. The library of interface functions to connect to my Teensy SDR control and display system is being developed and most of the normal used functions are working now. Work needs to be done to make it work with other CI-V radios in an wasy to configure way. The IC-905 uses a longer frequency message to support 10Ghz and the frequency variables needs to change from 32 to 64bit. For basic frequency messages, the message length changes between 5 and 6 bytes, 6 for bands 10G and higher.

Usage:

Plug the radio USB cable into the T4 USB host port. Connect your amps and relays via suitable driver interfaces. Configure IO in RadioConfig.h to create the correct output IO pin pattern you need for your relays, PTT, and transverters.

If using digital modes on a PC and audio is desired, use the LAN connection. It will operate at the same time as the USB port providing full control with serial, spectrum and audio. The USB connected box will then only handle the band decoding tasks. I use wfView at https://wfview.org/.

This is in active devlopment as of July 2024 and tested on my bench with a IC-905 with the 10Ghz transverter. Feel free to open Issues on the GitHub repository. I am borrowing an IC-705 and will use both the USB and BT serial connections. The driving demand for the 705 is to use it as an IF rig, often mixed with its native bands so full direct and transverted frequency display and easy transverter band select is desired.  This needs a display.  The 905 can likely jsut use the headless packaging, maybe add a small 1" OLED display.  I can be USB powered.

------------------------------------------------------------------------------------------------------------------

This is work in progress, changes can happen fast!   You can scan the details below for the major work items checked in.   See the Wiki Page ![History Wiki Page Link](https://github.com/K7MDL2/ICOM_IC-905_CIV/wiki/Change-History-and-Details) for the dev history with operational and coding details.

Here I am running the code on one of my Teensy SDR chassis (7" for now) since it has all the hardware needed.  I copy the extracted CI-V bus radio frequency into VFOA on the SDR chassis for display.  
Showing stripped down SDR screen with live VFOA from CI-V bus (USB ch'A')

![Showing stripped down SDR screen with live VFOA from CI-V bus (USB ch'A')](https://github.com/K7MDL2/ICOM_IC-905_CIV/blob/main/Pictures/20240703_234200.jpg)

Band Select screen configured for IC-905 supported bands. Can be configured for 160M through 122Ghz for other radios with or without transverters

![Band Select screen configured for IC-905 supported bands. Can be configured for 160M through 122Ghz for other radios with or without transverters](https://github.com/K7MDL2/ICOM_IC-905_CIV/blob/main/Pictures/20240703_234354.jpg)

GPS NMEA strings from USB ch 'B'

![GPS NMEA strings from USB ch 'B'](https://github.com/K7MDL2/ICOM_IC-905_CIV/blob/main/Pictures/905%20NMEA%20data.jpg)

