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

This is work in progress, changes can happen fast!   You can scan the details below for the major work items checked in.   See the Wiki Page ![History Wiki Page Link](https://github.com/K7MDL2/ICOM_IC-905_CIV/wiki/Change-History-and-Details) for the dev history with operational and coding details.

Here I am running the code on one of my Teensy SDR chassis (7" for now) since it has all the hardware needed.  I copy the extracted CI-V bus radio frequency into VFOA on the SDR chassis for display.  
Showing stripped down SDR screen with live VFOA from CI-V bus (USB ch'A')

![Showing stripped down SDR screen with live VFOA from CI-V bus (USB ch'A')](https://github.com/K7MDL2/ICOM_IC-905_CIV/blob/main/Pictures/20240703_234200.jpg)

Band Select screen configured for IC-905 supported bands. Can be configured for 160M through 122Ghz for other radios with or without transverters

![Band Select screen configured for IC-905 supported bands. Can be configured for 160M through 122Ghz for other radios with or without transverters](https://github.com/K7MDL2/ICOM_IC-905_CIV/blob/main/Pictures/20240703_234354.jpg)

GPS NMEA strings from USB ch 'B'

![GPS NMEA strings from USB ch 'B'](https://github.com/K7MDL2/ICOM_IC-905_CIV/blob/main/Pictures/905%20NMEA%20data.jpg)

