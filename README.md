### USB Band Decoder for Icom IC-905

Description:

This is example code that taps into the IC-905 control head USB-C serial port CI-V to extract frequency values to operate band decode outputs on GPIO pins.

The IC-905 connects to the Teensy host port which today cannot handle audio, so only the serial ports are dealt with.  Both serial ports are passed on to an optional PC USB connection.

For full audio and serial control you can use the IC-905 LAN connection to a PC.  The LAN operates in parallel with the USB OK.


Challenge:

The IC-905 does not have hardware outputs to select or key band specific antenna relays or amps.  There is only 1 PTT (aka SEND) port supporting up to 6 bands.  In the radio's config screens you can enable SEND for each band but on the wire there is no way to differentiate which band is active on the SEND port.  Further, 144, 432, and 1296 bands are combined on 1 N connector.  The 2.3, 5.7, and 10G SMA connectors are separate.  Keying 1 of 6 possible band amplifiers, or routing coax to a particular amp is a challenge.  While a PC could be used, many field operations do not use a PC so a small low power standalone solution is desired.

Solution:

A small CPU connects to the IC-905 USB port to control IO pins.  This is example Arduino code that runs on a on small Teensy 4.0 CPU module.  The Teensy 4.X CPUs have a USB2.0 host port which connects to the radio.   It can have multiple serial ports but cannot handle audio today.

The IC-905 USB port has 2 serial ports and USB audio on it.  One is CI-V for CAT control, the 2nd is configurable, usually set to the GPS ASCII data output useful to set time on a PC. This code will connect to the 2 serial ports and examine only the CI-V serial data for any frequency messages.  The frequency info is then translated to a pattern on the GPIO pins to control equipment.  You can modify the simple code to set the patterns you need for your equipment.  Be sure to consider the limits of the 3.3V IO pins and use driver interfaces as required.

As an option, a PC can be connected to the Teensy main USB port. The 2 radio serial ports are passed on to the PC. The PC connection is not required, this will operate standalone. The T4 CPU can be packaged in a very small box with a short USB cable to the radio.  Inside would be suitable buffers. Connectors added to control some combo of relays and amps. Long wires may then be run to the equipment including all the way up to the antenna, wherever that may be.

Could generate PTT (aka SEND) from the CI-V messages and combine in software for per-band PTT outputs.  
I have seen some say they identified and broke out the 'SEND" or PTT signal on the RF unit cable but still need band specific filtering.  Somehow the RF Unit needs to know what band to operate on so some sort of data exchange is likely over the cable from the control head.  I have never looked at it. Ideally the RF Unit would have band decode IO signals available using some of its spare accessory connector pins.  These pins are noted as "Do Not Connect" suggesting they may be connected to something inside for future use.

Leverages CI-V portion of band decoder code from RemoteTH.com and heavily modified - see source file for more info


Usage:

Plug the T4 host port into the Radio USB jack.  Connect your amps and relays via suitable driver interfaces.  Modify the example code to create the correct output IO pin pattern you need.

If using digital modes on a PC and audio is desired, use the LAN connection.  It will operate at the same time as the USB port providing full control with serial, spectrum and audio. The USB connected box will then only handle the band decoding tasks.  

This is minimally tested on my bench and will require some code customization to output the IO pin pattern you need.  Feel free to open Issues on the GitHub repository.  I intend to package one up later on for my use.

------------------------------------------------------------------------------------------------------------------

Dev update details:   This is still a early work in progress, changes can happen fast!

July 2024 Updates

Pulled in my Teensy SDR code and stubbed out audio and spectrum and PLL related code. Runnig the code on one of my Teensy SDR chassis (7" for now) since it has all the hardware needed.  I copy the extracted CI-V bus radio frequency into VFOA on the SDR chassis for display.  The encoders and swicthes change things on the display thinking it is still an SDR, but they have no effect today. Next steps are to extract other radio settings of interest and pull in the Band decoder project code.  Key features I want first up ande band menu select, VFO A and B display and control with teh VFO knobs, display mode, filter, and then add ability to control the radio more fully. Buffered IO hardware will be added to control antennas, transverters and amps.

In the case of at least the IC-905, can extract time from the USB ch 'B' NMEA data strings and update clock and Grid square on my display.  Currently time is form PC over USB when programmed or when ENET is enabled, NTP.  The GPS will work offline so is desireable to use.  Possible the grid square is already inteh CI-V but since they show it on the 905 info screens.

Showing stripped down SDR screen with live VFOA from CI-V bus (USB ch'A')

![Showing stripped down SDR screen with live VFOA from CI-V bus (USB ch'A')](https://github.com/K7MDL2/ICOM_IC-905_CIV/blob/main/Pictures/20240703_234200.jpg)

Band Select screen configured for IC-905 supported bands. Can be configured for 160M through 122Ghz for other radios with or without transverters

![Band Select screen configured for IC-905 supported bands. Can be configured for 160M through 122Ghz for other radios with or without transverters](https://github.com/K7MDL2/ICOM_IC-905_CIV/blob/main/Pictures/20240703_234354.jpg)

GPS NMEA strings from USB ch 'B'

![GPS NMEA strings from USB ch 'B'](https://github.com/K7MDL2/ICOM_IC-905_CIV/blob/main/Pictures/905%20NMEA%20data.jpg)


July 5, 2024

Significant rework done though still quite messy from pulling ion differnt project's code.  Lots to weed out and clean up.

You can now use touch or an encoder to control VFOA and change bands. So VFOA is now 2 way.  Change the radio dial and this box's VFOA will stay synced.  Mode and Filters are received but not yet dealt with fully.

This code update replaces the previous CI-V library with CIVMasterLib library found at https://github.com/WillyIoBrok/CIVmasterLib. It was not that bad to swap out but getting the send VFO frequency code to work was a challenge.  It wants const uint8_t values inteh writemsg fucntion and bytes in reverse order with a length byte at the front. Changes had to be made to the library and my frequency formatting code to account for the 905 using a 6 byte frequency message on bands 10GHz and higher, 5 bytes on lower bands.  Also replaced several unsigned longs with uint64_t to handle frequencies above 2.4Ghz.  Some of the higher level library fucntion I tried were causing slow perf so for now I am sticking with building my own as I need them.  The basics stuff seems to work fast enough. 

Code is in place to pass theough both directoin CI-V data so you can run WSJT-X.  If you you use the encoder to change frequency it causes WSJT-X to panic and offer a retry. Teh long term answer is probably to have thisbox be a separate CI_V address but more study required.  The library does have code to handle multile radios on the bus.

In SDR_Data.h I edited the Bandmem table defaults to turn off transverter support on IC-905 native bands.  I had to disable the SD Card coinfg as it was overriding the changes. Need to sort that out.

Other than the removed spectrum and audio related functions, the screen is not updated but should work well as-is until time for a new layout.  The database allows for multiple layouts so just takes time to figure new xy cordinates and sizes.

Still have to cut in band decoder logic and UI though I may do a simpler version instead.

Hooking this box up is simple.  Plug a USB Type-A cable into the Teensy 4.X host board. The other Type-C end goes into the IC-905. The IC-705, IC-9700, and maybe other radios should work by changing the CI-V addresses in the code.  Will extract that to a config file later.  Optionally you can have a PC connected to the main Teensy USB port (micro connector), used for programming and debug but there are 3 USB serial ports active.  The 2nd is for CAT to programs like WSJT-X.  The 3rd is the IC-905 GPS NMEA data streaming.  I use NMEATime2 to use this data to compute grid square and keep the PC time sync'd.

In theory it can work without any screen or encoders and act just as a band decoder once that code is added.  A read-only Screen at minimum, could be an OLED, or character LCD even. I think the touch screen, switches, and encoders will offer good value for things like CW macros, quick bands changes, more accurate touch tuning, less menu hunting.  Things like external amp temp and power can be read and displayed, perhaps over hardware serial or i2C bus connections.

I added a new folder containing my modified version of the CI-V library for the 905 and bands > 2GHz.  You can copy them to your normal Arduino libraries folder.  I expect they will need further udpates as I go, especially as I add higher level command support.

7/8/2024

Made several fixes.  Can now bidirectionally tune from either end on all bands including 10Ghz, followsws the configured band map.  The SD card config read at startup is bypassed until dev changes are stabilized more.  Next up are getting mode and filter to align.  There is a long list of settings and status stuff that I could sync with the radio (both directions) given time to code it up, mostly now a matter of priority order vs time vs usefulness.