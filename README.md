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

