### USB Band Decoder for Icom IC-905

1. Connect IC-905 to Teensy Host port.
2. Connect PC to Teensy USB
3. Connect Teensy IO pins to buffers and devices for band specific control

Leverages CI-V portion of band decoder code from RemoteTH.com and heavily modified - see source file for more info

Passes through 2 serial interfaces and taps into the CI-V frequency value to operate band decode outputs on GPIO pins.
The IC-905 connects to the Teensy host port which today cannot handle audio, so only the serial ports are dealt with.
Can use the IC-905 LAN connection to a PC for full audio and serial control.  The LAN operates in parallel with the USB OK.

Could generate PTT (aka SEND) from the CI-V messages and combine in software for per-band PTT outputs.  
I have seen some say they identified and broke out the 'SEND" or PTT signal on the RF unit cable but still band specific filtering.
Ideally the RF Unit would have band decode IO signals available using some of its spare accessory connector pins.  These pins are "Do Not Connect" 
suggesing they may be connected to something inside for future use.
