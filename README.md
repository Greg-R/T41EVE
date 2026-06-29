### Version T41EVE.01 T41 Software Defined Transceiver PlatformIO Arduino

This is the "T41 Embedded Video Engine" software for the 
T41-3 Software Defined Transceiver.  The T41EVE is derived from the T41EEE
software which targets the original T41 and T41-2 hardware. 

T41-3 hardware and the T41EVE firmware utilize the "Embedded Video Engine"
BT817 display controller for dramatically improved performance compared
to the original T41 and its RA8875 display controller.

Purchase the book "Digital Signal Processing and Software Defined Radio" by
Albert Peter and Jack Purdum here:

<https://www.amazon.com/Digital-Signal-Processing-Software-Defined/dp/B0F5BDQZW3>

Please bring your questions, bug reports, and complaints about this software to this
group:

<https://groups.io/g/SoftwareControlledHamRadio>

This software is licensed according to:  
GNU GENERAL PUBLIC LICENSE  
Version 3, 29 June 2007  
Please refer to the included LICENSE file.  
Greg Raven, May 10 2025

T41EVE will be hosted at this public Github repository:

<https://github.com/Greg-R/T41EVE>

I don't plan on publishing periodic releases.  The repository will be updated as bug fixes
and feature enhancements become available.  You will be able to update and FLASH your radio
with the revised software quickly.

Please note that the configuration structure is different than the predecessor T41EEE.
It is recommended to perform a full FLASH erase before loading a new version of T41EVE
unless otherwise noted.

## How to Compile T41EVE

T41EEE.93 was developed and compiled using PlatformIO.  All configuration parameters
are included in the platform.ini file.  You will need to install the following
package and libraries:

1.  Install TeensyDuino package using PlatformIO.  The current TeensyDuino version is 1.62.0.
    Please note that PlatformIO usually has a delay in updating to the latest package.
2.  You will need to install the ArduinoJson library by Benoit Blanchon.  Install via
    the IDE Library Manager.
3.  You will need to install the Etherkit Si5351 library.  Install via the IDE
    library manager.
4.  You will need to manually install the Open Audio Library.  The library was updated
    in November, 2025.  Update this library if you have installed an older copy!  
    T41EEE.91 uses the recently developed quad I2S output class, which was recently
    added to the library:

<https://github.com/chipaudette/OpenAudio_ArduinoLibrary>

5.  You will need to manually install the Rotary library:

<https://github.com/brianlow/Rotary>

6.  You will need to manually install the EVE library:

<https://github.com/RudolphRiedel/EmbeddedVideoEngine>


The file MyConfigurationFile.h includes several compiler options which add, subtract,
or adjust parameters of some features.  Please review this file prior to compilation.

Completing a FLASH erase of the Teensy is strongly recommended before uploading this
new version.  The instructions for performing a FLASH erase of the Teensy are here:

<https://www.pjrc.com/store/teensy41.html#programming>

The bullet "Memory Wipe & LED Blink Restore" has the instructions.

The compilation should conclude with results similar to this:
```
teensy_size: Memory Usage on Teensy 4.1:
teensy_size:   FLASH: code:195168, data:81216, headers:8284   free for files:7841796
teensy_size:    RAM1: variables:145216, code:168376, padding:28232   free for local variables:182464
Building .pio/build/teensy41/firmware.hex
teensy_size:    RAM2: variables:329792  free for malloc/new:194496
```
## Alternate Arduino Compilation

It is possible to compile using the current Arduino IDE.
Go here for the alternative Arduino sketch:

<https://github.com/Greg-R/T41EVE_Arduino>

Unfortunately it is not possible to use the EVE library as-is.  There are a couple of changes
required.  The modified library is included in the ZIP file download from the above repository.
Move the zipped EVE library file to your libraries folder, and expand it there.

Configure using the following list:

1.  Optimize is set to "Smallest Code" (Tools menu).
2.  CPU speed is set to 528 MHz (Tools menu).
3.  Select USB Type: "Dual Serial" in the Tools menu.
4.  Install TeensyDuino using the IDE Board Manager.  The TeensyDuino version is 1.62.0.  
5.  You will need to install the ArduinoJson library by Benoit Blanchon.  Install via
    the IDE Library Manager.
6.  You will need to install the Etherkit Si5351 library.  Install via the IDE
    library manager.
7.  You will need to manually install the Open Audio Library.  The library was updated
    in November, 2025.  Update this library if you have installed an older copy!  
    T41EEE.91 uses the recently developed quad I2S output class, which was recently
    added to the library:

<https://github.com/chipaudette/OpenAudio_ArduinoLibrary>

8.  You will need to manually install the Rotary library:

<https://github.com/brianlow/Rotary>

The compilation should conclude with results similar to this:
```
Opening Teensy Loader...
Memory Usage on Teensy 4.1:
  FLASH: code:194236, data:82768, headers:8684   free for files:7840776
   RAM1: variables:146464, code:168760, padding:27848   free for local variables:181216
   RAM2: variables:332384  free for malloc/new:191904
```