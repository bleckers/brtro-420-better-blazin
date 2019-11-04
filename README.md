# BRTRO-420 Better Blazin' Mod

Modify the BRTRO-420 reflow oven to have an Arduino based reflow firmware, USB interface and cold junction compensation.

Blog entries about the project are here - https://hackaday.io/project/167324-brtro-420-better-blazin-mod

Once the bootloader has been loaded (with your favourite SAM programming tool), the Arduino board can go into bootloader mode by issuing the "#" character over the serial terminal (at 115200 baud). This character gets issued automatically by the firmware loading process in the Arduino IDE, so you don't need to issue it manually when loading firmware.

Please see the pinout for the serial port here - https://cdn.hackaday.io/images/5605181570455859567.png
You can use any USB to serial adapter to connect to this port.

The board files are based on the Mattairtech Xeno Mini SAMC supported firmware here - https://github.com/mattairtech/ArduinoCore-samd . The easiest way to get it working is to load the Mattairtech board, then replace the installed board files with the ones from the mod zip in this repo. I will get something more streamlined if there is any demand, but this will get you up and running straight away.
