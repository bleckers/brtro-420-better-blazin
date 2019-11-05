# BRTRO-420 Better Blazin' Mod

Modify the BRTRO-420 reflow oven to have an Arduino based reflow firmware, USB interface and cold junction compensation.

![Mod Image](https://cdn.hackaday.io/images/2437681569845054828.jpg)

Blog entries about the project are here - https://hackaday.io/project/167324-brtro-420-better-blazin-mod

Since we don't have a reset line via the optocouplers and UART, we need a way to reset the boards to program new firmware without having to disconnect everything (since this mod board sits underneath the existing controller). Once the bootloader has been loaded (with your favourite SAM programming tool) and you have programmed your first firmware rev via the Arduino IDE, the Arduino board can go into bootloader mode by issuing the "#" character over the serial terminal (at 115200 baud). This character gets issued automatically by the firmware loading process in the Arduino IDE, so you don't need to issue it manually when loading firmware. There is a few functions in the firmware that you must not remove, otherwise this won't work without pressing the reset button. You also must use the modified bootloader from this repo and not the original Mattairtech one otherwise this won't work.

Please see the (topside) schematic/pinout for the serial port here:
![UART Pinout](https://cdn.hackaday.io/images/5605181570455859567.png)

You can use any USB to serial adapter to connect to this port.

The Arduino board files are based on the Mattairtech Xeno Mini SAMC (ATSAMC21G18A) supported firmware here - https://github.com/mattairtech/ArduinoCore-samd . The easiest way to get it working is to load the Mattairtech board, then replace the installed board files with the ones from the mod zip in this repo. I will get something more streamlined if there is any demand, but should get you up and running

While gerbers are supplied in the repo, you can also use this link to order from OSHPark - https://oshpark.com/shared_projects/wI3J9FGP

Alternatively if you don't mind waiting a couple of weeks, I can assemble, program and ship one for you for about $50USD inc shipping from Australia. Please contact me through the hackaday.io blog above if interested by leaving a comment or sending a message.
