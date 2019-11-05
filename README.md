# BRTRO-420 Better Blazin' Mod

Modify the [BRTRO-420](http://www.charmhigh-tech.com/sale-7840318-hot-2500w-reflow-oven-brt-420-charmhigh-hot-air-infrared-300-300mm-soldering-rework-station.html) reflow oven to have an Arduino based reflow firmware, USB interface and cold junction compensation. This mod consists of a board that connects to the existing controller board, which replaces all the control circuitry and microcontroller.

This oven has a few unique features over the popular T-962 oven. It has more elements (4) for starters to give a more even reflow, it also has an internal convection fan built in, it has an exhaust system which you can vent outside and it's insulation and thermal tape is actually decent, so it doesn't give of nasty smells during operation.

The new firmware keeps the same profile nomenclature as the original oven, so there isn't too much adjustment to the new firmware. However, it is completely written from scratch, so the profile modification workflow is a bit easier now. Note, to change the temperature in the profile edit menu, you hold the OK button on the value you want to edit and then you press the UP/DOWN arrows to adjust the value. Settings are stored automatically once exiting the profile editing screen (you will get a notification on the screen about settings being stored).

![Mod Image](https://cdn.hackaday.io/images/2437681569845054828.jpg)

Blog entries about the project are here - https://hackaday.io/project/167324-brtro-420-better-blazin-mod

Since we don't have a reset line via the optocouplers and UART, we need a way to reset the boards to program new firmware without having to disconnect everything (since this mod board sits underneath the existing controller). Once the bootloader has been loaded (with your favourite SAM programming tool) and you have programmed your first firmware rev via the Arduino IDE, the Arduino board can go into bootloader mode by issuing the "#" character over the serial terminal (at 115200 baud). This character gets issued automatically by the firmware loading process in the Arduino IDE, so you don't need to issue it manually when loading firmware. There is a few functions in the firmware that you must not remove, otherwise this won't work without pressing the reset button. You also must use the modified bootloader from this repo and not the original Mattairtech one otherwise this won't work.

The (topside) schematic/pinout for the serial port:
![UART Pinout](https://cdn.hackaday.io/images/5605181570455859567.png)

You can use any USB to UART adapter to connect to this port. Note MCU TX will connect to the RX port of your UART adapter and MCU RX to the TX port of the adapter.

The Arduino board files are based on the Mattairtech Xeno Mini SAMC (ATSAMC21G18A) supported firmware here - https://github.com/mattairtech/ArduinoCore-samd . The easiest way to get it working is to load the Mattairtech board, then replace the installed board files with the ones from the mod zip in this repo. I will get something more streamlined if there is any demand, but should get you up and running

While gerbers are supplied in the repo, you can also use this link to order from OSHPark - https://oshpark.com/shared_projects/wI3J9FGP

Alternatively if you don't mind waiting a couple of weeks, I can assemble, program and ship one for you for about $50USD inc shipping from Australia. Please contact me through the hackaday.io blog above if interested by leaving a comment or sending a message.
