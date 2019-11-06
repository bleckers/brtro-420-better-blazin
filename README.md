# BRTRO-420 Better Blazin' Mod

Modify the [BRTRO-420](http://www.charmhigh-tech.com/sale-7840318-hot-2500w-reflow-oven-brt-420-charmhigh-hot-air-infrared-300-300mm-soldering-rework-station.html) reflow oven to have an Arduino based reflow firmware, USB interface and cold junction compensation. This mod consists of a board that connects to the existing controller board, which replaces all the control circuitry and microcontroller.

This oven has a few unique features over the popular T-962 oven, which makes it slightly more desirable if you don't mind the higher cost:
 - an internal convection fan built in.
 - a fume exhaust system which you can vent outside.
 - insulation and thermal tape is actually decent, so it doesn't give of nasty smells during operation.
 - overall build quality is much better.
 
However, unlike the [Unified Engineering mods](https://github.com/UnifiedEngineering/T-962-improvements) to the T-692, the BRTRO-420 is slightly more difficult to modify, mostly due to the [unobtainium build chain](https://hackaday.io/project/167324-brtro-420-better-blazin-mod/log/167780-whats-going-to-happen) for its microcontroller along with a few other design issues such as a poor and noisy thermocouple interface. So the decision was made to spin up a mod board that can be soldered to the bottom of the existing board. This mod board is based on the ATSAMC21 which is a native 5V ARM processor, so it works well with the existing interfaces, without requiring level conversion.

This mod board is designed to be pretty simple. It just needs to be soldered onto the bottom of the PCB over the existing through hole leads (it lines up quite neatly, so difficult to get wrong). You also need to put a jumper on the W1 pins near the existing MCU (or bridge them with solder if it's missing the jumper); this is the DEBUG I/F pin. This places the original MCU into serial bootloader mode (MD = 0, DEBUG I/F = 0) and effectively renders it inoperable. You can now put the machine back together and start it up (or read further about UART and thermocouples). There is a reset jumper on the mod board you can short, which will disable the mod and allow you to run the existing firmware as well.

The new firmware keeps the same profile nomenclature as the original oven, so there isn't too much adjustment to the new firmware. However, it is completely written from scratch, so the profile modification workflow is a bit easier now. Note, to change the temperature in the profile edit menu, you hold the OK button on the value you want to edit and then you press the UP/DOWN arrows to adjust the value. Settings are stored automatically once exiting the profile editing screen (you will get a notification on the screen about settings being stored).

![Mod Image](https://cdn.hackaday.io/images/2437681569845054828.jpg)

Blog entries about the project are here - https://hackaday.io/project/167324-brtro-420-better-blazin-mod

Since we don't have a reset line via the optocouplers and UART, we need a way to reset the boards to program new firmware without having to disconnect everything (since this mod board sits underneath the existing controller). Once the bootloader has been loaded (with your favourite SAM programming tool) and you have programmed your first firmware rev via the Arduino IDE, the Arduino board can go into bootloader mode by issuing the "#" character over the serial terminal (at 115200 baud). This character gets issued automatically by the firmware loading process in the Arduino IDE, so you don't need to issue it manually when loading firmware. There is a few functions in the firmware that you must not remove, otherwise this won't work without pressing the reset button. You also must use the modified bootloader from this repo and not the original Mattairtech one otherwise this won't work.

The (topside) schematic/pinout for the serial port:
![UART Pinout](https://cdn.hackaday.io/images/5605181570455859567.png)

You can use any USB to UART adapter to connect to this port. Note MCU TX will connect to the RX port of your UART adapter and MCU RX to the TX port of the adapter.

The thermocouples will need to be replaced (the existing ones are glued in) and placed on the tray/PCBs for better thermal readouts (having them floating in the air with a black PCB can get the PCB up to 290 degrees Celsius). In the firmware, TC0 corresponds to the back two elements and TC1 corresponds to the front two (the back and front elements are individually controlled). Try to situate the thermocouples in the center of the elements on a sacrificial PCB or the PCB you are firing. If you aren't using a zone, just leave it where it is in the center of those elements. More info [here](https://hackaday.io/project/167324-brtro-420-better-blazin-mod/log/170604-getting-baked-gets-you-routed).

The Arduino board files are based on the Mattairtech Xeno Mini SAMC (ATSAMC21G18A) supported firmware here - https://github.com/mattairtech/ArduinoCore-samd . The easiest way to get it working is to load the Mattairtech board, then replace the installed board files with the ones from the mod zip in this repo. I will get something more streamlined if there is any demand, but should get you up and running

While gerbers are supplied in the repo, you can also use this link to order from OSHPark - https://oshpark.com/shared_projects/wI3J9FGP

Alternatively if you don't mind waiting a couple of weeks, I can assemble, program and ship one for you for about $50USD inc shipping from Australia. Please contact me through the hackaday.io blog above if interested by leaving a comment or sending a message.
