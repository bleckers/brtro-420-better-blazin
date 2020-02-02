# BRTRO-420 Better Blazin' Mod

Instructions are available in the wiki - https://github.com/bleckers/brtro-420-better-blazin/wiki

Now available on Tindie as a limited run - https://www.tindie.com/products/blecky/brtro-420-better-blazin-mod/

<img src="https://cdn.hackaday.io/images/192931573971102114.jpg" width="400">

Modify the [BRTRO-420](http://www.charmhigh-tech.com/sale-7840318-hot-2500w-reflow-oven-brt-420-charmhigh-hot-air-infrared-300-300mm-soldering-rework-station.html) reflow oven to have an Arduino based reflow firmware, serial interface, cold junction compensation and zero crossing detection. This mod consists of a board that connects to the existing controller board, which replaces all the control circuitry and microcontroller.

This oven has a few unique features over the popular T-962 oven, which makes it slightly more desirable if you don't mind the higher cost:
 - an internal convection fan built in.
 - a fume exhaust system which you can vent outside.
 - insulation and thermal tape is actually decent, so it doesn't give of nasty smells during operation.
 - overall build quality is much better.
 
However, unlike the [Unified Engineering mods](https://github.com/UnifiedEngineering/T-962-improvements) to the T-692, the BRTRO-420 is slightly more difficult to modify, mostly due to the [unobtainium build chain](https://hackaday.io/project/167324-brtro-420-better-blazin-mod/log/167780-whats-going-to-happen) for its microcontroller along with a few other design issues such as a poor and noisy thermocouple interface. So the decision was made to spin up a mod board that can be soldered to the bottom of the existing board. This mod board is based on the ATSAMC21 which is a native 5V ARM processor, so it works well with the existing interfaces, without requiring level conversion.

The new firmware keeps the same profile nomenclature as the original oven, so there isn't too much adjustment to the new firmware. However, it is completely written from scratch, so the profile modification workflow is a bit easier now. Note, to change the temperature in the profile edit menu, you hold the OK button on the value you want to edit and then you press the UP/DOWN arrows to adjust the value. Settings are stored automatically once exiting the profile editing screen (you will get a notification on the screen about settings being stored). 

![Mod Image](https://cdn.hackaday.io/images/2437681569845054828.jpg)

Blog entries about the project are here - https://hackaday.io/project/167324-brtro-420-better-blazin-mod


