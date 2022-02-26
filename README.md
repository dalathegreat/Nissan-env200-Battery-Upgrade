# Nissan-env200-Battery-Upgrade
Software for upgrading a 24kWh env200 to 40kWh using a Muxsan 3-port CAN-bridge. 

# Why is this needed when upgrading?
It is kind of not, but highly recommended. Technically you can install a 40kWh pack and just pair it with Leafspy, but then you will run into instrumentation issues and also isses with some fastchargers not charging the battery fully. The code here fixes that when installed on a CAN-bridge, which gets mounted between the battery and vehicle EV-CAN system.

## CAN-bridge installation videos
Below are some example installation videos:
https://www.youtube.com/watch?v=W-W7FwTNOnU

# CAN-bridge software
This section contains info on how to contribute/compile the software from scratch for the 3-port CAN-bridge

## Setting up compilation environment [Windows]
The firmware is compiled with Atmel Studio 7. The older version from 2020 is recommended, since it is less bloated and works with more non-genuine flashing tools. "Atmel Studio v7.0.1931" can be downloaded from here: https://www.microchip.com/en-us/development-tools-tools-and-software/avr-and-sam-downloads-archive

Once the Atmel Studio 7 has been installed, you can clone/download the CAN-bridge software from this repository: https://github.com/dalathegreat/Nissan-LEAF-CCS/

Once on your machine, double click the "can-bridge-env200.cproj" and it opens with Atmel Studio. Press F7 to build. This generates a .hex file in the build directory that can be flashed onto a CAN-bridge

## Making changes to the software
All changes are made in the "can-bridge-env200.c" file. Manipulations to messages happen after the **switch(frame.can_id){ ** line. Generated messages to be sent are handled in the **ISR(TCC0_OVF_vect){** interrupt. Happy programming!

## Flashing the 3-port CAN-bridge
If you want to simply flash a pre-made .hex file, follow this video: https://youtu.be/eLcNSo2Vn6U?t=170
A pre-made .hex file is found in the debug folder of this repository.
Please note that you don't need +12V to power the CAN-bridge, a standard 5V USB cable can also be used when flashing.

If you want to flash directly from Atmel Studio while making code changes, press CTRL+ALT+F5 to start a debugging session, that defaults to flashing the CAN-bridge
