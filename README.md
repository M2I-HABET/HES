# HES
Repository for the HABET Electrical System (HES). The HES system is the base electrical
system used by the HABET team for all payloads. It also has the base firmware
used with the hardware. Currently, we are working on version 4.0 of HES. This
new revision will include a modular hardware board with a shift to C/C++ code
and following NASA standards.

HES 4.0 will have the following features:

* Support for Adafruit Feather boards and BBC Micro-bit boards
* Support for both primary and secondary cells with power managment
* Support for QWICC connectors (I2C)
* Support for Raspberry Pi Computer Module (CM 4)
* Support
* Access to unregulated battery voltage (nominal 7.4 VDC) through PC104
* Access to regulated 5V and 3.3V power busses through PC104
* Access to other GPIO lines to the Raspberry Pi
