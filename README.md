# HES
Repository for the HABET Engineering System (HES). The HES system is the engineering hardware
system used by the HABET team for all spacecrafts. This includes the base electronics, software,
enclosure and any support hardware used for any HABET flight. Orignially this only included the 
base electronics used. In 2020 this was expanded to include all engineering systems in HABET and
rolled out as HES 4.0. 

This repo serves as a central hub for managing the systems and has submodules to the additional 
GitHub repos used that make up the HES family of hardware/software. These repos currently include

- BERT (Backup Emergency Recovery Transmitter)
- HABETOS (All HABET Firmware and Operating Systems used on board the spacecrafts)
- HAR (High Altitude Reporter)
- RAP (Remote Activated Pad)
- CyTrack - Online flight tracking, prediction and control software

HES 4.0 will have the following features:

* Support for Adafruit Feather boards and Adafruit Clue boards
* Custom Power Management Module for power management and regulation
* Support for QWICC connectors (I2C) for additional sensors
* Support for additional Feather Wing boards to be plugged
* 3D printable enclosure
* OLED and LED status system
* 433 LoRa system for TT&C (Telemetry, Tracking & Control)
* Support for standalone hardware through RJ45 connection (Power, Serial and I2C)

Future support will include:
* Support for Raspberry Pi Compute Modules
* POE Power management for WiFi adapaters (Rocket M5)

The eventually addition of the Raspberry Pi CM will alow for network connection for faster data uplinking/downlinking and for video support. 
The plan is to use the tested Rocket M5 WiFi adapters for this data link. 

More information can be found in our Wiki.
