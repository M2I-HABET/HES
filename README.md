# HES Revision 5.0
Repository for the HABET Engineering System (HES). The HES system is the engineering hardware
system used by the HABET team for all spacecraft. This includes the base electronics, software,
enclosure, and any support hardware used for any HABET flight. Originally, this only included the 
base electronics used. In 2020, this was expanded to include all engineering systems in HABET and
rolled out as HES 4.0. 

This repo serves as a central hub for managing the systems and has submodules for additional 
GitHub repos used that make up the HES family of hardware/software. These repos currently include

- HAR (High Altitude Reporter) - Main on-board computer and tracking system
- BERT (Back Emergency Recovery Transmitter) - Backup tracking system using a SatComm relay
- THOR (The High-voltage Output Regulator) - Main power distribution board
- ATHENA (Advanced Telemetry and High-Alitude Environmental Networking Assistant) - Payload computer system
- Gemini - Cube-shaped spacecraft with shelves used to divide different functional bays
- Apollo - Puck design to create a cylinder spacecraft with each puck containing hardware/payloads.

Others may be added as needed.

HES 5.0 will have the following features:

* Standardized to the PCIe 1x connector for interconnection. Backplane for Gemini and PCIe extension cables for Apollo
* Custom Power Management Module (THOR) to manage power to all sub-systems
* Support for QWICC connectors (I2C) for additional sensors through HAR and BERT
* Moved to the M.2 connectors and the SparkFun MicroMod board.
* Move to 3D printable enclosures for both Gemini and Apollo
* Front panel support for external power, switching on and off, and status lights
* 915 LoRa system for TT&C (Telemetry, Tracking & Control), 433 MHz LoRa for Payload 

Future support will include:
* Support for WiFi features using the ESP32 board on HAR
* POE Power management for WiFi adapters (Rocket M5)

POE support is a long-term goal as we have moved away from POE devices, but it would be useful in future missions. A shorter term goal is to utilize the ESP32 to provide real-time feedback for testing and pre-launch setup of the spacecraft.

More information can be found in our Wiki.
