<img align="right" src="Documentation/Logo/LOGO_github.png"/>

# MK4duo 3D Printer Firmware

[![Build Status](https://travis-ci.org/MKFirmware/MK4duo.svg?branch=master)](https://travis-ci.org/MKFirmware/MK4duo)
![GitHub](https://img.shields.io/github/license/MKFirmware/MK4duo?style=plastic)
![GitHub last commit](https://img.shields.io/github/last-commit/MKFirmware/MK4duo?style=plastic)
![GitHub repo size](https://img.shields.io/github/repo-size/MKFirmware/MK4duo?style=plastic)
![GitHub contributors](https://img.shields.io/github/contributors/MKFirmware/MK4duo?style=plastic)

## Version 4.4.0

NOW IS DIE...

## Hardware Abstraction Layer (HAL)

MK4duo introduces a layer of abstraction so that all the existing high-level code can be built for 32-bit platforms while still retaining full 8-bit AVR compatibility.

### Current HALs

  name|board|processor|speed|flash|sram|logic|fpu
  ----|-----|---------|-----|-----|----|-----|---
  [Arduino AVR](https://www.arduino.cc/)|[RAMPS 1_4](https://reprap.org/wiki/RAMPS_1.4)|ATmega2560|20MHz|256k|8k|5V|no
  [Arduino Due](https://www.arduino.cc/en/Guide/ArduinoDue)|[ULTRATRONICS](https://reprapworld.it/products/elettronica/ultratronics/ultratronics_pro_v1_0/)|[SAM3X8E](http://www.microchip.com/wwwproducts/en/ATsam3x8e) ARM-Cortex M3|84MHz|512k|64+32k|3.3V|no
  [Arduino Core STM32](https://github.com/stm32duino/Arduino_Core_STM32)|[RUMBA32](https://github.com/Aus3D/RUMBA32)|[STM32F446](https://www.st.com/en/microcontrollers-microprocessors/stm32f446.html) ARM-Cortex M4|120MHz|256k|128k|3.3V|no

### Special thanks
 - [Scott Lahteine](https://github.com/thinkyhead)
 - [Wurstnase](https://github.com/Wurstnase)
 - [esenapaj](https://github.com/esenapaj)
 - [bobc](https://github.com/bobc)
 - [repetier](https://github.com/repetier)
 - [developers of original Marlin](https://github.com/MarlinFirmware)

---
# MK4duo 3D Printer Firmware
  * [Configuration & Compilation AVR & DUE](/Documentation/Compilation.md)
  * [Configuration & Compilation STM32](/Documentation/STM32.md)
  * Supported
    * [Features](/Documentation/Features.md)
    * [Hardware](/Documentation/Hardware.md)
    * [GCodes](/Documentation/GCodes.md)
  * Notes
    * [Bresenham Algorithm](/Documentation/Bresenham.md)
    * [Auto Bed Leveling](/Documentation/BedLeveling.md)
    * [Filament Sensor](/Documentation/FilamentSensor.md)
    * [Ramps Servo Power](/Documentation/RampsServoPower.md)
    * [LCD Language - Font - System](Documentation/LCDLanguageFont.md)
  * Version
    * [Change Log](/Documentation/changelog.md)


## Configurator Tool Online

http://www.marlinkimbra.it


## Quick Information

The new features are:
* A single Firmware for all types of printers; Cartesian, Delta, CoreXY, CoreXZ & CoreYZ and their reverse.
* The possibility of having only one hotend independently from the extruders that you have.
* The addition of the 6th extruder.
* Management Color Mixing Extruder
* System Management MKr4 for 4 extruders with just two drivers and 8 relays.
* System Management MKr6 for 6 extruders with just two drivers and 8 relays.
* System Management MKr12 for 12 extruders with just four drivers and 16 relays.
* Management Dual Extruder DONDOLO.
* Step per unit varied for each extruder as well as the feedrate and the acceleration.
* Added commands to facilitate purging of hotend. 
* Added Delta Auto Calibration Algorithm of Minor Squares based on DC42 RepRapFirmware 7 points
* Added Delta Auto Calibration Algorithm based on LVD-AC(Luc Van Daele) 1 - 7 points + iteration
* Added Debug Dryrun used by repetier.
* Added total Power on time writed in SD CARD.
* Added total Power consumption writed in SD CARD.
* Added total filament printed writed in SD CARD.
* Added anti extruder idle oozing system.
* Added Hysteresis and Z-Wobble correction (only cartesian printers).
* Added support reader TAG with MFRC522
* Added support NEXTION HMI LCD TOUCH 4.3" normal/advanced, 5" advanced and 7" advanced
* Added support for 4 beds
* Added support for 4 Hot Chambers
* Added support for 1 Water Cooler
* Added Laser beam and raster base64
* Added CNC Router
* Added protection of the heaters if you do not print after 30 minutes on. It is important that the host gives the command M530 S1 for the start and M530 for the stop.
* Addes Mesh Bed Level (MBL)
* Added Restart for recovery job when power loss an return
* Added Nozzle Clean Features
* Added Nozzle Park Features
* Added Tool change Park
* Added RGB LED
* Added Adafruit Neopixel LED
* Added Case Light
* Added ABL or MBL leveling fade height
* Added save in EEPROM ABL, MBL or UBL
* Added Door switch for stop job when door is open
* Added Support for TMC2130 - TMC2208 - TMC2660 - TMC2160 - TMC5130 - TMC5160 motor driver
* Added Support for L6470 - L6474 - L6480 motor driver
* Added Power Check pin for restart job
* Added two serial for host.
* Added Probe BLTouch
* Added Probe BLTouch V3.0 or V3.1
* Added Probe Z Sensoreless for TMC
* Added Probe Manually
* Added LCD Bed Leveling
* Added User menu LCD
* Added DAV system for filament runout
* Added Extruder Encoder for control filament movement
* Added DHT Sensor Temperature and Humidity (DHT11, DHT21 and DHT22)
* Added Universal Bed Leveling (UBL) by Official Marlin
* Added Junction Deviation instead of traditional Jerk limiting
* Added Bézier Jerk Control see https://github.com/synthetos/TinyG/wiki/Jerk-Controlled-Motion-Explained
* Added Text Menu to Nextion Display
* Added Prompt support for Host
* Added Prusa MMU2 support


## Credits

The current MarlinKimbra dev team consists of:
  - [MagoKimbra - Alberto Cotronei](https://github.com/MagoKimbra)

More features have been added by:
  - Mr.Goblin
  - [iosonopersia](https://github.com/iosonopersia)

## License

MK4duo is published under the [GPL license](/LICENSE) because we believe in open development. The GPL comes with both rights and obligations. Whether you use MK4duo firmware as the driver for your open or closed-source product, you must keep MK4duo open, and you must provide your compatible MK4duo source code to end users upon request. The most straightforward way to comply with the MK4duo license is to make a fork of MK4duo on Github, perform your modifications, and direct users to your modified fork.

While we can't prevent the use of this code in products (3D printers, CNC, etc.) that are closed source or crippled by a patent, we would prefer that you choose another firmware or, better yet, make your own.
