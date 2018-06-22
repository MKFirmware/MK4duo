<img align="right" src="Documentation/Logo/MarlinKimbra%20Logo%20GitHub.png"/>

# MK4duo 3D Printer Firmware for all Atmel AVR boards and Arduino Due

## Version 4.3.6.

#### Please donate to support this project https://www.paypal.me/MagoKimbra

### Special thanks
 - [Scott Lahteine](https://github.com/thinkyhead)
 - [Wurstnase](https://github.com/Wurstnase)
 - [esenapaj](https://github.com/esenapaj)
 - [bobc](https://github.com/bobc)
 - [repetier](https://github.com/repetier)
 - [developers of original Marlin](https://github.com/MarlinFirmware)

### New features are:
* One version for all Atmel AVR boards and for Arduino Due or other boards based on Atmel SAM3X8E
* Stepping-algorithm optmized now for DRV8825 and A4988 (no need for double or quadstepping; no delays)
* High speed stepping of approx. 295.000 steps/s, if needed (maybe more with less DOUBLE_STEP_FREQUENCY?)

---
# MK4duo 3D Printer Firmware
  * [Configuration & Compilation](/Documentation/Compilation.md)
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

This version of Marlin was made to accommodate some requests made by the community RepRap Italy http://forums.reprap.org/index.php?349

The new features are:
* A single Firmware for all types of printers; Cartesian, Delta, MORGAN SCARA, MAKERARM SCARA, CoreXY, CoreXZ & CoreYZ and their reverse.
* The possibility of having only one hotend independently from the extruders that you have.
* The addition of the 6th extruder.
* Management Color Mixing Extruder
* System Management MKr4 for 4 extruders width just two drivers and 8 relay.
* System Management MKr6 for 6 extruders width just two drivers and 8 relay.
* System Management MKr12 for 12 extruders width just four drivers and 16 relay.
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
* Added support reader TAG width MFRC522
* Added support NEXTION lcd touch
* Added Cooler and Hot Chamber
* Added Laser beam and raster base64
* Added CNC Router
* Addes Mesh Bed Level (MBL)
* Added Stop and Save for Restart (SSR)
* Added Restart for recovery jov when power loss an return
* Added Nozzle Clean Features
* Added Nozzle Park Features
* Added RGB LED
* Added Case Light
* Added ABL or MBL leveling fade height
* Added save in EEPROM ABL, MBL or UBL
* Added Door switch
* Added TMC2130 motor driver
* Added TMC2208 motor driver
* Added Power Check for Stop and Save
* Added Probe Manually
* Added LCD Bed Leveling
* Added User menu LCD
* Added DAV system for filament runout
* Added Extruder Encoder for control filament movement
* Added Adafruit Neopixel LED
* Added DHT Sensor Temperature and Humidity (DHT11, DHT21 and DHT22)
* Added Universal Bed Leveling (UBL) by Official Marlin
* Added Junction Deviation instead of traditional Jerk limiting
* Added Bézier Jerk Control see https://github.com/synthetos/TinyG/wiki/Jerk-Controlled-Motion-Explained


## Credits

The current MarlinKimbra dev team consists of:
  - [MagoKimbra - Alberto Cotronei](https://github.com/MagoKimbra)

More features have been added by:
  - Mr.Goblin
  - [iosonopersia](https://github.com/iosonopersia)
  - [Franco (nextime) Lanza](https://git.nexlab.net/u/nextime)

## License

MK4duo is published under the [GPL license](/Documentation/COPYING.md) because I believe in open development.
Do not use this code in products (3D printers, CNC etc) that are closed source or are crippled by a patent.
