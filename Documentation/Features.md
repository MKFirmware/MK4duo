NEED UPGRADE

# Features

*   Interrupt based movement with real linear acceleration
*   High steprate
*   Look ahead (Keep the speed high when possible. High cornering speed)
*   Interrupt based temperature protection
*   preliminary support for Matthew Roberts advance algorithm
    For more info see: http://reprap.org/pipermail/reprap-dev/2011-May/003323.html
*   Full endstop support
*   SD Card support
*   SD Card folders (works in pronterface)
*   SD Card autostart support
*   LCD support (ideally 20x4)
*   LCD menu system for autonomous SD card printing, controlled by an click-encoder.
*   EEPROM storage of e.g. max-velocity, max-acceleration, and similar variables
*   many small but handy things originating from bkubicek's fork.
*   Arc support
*   Temperature oversampling
*   Dynamic Temperature setpointing aka "AutoTemp"
*   Support for QTMarlin, a very beta GUI for PID-tuning and velocity-acceleration testing. https://github.com/bkubicek/QTMarlin
*   Endstop trigger reporting to the host software.
*   Updated sdcardlib
*   Heater power reporting. Useful for PID monitoring.
*   PID tuning
*   Cartesian kinematics
*   CoreXY kinematics (www.corexy.com/theory.html)
*   CoreXZ kinematics
*   Delta kinematics
*   SCARA kinematics
*   One firmware for all printers, see configurations.h.
*   Dual X-carriage support for multiple extruder systems
*   Configurable serial port to support connection of wireless adaptors.
*   Automatic operation of extruder/cold-end cooling fans based on nozzle temperature
*   RC Servo Support, specify angle or duration for continuous rotation servos.
*   Manual procedure for bed setting with LCD
*   Bed Auto Leveling for cartesian and delta printer
*   Z probe repetability test
*   Setting step for unit and feedrate for extruders
*   Setting PID for any extruder
*   Real-time filament diameter measurement and control
*   COLOR MIXING EXTRUDE
*   MKR4 suppport for 4 extruder but width only one driver
*   Singlenozzle support
*   NPr2 support, multiextruder by NicolaP http://www.3dmakerlab.it/extruder-npr2.html
*   Laserbeam support
*   Firmware test
*   Support for a filament diameter sensor, which adjusts extrusion volume
*   Support for a hall effect sensor for calucalte Wh. Example sensor ACS712 20A range Current Sensor Module. http://i.ebayimg.com/images/i/310506962976-0-1/s-l1000.jpg
*   Anti extruder idle oozing system
*   Statistics save on SD

The default baudrate is 250000. This baudrate has less jitter and hence errors than the usual 115200 baud, but is less supported by drivers and host-environments.

## Differences and additions to the already good Sprinter firmware

### Look-ahead

Marlin has look-ahead. While sprinter has to break and re-accelerate at each corner,
lookahead will only decelerate and accelerate to a velocity,
so that the change in vectorial velocity magnitude is less than the xy_jerk_velocity.
This is only possible, if some future moves are already processed, hence the name.
It leads to less over-deposition at corners, especially at flat angles.

### Arc support

Slic3r can find curves that, although broken into segments, were ment to describe an arc.
Marlin is able to print those arcs. The advantage is the firmware can choose the resolution,
and can perform the arc with nearly constant velocity, resulting in a nice finish.
Also, less serial communication is needed.

### Temperature Oversampling

To reduce noise and make the PID-differential term more useful, 16 ADC conversion results are averaged.

### AutoTemp

If your gcode contains a wide spread of extruder velocities, or you realtime change the building speed, the temperature should be changed accordingly.
Usually, higher speed requires higher temperature.
This can now be performed by the AutoTemp function
By calling M109 S<mintemp> B<maxtemp> F<factor> you enter the autotemp mode.

You can leave it by calling M109 without any F.
If active, the maximal extruder stepper rate of all buffered moves will be calculated, and named "maxerate" [steps/sec].
The wanted temperature then will be set to t=tempmin+factor*maxerate, while being limited between tempmin and tempmax.
If the target temperature is set manually or by gcode to a value less then tempmin, it will be kept without change.
Ideally, your gcode can be completely free of temperature controls, apart from a M109 S T F in the start.gcode, and a M109 S0 in the end.gcode.

### EEPROM

If you know your PID values, the acceleration and max-velocities of your unique machine, you can set them, and finally store them in the EEPROM.
After each reboot, it will magically load them from EEPROM, independent what your Configuration.h says.

### LCD Menu

If your hardware supports it, you can build yourself a LCD-CardReader+Click+encoder combination. It will enable you to realtime tune temperatures,
accelerations, velocities, flow rates, select and print files from the SD card, preheat, disable the steppers, and do other fancy stuff.
One working hardware is documented here: http://www.thingiverse.com/thing:12663
Also, with just a 20x4 or 16x2 display, useful data is shown.

### SD card directories

If you have an SD card reader attached to your controller, also folders work now. Listing the files in pronterface will show "/path/subpath/file.g".
You can write to file in a subfolder by specifying a similar text using small letters in the path.
Also, backup copies of various operating systems are hidden, as well as files not ending with ".g".

### Autostart

If you place a file auto[0-9].g into the root of the sd card, it will be automatically executed if you boot the printer. The same file will be executed by selecting "Autostart" from the menu.
First *0 will be performed, than *1 and so on. That way, you can heat up or even print automatically without user interaction.

### Endstop trigger reporting

If an endstop is hit while moving towards the endstop, the location at which the firmware thinks that the endstop was triggered is outputed on the serial port.
This is useful, because the user gets a warning message.
However, also tools like QTMarlin can use this for finding acceptable combinations of velocity+acceleration.

### Coding paradigm

Not relevant from a user side, but Marlin was split into thematic junks, and has tried to partially enforced private variables.
This is intended to make it clearer, what interacts which what, and leads to a higher level of modularization.
We think that this is a useful prestep for porting this firmware to e.g. an ARM platform in the future.
A lot of RAM (with enabled LCD ~2200 bytes) was saved by storing char []="some message" in Program memory.
In the serial communication, a #define based level of abstraction was enforced, so that it is clear that
some transfer is information (usually beginning with "echo:"), an error "error:", or just normal protocol,
necessary for backwards compatibility.

### Interrupt based temperature measurements

An interrupt is used to manage ADC conversions, and enforce checking for critical temperatures.
This leads to less blocking in the heater management routine.


### Different printer one firmware

I put in a single firmware all the firmware that I found online for the various printers, especially the one for Delta, I standardized the firmware. There are 4 files one for each type of printer, just edit the file in question and say configuration.h the printer you want to use ...

* \#define CARTESIAN
* \#define COREXY
* \#define DELTA
* \#define SCARA


### Different axis step per unit for all extruder

* \#define DEFAULT_AXIS_STEPS_PER_UNIT     {80,80,3200,625,625,625,625}    // X, Y, Z, E0, E1, E2, E3 default steps per unit


### Different feedrate for all extruder

* \#define DEFAULT_MAX_FEEDRATE            {300,300,2,100,100,100,100}     // X, Y, Z, E0, E1, E2, E3 (mm/sec)


### Different acceleration for all extruder

* #define DEFAULT_MAX_ACCELERATION {3000,3000,50,1000,1000,1000,1000}     // X, Y, Z, E0, E1, E2, E3 maximum start speed for accelerated moves.


### Different PID for all hotend

* \#define  DEFAULT_Kp {41.51,50,0,0} // Kp for E0, E1, E2, E3
* \#define  DEFAULT_Ki {7.28,15,0,0}  // Ki for E0, E1, E2, E3
* \#define  DEFAULT_Kd {59.17,90,0,0} // Kd for E0, E1, E2, E3


### Add Feedrate for retraction

* \#define DEFAULT_RETRACTION_MAX_FEEDRATE {150,150,150,150}               // E0, E1, E2, E3 (mm/sec)


### Singlenozzle

If have one hotend and more extruder define SINGLENOZZLE for unique temperature.
* Uncomment below to enable SINGLENOZZLE (One Hotend)
* \#define SINGLENOZZLE //This is used for singlenozzled multiple extrusion configuration


### COLOR MIXING EXTRUDER
* Support is added for the [Repetier Host compatible](http://reprap.org/wiki/Repetier_Color_Mixing) GCode M163 Sn Pn which sets a single mix factor at a time. For example M163 S0 P0.5 and M163 S1 P0.5 would be used to set a mix of half color 1 plus half color 2.
* A shorthand gcode M165 is proposed to set the mixing parameters globally using parameters ABCDHI as a shortcut over multiple M163. For example, M165 A0.5 B0.5 would be used to set a mix of half color 1 plus half color 2.
* If MIXING_VIRTUAL_TOOLS is set to 2 or greater the Repetier Host compatible M164 Sn command is also made available to save the current mix factors as a virtual tool that can be recalled later. This option changes the behavior of the gcode_T() function so it restores a saved mixture rather than setting a new extruder.
* Following Pia Taubert's proposal, the G1 command is extended to accept up to 6 mixing parameters (ABCDHI). For example G1 A0.25 B0.6 C0.15 X10.0 Y99.2 E12.34 F9000 will set a mixture for 3 channels starting with the current move. The mix parameters must add up to 1.0. If they don't they will be normalized – scaled up or down to add up to 1.0. The mix is persistent, so further moves will continue to use the same mix.


### MKR4 System

The system MKR4 allows two extruders for each driver on the motherboard. So with two drivers available you get to have 4 extruders. This is due to the relays controlled by the same motherboard with the pins. Look at the bottom of the file pins.h to set the right pin. This system allows the use of flux channeler to print in color. See http://www.immaginaecrea.it/index.php/blog-wordpress/post/150-flusso-canalizzatore-a-4-vie-la-stampa-3d-a-4-colori-e-gia-realta-per-lambiente-reprap-prima-parte


### NPr2 System

soon
http://www.3dmakerlab.it/extruder-npr2.html


### Debug Dryrun Repetier

In dry run mode, the firmware will ignore all commands to set temperature or extrude. That way you can send a file without using any filament. This is handy if your printer loses steps during print and you are doing some research on when and why. If you seem to have troubles with your extruder, check if you have that option enabled!


### Laserbeam Support

Support for laserbeam.
M3 Sxxx put output LASER_TTL_PIN in PWM
M4 switch on Laser, put high LASER_PWR_PIN
M5 switch off Laser, put low LASER_PWR_PIN
Setting LASER_TTL_PIN and LASER_PWR_PIN in pins.h


### G30 Autocalibartion for DELTA

G30 This command is used to perform reporting and autocalibration of a delta printer and has several options, as follows: G30 Probe bed and produce a report of the current state of the printer, e.g.:
* Z-Tower			Endstop Offsets
* -0.0125			X:-3.05 Y:-1.83 Z:-2.69
* -0.0000	-0.0000	Tower Position Adjust
* -0.0625			A:-0.04 B:0.05 C:-0.01
* -0.0375 -0.0250 	I:0.25 J:-1.25 K:-0.37 -0.0250
* Delta Radius: 109.5965
* Diag Rod: 224.5935 This option does not change any settings, but is useful when manually calibrating a printer, using the M666 command to change values.

G30 Xnn Ynn Probe bed at specified X,Y point and show z-height and delta carriage positions, e.g.: Bed Z-Height at X:30.00 Y:30.00 = 0.0000 Carriage Positions: [176.40, 207.77, 209.52].

G30 A Start auto-calibration. This will attempt to calibrate the printer, adjusting all parameters automatically, and will repeat the bed probing sequence show above several times adjusting each time until calibration is complete. It is recommended that you use M502 to load default values and then M500 to save them prior to starting the auto-calibration.


### M666 for all printers

For not DELTA:
M666 Pzzz adjust Z-Probe Offset if you have Auto bed level.
M666 L view value in memory for Z-Probe Offset.

For DELTA:
M666 L   List all current configuration values , e.g.:
Current Delta geometry/print values:
* X (Endstop Adj): -3.05
* Y (Endstop Adj): -1.83
* Z (Endstop Adj): -2.69
* A (Tower A Diagonal Rod Correction): 0.04
* B (Tower B Diagonal Rod Correction): 0.05
* C (Tower C Diagonal Rod Correction): 0.00
* I (Tower A Angle Correction): 0.25
* J (Tower B Angle Correction): -1.25
* K (Tower C Angle Correction): 0.00
* U (Tower A Radius Correction): -0.04
* V (Tower B Radius Correction): 0.05
* W (Tower C Radius Correction): -0.02
* R (Delta Radius): 109.60
* D (Diagonal Rod Length): 224.59
* S (Segments per Second): 200
* O (Print Radius): 120.0
* P (Probe radius): 100.0
* H (Z-Height): 255.73

All of these values can also be adjusted using the M666 command, e.g. to set the delta radius to 200mm, use:
* M666 R200

Or to change the Z-Height to 350.5 mm:
* M666 H350.5

Commands can also be combined, e.g. to set endstop values:
* M666 X-2.04 Y-1.02 Z-1.52

All of these values can be saved/loaded to/from EEPROM using standard M500/M501 G-Code commands (to save the settings at any time just type M500). This makes manual configuration of a printer much easier as there is no longer a requirement to edit the configuration.h file and re-upload firmware for each time a change needs to be made.

Configuration_delta.h  now includes the following additional parameters:
Set start and end locations used to deploy the Z-Probe:

* \#define Z_PROBE_DEPLOY_START_LOCATION {20, 96, 30, 0}
* \#define Z_PROBE_DEPLOY_END_LOCATION {5, 96, 30, 0}
* \#define Z_PROBE_RETRACT_START_LOCATION {49, 84, 20, 0}
* \#define Z_PROBE_RETRACT_END_LOCATION {49, 84, 1, 0}

Set precision for autocalibration G30 function . calibration will complete when this value is reached . all probed point have to be at 0 +/- 0.015mm (for 0.03 setting below)

* \#define AUTOCALIBRATION_PRECISION 0.03 // mm

Set distance to probe bed at for G30 function
 
* \#define BED_DIAMETER 170 // mm


### Firmware test tools

Test firmware uncomment
* \#define FIRMWARE_TEST in configuration.h
Use baudrate 115200 and use Arduino serial monitor.
