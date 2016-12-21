/**
 * MK4duo 3D Printer Firmware and Laser
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2016 Alberto Cotronei @MagoKimbra
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * Look here for descriptions of G-codes:
 *  - http://linuxcnc.org/handbook/gcode/g-code.html
 *  - http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes
 *
 * Help us document these G-codes online:
 *  - http://reprap.org/wiki/G-code
 *  - https://github.com/MagoKimbra/MK4duo/blob/master/Documentation/GCodes.md
 *
 * -----------------
 * Implemented Codes
 * -----------------
 *
 * "G" Codes
 *
 * G0  -> G1 except for laser where G0 is "move without firing"
 * G1  - Coordinated Movement X Y Z E F(feedrate) P(Purge), for laser move by firing
 * G2  - CW ARC
 * G3  - CCW ARC
 * G4  - Dwell S[seconds] or P[milliseconds], delay in Second or Millisecond
 * G5  - Bezier curve - from http://forums.reprap.org/read.php?147,93577
 * G7  - Laser raster base64
 * G10 - retract filament according to settings of M207
 * G11 - retract recover filament according to settings of M208
 * G12 - Nozzle Clean
 * G20 - Set input units to inches
 * G21 - Set input units to millimeters
 * G27 - Nozzle Park
 * G28 - X Y Z Home all Axis. M for bed manual setting with LCD. B return to back point
 * G29 - Detailed Z-Probe, probes the bed at 3 or more points. Will fail if you haven't homed yet.
   G29   Fyyy Lxxx Rxxx Byyy for customer grid.
 * G30 - Single Z probe, probes bed at X Y location (defaults to current XY location)
         and Delta geometry Autocalibration
 * G31 - Dock sled (Z_PROBE_SLED only)
 * G32 - Undock sled (Z_PROBE_SLED only)
 * G60 - Save current position coordinates (all axes, for active extruder).
 *        S<SLOT> - specifies memory slot # (0-based) to save into (default 0).
 * G61 - Apply/restore saved coordinates to the active extruder.
 *        X Y Z E - Value to add at stored coordinates.
 *        F<speed> - Set Feedrate.
 *        S<SLOT> - specifies memory slot # (0-based) to restore from (default 0).
 * G90 - Use Absolute Coordinates
 * G91 - Use Relative Coordinates
 * G92 - Set current position to coordinates given
 *
 * "M" Codes
 *
 * M0   - Unconditional stop - Wait for user to press a button on the LCD (Only if ULTRA_LCD is enabled)
 * M1   - Same as M0
 * M3   - Put S<value> in laser beam control. (Requires LASERBEAM)
 * M4   - Turn on laser beam. (Requires LASERBEAM)
 * M5   - Turn off laser beam. (Requires LASERBEAM)
 * M17  - Enable/Power all stepper motors
 * M18  - Disable all stepper motors; same as M84
 * M20  - List SD card
 * M21  - Init SD card
 * M22  - Release SD card
 * M23  - Select SD file (M23 filename.g)
 * M24  - Start/resume SD print
 * M25  - Pause SD print
 * M26  - Set SD position in bytes (M26 S12345)
 * M27  - Report SD print status
 * M28  - Start SD write (M28 filename.g)
 * M29  - Stop SD write
 * M30  - Delete file from SD (M30 filename.g)
 * M31  - Output time since last M109 or SD card start to serial
 * M32  - Make directory
 * M33  - Stop printing, close file and save restart.gcode
 * M34  - Open file and start print
 * M35  - Upload Firmware to Nextion from SD
 * M42  - Change pin status via gcode Use M42 Px Sy to set pin x to value y, when omitting Px the onboard led will be used.
 * M43  - Monitor pins & report changes - report active pins
 * M48  - Measure Z_Probe repeatability. M48 [P # of points] [X position] [Y position] [V_erboseness #] [E_ngage Probe] [L # of legs of travel]
 * M70  - Power consumption sensor calibration
 * M75  - Start the print job timer
 * M76  - Pause the print job timer
 * M77  - Stop the print job timer
 * M78  - Show statistical information about the print jobs
 * M80  - Turn on Power Supply
 * M81  - Turn off Power Supply
 * M82  - Set E codes absolute (default)
 * M83  - Set E codes relative while in Absolute Coordinates (G90) mode
 * M84  - Disable steppers until next move,
 *        or use S<seconds> to specify an inactivity timeout, after which the steppers will be disabled.  S0 to disable the timeout.
 * M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
 * M92  - Set axis_steps_per_unit - same syntax as G92
 * M96  - Print ZWobble value
 * M97  - Set ZWobble parameter M97 A<Amplitude_in_mm> W<period_in_mm> P<phase_in_degrees>
 * M98  - Print Hysteresis value
 * M99  - Set Hysteresis parameter M99 X<in mm> Y<in mm> Z<in mm> E<in mm>
 * M100 - Watch Free Memory (For Debugging Only)
 * M104 - Set hotend target temp
 * M105 - Read current temp
 * M106 - Fan on
 * M107 - Fan off
 * M109 - Sxxx Wait for hotend current temp to reach target temp. Waits only when heating
 *        Rxxx Wait for hotend current temp to reach target temp. Waits when heating and cooling
 *        IF AUTOTEMP is enabled, S<mintemp> B<maxtemp> F<factor>. Exit autotemp by any M109 without F
 * M110 - Set the current line number
 * M111 - Set debug flags with S<mask>.
 * M112 - Emergency stop
 * M114 - Output current position to serial port
 * M115 - Report capabilities. (Extended capabilities requires EXTENDED_CAPABILITIES_REPORT)
 * M117 - Display a message on the controller screen
 * M119 - Output Endstop status to serial port
 * M120 - Enable endstop detection
 * M121 - Disable endstop detection
 * M122 - S<1=true/0=false> Enable or disable check software endstop
 * M126 - Solenoid Air Valve Open (BariCUDA support by jmil)
 * M127 - Solenoid Air Valve Closed (BariCUDA vent to atmospheric pressure by jmil)
 * M128 - EtoP Open (BariCUDA EtoP = electricity to air pressure transducer by jmil)
 * M129 - EtoP Closed (BariCUDA EtoP = electricity to air pressure transducer by jmil)
 * M140 - Set hot bed target temp
 * M141 - Set hot chamber target temp
 * M142 - Set cooler target temp
 * M145 - Set the heatup state H<hotend> B<bed> F<fan speed> for S<material> (0=PLA, 1=ABS)
 * M149 - Set temperature units
 * M150 - Set BlinkM Color Output or RGB LED R: Red<0-255> U(!): Green<0-255> B: Blue<0-255> over i2c, G for green does not work.
 * M155 - Auto-report temperatures with interval of S<seconds>. (Requires AUTO_REPORT_TEMPERATURES)
 * M163 - Set a single proportion for a mixing extruder. (Requires MIXING_EXTRUDER)
 * M164 - Save the mix as a virtual extruder. (Requires MIXING_EXTRUDER and MIXING_VIRTUAL_TOOLS)
 * M165 - Set the proportions for a mixing extruder. Use parameters ABCDHI to set the mixing factors. (Requires MIXING_EXTRUDER)
 * M190 - Sxxx Wait for bed current temp to reach target temp. Waits only when heating
 *        Rxxx Wait for bed current temp to reach target temp. Waits when heating and cooling
 * M191 - Sxxx Wait for chamber current temp to reach target temp. Waits only when heating
 *        Rxxx Wait for chamber current temp to reach target temp. Waits when heating and cooling
 * M192 - Sxxx Wait for cooler current temp to reach target temp. Waits only when heating
 *        Rxxx Wait for cooler current temp to reach target temp. Waits when heating and cooling
 * M200 - set filament diameter and set E axis units to cubic millimeters (use S0 to set back to millimeters).:D<millimeters>- 
 * M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
 * M202 - Set max acceleration in units/s^2 for travel moves (M202 X1000 Y1000) Unused in Marlin!!
 * M203 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
 * M204 - Set default acceleration: P for Printing moves, R for Retract only (no X, Y, Z) moves and T for Travel (non printing) moves (ex. M204 P800 T3000 R9000) in mm/sec^2
 * M205 -  advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk, E=maximum E jerk
 * M206 - Set additional homing offset
 * M207 - Set retract length S[positive mm] F[feedrate mm/min] Z[additional zlift/hop], stays in mm regardless of M200 setting
 * M208 - Set recover=unretract length S[positive mm surplus to the M207 S*] F[feedrate mm/min]
 * M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
 * M218 - Set hotend offset (in mm): T<extruder_number> X<offset_on_X> Y<offset_on_Y>
 * M220 - Set speed factor override percentage: S<factor in percent>
 * M221 - T<extruder> S<factor in percent> - set extrude factor override percentage
 * M222 - T<extruder> S<factor in percent> - set density extrude factor percentage for purge
 * M226 - Wait until the specified pin reaches the state required: P<pin number> S<pin state>
 * M240 - Trigger a camera to take a photograph
 * M250 - Set LCD contrast C<contrast value> (value 0..63)
 * M280 - Set servo position absolute. P: servo index, S: angle or microseconds
 * M300 - Play beep sound S<frequency Hz> P<duration ms>
 * M301 - Set PID parameters P I D and C
 * M302 - Allow cold extrudes, or set the minimum extrude S<temperature>.
 * M303 - PID relay autotune S<temperature> sets the target temperature (default target temperature = 150C). H<hotend> C<cycles> U<Apply result>
 * M304 - Set hot bed PID parameters P I and D
 * M305 - Set hot chamber PID parameters P I and D
 * M306 - Set cooler PID parameters P I and D
 * M320 - Enable/Disable S1=enable S0=disable, V[bool] Print the leveling grid, Z<height> for leveling fade height (Requires ENABLE_LEVELING_FADE_HEIGHT)
 * M321 - Set a single Auto Bed Leveling Z coordinate - X<gridx> Y<gridy> Z<level val> S<level add>
 * M322 - Reset Auto Bed Leveling matrix
 * M323 - Set Level bilinear manual - X<gridx> Y<gridy> Z<level val> S<level add>
 * M350 - Set microstepping mode.
 * M351 - Toggle MS1 MS2 pins directly.
 * M355 - Turn case lights on/off
 * M380 - Activate solenoid on active extruder
 * M381 - Disable all solenoids
 * M400 - Finish all moves
 * M401 - Lower z-probe if present
 * M402 - Raise z-probe if present
 * M404 - N<dia in mm> Enter the nominal filament width (3mm, 1.75mm ) or will display nominal filament width without parameters
 * M405 - Turn on Filament Sensor extrusion control.  Optional D<delay in cm> to set delay in centimeters between sensor and extruder
 * M406 - Turn off Filament Sensor extrusion control
 * M407 - Display measured filament diameter
 * M408 - Report JSON-style response
 * M410 - Quickstop. Abort all the planned moves
 * M420 - Enable/Disable Mesh Bed Leveling (with current values) S1=enable S0=disable (Requires MESH_BED_LEVELING)
 *        Z<height> for leveling fade height (Requires ENABLE_LEVELING_FADE_HEIGHT)
 * M421 - Set a single Mesh Bed Leveling Z coordinate. M421 X<mm> Y<mm> Z<mm>' or 'M421 I<xindex> J<yindex> Z<mm>
 * M428 - Set the home_offset logically based on the current_position
 * M500 - Store parameters in EEPROM
 * M501 - Read parameters from EEPROM (if you need reset them after you changed them temporarily).
 * M502 - Revert to the default "factory settings". You still need to store them in EEPROM afterwards if you want to.
 * M503 - Print the current settings (from memory not from EEPROM). Use S0 to leave off headings.
 * M522 - Read or Write on card. M522 T<extruders> R<read> or W<write> L<list>
 * M530 - Enables explicit printing mode (S1) or disables it (S0). L can set layer count
 * M531 - filename - Define filename being printed
 * M532 - X<percent> L<curLayer> - update current print state progress (X=0..100) and layer L
 * M540 - Use S[0|1] to enable or disable the stop print on endstop hit (requires ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
 * M595 - Set hotend AD595 O<offset> and S<gain>
 * M600 - Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
 * M605 - Set dual x-carriage movement mode: S<mode> [ X<duplication x-offset> R<duplication temp offset> ]
 * M649 - Set laser options. S<intensity> L<duration> P<ppm> B<set mode> R<raster mm per pulse> F<feedrate>
 * M666 - Set z probe offset or Endstop and delta geometry adjustment
 * M906 - Set motor currents XYZ T0-4 E
 * M907 - Set digital trimpot motor current using axis codes.
 * M908 - Control digital trimpot directly.
 *
 * ************ SCARA Specific - This can change to suit future G-code regulations
 * M360 - SCARA calibration: Move to cal-position ThetaA (0 deg calibration)
 * M361 - SCARA calibration: Move to cal-position ThetaB (90 deg calibration - steps per degree)
 * M362 - SCARA calibration: Move to cal-position PsiA (0 deg calibration)
 * M363 - SCARA calibration: Move to cal-position PsiB (90 deg calibration - steps per degree)
 * M364 - SCARA calibration: Move to cal-position PSIC (90 deg to Theta calibration position)
 * ************* SCARA End ***************
 *
 * M928 - Start SD logging (M928 filename.g) - ended by M29
 * M995 - X Y Z Set origin for graphic in NEXTION
 * M996 - S<scale> Set scale for graphic in NEXTION
 * M997 - NPR2 Color rotate
 * M999 - Restart after being stopped by error
 *
 * "T" Codes
 *
 * T0-T5 - Select a tool by index (usually an extruder) [ F<mm/min> ]
 *
 */
 
#include "base.h"

#if ENABLED(DIGIPOT_I2C) || ENABLED(BLINKM)
  #include <Wire.h>
#endif
#if ENABLED(ULTRA_LCD)
  #if ENABLED(LCD_I2C_TYPE_PCF8575)
    #include <Wire.h>
    #include <LiquidCrystal_I2C.h>
  #elif ENABLED(LCD_I2C_TYPE_MCP23017) || ENABLED(LCD_I2C_TYPE_MCP23008)
    #include <Wire.h>
    #include <LiquidTWI2.h>
  #elif ENABLED(DOGLCD)
    #include <U8glib.h> // library for graphics LCD by Oli Kraus (https://code.google.com/p/u8glib/)
  #else
    #include <LiquidCrystal.h> // library for character LCD
  #endif
#endif
#if HAS(DIGIPOTSS)
  #include <SPI.h>
#endif
