/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
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
 * Help to document MK4duo's G-codes online:
 *  - http://reprap.org/wiki/G-code
 *  - https://github.com/MagoKimbra/MK4duo/blob/master/Documentation/GCodes.md
 *
 * -----------------
 * Implemented Codes
 * -----------------
 *
 * "G" Codes
 *
 * G0   -> G1 except for laser where G0 is "move without firing"
 * G1   - Coordinated Movement X Y Z E F(feedrate) P(Purge), for laser move by firing
 * G2   - CW ARC
 * G3   - CCW ARC
 * G4   - Dwell S[seconds] or P[milliseconds], delay in Second or Millisecond
 * G5   - Bezier curve - from http://forums.reprap.org/read.php?147,93577
 * G7   - Laser raster base64
 * G10  - Retract filament according to settings of M207
 * G11  - Retract recover filament according to settings of M208
 * G12  - Clean tool
 * G17  - Select Plane XY (Requires CNC_WORKSPACE_PLANES)
 * G18  - Select Plane ZX (Requires CNC_WORKSPACE_PLANES)
 * G19  - Select Plane YZ (Requires CNC_WORKSPACE_PLANES)
 * G20  - Set input units to inches
 * G21  - Set input units to millimeters
 * G26  - Mesh Validation Pattern (Requires G26_MESH_VALIDATION) 
 * G27  - Park Nozzle (Requires NOZZLE_PARK_FEATURE)
 * G28  - X Y Z Home all Axis. M for bed manual setting with LCD. B return to back point. O Home only if position is unknown.
 * G29  - Detailed Z-Probe, probes the bed at 3 or more points. Will fail if you haven't homed yet.
 *          Fyyy Lxxx Rxxx Byyy for customer grid.
 * G30  - Single Z probe, probes bed at X Y location (defaults to current XY location)
 *          G30 [X#] [Y#] [S#] [Z#] [P#]
 *          X = Probe X position (default=current probe position)
 *          Y = Probe Y position (default=current probe position)
 *          S = [bool] Stows the probe if 1 (default=1)
 *          Z = [bool] with a non-zero value will apply the result to current delta_height (ONLY DELTA)
 *          P = [bool] with a non-zero value will apply the result to current probe_offset_Z (ONLY DELTA)
 * G31  - Dock sled (PROBE_SLED only)
 * G32  - Undock sled (PROBE_SLED only)
 * G33  - Delta geometry Autocalibration (Requires DELTA_AUTO_CALIBRATION_?)
 *          F[nfactor] p[npoint] Q[debugging] (Requires DELTA_AUTO_CALIBRATION_1)
 *          P[points] [F] [O] [T] V[verbose] (Requires DELTA_AUTO_CALIBRATION_2)
 * G34  - Set Delta Height calculated from toolhead position (only DELTA)
 * G34  - I<iterations> T<accuracy> A<amplification> (Requires Z_STEPPER_AUTO_ALIGN)
 * G38  - Probe target - similar to G28 except it uses the Z_MIN endstop for all three axes
 * G42  - Coordinated move to a mesh point. (Requires MESH_BED_LEVELING or AUTO_BED_LEVELING_BILINEAR)
 * G60  - Save current position coordinates (all axes, for active extruder).
 *          S[SLOT] - specifies memory slot # (0-based) to save into (default 0).
 * G61  - Apply/restore saved coordinates.
 *          X Y Z E - Value to add at stored coordinates.
 *          F[speed] - Set Feedrate.
 *          S[SLOT] - specifies memory slot # (0-based) to restore from (default 0).
 * G90  - Use Absolute Coordinates
 * G91  - Use Relative Coordinates
 * G92  - Set current position to coordinates given
 *
 * "M" Codes
 *
 * M0   - Unconditional stop - Wait for user to press a button on the LCD.
 * M1   -> M0
 * M3   - S[value] L[duration] P[ppm] D[diagnostic] B[set mode] in laser beam control. (Requires LASER)
 *        S[value] CNC clockwise speed. (Requires CNCROUTERS)
 * M4   - S[value] CNC counter clockwise speed. (Requires CNCROUTERS)
 * M5   - Turn laser/spindle off. (Requires LASER or Requires CNCROUTERS)
 * M6   - Tool change CNC. (Requires CNCROUTERS)
 * M16  - Expected printer check
 * M17  - Enable stepper motors
 * M18  - Disable stepper motors; same as M84
 * M20  - List SD card. (Requires SDSUPPORT)
 * M21  - Init SD card. (Requires SDSUPPORT)
 * M22  - Release SD card. (Requires SDSUPPORT)
 * M23  - Select SD file (M23 filename.g). (Requires SDSUPPORT)
 * M24  - Start/resume SD print. (Requires SDSUPPORT)
 * M25  - Pause SD print. (Requires SDSUPPORT)
 * M26  - Set SD position in bytes (M26 S12345). (Requires SDSUPPORT)
 * M27  - Report SD print status. With S[bool] set the SD status auto-report. (Requires SDSUPPORT) 
 * M28  - Start SD write (M28 filename.g). (Requires SDSUPPORT)
 * M29  - Stop SD write. (Requires SDSUPPORT)
 * M30  - Delete file from SD (M30 filename.g). (Requires SDSUPPORT)
 * M31  - Get the time since the start of SD Print
 * M32  - Open file and start print
 * M34  - Set SD Card Sorting Options
 * M35  - Upload Firmware to Nextion from SD
 * M39  - SD info and formatting
 * M42  - Change pin status via gcode Use M42 Px Sy to set pin x to value y, when omitting Px the onboard led will be used.
 * M43  - Display pin status, watch pins for changes, watch endstops & toggle LED, Z servo probe test, toggle pins
 *
 * M43         - report name and state of pin(s)
 *                 P[pin]  Pin to read or watch. If omitted, reads all pins.
 *                 I       Flag to ignore MK4duo's pin protection.
 *
 * M43 W       - Watch pins -reporting changes- until reset, click, or M108.
 *                 P[pin]  Pin to read or watch. If omitted, read/watch all pins.
 *                 I       Flag to ignore MK4duo's pin protection.
 *
 * M43 E[bool] - Enable / disable background endstop monitoring
 *                 - Machine continues to operate
 *                 - Reports changes to endstops
 *                 - Toggles LED when an endstop changes
 *                 - Can not reliably catch the 5mS pulse from BLTouch type probes
 *
 * M43 T       - Toggle pin(s) and report which pin is being toggled
 *                 S[pin]  - Start Pin number.   If not given, will default to 0
 *                 L[pin]  - End Pin number.   If not given, will default to last pin defined for this board
 *                 I       - Flag to ignore MK4duo's pin protection.   Use with caution!!!!
 *                 R       - Repeat pulses on each pin this number of times before continueing to next pin
 *                 W       - Wait time (in miliseconds) between pulses.  If not given will default to 500
 *
 * M43 S       - Servo probe test
 *                 P[index] - Probe index (optional - defaults to 0
 * M44  - Codes debug - report codes available (and how many of them there are)
 *          I G-code list, J M-code list
 * M48  - Measure Z_Probe repeatability. M48 [P # of points] [X position] [Y position] [V_erboseness #] [E_ngage Probe] [L # of legs of travel]
 * M49  - Turn on or off G26 debug flag for verbose output (Requires G26_MESH_VALIDATION)
 * M70  - Power consumption sensor calibration
 * M73  - P[percent] Set percentage complete (compatibility with Marlin)
 * M75  - Start the print job timer
 * M76  - Pause the print job timer
 * M77  - Stop the print job timer
 * M78  - Show statistical information about the print jobs
 *        S78 reset statistics, R[index] reset service index
 * M80  - Turn on Power Supply
 * M81  - Turn off Power Supply
 * M82  - Set E codes absolute (default)
 * M83  - Set E codes relative while in Absolute Coordinates (G90) mode
 * M84  - Disable steppers until next move,
 *        or use S[seconds] to specify an inactivity timeout, after which the steppers will be disabled.  S0 to disable the timeout.
 * M85  - Set inactivity shutdown timer with parameter S[seconds]. To disable set zero (default)
 * M86  - Set safety timer expiration with parameter M[minutes]. To disable set zero
 * M92  - Set axis_steps_per_unit - same syntax as G92
 * M99  - Set Hysteresis parameter X[in mm] Y[in mm] Z[in mm] E[in mm] F[float] Enable/disable/fade-out hysteresis correction (0.0 to 1.0)
 * M100 - Watch Free Memory (For Debugging Only)
 * M104 - S[C°] Set hotend target temperature, R[C°] Set hotend idle temperature
 *        T[int] 0-5 For Select Hotends (default 0)
 * M105 - Read current temp
 * M106 - P[fan] S[speed] F[frequency] U[pin] L[min speed] X[max speed] I[inverted logic] H[int] Set Auto mode - H=7 for controller - H-1 for disabled T[trig temperaure]
 * M107 - P[fan] Fan off
 * M108 - Break out of heating loops (M109, M190, M303). With no controller, breaks out of M0/M1. (Requires EMERGENCY_PARSER)
 * M109 - Sxxx Wait for hotend current temp to reach target temp. Waits only when heating
 *        Rxxx Wait for hotend current temp to reach target temp. Waits when heating and cooling
 *        IF AUTOTEMP is enabled, S[mintemp] B[maxtemp] F[factor]. Exit autotemp by any M109 without F
 * M110 - Set the current line number
 * M111 - Set debug flags with S[mask].
 * M112 - Full Shutdown
 * M113 - Set Host Keepalive interval with parameter S[seconds]. To disable set zero
 * M114 - Report the current position to host. D Report more detail, R Report the realtime position instead
 * M115 - Report capabilities. (Extended capabilities requires EXTENDED_CAPABILITIES_REPORT)
 * M116 - Wait for all heaters to reach target temperature
 * M117 - Display a message on the controller screen
 * M118 - Display a message in the host console.
 * M119 - Output Endstop status to serial port
 * M120 - Enable endstop detection
 * M121 - Disable endstop detection
 * M122 - S[1=true|0=false] Enable or disable check software endstop. (Requires MIN_SOFTWARE_ENDSTOPS or MAX_SOFTWARE_ENDSTOPS)
 * M123 - Set Endstop Logic X[bool] Y[bool] Z[bool] I[X2 bool] J[Y2 bool] K[Z2 bool] P[Probe bool] D[Door bool]
 * M124 - Set Endstop Pullup X[bool] Y[bool] Z[bool] I[X2 bool] J[Y2 bool] K[Z2 bool] P[Probe bool] D[Door bool]
 * M125 - Save current position and move to pause park position. (Requires PARK_HEAD_ON_PAUSE)
 * M126 - Solenoid Air Valve Open (BariCUDA support by jmil)
 * M127 - Solenoid Air Valve Closed (BariCUDA vent to atmospheric pressure by jmil)
 * M128 - EtoP Open (BariCUDA EtoP = electricity to air pressure transducer by jmil)
 * M129 - EtoP Closed (BariCUDA EtoP = electricity to air pressure transducer by jmil)
 * M140 - S[C°] Set hot bed target temperature, R[C°] Set hot bed idle temperature
 *        T[int] 0-3 For Select Beds (default 0)
 * M141 - S[C°] Set hot chamber target temperature, R[C°] Set hot chamber idle temperature
 *        T[int] 0-3 For Select Chambers (default 0)
 * M142 - S[C°] Set cooler target temperature
 * M145 - Set the heatup state H[hotend] B[bed] C[chamber] F[fan speed] for S[material] (0=PLA, 1=ABS, 2=GUM)
 * M149 - Set temperature units
 * M150 - Set Status LED Color as R[red] U[green] B[blue]. Values 0-255. (Requires BLINKM, RGB_LED, RGBW_LED, or PCA9632)
 * M155 - S[1/0] Enable/disable auto report temperatures.
 * M163 - S[index] P[float] Set a single proportion for a mixing extruder. (Requires COLOR_MIXING_EXTRUDER)
 * M164 - S[index] Save the mix as a virtual extruder. (Requires COLOR_MIXING_EXTRUDER and MIXING_VIRTUAL_TOOLS)
 * M165 - Set the proportions for a mixing extruder. Use parameters ABCDHI to set the mixing factors. (Requires COLOR_MIXING_EXTRUDER)
 * M166 - S[bool] A[float] Z[float] I[index] J[index] Set the Gradient Mix for the mixing extruder. (Requires COLOR_MIXING_EXTRUDER)
 * M190 - Sxxx Wait for bed current temp to reach target temp. Waits only when heating
 *        Rxxx Wait for bed current temp to reach target temp. Waits when heating and cooling
 *        T[int] 0-3 For Select Beds (default 0)
 * M191 - Sxxx Wait for chamber current temp to reach target temp. Waits only when heating
 *        Rxxx Wait for chamber current temp to reach target temp. Waits when heating and cooling
 *        T[int] 0-3 For Select Chambers (default 0)
 * M192 - Sxxx Wait for cooler current temp to reach target temp. Waits only when cooling
 * M200 - set filament diameter and set E axis units to cubic millimeters (use S0 to set back to millimeters).:D[millimeters]- 
 * M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
 * M203 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z10 E50) in mm/sec
 * M204 - Set default acceleration in mm/sec^2: P for Printing moves, R for Retract moves and V for Travel (non printing) moves (ex. M204 P800 V3000 T0 R9000)
 * M205 - Set advanced settings:  minimum travel speed S=while printing T=travel only, B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk, E=maximum E jerk, J=Junction mm
 * M206 - Set additional homing offset
 * M207 - Set retract length S[positive mm] F[feedrate mm/min] Z[additional zlift/hop], stays in mm regardless of M200 setting
 * M208 - Set recover=unretract length S[positive mm surplus to the M207 S*] F[feedrate mm/min]
 * M209 - S[1/0] enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
 * M217 - Set Park position and tool change parameters. (Requires NOZZLE_PARK_FEATURE or TOOL_CHANGE_FIL_SWAP)
 * M218 - Set hotend offset (in mm): T[tool] X[offset_on_X] Y[offset_on_Y] Z[offset_on_Z]
 * M220 - Set speed factor override percentage: S[factor in percent], B to backup, R to restore currently set override
 * M221 - T[extruder] S[factor in percent] - set extrude factor override percentage
 * M222 - T[extruder] S[factor in percent] - set density extrude factor percentage for purge
 * M223 - T[extruder] S[bool] set Filrunout Logic. (Requires FILAMENT_RUNOUT_SENSOR)
 * M224 - T[extruder] S[bool] set Filrunout Pullup. (Requires FILAMENT_RUNOUT_SENSOR)
 * M226 - Wait until the specified pin reaches the state required: P[pin number] S[pin state]
 * M228 - Set axis min/max travel. S[bool] 0 = set axis maximum (default), 1 = set axis minimum, Xnnn X axis limit, Ynnn Y axis limit, Znnn Z axis limit
 * M240 - Trigger a camera to take a photograph
 * M250 - Set LCD contrast C[contrast value] (value 0..63)
 * M280 - Set servo position absolute. P<index> S<angle or microseconds>. (Requires servos)
 * M281 - Set servo low|up angles position. P<index> L<low> U<up>. (Requires servos)
 * M290 - Babystepping (Requires BABYSTEPPING)
 * M300 - Play beep sound S[frequency Hz] P[duration ms]
 * M301 - Set PID parameters P I D and C.
 *          H[heaters] 0-5 Hotend, -1 BED, -2 CHAMBER, -3 COOLER
 *          T[int] 0-3 For Select Beds or Chambers (default 0)
 *          P[float] Kp term, I[float] Ki term, D[float] Kd term
 *          With PID_ADD_EXTRUSION_RATE: C[float] Kc term, L[int] LPQ length
 * M302 - Allow cold extrudes, or set the minimum extrude S[temperature].
 * M303 - PID relay autotune.
 *          H[heaters] 0-5 Hotend, -1 BED, -2 CHAMBER, -3 COOLER
 *          T[int] 0-3 For Select Beds or Chambers (default 0)
 *          S[temperature] sets the target temperature (default target temperature = 150C), C[cycles], U[Apply result],
 *          R[Method] 0 = Classic Pid, 1 = Some overshoot, 2 = No Overshoot, 3 = Pessen Pid
 * M305 - Set thermistor and ADC parameters.
 *          H[heaters] 0-5 Hotend, -1 BED, -2 CHAMBER, -3 COOLER
 *          T[int] 0-3 For Select Beds or Chambers (default 0)
 *          A[float] Thermistor resistance at 25°C, B[float] BetaK, C[float] Steinhart-Hart C coefficien, R[float] Pullup resistor value,
 *          L[int] ADC low offset correction, O[int] ADC high offset correction, P[int] Sensor Pin
 *          Set DHT sensor parameter: D0 P[int] Sensor Pin, S[int] Sensor Type (11, 21, 22).
 * M306 - Set Heaters parameters.
 *          H[heaters] 0-5 Hotend, -1 BED, -2 CHAMBER, -3 COOLER
 *          T[int] 0-3 For Select Beds or Chambers (default 0)
 *          A[int] Power Drive Min, B[int] Power Drive Max, C[int] Power Max, F[int] Frequency
 *          L[int] Min temperature, O[int] Max temperature, U[bool] Use Pid/bang bang,
 *          I[bool] Hardware Inverted, T[bool] Thermal Protection, P[int] Pin, Q[bool] PWM Hardware
 * M350 - Set microstepping mode. (Requires digital microstepping pins.)
 * M351 - Toggle MS1 MS2 pins directly. (Requires digital microstepping pins.)
 * M352 - Set driver pins and logic. X X2 Y Y2 Z Z2 Z3 T0-5 E[Enable pin] D[Dir pin] S[Step pin] L[enable logic] M[step logic]
 * M353 - Set number total Tools. D[int] Set number driver extruder, E[int] Set number extruder, H[int] Set number hotend, B[int] Set number bed, C[int] Set number chamber, F[int] Set number fan
 * M355 - Turn case lights on/off
 * M380 - Activate solenoid on active extruder
 * M381 - Disable all solenoids
 * M400 - Finish all moves
 * M401 - Lower z-probe if present
 * M402 - Raise z-probe if present
 * M404 - N[dia in mm] Enter the nominal filament width (3mm, 1.75mm ) or will display nominal filament width without parameters
 * M405 - Turn on Filament Sensor extrusion control. Optional D[delay in cm] to set delay in centimeters between sensor and extruder
 * M406 - Turn off Filament Sensor extrusion control
 * M407 - Display measured filament diameter
 * M408 - Report JSON-style response
 * M410 - Quickstop. Abort all the planned moves
 * M412 - Filament Runout Sensor. (Requires FILAMENT_RUNOUT_SENSOR)
 *          S[bool]   Enable / Disable Sensor control
 *          H[bool]   Enable / Disable Host control
 *          R[bool]   Reset control
 *          D[float]  Distance mm
 * M413 - S[bool] Enable / Disable Restart Job. (Requires SD_RESTART_FILE)
 * M420 - Enable/Disable Leveling (with current values) S1=enable S0=disable (Requires MBL, UBL or ABL)
 *          Z[height] for leveling fade height (Requires ENABLE_LEVELING_FADE_HEIGHT)
 * M421 - Set a single Z coordinate in the Mesh Leveling grid. X[units] Y[units] Z[units] (Requires MESH_BED_LEVELING, AUTO_BED_LEVELING_BILINEAR, or AUTO_BED_LEVELING_UBL)
 * M428 - Set the home_offset logically based on the position
 * M450 - Report Printer Mode
 * M451 - Select FFF Printer Mode
 * M452 - Select Laser Printer Mode
 * M453 - Select CNC Printer Mode
 * M500 - Store parameters in EEPROM
 * M501 - Read parameters from EEPROM (if you need reset them after you changed them temporarily).
 * M502 - Revert to the default "factory settings". You still need to store them in EEPROM afterwards if you want to.
 * M503 - Print the current settings (from memory not from EEPROM)
 * M504 - Validate EEPROM Contents
 * M505 - Clear EEPROM and RESET Printer
 * M522 - Read or Write on card. M522 T[extruders] R[read] or W[write] L[list]
 * M524 - Abort the current SD print job (started with M24). (Requires SDSUPPORT)
 * M530 - Enables explicit printing mode (S1) or disables it (S0). L can set layer count
 * M531 - filename - Define filename being printed
 * M532 - X[percent] L[curLayer] - update current print state progress (X=0..100) and layer L
 * M540 - Use S[0|1] to enable or disable the stop print on endstop hit (requires SD_ABORT_ON_ENDSTOP_HIT)
 * M563 - Set Tools Driver Hotend assignment
 *          T[tools]  - Set Tool
 *          D[int]    - Set Driver for tool
 *          H[int]    - Set Hotend for tool
 * M569 - Stepper driver control X[bool] Y[bool] Z[bool] T[extruders] E[bool] set direction,
 *          D[long] set direction delay, P[int] set minimum pulse, R[long] set maximum rate, Q[bool] Enable/Disable double/quad stepping.
 * M575 - Change serial baud rate P[Port index] B[Baudrate]
 * M595 - Set AD595 or AD8495 O[offset] and S[gain]
 * M600 - Pause for filament change T[toolhead] X[pos] Y[pos] Z[relative lift]
 *          E[initial retract] U[Retract distance] L[Extrude distance] S[new temp] B[Number of beep]
 * M603 - Set filament change T[toolhead] U[Retract distance] L[Extrude distance]
 * M605 - Set dual x-carriage movement mode: S[mode] [ X[duplication x-offset] R[duplication temp offset] ]
 * M649 - Set laser options. S[intensity] L[duration] P[ppm] B[set mode] R[raster mm per pulse] F[feedrate]
 * M666 - Delta geometry adjustment
 * M666 - Set Two Endstops offsets for X, Y, and/or Z (requires TWO ENDSTOPS)
 * M672 - Set/reset Probe Smart Effector sensitivity, S[sensitivity] 0-255, R reset sensitivity to default
 * M701 - Load Filament T[toolhead] Z[distance] L[Extrude distance]
 * M702 - Unload Filament T[toolhead] Z[distance] U[Retract distance]
 * M800 - S goto to lcd menu. With no parameters run restart commands.
 * M851 - Set X Y Z Probe Offset in current units, set speed [F]ast and [S]low, [R]epetititons. (Requires Probe)
 * M876 - Host dialog handling.
 * M890 - Run User Gcode. S[int] Start User Gcode 1 - 5.
 * M900 - T[tool] K[factor] Set Linear Advance K-factor. (Requires LIN_ADVANCE)
 * M906 - Set motor currents XYZ T0-4 E (Requires ALLIGATOR)
 *        Set or get motor current in milliamps using axis codes X, Y, Z, E. Report values if no axis codes given. (Requires TRINAMIC)
 * M907 - Set digital trimpot motor current using axis codes. (Requires a board with digital trimpots)
 * M908 - Control digital trimpot directly. (Requires DIGIPOTSS_PIN)
 * M911 - Report stepper driver overtemperature pre-warn condition. (Requires TRINAMIC)
 * M912 - Clear stepper driver overtemperature pre-warn condition flag. (Requires TRINAMIC)
 * M913 - Set HYBRID_THRESHOLD speed. (Requires HYBRID_THRESHOLD)
 * M914 - Set StallGuard sensitivity. (Requires SENSORLESS_HOMING)
 * M915 - TMC Z axis calibration routine. (Requires TMC)
 * M922 - Enable/disable TMC debug. (Requires TMC_DEBUG)
 * M930 - TMC set blank_time.
 * M931 - TMC set off_time.
 * M932 - TMC set hysteresis_start.
 * M933 - TMC set hysteresis_end/sine_offset (chm = 0/1).
 * M934 - TMC set fast_decay_time (chm = 1).
 * M935 - TMC set disable_I_comparator (chm = 1).
 * M936 - TMC set stealth_gradient.
 * M937 - TMC set stealth_amplitude.
 * M938 - TMC set stealth_freq.
 * M939 - TMC switch stealth_autoscale.
 * M940 - TMC switch StealthChop.
 * M941 - TMC switch ChopperMode.
 * M942 - TMC switch interpolation.
 *
 * ************ SCARA Specific - This can change to suit future G-code regulations
 * M360 - SCARA calibration: Move to cal-position ThetaA (0 deg calibration)
 * M361 - SCARA calibration: Move to cal-position ThetaB (90 deg calibration - steps per degree)
 * M362 - SCARA calibration: Move to cal-position PsiA (0 deg calibration)
 * M363 - SCARA calibration: Move to cal-position PsiB (90 deg calibration - steps per degree)
 * M364 - SCARA calibration: Move to cal-position PsIC (90 deg to Theta calibration position)
 * ************* SCARA End ***************
 *
 * M928 - Start SD logging (M928 filename.g) - ended by M29
 * M995 - X Y Z Set origin for graphic in NEXTION
 * M996 - S[scale] Set scale for graphic in NEXTION
 * M999 - Restart after being stopped by error
 *
 * "T" Codes
 *
 * T0-T5 - Select a tool by index (usually an extruder) F[mm/min]
 *
 */

#include "MK4duo.h"
