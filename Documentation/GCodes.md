# Implemented Codes

## G Codes

| Code | Descripion |
| ---: | :--- |
|   G0 | G1
|   G1 | Coordinated Movement X Y Z E F(feedrate) P(Purge)
|   G2 | CW ARC
|   G3 | CCW ARC
|   G4 | Dwell S[seconds] or P[milliseconds], delay in Second or Millisecond
|   G5 | Bezier curve - from [forums.reprap.org](http://forums.reprap.org/read.php?147,93577)
|   G7 | Laser raster base64
|  G10 | retract filament according to settings of M207
|  G11 | retract recover filament according to settings of M208
|  G12 | Nozzle Clean
|  G20 | Set input units to inches
|  G21 | Set input units to millimeters
|  G26 | Mesh Validation Pattern. (Requires G26 MESH VALIDATION & **AUTO BED LEVELING UBL** or **MESH BED LEVELING** or **AUTO BED LEVELING BILINEAR**) 
|  G27 | Nozzle Park
|  G28 | X Y Z Home all Axis. M for bed manual setting with LCD. B return to back point. O Home only if position is unknown
|  G29 | Detailed Z probe, probes the bed at 3 or more points. Will fail if you haven't homed yet.<br/>`G29   Fyyy Lxxx Rxxx Byyy` for customer grid.
|  G30 | Single Z Probe, probes bed at current XY location.
|  G31 | Dock Z Probe sled (if enabled)
|  G32 | Undock Z Probe sled (if enabled)
|  G33 | Delta geometry Autocalibration[br/>F[nfactor] P[npoint] Q[debugging> (Requires **DELTA AUTO CALIBRATION 1**)<br/>P[npoints] V[nverbose> (Requires **DELTA AUTO CALIBRATION 2**)
|  G38 | Probe target - similar to **G28** except it uses the Z MIN endstop for all three axes
|  G42 | Coordinated move to a mesh point. (Requires **MESH BED LEVELING** or **AUTO BED LEVELING BILINEAR**)
|  G60 | Save current position coordinates (all axes, for active extruder).<br/>S[SLOT] - specifies memory slot # (0-based) to save into (default 0)
|  G61 | Apply/restore saved coordinates to the active extruder.<br/>X Y Z E - Value to add at stored coordinates<br/>F[speed] - Set Feedrate<br/>S[SLOT] - specifies memory slot # (0-based) to restore from (default 0)
|  G90 | Use Absolute Coordinates
|  G91 | Use Relative Coordinates
|  G92 | Set current position to cordinates given

## M codes
| Code | Requires | Descripion |
| ---: | :---: | :--- |
|   M0 | ULTRA LCD | Unconditional stop - Wait for user to press a button on the LCD
|   M1 | \\\ | Same as M0
|   M3 | LASER | S[value] L[duration] P[ppm] D[diagnostic] B[set mode> in laser beam control
|   M4 | CNCROUTERS | S[value] - CNC clockwise speed
|   M5 | LASER or CNCROUTERS | Turn off laser beam or CNC
|   M6 | CNCROUTERS | Tool change CNC
|  M17 | - | Enable/Power all stepper motors
|  M18 | - | Disable all stepper motors; same as M84
|  M20 | SDCARD | List SD card
|  M21 | SDCARD | Init SD card
|  M22 | SDCARD | Release SD card
|  M23 | SDCARD | Select SD file (M23 filename.g)
|  M24 | SDCARD | Start/resume SD print
|  M25 | SDCARD | Pause SD print
|  M26 | SDCARD | Set SD position in bytes (M26 S12345)
|  M27 | SDCARD | Report SD print status
|  M28 | SDCARD | Start SD write (M28 filename.g)
|  M29 | SDCARD | Stop SD write
|  M30 | SDCARD | Delete file from SD (M30 filename.g)
|  M31 | SDCARD | Output time since last M109 or SD card start to serial
|  M32 | SDCARD | Make directory
|  M34 | SDCARD | Set SD Card Sorting Options
|  M35 | NEXTION | Upload Firmware to Nextion from SD
|  M36 | SDCARD | Set SD Card Sorting Options
|  M42 | ? | Change pin status via gcode Use M42 Px Sy to set pin x to value y, when omitting Px the onboard led will be used.
|  M43 | PINS DEBUGGING | Display pin status, watch pins for changes, watch endstops & toggle LED, Z servo probe test, toggle pins[br/>Report name and state of pin(s)<br/>P[pin] - Pin to read or watch. If omitted, reads all pins<br/>I - Flag to ignore Mk4duo's pin protection<br/>`W` - Watch pins -reporting changes- until reset, click, or M108[br/>P[pin] - Pin to read or watch. If omitted, read/watch all pins<br/>I - Flag to ignore Marlin's pin protection<br/>`E[bool>` - Enable / disable background endstop monitoring[br/>- Machine continues to operate[br/>- Reports changes to endstops[br/>- Toggles LED when an endstop changes[br/>- Can not reliably catch the 5mS pulse from BLTouch type probes[br/>`T` - Toggle pin(s) and report which pin is being toggled[br/>S[pin] - Start Pin number.   If not given, will default to 0<br/>L[pin] - End Pin number.   If not given, will default to last pin defined for this board<br/>I - Flag to ignore Mk4duo's pin protection **Use with caution!!!**<br/>R - Repeat pulses on each pin this number of times before continueing to next pin<br/>W - Wait time (in miliseconds) between pulses.  If not given will default to 500<br/>`S` - Servo probe test[br/>P[index] - Probe index (optional - defaults to 0<br/>
|  M43 | ? | M43 S1 P[servo] Z servo probe test.
|  M44 | ? | Codes debug - report codes available (and how many of them there are)<br/>I - G-code list<br/>J - M-code list
|  M48 | ? | Measure Z Probe repeatability. M48 [P # of points] [X position] [Y position] [V_erboseness #] [E_ngage Probe] [L # of legs of travel]
|  M48 | G26 MESH VALIDATION | Turn on or off G26 debug flag for verbose output.
|  M70 | ? | Power consumption sensor calibration
|  M75 | ? | Start the print job timer
|  M76 | ? | Pause the print job timer
|  M77 | ? | Stop the print job timer
|  M78 | ? | Show statistical information about the print jobs
|  M80 | ? | Turn on Power Supply
|  M81 | ? | Turn off Power, including Power Supply, if possible
|  M82 | ? | Set E codes absolute (default)
|  M83 | ? | Set E codes relative while in Absolute Coordinates (G90) mode
|  M84 | ? | Disable steppers until next move, or use S[seconds] to specify an inactivity timeout, after which the steppers will be disabled.  S0 to disable the timeout.
|  M85 | ? | Set inactivity shutdown timer with parameter S[seconds]. To disable set zero (default)
|  M92 | ? | Set axis steps per unit - same syntax as G92
|  M99 | HYSTERESIS FEATURE | Set Hysteresis parameter M99 X[in mm] Y[in mm] Z[in mm] F[float] Enable/disable/fade-out hysteresis correction (0.0 to 1.0)
| M100 | ? | Watch Free Memory (For Debugging Only)
| M104 | ? | Set hotend target temp
| M105 | ? | Read current temp
| M106 | ? | P[fan] S[speed] F[frequency] U[pin] L[min speed] I[inverted logic] H[int] Set Auto mode - H=7 for controller - H-1 for disabled
| M107 | ? | P[fan] Fan off
| M108 | EMERGENCY PARSER | Break out of heating loops (M109, M190, M303). With no controller, breaks out of M0/M1.
| M109 | ? | S[xxx] - Wait for hotend current temp to reach target temp. Waits only when heating<br/>R[xxx] - Wait for hotend current temp to reach target temp. Waits when heating and cooling<br/>IF AUTOTEMP is enabled, S[mintemp] B[maxtemp] F[factor>. Exit autotemp by any M109 without F
| M110 | ? | Set the current line number
| M111 | ? | Set debug flags with S[mask>.
| M112 | ? | Emergency stop
| M114 | ? | Output current position to serial port
| M115 | EXTENDED CAPABILITIES REPORT* | Report capabilities. (* for extended capabilities)
| M117 | ? | Display a message on the controller screen
| M118 | ? | Display a message in the host console
| M119 | ? | Output Endstop status to serial port
| M120 | ? | Enable endstop detection
| M121 | ? | Disable endstop detection
| M122 | MIN SOFTWARE ENDSTOPS or MAX SOFTWARE ENDSTOPS | S[bool] Enable or disable check software endstop
| M123 | ENDSTOP | Set Endstop Logic[br/>X[bool] Y[bool] Z[bool] I[X2 bool] J[Y2 bool] K[Z2 bool] P[Probe bool] D[Door bool] F[Filrunout bool] W[Power Check bool>
| M124 | ENDSTOP | Set Endstop Pullup[br/>X[bool] Y[bool] Z[bool] I[X2 bool] J[Y2 bool] K[Z2 bool] P[Probe bool] D[Door bool] F[Filrunout bool] W[Power Check bool>
| M125 | PARK HEAD ON PAUSE | Save current position and move to pause park position 
| M126 | ? | Solenoid Air Valve Open (BariCUDA support by jmil)
| M127 | ? | Solenoid Air Valve Closed (BariCUDA vent to atmospheric pressure by jmil)
| M128 | ? | EtoP Open (BariCUDA EtoP = electricity to air pressure transducer by jmil)
| M129 | ? | EtoP Closed (BariCUDA EtoP = electricity to air pressure transducer by jmil)
| M140 | ? | Set hot bed target temp
| M141 | ? | Set hot chamber target temp
| M142 | ? | Set cooler target temp
| M145 | ? | Set the heatup state H[hotend] B[bed] F[fan speed] for S[material] (0=PLA, 1=ABS, 2=GUM)
| M150 | BLINKM, RGB LED, RGBW LED, or PCA9632 | Set Status LED Color as R[red] U[green] B[blue] values 0-255
| M155 | ? | Auto report temperatures S[bool] Enable/disable
| M163 | COLOR MIXING EXTRUDER | Set a single proportion for a mixing extruder 
| M164 | COLOR MIXING EXTRUDER and MIXING VIRTUAL TOOLS | Save the mix as a virtual extruder 
| M165 | COLOR MIXING EXTRUDER | Set the proportions for a mixing extruder. Use parameters ABCDHI to set the mixing factors
| M190 | ? | Sxxx - Wait for bed current temp to reach target temp. Waits only when heating<br/>Rxxx - Wait for bed current temp to reach target temp. Waits when heating and cooling
| M191 | ? | Sxxx - Wait for chamber current temp to reach target temp. Waits only when heating<br/>Rxxx Wait for chamber current temp to reach target temp. Waits when heating and cooling
| M192 | ? | Sxxx Wait for cooler current temp to reach target temp. Waits only when heating<br/>Rxxx Wait for cooler current temp to reach target temp. Waits when heating and cooling
| M200 | ? | D[millimeters]- set filament diameter and set E axis units to cubic millimeters (use S0 to set back to millimeters).
| M201 | ? | Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000 Z1000 E0 S1000 E1 S1000 E2 S1000 E3 S1000) in mm/sec^2
| M203 | ? | Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E0 S1000 E1 S1000 E2 S1000 E3 S1000) in mm/sec
| M204 | ? | Set Accelerations in mm/sec^2: S printing moves, R Retract moves(only E), T travel moves (M204 P1200 R3000 T2500) im mm/sec^2  also sets minimum segment time in ms (B20000) to prevent buffer underruns and M20 minimum feedrate.
| M205 | ? | Set Advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk, E=maximum E jerk, J=Junction deviation mm
| M206 | ? | set additional homing offset
| M207 | ? | set retract length S[positive mm] F[feedrate mm/min] Z[additional zlift/hop], stays in mm regardless of M200 setting
| M208 | ? | set recover=unretract length S[positive mm surplus to the M207 S*] F[feedrate mm/min]
| M209 | ? | S[1=true/0=false] enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
| M218 | ? | set hotend offset (in mm): H[hotend_number] X[offset_on_X] Y[offset_on_Y]> Z[offset_on_Z]
| M220 | ? | S[factor in percent] - set speed factor override percentage
| M221 | ? | T[extruder] S[factor in percent] - set extrude factor override percentage
| M222 | ? | T[extruder] S[factor in percent] - set density extrude factor percentage for purge
| M240 | ? | Trigger a camera to take a photograph
| M280 | SERVO | Position an RC Servo P[index] S[angle/microseconds], ommit S to report back current angle
| M281 | SERVO | Set servo low|up angles position. P[index] L[low] U[up]
| M300 | ? | Play beep sound S[frequency Hz] P[duration ms]
| M301 | ? | Set PID parameters P I D and C. H[heaters] H = 0-3 Hotend, H = -1 BED, H = -2 CHAMBER, H = -3 COOLER, P[float] Kp term, I[float] Ki term, D[float] Kd term. With PID ADD EXTRUSION RATE: C[float] Kc term, L[float] LPQ length
| M302 | ? | Allow cold extrudes, or set the minimum extrude S[temperature>.
| M303 | ? | PID relay autotune: H[heaters] H = 0-3 Hotend, H = -1 BED, H = -2 CHAMBER, H = -3 COOLER, S[temperature] sets the target temperature (default target temperature = 200C), C[cycles>, R[method>, U[Apply result>, R[Method] 0 = Classic Pid, 1 = Some overshoot, 2 = No Overshoot, 3 = Pessen Pid.
| M305 | ? | Set thermistor and ADC parameters: H[heaters] H = 0-3 Hotend, H = -1 BED, H = -2 CHAMBER, H = -3 COOLER, A[float] Thermistor resistance at 25°C, B[float] BetaK, C[float] Steinhart-Hart C coefficien, R[float] Pullup resistor value, L[int] ADC low offset correction, O[int] ADC high offset correction, P[int] Sensor Pin. Set DHT sensor parameter: D0 P[int] Sensor Pin, S[int] Sensor Type (11, 21, 22).
| M306 | ? | Set Heaters parameters: H[heaters] H = 0-3 Hotend, H = -1 BED, H = -2 CHAMBER, H = -3 COOLER, A[int] Pid Drive Min, B[int] Pid Drive Max, C[int] Pid Max, L[int] Min temperature, O[int] Max temperature, U[bool] Use Pid/bang bang, I[bool] Hardware Inverted, P[int] Pin
| M350 | ? | Set microstepping mode.
| M351 | ? | Toggle MS1 MS2 pins directly.
| M355 | ? | Turn case lights on/off S[bool] on-off, P[brightness]
| M360 | MECH ? SCARA | Move to cal-position ThetaA (0 deg calibration)
| M361 | MECH ? SCARA | Move to cal-position ThetaB (90 deg calibration - steps per degree)
| M362 | MECH ? SCARA | Move to cal-position PsiA (0 deg calibration)
| M363 | MECH ? SCARA | Move to cal-position PsiB (90 deg calibration - steps per degree)
| M364 | MECH ? SCARA | Move to cal-position PSIC (90 deg to Theta calibration position)
| M380 | ? | Activate solenoid on active extruder
| M381 | ? | Disable all solenoids
| M400 | ? | Finish all moves
| M401 | ? | Lower z-probe if present
| M402 | ? | Raise z-probe if present
| M404 | ? | D[dia in mm] Enter the nominal filament width (3mm, 1.75mm) or will display nominal filament width without parameters
| M405 | ? | Turn on Filament Sensor extrusion control. Optional D[delay in cm] to set delay in centimeters between sensor and extruder
| M406 | ? | Turn off Filament Sensor extrusion control
| M407 | ? | Displays measured filament diameter
| M408 | ? | Report JSON-style response
| M410 | ? | Quickstop. Abort all the planned moves
| M420 | ? | Enable/Disable Leveling (with current values) S1=enable S0=disable (Requires MBL, UBL or ABL), Z[height] for leveling fade height (Requires ENABLE LEVELING FADE HEIGHT)
| M421 | ? | Set a single Z coordinate in the Mesh Leveling grid. M421 X[mm] Y[mm] Z[mm>' or 'M421 I[xindex] J[yindex] Z[mm] (Requires MBL, UBL or ABL BILINEAR)
| M428 | ? | Set the home_offset logically based on the current_position
| M450 | ? | Report Printer Mode
| M451 | ? | Select FFF Printer Mode
| M452 | ? | Select Laser Printer Mode
| M453 | ? | Select CNC Printer Mode
| M500 | ? | stores paramters in EEPROM
| M501 | ? | reads parameters from EEPROM (if you need reset them after you changed them temporarily).
| M502 | ? | reverts to the default "factory settings". You still need to store them in EEPROM afterwards if you want to.
| M503 | ? | print the current settings (from memory not from EEPROM)
| M512 | ? | Print Extruder Encoder status Pin. (Requires Extruder Encoder)
| M522 | ? | Use for reader o writer tag with MFRC522. M522 T[extruder] R(read) W(write) L(print list data on tag)
| M530 | ? | Enables explicit printing mode (S1) or disables it (S0). L can set layer count
| M531 | ? | filename - Define filename being printed
| M532 | ? | X[percent] L[curLayer] - update current print state progress (X=0..100) and layer L
| M540 | ABORT ON ENDSTOP HIT  FEATURE ENABLED | Use S[0\|1] to enable or disable the stop print on endstop hit
| M569 | ? | Stepper driver control X[bool] Y[bool] Z[bool] T[extruders] E[bool] set direction, D[long] set direction delay, P[int] set minimum pulse, R[long] set maximum rate.
| M595 | ? | Set AD595 or AD8495 offset & Gain H[hotend] O[offset] S[gain]
| M600 | ADVANCED PAUSE FEATURE | Pause for filament change T[toolhead] X[pos] Y[pos] Z[relative lift] E[initial retract] U[Retract distance] L[Extrude distance] S[new temp] B[Number of beep]
| M603 | ADVANCED PAUSE FEATURE | Set filament change T[toolhead] U[Retract distance] L[Extrude distance]
| M605 | ? | Set dual x-carriage movement mode: Smode [ X[duplication x-offset] Rduplication temp offset ]
| M649 | ? | Set laser options. S[intensity] L[duration] P[ppm] B[set mode] R[raster mm per pulse] F[feedrate]
| M666 | DELTA | Delta geometry adjustment.
| M666 | TWO ENDSTOPS | Set Two Endstops offsets for X, Y, and/or Z. X[float] Y[float] Z[float]
| M701 | ADVANCED PAUSE FEATURE | Load Filament T[toolhead] Z[distance] L[Extrude distance]
| M702 | ADVANCED PAUSE FEATURE | Unload Filament T[toolhead] Z[distance] U[Retract distance]
| M851 | ? | Set X Y Z Probe Offset in current units. (Requires Probe)
| M900 | LIN ADVANCE | K[factor] Set Linear Advance K-factor.
| M906 | ALLIGATOR or TRINAMIC | Set motor currents XYZ T0-4 E. Set or get motor current in milliamps using axis codes X, Y, Z, E. Report values if no axis codes given.
| M907 | a board with digital trimpots | Set digital trimpot motor current using axis codes
| M908 | DIGIPOTSS PIN | Control digital trimpot directly
| M911 | TRINAMIC | Report stepper driver overtemperature pre-warn condition
| M912 | TRINAMIC | Clear stepper driver overtemperature pre-warn condition flag
| M913 | HYBRID THRESHOLD | Set HYBRID THRESHOLD speed
| M914 | SENSORLESS HOMING | Set SENSORLESS HOMING sensitivity
| M915 | TRINAMIC | TMC Z axis calibration routine
| M922 | TRINAMIC | S[1/0] Enable/disable TMC debug
| M928 | ? | Start SD logging (M928 filename.g) - ended by M29
| M995 | ? | X Y Z Set origin for graphic in NEXTION
| M996 | ? | S[scale] Set scale for graphic in NEXTION
| M999 | ? | Restart after being stopped by error
