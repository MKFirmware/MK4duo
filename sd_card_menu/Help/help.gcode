M118 list g-codes commands
M118 g0  - g1 except for laser where g0 is move without firing
M118 g1  - coordinated movement x y z e f(feedrate) p(purge), for laser move by firing
M118 g2  - cw arc
M118 g3  - ccw arc
M118 g4  - dwell s[seconds] or p[milliseconds], delay in second or millisecond
M118 g5  - bezier curve
M118 g7  - laser raster base64
M118 g10 - retract filament according to settings of m207
M118 g11 - retract recover filament according to settings of m208
M118 g12 - nozzle clean
M118 g20 - set input units to inches
M118 g21 - set input units to millimeters
M118 g27 - nozzle park
M118 g28 - x y z home all axis. m for bed manual setting with lcd. b return to back point
M118 g29 - detailed z-probe, probes the bed at 3 or more points. will fail if you havent homed yet.
M118 .      fyyy lxxx rxxx byyy for customer grid.
M118 g30 - single z probe, probes bed at x y location (defaults to current xy location)
M118 g31 - dock sled (z_probe_sled only)
M118 g32 - undock sled (z_probe_sled only)
M118 g33 - delta geometry autocalibration
M118 .      fnfactor pnpoint qdebugging (requires delta_auto_calibration_1)
M118 .      pnpoints vnverbose (requires delta_auto_calibration_2)
M118 .      aprecision eprecision rprecision i d t s (requires delta_auto_calibration_3)
M118 g38 - probe target - similar to g28 except it uses the z_min endstop for all three axes
M118 g60 - save current position coordinates (all axes, for active extruder).
M118 .      sslot - specifies memory slot n (0-based) to save into (default 0).
M118 g61 - apply/restore saved coordinates to the active tools.
M118 .      x y z e - value to add at stored coordinates.
M118 .      fspeed - set feedrate.
M118 .      sslot - specifies memory slot n (0-based) to restore from (default 0).
M118 g90 - use absolute coordinates
M118 g91 - use relative coordinates
M118 g92 - set current position to coordinates given
M118 m codes
M118 m0   - unconditional stop - wait for user to press a button on the lcd (only if ultra_lcd is enabled)
M118 m1   - same as m0
M118 m3   - svalue lduration pppm ddiagnostic bset mode in laser beam control. (requires laserbeam)
M118 .      svalue cnc clockwise speed. (requires cncrouters)
M118 m4   - svalue cnc counter clockwise speed. (requires cncrouters)
M118 m5   - turn off laser beam. (requires laserbeam) - turn off cnc. (requires cncrouters)
M118 m6   - tool change cnc. (requires cncrouters)
M118 m17  - enable/power all stepper motors
M118 m18  - disable all stepper motors; same as m84
M118 m20  - list sd card
M118 m21  - init sd card
M118 m22  - release sd card
M118 m23  - select sd file (m23 filename.g)
M118 m24  - start/resume sd print
M118 m25  - pause sd print
M118 m26  - set sd position in bytes (m26 s12345)
M118 m27  - report sd print status
M118 m28  - start sd write (m28 filename.g)
M118 m29  - stop sd write
M118 m30  - delete file from sd (m30 filename.g)
M118 m31  - output time since last m109 or sd card start to serial
M118 m32  - make directory
M118 m33  - stop printing, close file and save restart.gcode
M118 m34  - open file and start print
M118 m35  - upload firmware to nextion from sd
M118 m42  - change pin status via gcode use m42 px sy to set pin x to value y, when omitting px the onboard led will be used.
M118 m43  - display pin status, watch pins for changes, watch endstops & toggle led, z servo probe test, toggle pins
M118 m48  - measure z_probe repeatability. m48 [p n of points] [x position] [y position] [v_erboseness n] [e_ngage probe] [l n of legs of travel]
M118 m70  - power consumption sensor calibration
M118 m75  - start the print job timer
M118 m76  - pause the print job timer
M118 m77  - stop the print job timer
M118 m78  - show statistical information about the print jobs
M118 m80  - turn on power supply
M118 m81  - turn off power supply
M118 m82  - set e codes absolute (default)
M118 m83  - set e codes relative while in absolute coordinates (g90) mode
M118 m84  - disable steppers until next move,
M118 .      or use sseconds to specify an inactivity timeout, after which the steppers will be disabled.  s0 to disable the timeout.
M118 m85  - set inactivity shutdown timer with parameter sseconds. to disable set zero (default)
M118 m92  - set axis_steps_per_unit - same syntax as g92
M118 m96  - print zwobble value
M118 m97  - set zwobble parameter m97 aamplitude_in_mm wperiod_in_mm pphase_in_degrees
M118 m98  - print hysteresis value
M118 m99  - set hysteresis parameter m99 xin mm yin mm zin mm ein mm
M118 m100 - watch free memory (for debugging only)
M118 m104 - set hotend target temp
M118 m105 - read current temp
M118 m106 - sspeed pfan fan on
M118 m107 - pfan fan off
M118 m109 - sxxx wait for hotend current temp to reach target temp. waits only when heating
M118 .      rxxx wait for hotend current temp to reach target temp. waits when heating and cooling
M118 .      if autotemp is enabled, smintemp bmaxtemp ffactor. exit autotemp by any m109 without f
M118 m110 - set the current line number
M118 m111 - set debug flags with smask.
M118 m112 - emergency stop
M118 m114 - output current position to serial port
M118 m115 - report capabilities. (extended capabilities requires extended_capabilities_report)
M118 m117 - display a message on the controller screen
M118 m119 - output endstop status to serial port
M118 m120 - enable endstop detection
M118 m121 - disable endstop detection
M118 m122 - s1=true|0=false enable or disable check software endstop. (requires min_software_endstops or max_software_endstops)
M118 m126 - solenoid air valve open (baricuda support by jmil)
M118 m127 - solenoid air valve closed (baricuda vent to atmospheric pressure by jmil)
M118 m128 - etop open (baricuda etop = electricity to air pressure transducer by jmil)
M118 m129 - etop closed (baricuda etop = electricity to air pressure transducer by jmil)
M118 m140 - set hot bed target temp
M118 m141 - set hot chamber target temp
M118 m142 - set cooler target temp
M118 m145 - set the heatup state hhotend bbed ffan speed for smaterial (0=pla, 1=abs)
M118 m149 - set temperature units
M118 m150 - set blinkm color output or rgb led r red0-255 u green0-255 b blue0-255 over i2c, g for green does not work.
M118 m155 - auto-report temperatures with interval of sseconds. (requires auto_report_temperatures)
M118 m163 - set a single proportion for a mixing tools. (requires mixing_extruder)
M118 m164 - save the mix as a virtual tools. (requires mixing_extruder and mixing_virtual_tools)
M118 m165 - set the proportions for a mixing tools. use parameters abcdhi to set the mixing factors. (requires mixing_extruder)
M118 m190 - sxxx wait for bed current temp to reach target temp. waits only when heating
M118 .      rxxx wait for bed current temp to reach target temp. waits when heating and cooling
M118 m191 - sxxx wait for chamber current temp to reach target temp. waits only when heating
M118 .      rxxx wait for chamber current temp to reach target temp. waits when heating and cooling
M118 m192 - sxxx wait for cooler current temp to reach target temp. waits only when heating
M118 .      rxxx wait for cooler current temp to reach target temp. waits when heating and cooling
M118 m200 - set filament diameter and set e axis units to cubic millimeters (use s0 to set back to millimeters).dmillimeters- 
M118 m201 - set max acceleration in units/s2 for print moves (m201 x1000 y1000)
M118 m202 - set max acceleration in units/s2 for travel moves (m202 x1000 y1000) unused in marlin!!
M118 m203 - set maximum feedrate that your machine can sustain (m203 x200 y200 z300 e10000) in mm/sec
M118 m204 - set default acceleration p for printing moves, r for retract only (no x, y, z) moves and t for travel (non printing) moves (ex. m204 p800 t3000 r9000) in mm/sec2
M118 m205 - advanced settings  minimum travel speed s=while printing t=travel only,  b=minimum segment time x= maximum xy jerk, z=maximum z jerk, e=maximum e jerk
M118 m206 - set additional homing offset
M118 m207 - set retract length s[positive mm] f[feedrate mm/min] z[additional zlift/hop], stays in mm regardless of m200 setting
M118 m208 - set recover=unretract length s[positive mm surplus to the m207 s] f[feedrate mm/min]
M118 m209 - s1=true/0=false enable automatic retract detect if the slicer did not support g10/11 every normal extrude-only move will be classified as retract depending on the direction.
M118 m218 - set hotend offset (in mm) textruder_number xoffset_on_x yoffset_on_y
M118 m220 - set speed factor override percentage sfactor in percent
M118 m221 - textruder sfactor in percent - set extrude factor override percentage
M118 m222 - textruder sfactor in percent - set density extrude factor percentage for purge
M118 m226 - wait until the specified pin reaches the state required ppin number spin state
M118 m240 - trigger a camera to take a photograph
M118 m250 - set lcd contrast ccontrast value (value 0..63)
M118 m280 - set servo position absolute. p servo index, s angle or microseconds
M118 m300 - play beep sound sfrequency hz pduration ms
M118 m301 - set pid parameters p i d and c
M118 m302 - allow cold extrudes, or set the minimum extrude stemperature.
M118 m303 - pid relay autotune stemperature sets the target temperature (default target temperature = 150c). hhotend ccycles uapply result
M118 m304 - set hot bed pid parameters p i and d
M118 m305 - set hot chamber pid parameters p i and d
M118 m306 - set cooler pid parameters p i and d
M118 m320 - enable/disable s1=enable s0=disable, v[bool] print the leveling grid, zheight for leveling fade height (requires enable_leveling_fade_height)
M118 m321 - set a single auto bed leveling z coordinate - xgridx ygridy zlevel val slevel add
M118 m322 - reset auto bed leveling matrix
M118 m323 - set level bilinear manual - xgridx ygridy zlevel val slevel add
M118 m350 - set microstepping mode.
M118 m351 - toggle ms1 ms2 pins directly.
M118 m355 - turn case lights on/off
M118 m380 - activate solenoid on active extruder
M118 m381 - disable all solenoids
M118 m400 - finish all moves
M118 m401 - lower z-probe if present
M118 m402 - raise z-probe if present
M118 m404 - ndia in mm enter the nominal filament width (3mm, 1.75mm ) or will display nominal filament width without parameters
M118 m405 - turn on filament sensor extrusion control.  optional ddelay in cm to set delay in centimeters between sensor and extruder
M118 m406 - turn off filament sensor extrusion control
M118 m407 - display measured filament diameter
M118 m408 - report json-style response
M118 m410 - quickstop. abort all the planned moves
M118 m420 - enable/disable mesh bed leveling (with current values) s1=enable s0=disable (requires mesh_bed_leveling)
M118 .      zheight for leveling fade height (requires enable_leveling_fade_height)
M118 m421 - set a single mesh bed leveling z coordinate. m421 xmm ymm zmm or m421 ixindex jyindex zmm
M118 m428 - set the home_offset logically based on the current_position
M118 m450 - report printer mode
M118 m451 - select fff printer mode
M118 m452 - select laser printer mode
M118 m453 - select cnc printer mode
M118 m500 - store parameters in eeprom
M118 m501 - read parameters from eeprom (if you need reset them after you changed them temporarily).
M118 m502 - revert to the default factory settings. you still need to store them in eeprom afterwards if you want to.
M118 m503 - print the current settings (from memory not from eeprom). use s0 to leave off headings.
M118 m522 - read or write on card. m522 textruders rread or wwrite llist
M118 m530 - enables explicit printing mode (s1) or disables it (s0). l can set layer count
M118 m531 - filename - define filename being printed
M118 m532 - xpercent lcurlayer - update current print state progress (x=0..100) and layer l
M118 m540 - use s[0|1] to enable or disable the stop print on endstop hit (requires abort_on_endstop_hit_feature_enabled)
M118 m595 - set hotend ad595 ooffset and sgain
M118 m600 - pause for filament change x[pos] y[pos] z[relative lift] e[initial retract] l[later retract distance for removal]
M118 m605 - set dual x-carriage movement mode smode [ xduplication x-offset rduplication temp offset ]
M118 m649 - set laser options. sintensity lduration pppm bset mode rraster mm per pulse ffeedrate
M118 m666 - set z probe offset or endstop and delta geometry adjustment
M118 m900 - kfactor rratio wwidth hheight ddiam - set and/or get advance k factor and wh/d ratio
M118 m906 - set motor currents xyz t0-4 e (requires alligator)
M118 .      set or get motor current in milliamps using axis codes x, y, z, e. report values if no axis codes given. (requires have_tmc2130)
M118 m907 - set digital trimpot motor current using axis codes. (requires a board with digital trimpots)
M118 m908 - control digital trimpot directly. (requires digipotss_pin)
M118 m911 - report stepper driver overtemperature pre-warn condition. (requires have_tmc2130)
M118 m912 - clear stepper driver overtemperature pre-warn condition flag. (requires have_tmc2130)
M118 m913 - set hybrid_threshold speed. (requires hybrid_threshold)
M118 m914 - set sensorless_homing sensitivity. (requires sensorless_homing)
M118 m360 - scara calibration move to cal-position thetaa (0 deg calibration)
M118 m361 - scara calibration move to cal-position thetab (90 deg calibration - steps per degree)
M118 m362 - scara calibration move to cal-position psia (0 deg calibration)
M118 m363 - scara calibration move to cal-position psib (90 deg calibration - steps per degree)
M118 m364 - scara calibration move to cal-position psic (90 deg to theta calibration position)
M118 m928 - start sd logging (m928 filename.g) - ended by m29
M118 m995 - x y z set origin for graphic in nextion
M118 m996 - sscale set scale for graphic in nextion
M118 m997 - npr2 color rotate
M118 m999 - restart after being stopped by error
M118 t codes
M118 t0-t5 - select a tool by index (usually an extruder) [ fmm/min ]
