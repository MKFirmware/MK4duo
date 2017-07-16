/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2017 Alberto Cotronei @MagoKimbra
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
 *
 */

/**
 * mcode.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#if DISABLED(EMERGENCY_PARSER)
  /**
   * M410: Quickstop - Abort all planned moves
   *
   * This will stop the carriages mid-move, so most likely they
   * will be out of sync with the stepper position after this.
   */
  inline void gcode_M410() { printer.quickstop_stepper(); }
#endif

#if ENABLED(MESH_BED_LEVELING)
  /**
   * M420: Enable/Disable Bed Leveling and/or set the Z fade height.
   *
   *    S[bool]   Turns leveling on or off
   *    Z[height] Sets the Z fade height (0 or none to disable)
   *    V[bool]   Verbose - Print the leveling grid
   */
  inline void gcode_M420() {
    bool to_enable = false;

    // V to print the matrix or mesh
    if (parser.seen('V') && bedlevel.leveling_is_valid()) {
      SERIAL_EM("Mesh Bed Level data:");
      bedlevel.mbl_mesh_report();
    }

    if (parser.seen('S')) {
      to_enable = parser.value_bool();
      bedlevel.set_bed_leveling_enabled(to_enable);
    }

    #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
      if (parser.seen('Z')) bedlevel.set_z_fade_height(parser.value_linear_units());
    #endif

    const bool new_status = bedlevel.leveling_is_active();

    if (to_enable && !new_status)
      SERIAL_LM(ER, MSG_ERR_M320_M420_FAILED);

    SERIAL_LMV(ECHO, "MBL: ", new_status ? MSG_ON : MSG_OFF);
  }

  /**
   * M421: Set a single Mesh Bed Leveling Z coordinate
   *
   * Usage:
   *   M421 X<linear> Y<linear> Z<linear>
   *   M421 X<linear> Y<linear> Q<offset>
   *   M421 I<xindex> J<yindex> Z<linear>
   *   M421 I<xindex> J<yindex> Q<offset>
   */
  inline void gcode_M421() {
    const bool hasX = parser.seen('X'), hasI = parser.seen('I');
    const int8_t ix = hasI ? parser.value_int() : hasX ? mbl.probe_index_x(RAW_X_POSITION(parser.value_linear_units())) : -1;
    const bool hasY = parser.seen('Y'), hasJ = parser.seen('J');
    const int8_t iy = hasJ ? parser.value_int() : hasY ? mbl.probe_index_y(RAW_Y_POSITION(parser.value_linear_units())) : -1;
    const bool hasZ = parser.seen('Z'), hasQ = !hasZ && parser.seen('Q');

    if (int(hasI && hasJ) + int(hasX && hasY) != 1 || !(hasZ || hasQ)) {
      SERIAL_LM(ER, MSG_ERR_M421_PARAMETERS);
    }
    else if (ix < 0 || iy < 0) {
      SERIAL_LM(ER, MSG_ERR_MESH_XY);
    }
    else
      mbl.set_z(ix, iy, parser.value_linear_units() + (hasQ ? mbl.z_values[ix][iy] : 0));
  }
#endif // MESH_BED_LEVELING

#if ENABLED(WORKSPACE_OFFSETS)

  /**
   * M428: Set home_offset based on the distance between the
   *       current_position and the nearest "reference point."
   *       If an axis is past center its Endstop position
   *       is the reference-point. Otherwise it uses 0. This allows
   *       the Z offset to be set near the bed when using a max Endstop.
   *
   *       M428 can't be used more than 2cm away from 0 or an Endstop.
   *
   *       Use M206 to set these values directly.
   */
  inline void gcode_M428() {
    bool err = false;
    LOOP_XYZ(i) {
      if (mechanics.axis_homed[i]) {
        const float base = (mechanics.current_position[i] > (endstops.soft_endstop_min[i] + endstops.soft_endstop_max[i]) * 0.5) ? mechanics.base_home_pos[(AxisEnum)i] : 0,
                    diff = base - RAW_POSITION(mechanics.current_position[i], i);
        if (WITHIN(diff, -20, 20)) {
          set_home_offset((AxisEnum)i, diff);
        }
        else {
          SERIAL_LM(ER, MSG_ERR_M428_TOO_FAR);
          LCD_ALERTMESSAGEPGM("Err: Too far!");
          BUZZ(200, 40);
          err = true;
          break;
        }
      }
    }

    if (!err) {
      mechanics.sync_plan_position();
      mechanics.report_current_position();
      LCD_MESSAGEPGM(MSG_HOME_OFFSETS_APPLIED);
      BUZZ(100, 659);
      BUZZ(100, 698);
    }
  }

#endif // ENABLED(WORKSPACE_OFFSETS)

#if HAS_MULTI_MODE

  /**
   * Shared function for Printer Mode GCodes
   */
  static void gcode_printer_mode(const int8_t new_mode) {
    const static char str_tooltype_0[] PROGMEM = "FFF";
    const static char str_tooltype_1[] PROGMEM = "Laser";
    const static char str_tooltype_2[] PROGMEM = "CNC";
    const static char* const tool_strings[] PROGMEM = { str_tooltype_0, str_tooltype_1, str_tooltype_2 };
    if (new_mode >= 0 && (PrinterMode)new_mode < PRINTER_MODE_COUNT) printer.mode = (PrinterMode)new_mode;
    SERIAL_SM(ECHO, "Printer-Mode: ");
    SERIAL_PS((char*)pgm_read_word(&(tool_strings[printer.mode])));
    SERIAL_CHR(' ');
    SERIAL_EV((int)(printer.mode == PRINTER_MODE_FFF ? printer.active_extruder : 0));
  }

  /**
   * M450: Set and/or report current tool type
   *
   *  S<type> - The new tool type
   */
  inline void gcode_M450() {
    gcode_printer_mode(parser.seen('S') ? parser.value_byte() : -1);
  }

  /**
   * M451: Select FFF printer mode
   */
  inline void gcode_M451() { gcode_printer_mode(PRINTER_MODE_FFF); }

  #if ENABLED(LASER)
    /**
     * M452: Select Laser printer mode
     */
    inline void gcode_M452() { gcode_printer_mode(PRINTER_MODE_LASER); }
  #endif

  #if HAS_CNCROUTER
    /**
     * M453: Select CNC printer mode
     */
    inline void gcode_M453() { gcode_printer_mode(PRINTER_MODE_CNC); }
  #endif

#endif // HAS_MULTI_MODE

/**
 * M500: Store settings in EEPROM
 */
inline void gcode_M500() {
  (void)eeprom.Store_Settings();
}

/**
 * M501: Read settings from EEPROM
 */
inline void gcode_M501() {
  (void)eeprom.Load_Settings();
}

/**
 * M502: Revert to default settings
 */
inline void gcode_M502() {
  (void)eeprom.Factory_Settings();
}

/**
 * M503: print settings currently in memory
 */
inline void gcode_M503() {
  (void)eeprom.Print_Settings(parser.seen('S') && !parser.value_bool());
}

#if HAS_EXT_ENCODER
  /**
   * M512: Print Extruder Encoder status Pin
   */
  inline void gcode_M512() {

    #if HAS_E0_ENC
      SERIAL_EMV("Enc0 signal:", (int16_t)READ_ENCODER(E0_ENC_PIN));
    #endif

  }
#endif

#if ENABLED(RFID_MODULE)
  /**
   * M522: Read or Write on card. M522 T<extruders> R<read> or W<write> L<list>
   */
  inline void gcode_M522() {

    GET_TARGET_EXTRUDER(522);
    if (!RFID_ON) return;

    if (parser.seen('R')) {
      SERIAL_EM("Put RFID on tag!");
      #if ENABLED(NEXTION)
        rfid_setText("Put RFID on tag!");
      #endif
      Spool_must_read[TARGET_EXTRUDER] = true;
    }
    if (parser.seen('W')) {
      if (Spool_ID[TARGET_EXTRUDER] != 0) {
        SERIAL_EM("Put RFID on tag!");
        #if ENABLED(NEXTION)
          rfid_setText("Put RFID on tag!");
        #endif
        Spool_must_write[TARGET_EXTRUDER] = true;
      }
      else {
        SERIAL_LM(ER, "You have not read this Spool!");
        #if ENABLED(NEXTION)
          rfid_setText("You have not read this Spool!", 64488);
        #endif
      }
    }

    if (parser.seen('L')) RFID522.printInfo(TARGET_EXTRUDER);
  }
#endif // RFID_MODULE

/**
 * M530: S<printing> L<layer> - Enables explicit printing mode (S1) or disables it (S0). L can set layer count
 */
inline void gcode_M530() {

  if (parser.seen('L')) printer.maxLayer = parser.value_long();

  if (parser.seen('S') && parser.value_bool()) {
    printer.print_job_counter.start();

    SERIAL_MSG("Start Printing");
    if (printer.maxLayer > 0) SERIAL_EMV(" - MaxLayer:", printer.maxLayer);
    else SERIAL_EOL();

    #if ENABLED(START_GCODE)
      commands.enqueue_and_echo_commands_P(PSTR(START_PRINTING_SCRIPT));
    #endif
    #if HAS_FIL_RUNOUT
      printer.filament_ran_out = false;
      SERIAL_EM("Filament runout activated.");
      SERIAL_STR(RESUME);
      SERIAL_EOL();
    #endif
    #if HAS_POWER_CONSUMPTION_SENSOR
      startpower = power_consumption_hour;
    #endif
  }
  else {
    printer.print_job_counter.stop();
    SERIAL_EM("Stop Printing");
    #if ENABLED(STOP_GCODE)
      commands.enqueue_and_echo_commands_P(PSTR(STOP_PRINTING_SCRIPT));
    #endif
    #if HAS_FIL_RUNOUT
      printer.filament_ran_out = false;
      SERIAL_EM("Filament runout deactivated.");
    #endif
  }
}

/**
 * M531: filename - Define filename being printed
 */
inline void gcode_M531() {
  strncpy(printer.printName, parser.string_arg, 20);
  printer.printName[20] = 0;
}

/**
 * M532: X<percent> L<curLayer> - update current print state printer.progress (X=0..100) and layer L
 */
inline void gcode_M532() {
  if (parser.seen('X'))
    printer.progress = parser.value_float();
  if (printer.progress > 100.0)
    printer.progress = 100.0;
  else if (printer.progress < 0)
    printer.progress = 0;

  if (parser.seen('L'))
    printer.currentLayer = parser.value_long();
}

#if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)

  /**
   * M540: Set whether SD card print should abort on endstop hit (M540 S<0|1>)
   */
  inline void gcode_M540() {
    if (parser.seen('S')) stepper.abort_on_endstop_hit = parser.value_bool();
  }

#endif // ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED

#if HEATER_USES_AD595
  /**
   * M595 - set Hotend AD595 offset & Gain H<hotend_number> O<offset> S<gain>
   */
  inline void gcode_M595() {

    GET_TARGET_HOTEND(595);

    if (parser.seen('O')) ad595_offset[TARGET_EXTRUDER] = parser.value_float();
    if (parser.seen('S')) ad595_gain[TARGET_EXTRUDER] = parser.value_float();

    for (int8_t h = 0; h < HOTENDS; h++) {
      // if gain == 0 you get MINTEMP!
      if (ad595_gain[h] == 0) ad595_gain[h]= 1;
    }

    SERIAL_EM(MSG_AD595);
    for (int8_t h = 0; h < HOTENDS; h++) {
      SERIAL_MV(" T", h);
      SERIAL_MV(" Offset: ", ad595_offset[h]);
      SERIAL_EMV(", Gain: ", ad595_gain[h]);
    }
  }
#endif // HEATER_USES_AD595

#if ENABLED(ADVANCED_PAUSE_FEATURE)

  /**
   * M600: Pause Park and filament change
   *
   *  E[distance] - Retract the filament this far (negative value)
   *  Z[distance] - Move the Z axis by this distance
   *  X[position] - Move to this X position, with Y
   *  Y[position] - Move to this Y position, with X
   *  U[distance] - Retract distance for removal (negative value) (manual reload)
   *  L[distance] - Extrude distance for insertion (positive value) (manual reload)
   *  B[count]    - Number of times to beep, -1 for indefinite (if equipped with a buzzer)
   *
   *  Default values are used for omitted arguments.
   *
   */
  inline void gcode_M600() {

    // Homing first
    if (mechanics.axis_unhomed_error()) mechanics.Home(true);

    // Initial retract before move to pause park position
    const float retract = parser.seen('E') ? parser.value_axis_units(E_AXIS) : 0
      #if ENABLED(PAUSE_PARK_RETRACT_LENGTH) && PAUSE_PARK_RETRACT_LENGTH > 0
        - (PAUSE_PARK_RETRACT_LENGTH)
      #endif
    ;

    // Second retract after cooldown hotend
    const float retract2 = 0
      #if ENABLED(PAUSE_PARK_RETRACT_2_LENGTH) && PAUSE_PARK_RETRACT_2_LENGTH > 0
        - (PAUSE_PARK_RETRACT_2_LENGTH)
      #endif
    ;

    // Lift Z axis
    const float z_lift = parser.seen('Z') ? parser.value_linear_units() :
      #if ENABLED(PAUSE_PARK_Z_ADD) && PAUSE_PARK_Z_ADD > 0
        PAUSE_PARK_Z_ADD
      #else
        0
      #endif
    ;

    // Move XY axes to filament exchange position
    const float x_pos = parser.seen('X') ? parser.value_linear_units() : 0
      #if ENABLED(PAUSE_PARK_X_POS)
        + PAUSE_PARK_X_POS
      #endif
    ;
    const float y_pos = parser.seen('Y') ? parser.value_linear_units() : 0
      #if ENABLED(PAUSE_PARK_Y_POS)
        + PAUSE_PARK_Y_POS
      #endif
    ;

    // Unload filament
    const float unload_length = parser.seen('U') ? parser.value_axis_units(E_AXIS) : 0
      #if ENABLED(PAUSE_PARK_UNLOAD_LENGTH) && PAUSE_PARK_UNLOAD_LENGTH > 0
        - (PAUSE_PARK_UNLOAD_LENGTH)
      #endif
    ;

    // Load filament
    const float load_length = parser.seen('L') ? parser.value_axis_units(E_AXIS) : 0
      #if ENABLED(PAUSE_PARK_LOAD_LENGTH)
        + PAUSE_PARK_LOAD_LENGTH
      #endif
    ;

    const int beep_count = parser.seen('B') ? parser.value_int() :
      #if ENABLED(PAUSE_PARK_NUMBER_OF_ALERT_BEEPS)
        PAUSE_PARK_NUMBER_OF_ALERT_BEEPS
      #else
        -1
      #endif
    ;

    const bool job_running = printer.print_job_counter.isRunning();

    if (printer.pause_print(retract, retract2, z_lift, x_pos, y_pos, unload_length, beep_count, true)) {
      printer.wait_for_filament_reload(beep_count);
      printer.resume_print(load_length, PAUSE_PARK_EXTRUDE_LENGTH, beep_count);
    }

    // Resume the print job timer if it was running
    if (job_running) printer.print_job_counter.start();

  }

#endif // ADVANCED_PAUSE_FEATURE

#if HAS_EXT_ENCODER
  /**
   * M602: Enable or disable Extruder Encoder
   */

  /**
   * M604: Set data Extruder Encoder
   *
   *  S[step] - Set Error Steps
   *
   */
  inline void gcode_M604() {
    GET_TARGET_EXTRUDER(604);
    stepper.synchronize();
    printer.encErrorSteps[printer.target_extruder] = parser.intval('S', ENC_ERROR_STEPS);
    SERIAL_EMV("Encoder Error Steps: ", printer.encErrorSteps[printer.target_extruder]);
  }
#endif

#if ENABLED(DUAL_X_CARRIAGE)

  /**
   * M605: Set dual x-carriage movement mode
   *
   *    M605 S0: Full control mode. The slicer has full control over x-carriage movement
   *    M605 S1: Auto-park mode. The inactive head will auto park/unpark without slicer involvement
   *    M605 S2 [Xnnn] [Rmmm]: Duplication mode. The second extruder will duplicate the first with nnn
   *                         units x-offset and an optional differential hotend temperature of
   *                         mmm degrees. E.g., with "M605 S2 X100 R2" the second extruder will duplicate
   *                         the first with a spacing of 100mm in the x direction and 2 degrees hotter.
   *
   *    Note: the X axis should be homed after changing dual x-carriage mode.
   */
  inline void gcode_M605() {
    stepper.synchronize();
    if (parser.seen('S')) mechanics.dual_x_carriage_mode = (DualXMode)parser.value_byte();
    switch(mechanics.dual_x_carriage_mode) {
      case DXC_FULL_CONTROL_MODE:
      case DXC_AUTO_PARK_MODE:
        break;
      case DXC_DUPLICATION_MODE:
        if (parser.seen('X')) mechanics.duplicate_hotend_x_offset = max(parser.value_linear_units(), X2_MIN_POS - mechanics.x_home_pos(0));
        if (parser.seen('R')) mechanics.duplicate_hotend_temp_offset = parser.value_celsius_diff();
        SERIAL_SM(ECHO, MSG_HOTEND_OFFSET);
        SERIAL_CHR(' ');
        SERIAL_VAL(printer.hotend_offset[X_AXIS][0]);
        SERIAL_CHR(',');
        SERIAL_VAL(printer.hotend_offset[Y_AXIS][0]);
        SERIAL_CHR(' ');
        SERIAL_VAL(mechanics.duplicate_hotend_x_offset);
        SERIAL_CHR(',');
        SERIAL_EV(printer.hotend_offset[Y_AXIS][1]);
        break;
      default:
        mechanics.dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;
        break;
    }
    mechanics.active_hotend_parked = false;
    mechanics.hotend_duplication_enabled = false;
    mechanics.delayed_move_time = 0;
  }

#endif // DUAL_X_CARRIAGE

#if ENABLED(LASER)

  // M649 set laser options
  inline void gcode_M649() {
    // do this at the start so we can debug if needed!
    if (parser.seen('D') && IsRunning()) laser.diagnostics = parser.value_bool();

    // Wait for the rest
    // stepper.synchronize();
    if (parser.seen('S') && IsRunning()) {
      laser.intensity = parser.value_float();
      laser.rasterlaserpower =  laser.intensity;
    }

    if (IsRunning()) {
      if (parser.seen('L')) laser.duration = parser.value_ulong();
      if (parser.seen('P')) laser.ppm = parser.value_float();
      if (parser.seen('B')) laser.set_mode(parser.value_int());
      if (parser.seen('R')) laser.raster_mm_per_pulse = (parser.value_float());
    }

    if (parser.seen('F')) {
      float next_feedrate = parser.value_linear_units();
      if (next_feedrate > 0.0) mechanics.feedrate_mm_s = next_feedrate;
    }
  }

#endif // LASER

#if MECH(MUVE3D)

  // M650: Set peel distance
  inline void gcode_M650() {

    stepper.synchronize();

    peel_distance   = (parser.seen('D') ? parser.value_float() : 2.0);
    peel_speed      = (parser.seen('S') ? parser.value_float() : 2.0);
    retract_speed   = (parser.seen('R') ? parser.value_float() : 2.0);
    peel_pause      = (parser.seen('P') ? parser.value_float() : 0.0);
    tilt_distance   = (parser.seen('T') ? parser.value_float() : 20.0);
    layer_thickness = (parser.seen('H') ? parser.value_float() : 0.0);

    // Initialize tilted to false. The intent here is that you would send this command at the start of a print job, and
    // the platform would be level when you do. As such, we assume that you either hand-cranked it to level, or executed
    // an M654 command via manual GCode before running a new print job. If not, then the platform is currently tilted, and
    // your print job is going to go poorly.
    tilted = false;
  }

  // M651: Run peel move and return back to start.
  inline void gcode_M651() {

    if (peel_distance > 0) {
      planner.buffer_line(mechanics.destination[X_AXIS], mechanics.destination[Y_AXIS], mechanics.destination[Z_AXIS] + peel_distance, mechanics.destination[Z_AXIS], peel_speed, printer.active_extruder);
      planner.buffer_line(mechanics.destination[X_AXIS], mechanics.destination[Y_AXIS], mechanics.destination[Z_AXIS] + peel_distance, mechanics.destination[Z_AXIS] + peel_distance, peel_speed, printer.active_extruder);
      stepper.synchronize();
      if (peel_pause > 0) printer.safe_delay(peel_pause);
    }

    planner.buffer_line(mechanics.destination[X_AXIS], mechanics.destination[Y_AXIS], mechanics.destination[Z_AXIS], mechanics.destination[Z_AXIS], retract_speed, printer.active_extruder);
    stepper.synchronize();
  }

  // M653: Execute tilt move
  inline void gcode_M653() {
    // Double tilts are not allowed.
    if (!tilted) {
      planner.buffer_line(mechanics.destination[X_AXIS], mechanics.destination[Y_AXIS], mechanics.destination[Z_AXIS] + tilt_distance, mechanics.destination[Z_AXIS], retract_speed, printer.active_extruder);
      stepper.synchronize();
    }
  }

  // M654 - execute untilt move
  inline void gcode_M654() {
    // Can only untilt if tilted
    if (tilted) {
       // To prevent subsequent commands from not knowing our
       // actual position, update the Z axis, then move to it.
       mechanics.destination[Z_AXIS] += tilt_distance;
       planner.buffer_line(mechanics.destination[X_AXIS], mechanics.destination[Y_AXIS], mechanics.destination[Z_AXIS], mechanics.destination[Z_AXIS], retract_speed, printer.active_extruder);
       // And save it away as our current position, because we're there.
       mechanics.set_current_to_destination();
       stepper.synchronize();
       tilted = false;
    }
  }

  // M655: Send projector control commands via serial
  inline void gcode_M655() {

    // Viewsonic commands
    if (parser.seen('V')) {
      int tempVal = parser.value_int();

      switch(tempVal) {
        // Power Off
        case 0: {
          // 0614000400341101005E
          const byte off[] = {0x06, 0x14, 0x00, 0x04, 0x00,
                              0x34, 0x11, 0x01, 0x00, 0x5E};
          DLPSerial.write(off, sizeof(off));
        }
        break;
        // Power On
        case 1: {
          // 0614000400341100005D
          const byte on[] = {0x06, 0x14, 0x00, 0x04, 0x00,
                             0x34, 0x11, 0x00, 0x00, 0x5D};
          DLPSerial.write(on, sizeof(on));
        }
        break;
        // Factory Reset
        case 2: {
          // 0614000400341102005F
          const byte reset[] = {0x06, 0x14, 0x00, 0x04, 0x00,
                                0x34, 0x11, 0x02, 0x00, 0x5F};
          DLPSerial.write(reset, sizeof(reset));
        }
        break;
        // Splash Screen Black
        case 3: {
          // 061400040034110A0067
          const byte blackScreen[] = {0x06, 0x14, 0x00, 0x04, 0x00,
                                      0x34, 0x11, 0x0A, 0x00, 0x67};
          DLPSerial.write(blackScreen, sizeof(blackScreen));
        }
        break;
        // High Altitude On
        case 4: {
          // 061400040034110C016A
          const byte HAOn[] = {0x06, 0x14, 0x00, 0x04, 0x00,
                               0x34, 0x11, 0x0C, 0x01, 0x6A};
          DLPSerial.write(HAOn, sizeof(HAOn));
        }
        break;
        // High Altitude Off
        case 5: {
          // 061400040034110C0069
          const byte HAOff[] = {0x06, 0x14, 0x00, 0x04, 0x00,
                                0x34, 0x11, 0x0C, 0x00, 0x69};
          DLPSerial.write(HAOff, sizeof(HAOff));
        }
        break;
        // Lamp Mode Normal
        case 6: {
          // 0614000400341110006D
          const byte lampNormal[] = {0x06, 0x14, 0x00, 0x04, 0x00,
                                     0x34, 0x11, 0x10, 0x00, 0x6D};
          DLPSerial.write(lampNormal, sizeof(lampNormal));
        }
        break;
        // Contrast Decrease
        case 7: {
          // 06140004003412020060
          const byte contDec[] = {0x06, 0x14, 0x00, 0x04, 0x00,
                                  0x34, 0x12, 0x02, 0x00, 0x60};
          DLPSerial.write(contDec, sizeof(contDec));
        }
        break;
        // Contrast Increase
        case 8: {
          // 06140004003412020161
          const byte contInc[] = {0x06, 0x14, 0x00, 0x04, 0x00,
                                  0x34, 0x12, 0x02, 0x01, 0x61};
          DLPSerial.write(contInc, sizeof(contInc));
        }
        break;
        // Brightness Decrease
        case 9: {
          // 06140004003412030061
          const byte brightDec[] = {0x06, 0x14, 0x00, 0x04, 0x00,
                                    0x34, 0x12, 0x03, 0x00, 0x61};
          DLPSerial.write(brightDec, sizeof(brightDec));
        }
        break;
        // Brightness Increase
        case 10: {
          // 06140004003412030162
          const byte brightInc[] = {0x06, 0x14, 0x00, 0x04, 0x00,
                                    0x34, 0x12, 0x03, 0x01, 0x62};
          DLPSerial.write(brightInc, sizeof(brightInc));
        }
        break;
      }
    }
  }

#endif // MECH(MUVE3D)

#if HAS_BED_PROBE && NOMECH(DELTA)

  // M666: Set Z probe offset
  inline void gcode_M666() {

    SERIAL_SM(ECHO, MSG_ZPROBE_ZOFFSET);
    SERIAL_CHR(' ');

    if (parser.seen('P')) {
      float p_val = parser.value_linear_units();
      if (Z_PROBE_OFFSET_RANGE_MIN <= p_val && p_val <= Z_PROBE_OFFSET_RANGE_MAX) {

        #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
          // Correct bilinear grid for new probe offset
          const float diff = p_val - probe.z_offset;
          if (diff) {
            for (uint8_t x = 0; x < GRID_MAX_POINTS_X; x++)
              for (uint8_t y = 0; y < GRID_MAX_POINTS_Y; y++)
                bedlevel.z_values[x][y] += diff;
          }
          #if ENABLED(ABL_BILINEAR_SUBDIVISION)
            bedlevel.bed_level_virt_interpolate();
          #endif
        #endif

        probe.z_offset = p_val;
        SERIAL_VAL(probe.z_offset);
      }
      else {
        SERIAL_MT(MSG_Z_MIN, Z_PROBE_OFFSET_RANGE_MIN);
        SERIAL_CHR(' ');
        SERIAL_MT(MSG_Z_MAX, Z_PROBE_OFFSET_RANGE_MAX);
      }
    }
    else {
      SERIAL_MV(": ", probe.z_offset, 3);
    }

    SERIAL_EOL();
  }

#elif MECH(DELTA)

  /**
   * M666: Set delta endstop and geometry adjustment
   *
   *    D = Diagonal Rod
   *    R = Delta Radius
   *    S = Segments per Second
   *    A = Alpha (Tower 1) Diagonal Rod Adjust
   *    B = Beta  (Tower 2) Diagonal Rod Adjust
   *    C = Gamma (Tower 3) Diagonal Rod Adjust
   *    I = Alpha (Tower 1) Tower Radius Adjust
   *    J = Beta  (Tower 2) Tower Radius Adjust
   *    K = Gamma (Tower 3) Tower Radius Adjust
   *    U = Alpha (Tower 1) Tower Position Adjust
   *    V = Beta  (Tower 2) Tower Position Adjust
   *    W = Gamma (Tower 3) Tower Position Adjust
   *    X = Alpha (Tower 1) Endstop Adjust
   *    Y = Beta  (Tower 2) Endstop Adjust
   *    Z = Gamma (Tower 3) Endstop Adjust
   *    O = Print radius
   *    Q = Probe radius
   *    P = Z probe offset
   *    H = Z Height
   */
  inline void gcode_M666() {

    if (parser.seen('H')) {
      const float old_delta_height = mechanics.delta_height;
      mechanics.delta_height = parser.value_linear_units();
      mechanics.current_position[Z_AXIS] += mechanics.delta_height - old_delta_height;
    }

    if (parser.seen('D')) mechanics.delta_diagonal_rod              = parser.value_linear_units();
    if (parser.seen('R')) mechanics.delta_radius                    = parser.value_linear_units();
    if (parser.seen('S')) mechanics.delta_segments_per_second       = parser.value_float();
    if (parser.seen('A')) mechanics.delta_diagonal_rod_adj[A_AXIS]  = parser.value_linear_units();
    if (parser.seen('B')) mechanics.delta_diagonal_rod_adj[B_AXIS]  = parser.value_linear_units();
    if (parser.seen('C')) mechanics.delta_diagonal_rod_adj[C_AXIS]  = parser.value_linear_units();
    if (parser.seen('I')) mechanics.delta_tower_radius_adj[A_AXIS]  = parser.value_linear_units();
    if (parser.seen('J')) mechanics.delta_tower_radius_adj[B_AXIS]  = parser.value_linear_units();
    if (parser.seen('K')) mechanics.delta_tower_radius_adj[C_AXIS]  = parser.value_linear_units();
    if (parser.seen('U')) mechanics.delta_tower_pos_adj[A_AXIS]     = parser.value_linear_units();
    if (parser.seen('V')) mechanics.delta_tower_pos_adj[B_AXIS]     = parser.value_linear_units();
    if (parser.seen('W')) mechanics.delta_tower_pos_adj[C_AXIS]     = parser.value_linear_units();
    if (parser.seen('O')) mechanics.delta_print_radius              = parser.value_linear_units();
    if (parser.seen('Q')) mechanics.delta_probe_radius              = parser.value_linear_units();

    mechanics.recalc_delta_settings();

    #if HAS_BED_PROBE

      if (parser.seen('P')) {

        SERIAL_SM(ECHO, MSG_ZPROBE_ZOFFSET);
        SERIAL_CHR(' ');

        float p_val = parser.value_linear_units();
        if (Z_PROBE_OFFSET_RANGE_MIN <= p_val && p_val <= Z_PROBE_OFFSET_RANGE_MAX) {

          #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
            // Correct bilinear grid for new probe offset
            const float diff = p_val - probe.z_offset;
            if (diff) {
              for (uint8_t x = 0; x < GRID_MAX_POINTS_X; x++)
                for (uint8_t y = 0; y < GRID_MAX_POINTS_Y; y++)
                  bedlevel.z_values[x][y] += diff;
            }
            #if ENABLED(ABL_BILINEAR_SUBDIVISION)
              bedlevel.bed_level_virt_interpolate();
            #endif
          #endif

          probe.z_offset = p_val;
          SERIAL_VAL(probe.z_offset);
        }
        else {
          SERIAL_MT(MSG_Z_MIN, Z_PROBE_OFFSET_RANGE_MIN);
          SERIAL_CHR(' ');
          SERIAL_MT(MSG_Z_MAX, Z_PROBE_OFFSET_RANGE_MAX);
        }

        SERIAL_EOL();
      }

    #endif // HAS_BED_PROBE

    LOOP_XYZ(i) {
      if (parser.seen(axis_codes[i])) mechanics.delta_endstop_adj[i] = parser.value_linear_units();
    }

    if (parser.seen('L')) {
      SERIAL_LM(CFG, "Current Delta geometry values:");
      LOOP_XYZ(i) {
        SERIAL_SV(CFG, axis_codes[i]);
        SERIAL_EMV(" (Endstop Adj): ", mechanics.delta_endstop_adj[i], 3);
      }

      #if HAS_BED_PROBE
        SERIAL_LMV(CFG, "P (ZProbe ZOffset): ", probe.z_offset, 3);
      #endif

      SERIAL_LMV(CFG, "A (Tower A Diagonal Rod Correction): ",  mechanics.delta_diagonal_rod_adj[0], 3);
      SERIAL_LMV(CFG, "B (Tower B Diagonal Rod Correction): ",  mechanics.delta_diagonal_rod_adj[1], 3);
      SERIAL_LMV(CFG, "C (Tower C Diagonal Rod Correction): ",  mechanics.delta_diagonal_rod_adj[2], 3);
      SERIAL_LMV(CFG, "I (Tower A Radius Correction): ",        mechanics.delta_tower_radius_adj[0], 3);
      SERIAL_LMV(CFG, "J (Tower B Radius Correction): ",        mechanics.delta_tower_radius_adj[1], 3);
      SERIAL_LMV(CFG, "K (Tower C Radius Correction): ",        mechanics.delta_tower_radius_adj[2], 3);
      SERIAL_LMV(CFG, "U (Tower A Position Correction): ",      mechanics.delta_tower_pos_adj[0], 3);
      SERIAL_LMV(CFG, "V (Tower B Position Correction): ",      mechanics.delta_tower_pos_adj[1], 3);
      SERIAL_LMV(CFG, "W (Tower C Position Correction): ",      mechanics.delta_tower_pos_adj[2], 3);
      SERIAL_LMV(CFG, "R (Delta Radius): ",                     mechanics.delta_radius, 4);
      SERIAL_LMV(CFG, "D (Diagonal Rod Length): ",              mechanics.delta_diagonal_rod, 4);
      SERIAL_LMV(CFG, "S (Delta Segments per second): ",        mechanics.delta_segments_per_second);
      SERIAL_LMV(CFG, "O (Delta Print Radius): ",               mechanics.delta_print_radius);
      SERIAL_LMV(CFG, "Q (Delta Probe Radius): ",               mechanics.delta_probe_radius);
      SERIAL_LMV(CFG, "H (Z-Height): ",                         mechanics.delta_height, 3);
    }
  }

#endif // MECH DELTA

#if ENABLED(LIN_ADVANCE)

  /**
   * M900: Set and/or Get advance K factor and WH/D ratio
   *
   *  K<factor>                  Set advance K factor
   *  R<ratio>                   Set ratio directly (overrides WH/D)
   *  W<width> H<height> D<diam> Set ratio from WH/D
   */
  inline void gcode_M900() {
    stepper.synchronize();

    const float newK = parser.seen('K') ? parser.value_float() : -1;
    if (newK >= 0) planner.extruder_advance_k = newK;

    float newR = parser.seen('R') ? parser.value_float() : -1;
    if (newR < 0) {
      const float newD = parser.seen('D') ? parser.value_float() : -1,
                  newW = parser.seen('W') ? parser.value_float() : -1,
                  newH = parser.seen('H') ? parser.value_float() : -1;
      if (newD >= 0 && newW >= 0 && newH >= 0)
        newR = newD ? (newW * newH) / (sq(newD * 0.5) * M_PI) : 0;
    }
    if (newR >= 0) planner.advance_ed_ratio = newR;

    SERIAL_SMV(ECHO, "Advance K=", planner.extruder_advance_k);
    SERIAL_MSG(" E/D=");
    if (planner.advance_ed_ratio) SERIAL_VAL(planner.advance_ed_ratio);
    else SERIAL_MSG("Auto");
    SERIAL_EOL();
  }

#endif // LIN_ADVANCE

#if MB(ALLIGATOR) || MB(ALLIGATOR_V3)

  /**
   * M906: Set motor currents
   */
  inline void gcode_M906() {

    GET_TARGET_EXTRUDER(906);

    LOOP_XYZE(i) {
      if (parser.seen(axis_codes[i])) {
        const uint8_t a = i + (i == E_AXIS ? TARGET_EXTRUDER : 0);
        printer.motor_current[a] = parser.value_float();
      }
    }
    stepper.set_driver_current();
  }

#elif ENABLED(HAVE_TMC2130)

  static void tmc2130_get_current(TMC2130Stepper &st, const char name) {
    SERIAL_CHR(name);
    SERIAL_MSG(" axis driver current: ");
    SERIAL_EV(st.getCurrent());
  }
  static void tmc2130_set_current(TMC2130Stepper &st, const char name, const int mA) {
    st.setCurrent(mA, R_SENSE, HOLD_MULTIPLIER);
    tmc2130_get_current(st, name);
  }

  static void tmc2130_report_otpw(TMC2130Stepper &st, const char name) {
    SERIAL_CHR(name);
    SERIAL_MSG(" axis temperature prewarn triggered: ");
    SERIAL_PS(st.getOTPW() ? PSTR("true") : PSTR("false"));
    SERIAL_EOL();
  }
  static void tmc2130_clear_otpw(TMC2130Stepper &st, const char name) {
    st.clear_otpw();
    SERIAL_CHR(name);
    SERIAL_EM(" prewarn flag cleared");
  }

  static void tmc2130_get_pwmthrs(TMC2130Stepper &st, const char name, const uint16_t spmm) {
    SERIAL_CHR(name);
    SERIAL_MSG(" stealthChop max speed set to ");
    SERIAL_EV(12650000UL * st.microsteps() / (256 * st.stealth_max_speed() * spmm));
  }
  static void tmc2130_set_pwmthrs(TMC2130Stepper &st, const char name, const int32_t thrs, const uint32_t spmm) {
    st.stealth_max_speed(12650000UL * st.microsteps() / (256 * thrs * spmm));
    tmc2130_get_pwmthrs(st, name, spmm);
  }

  static void tmc2130_get_sgt(TMC2130Stepper &st, const char name) {
    SERIAL_CHR(name);
    SERIAL_MSG(" driver homing sensitivity set to ");
    SERIAL_EV(st.sgt());
  }
  static void tmc2130_set_sgt(TMC2130Stepper &st, const char name, const int8_t sgt_val) {
    st.sgt(sgt_val);
    tmc2130_get_sgt(st, name);
  }

  /**
   * M906: Set motor current in milliamps using axis codes X, Y, Z, E
   * Report driver currents when no axis specified
   *
   * S1: Enable automatic current control
   * S0: Disable
   */
  inline void gcode_M906() {
    uint16_t values[NUM_AXIS];
    LOOP_XYZE(i)
      values[i] = parser.seen(axis_codes[i]) ? parser.value_int() : 0;

    #if ENABLED(X_IS_TMC2130)
      if (values[X_AXIS]) tmc2130_set_current(values[X_AXIS], stepperX, 'X');
      else tmc2130_get_current(stepperX, 'X');
    #endif
    #if ENABLED(Y_IS_TMC2130)
      if (values[Y_AXIS]) tmc2130_set_current(values[Y_AXIS], stepperY, 'Y');
      else tmc2130_get_current(stepperY, 'Y');
    #endif
    #if ENABLED(Z_IS_TMC2130)
      if (values[Z_AXIS]) tmc2130_set_current(values[Z_AXIS], stepperZ, 'Z');
      else tmc2130_get_current(stepperZ, 'Z');
    #endif
    #if ENABLED(E0_IS_TMC2130)
      if (values[E_AXIS]) tmc2130_set_current(values[E_AXIS], stepperE0, 'E');
      else tmc2130_get_current(stepperE0, 'E');
    #endif

    #if ENABLED(AUTOMATIC_CURRENT_CONTROL)
      if (parser.seen('S')) auto_current_control = parser.value_bool();
    #endif
  }

  /**
   * M911: Report TMC2130 stepper driver overtemperature pre-warn flag
   * The flag is held by the library and persist until manually cleared by M912
   */
  inline void gcode_M911() {
    const bool reportX = parser.seen('X'), reportY = parser.seen('Y'), reportZ = parser.seen('Z'), reportE = parser.seen('E'),
             reportAll = (!reportX && !reportY && !reportZ && !reportE) || (reportX && reportY && reportZ && reportE);
    #if ENABLED(X_IS_TMC2130)
      if (reportX || reportAll) tmc2130_report_otpw(stepperX, 'X');
    #endif
    #if ENABLED(Y_IS_TMC2130)
      if (reportY || reportAll) tmc2130_report_otpw(stepperY, 'Y');
    #endif
    #if ENABLED(Z_IS_TMC2130)
      if (reportZ || reportAll) tmc2130_report_otpw(stepperZ, 'Z');
    #endif
    #if ENABLED(E0_IS_TMC2130)
      if (reportE || reportAll) tmc2130_report_otpw(stepperE0, 'E');
    #endif
  }

  /**
   * M912: Clear TMC2130 stepper driver overtemperature pre-warn flag held by the library
   */
  inline void gcode_M912() {
    const bool clearX = parser.seen('X'), clearY = parser.seen('Y'), clearZ = parser.seen('Z'), clearE = parser.seen('E'),
             clearAll = (!clearX && !clearY && !clearZ && !clearE) || (clearX && clearY && clearZ && clearE);
    #if ENABLED(X_IS_TMC2130)
      if (clearX || clearAll) tmc2130_clear_otpw(stepperX, 'X');
    #endif
    #if ENABLED(Y_IS_TMC2130)
      if (clearY || clearAll) tmc2130_clear_otpw(stepperY, 'Y');
    #endif
    #if ENABLED(Z_IS_TMC2130)
      if (clearZ || clearAll) tmc2130_clear_otpw(stepperZ, 'Z');
    #endif
    #if ENABLED(E0_IS_TMC2130)
      if (clearE || clearAll) tmc2130_clear_otpw(stepperE0, 'E');
    #endif
  }

  /**
   * M913: Set HYBRID_THRESHOLD speed.
   */
  #if ENABLED(HYBRID_THRESHOLD)
    inline void gcode_M913() {
      uint16_t values[XYZE];
      LOOP_XYZE(i)
        values[i] = parser.seen(axis_codes[i]) ? parser.value_int() : 0;

      #if ENABLED(X_IS_TMC2130)
        if (values[X_AXIS]) tmc2130_set_pwmthrs(stepperX, 'X', values[X_AXIS], mechanics.axis_steps_per_mm[X_AXIS]);
        else tmc2130_get_pwmthrs(stepperX, 'X', mechanics.axis_steps_per_mm[X_AXIS]);
      #endif
      #if ENABLED(Y_IS_TMC2130)
        if (values[Y_AXIS]) tmc2130_set_pwmthrs(stepperY, 'Y', values[Y_AXIS], mechanics.axis_steps_per_mm[Y_AXIS]);
        else tmc2130_get_pwmthrs(stepperY, 'Y', mechanics.axis_steps_per_mm[Y_AXIS]);
      #endif
      #if ENABLED(Z_IS_TMC2130)
        if (values[Z_AXIS]) tmc2130_set_pwmthrs(stepperZ, 'Z', values[Z_AXIS], mechanics.axis_steps_per_mm[Z_AXIS]);
        else tmc2130_get_pwmthrs(stepperZ, 'Z', mechanics.axis_steps_per_mm[Z_AXIS]);
      #endif
      #if ENABLED(E0_IS_TMC2130)
        if (values[E_AXIS]) tmc2130_set_pwmthrs(stepperE0, 'E', values[E_AXIS], mechanics.axis_steps_per_mm[E_AXIS]);
        else tmc2130_get_pwmthrs(stepperE0, 'E', mechanics.axis_steps_per_mm[E_AXIS]);
      #endif
    }
  #endif // HYBRID_THRESHOLD

  /**
   * M914: Set SENSORLESS_HOMING sensitivity.
   */
  #if ENABLED(SENSORLESS_HOMING)
    inline void gcode_M914() {
      #if ENABLED(X_IS_TMC2130)
        if (parser.seen(axis_codes[X_AXIS])) tmc2130_set_sgt(stepperX, 'X', parser.value_int());
        else tmc2130_get_sgt(stepperX, 'X');
      #endif
      #if ENABLED(Y_IS_TMC2130)
        if (parser.seen(axis_codes[Y_AXIS])) tmc2130_set_sgt(stepperY, 'Y', parser.value_int());
        else tmc2130_get_sgt(stepperY, 'Y');
      #endif
    }
  #endif // SENSORLESS_HOMING

#endif // HAVE_TMC2130

/**
 * M907: Set digital trimpot motor current using axis codes X, Y, Z, E, B, S
 */
inline void gcode_M907() {
  #if HAS_DIGIPOTSS
    LOOP_XYZE(i)
      if (parser.seen(axis_codes[i])) stepper.digipot_current(i, parser.value_int());
    if (parser.seen('B')) stepper.digipot_current(4, parser.value_int());
    if (parser.seen('S')) for (uint8_t i = 0; i <= 4; i++) stepper.digipot_current(i, parser.value_int());
  #elif HAS_MOTOR_CURRENT_PWM
    #if PIN_EXISTS(MOTOR_CURRENT_PWM_XY)
      if (parser.seen('X')) stepper.digipot_current(0, parser.value_int());
    #endif
    #if PIN_EXISTS(MOTOR_CURRENT_PWM_Z)
      if (parser.seen('Z')) stepper.digipot_current(1, parser.value_int());
    #endif
    #if PIN_EXISTS(MOTOR_CURRENT_PWM_E)
      if (parser.seen('E')) stepper.digipot_current(2, parser.value_int());
    #endif
  #endif
  #if ENABLED(DIGIPOT_I2C)
    // this one uses actual amps in floating point
    LOOP_XYZE(i) if (parser.seen(axis_codes[i])) digipot_i2c_set_current(i, parser.value_float());
    // for each additional extruder (named B,C,D,E..., channels 4,5,6,7...)
    for (uint8_t i = NUM_AXIS; i < DIGIPOT_I2C_NUM_CHANNELS; i++) if(parser.seen('B' + i - (NUM_AXIS))) digipot_i2c_set_current(i, parser.value_float());
  #endif
}

#if HAS_DIGIPOTSS
  /**
   * M908: Control digital trimpot directly (M908 P<pin> S<current>)
   */
  inline void gcode_M908() {
    digitalPotWrite(
      parser.intval('P'),
      parser.intval('S')
    );
  }
#endif // HAS_DIGIPOTSS

#if ENABLED(NEXTION) && ENABLED(NEXTION_GFX)

  /**
   * M995: Nextion Origin
   */
  inline void gcode_M995() { gfx_origin(parser.linearval('X'), parser.linearval('Y'), parser.linearval('Z')); }

  /**
   * M996: Nextion Scale
   */
  inline void gcode_M996() {
    if (parser.seenval('S')) gfx_scale(parser.value_float());
  }

#endif

#if ENABLED(NPR2)

  /**
   * M997: Cxx Move Carter xx gradi
   */
  inline void gcode_M997() {
    long csteps;
    if (parser.seenval('C')) {
      csteps = parser.value_ulong() * color_step_moltiplicator;
      SERIAL_EMV("csteps: ", csteps);
      if (csteps < 0) stepper.colorstep(-csteps, false);
      if (csteps > 0) stepper.colorstep(csteps, true);
    }
  }

#endif

/**
 * M999: Restart after being stopped
 *
 * Default behaviour is to flush the serial buffer and request
 * a resend to the host starting on the last N line received.
 *
 * Sending "M999 S1" will resume printing without flushing the
 * existing command buffer.
 *
 */
inline void gcode_M999() {
  printer.setRunning(true);
  lcd_reset_alert_level();

  if (parser.boolval('S')) return;

  commands.FlushSerialRequestResend();
}
