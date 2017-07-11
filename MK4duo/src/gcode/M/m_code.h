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

#if HAS_RESUME_CONTINUE

  /**
   * M0: Unconditional stop - Wait for user button press on LCD
   * M1: Conditional stop   - Wait for user button press on LCD
   */
  inline void gcode_M0_M1() {
    const char * const args = parser.string_arg;

    millis_t ms = 0;
    bool hasP = false, hasS = false;
    if (parser.seenval('P')) {
      ms = parser.value_millis(); // milliseconds to wait
      hasP = ms > 0;
    }
    if (parser.seenval('S')) {
      ms = parser.value_millis_from_seconds(); // seconds to wait
      hasS = ms > 0;
    }

    #if ENABLED(ULTIPANEL)

      if (!hasP && !hasS && *args != '\0')
        lcd_setstatus(args, true);
      else {
        LCD_MESSAGEPGM(MSG_USERWAIT);
        #if ENABLED(LCD_PROGRESS_BAR) && PROGRESS_MSG_EXPIRE > 0
          dontExpireStatus();
        #endif
      }

    #else

      if (!hasP && !hasS && *args != '\0')
        SERIAL_LV(ECHO, args);

    #endif

    printer.wait_for_user = true;
    KEEPALIVE_STATE(PAUSED_FOR_USER);

    stepper.synchronize();
    commands.refresh_cmd_timeout();

    if (ms > 0) {
      ms += commands.previous_cmd_ms;  // wait until this time for a click
      while (PENDING(millis(), ms) && printer.wait_for_user) printer.idle();
    }
    else {
      #if ENABLED(ULTIPANEL)
        if (lcd_detected()) {
          while (printer.wait_for_user) printer.idle();
          IS_SD_PRINTING ? LCD_MESSAGEPGM(MSG_RESUMING) : LCD_MESSAGEPGM(WELCOME_MSG);
        }
      #else
        while (printer.wait_for_user) printer.idle();
      #endif
    }

    printer.wait_for_user = false;
    KEEPALIVE_STATE(IN_HANDLER);
  }

#endif // HAS_RESUME_CONTINUE

#if HAS_MULTI_MODE

  /**
   * M3: Setting laser beam or fire laser - CNC clockwise speed
   * M4: Turn on laser beam or CNC counter clockwise speed
   *      S - Laser intensity or CNC speed
   *      L - Laser Duration
   *      P - PPM
   *      D - Diagnostic
   *      B - Set mode
   */
  inline void gcode_M3_M4(bool clockwise) {
    stepper.synchronize();

    switch (printer.mode) {

      #if ENABLED(LASER) && ENABLED(LASER_FIRE_SPINDLE)
        case PRINTER_MODE_LASER: {
          if (IsRunning()) {
            #if ENABLED(INTENSITY_IN_BYTE)
              if (parser.seenval('S')) laser.intensity = (float)(parser.value_byte() / 255) * 100.0;
            #else
              if (parser.seenval('S')) laser.intensity = parser.value_float();
            #endif
            if (parser.seenval('L')) laser.duration = parser.value_ulong();
            if (parser.seenval('P')) laser.ppm = parser.value_float();
            if (parser.seenval('D')) laser.diagnostics = parser.value_bool();
            if (parser.seenval('B')) laser.set_mode(parser.value_int());
          }
          laser.status = LASER_ON;
        }
        break;
      #endif

      #if ENABLED(CNCROUTER)
        case PRINTER_MODE_CNC:
          if (parser.seenval('S')) setCNCRouterSpeed(parser.value_ulong(), clockwise);
        break;
      #endif

      default: break; // other tools

    } // printer.mode

    mechanics.prepare_move_to_destination();
  }

  /**
   * M5: Turn off laser beam - CNC off
   */
  inline void gcode_M5() {
    stepper.synchronize();

    switch (printer.mode) {

      #if ENABLED(LASER)
        case PRINTER_MODE_LASER: {
          if (laser.status != LASER_OFF) {
            laser.status = LASER_OFF;
            laser.mode = CONTINUOUS;
            laser.duration = 0;

            if (laser.diagnostics)
              SERIAL_EM("Laser M5 called and laser OFF");
          }
        }
        break;
      #endif

      #if ENABLED(CNCROUTER)
        case PRINTER_MODE_CNC:
          disable_cncrouter();
        break;
      #endif

      default: break; // other tools

    } // printer.mode

    mechanics.prepare_move_to_destination();

  }

  #if ENABLED(CNCROUTER)
    /*
     * M6: CNC tool change
     */
    inline void gcode_M6() { tool_change_cnc(CNC_M6_TOOL_ID); }
  #endif

#endif // HAS_MULTI_MODE

/**
 * M17: Enable power on all stepper motors
 */
inline void gcode_M17() {
  LCD_MESSAGEPGM(MSG_NO_MOVE);
  stepper.enable_all_steppers();
}

#if HAS_SDSUPPORT

  /**
   * M20: List SD card to serial output
   */
  inline void gcode_M20() {
    SERIAL_EM(MSG_BEGIN_FILE_LIST);
    card.ls();
    SERIAL_EM(MSG_END_FILE_LIST);
  }

  /**
   * M21: Init SD Card
   */
  inline void gcode_M21() {
    card.mount();
  }

  /**
   * M22: Release SD Card
   */
  inline void gcode_M22() {
    card.unmount();
  }

  /**
   * M23: Select a file
   */
  inline void gcode_M23() {
    // Simplify3D includes the size, so zero out all spaces (#7227)
    for (char *fn = parser.string_arg; *fn; ++fn) if (*fn == ' ') *fn = '\0';
    card.selectFile(parser.string_arg);
  }

  /**
   * M24: Start or Resume SD Print
   */
  inline void gcode_M24() {
    #if ENABLED(PARK_HEAD_ON_PAUSE)
      printer.resume_print();
    #endif

    card.startFileprint();
    printer.print_job_counter.start();
    #if HAS_POWER_CONSUMPTION_SENSOR
      startpower = power_consumption_hour;
    #endif
  }

  /**
   * M25: Pause SD Print
   */
  inline void gcode_M25() {
    card.pauseSDPrint();
    printer.print_job_counter.pause();
    SERIAL_LM(REQUEST_PAUSE, "SD pause");

    #if ENABLED(PARK_HEAD_ON_PAUSE)
      commands.enqueue_and_echo_commands_P(PSTR("M125")); // Must be enqueued with pauseSDPrint set to be last in the buffer
    #endif
  }

  /**
   * M26: Set SD Card file index
   */
  inline void gcode_M26() {
    if (card.cardOK && parser.seen('S'))
      card.setIndex(parser.value_long());
  }

  /**
   * M27: Get SD Card status
   */
  inline void gcode_M27() {
    card.printStatus();
  }

  /**
   * M28: Start SD Write
   */
  inline void gcode_M28() {
    card.startWrite(parser.string_arg, false);
  }

  /**
   * M29: Stop SD Write
   * Processed in write to file routine above
   */
  inline void gcode_M29() {
    // card.saving = false;
  }

  /**
   * M30 <filename>: Delete SD Card file
   */
  inline void gcode_M30() {
    if (card.cardOK) {
      card.closeFile();
      card.deleteFile(parser.string_arg);
    }
  }

#endif // SDSUPPORT

/**
 * M31: Get the time since the start of SD Print (or last M109)
 */
inline void gcode_M31() {
  char buffer[21];
  duration_t elapsed = printer.print_job_counter.duration();
  elapsed.toString(buffer);
  lcd_setstatus(buffer);

  SERIAL_LMT(ECHO, MSG_PRINT_TIME, buffer);
}

#if HAS_SDSUPPORT

  /**
   * M32: Make Directory
   */
  inline void gcode_M32() {
    if (card.cardOK) {
      card.makeDirectory(parser.string_arg);
      card.mount();
    }
  }

  /**
   * M33: Close File and store location in restart.gcode
   */
  inline void gcode_M33() {
    printer.stopSDPrint(true);
  }

  /**
   * M34: Select file and start SD print
   */
  inline void gcode_M34() {
    if (card.sdprinting)
      stepper.synchronize();

    if (card.cardOK) {
      char* namestartpos = (strchr(parser.string_arg, '@'));
      if (namestartpos == NULL) {
        namestartpos = parser.string_arg ; // default name position
      }
      else
        namestartpos++; // to skip the '@'

      SERIAL_MV("Open file: ", namestartpos);
      SERIAL_EM(" and start print.");
      card.selectFile(namestartpos);
      if (parser.seenval('S')) card.setIndex(parser.value_long());

      mechanics.feedrate_mm_s       = 20.0; // 20 units/sec
      mechanics.feedrate_percentage = 100;  // 100% mechanics.feedrate_mm_s
      card.startFileprint();
      printer.print_job_counter.start();
      #if HAS_POWER_CONSUMPTION_SENSOR
        startpower = power_consumption_hour;
      #endif
    }
  }

  #if ENABLED(NEXTION)
    /**
     * M35: Upload Firmware to Nextion from SD
     */
    inline void gcode_M35() {
      UploadNewFirmware();
    }
  #endif

#endif // SDSUPPORT

/**
 * M42: Change pin status via GCode
 *
 *  P<pin>  Pin number (LED if omitted)
 *  S<byte> Pin status from 0 - 255
 */
inline void gcode_M42() {
  if (!parser.seenval('S')) return;
  const byte pin_status = parser.value_byte();

  const Pin pin_number = parser.intval('P', LED_PIN);
  if (pin_number < 0) return;

  if (printer.pin_is_protected(pin_number)) {
    SERIAL_LM(ER, MSG_ERR_PROTECTED_PIN);
    return;
  }

  HAL::pinMode(pin_number, OUTPUT);
  HAL::digitalWrite(pin_number, pin_status);
  HAL::analogWrite(pin_number, pin_status);

  #if FAN_COUNT > 0
    switch (pin_number) {
      #if HAS_FAN0
        case FAN_PIN: printer.fanSpeeds[0] = pin_status; break;
      #endif
      #if HAS_FAN1
        case FAN1_PIN: printer.fanSpeeds[1] = pin_status; break;
      #endif
      #if HAS_FAN2
        case FAN2_PIN: printer.fanSpeeds[2] = pin_status; break;
      #endif
      #if HAS_FAN3
        case FAN3_PIN: printer.fanSpeeds[3] = pin_status; break;
      #endif
    }
  #endif
}

#if ENABLED(PINS_DEBUGGING)

  #include "utility/pinsdebug.h"

  inline void toggle_pins() {
    const bool  I_flag  = parser.boolval('I');
    const int   repeat  = parser.intval('R', 1),
                start   = parser.intval('S'),
                end     = parser.intval('E', LAST_PIN - 1),
                wait    = parser.intval('W', 500);

    for (Pin pin = start; pin <= end; pin++) {

      if (!I_flag && printer.pin_is_protected(pin)) {
        SERIAL_MV("Sensitive Pin: ", pin);
        SERIAL_EM(" untouched.");
      }
      else {
        SERIAL_MV("Pulsing Pin: ", pin);
        HAL::pinMode(pin, OUTPUT);
        for (int16_t j = 0; j < repeat; j++) {
          HAL::digitalWrite(pin, 0);
          printer.safe_delay(wait);
          HAL::digitalWrite(pin, 1);
          printer.safe_delay(wait);
          HAL::digitalWrite(pin, 0);
          printer.safe_delay(wait);
        }
      }
      SERIAL_EOL();
    }
    SERIAL_EM("Done");
  } // toggle_pins

  inline void servo_probe_test(){
    #if !(NUM_SERVOS >= 1 && HAS_SERVO_0)
      SERIAL_LM(ER, "SERVO not setup");
    #elif !HAS_Z_SERVO_PROBE
      SERIAL_LM(ER, "Z_ENDSTOP_SERVO_NR not setup");
    #else

      const uint8_t probe_index = parser.seen('P') ? parser.value_byte() : Z_ENDSTOP_SERVO_NR;

      SERIAL_EM("Servo probe test");
      SERIAL_EMV(".  Using index:  ", probe_index);
      SERIAL_EMV(".  Deploy angle: ", probe.z_servo_angle[0]);
      SERIAL_EMV(".  Stow angle:   ", probe.z_servo_angle[1]);

      bool probe_inverting;

      #if HAS_Z_PROBE_PIN

        #define PROBE_TEST_PIN Z_PROBE_PIN

        SERIAL_EMV("Probe uses Z_MIN_PROBE_PIN: ", PROBE_TEST_PIN);
        SERIAL_EM(".  Uses Z_PROBE_ENDSTOP_INVERTING (ignores Z_MIN_ENDSTOP_INVERTING)");
        SERIAL_MSG(".  Z_PROBE_ENDSTOP_INVERTING: ");

        #if Z_PROBE_ENDSTOP_INVERTING
          SERIAL_EM("true");
        #else
          SERIAL_EM("false");
        #endif

        probe_inverting = Z_PROBE_ENDSTOP_INVERTING;

      #elif HAS_Z_MIN

        #define PROBE_TEST_PIN Z_MIN_PIN

        SERIAL_EMV("Probe uses Z_MIN pin: ", PROBE_TEST_PIN);
        SERIAL_EM(".  Uses Z_MIN_ENDSTOP_INVERTING (ignores Z_PROBE_ENDSTOP_INVERTING)");
        SERIAL_MSG(".  Z_MIN_ENDSTOP_INVERTING: ");

        #if Z_MIN_ENDSTOP_INVERTING
          SERIAL_EM("true");
        #else
          SERIAL_EM("false");
        #endif

        probe_inverting = Z_MIN_ENDSTOP_INVERTING;

      #endif

      SERIAL_EM("Deploy & stow 4 times");
      SET_INPUT_PULLUP(PROBE_TEST_PIN);
      bool deploy_state, stow_state;
      for (uint8_t i = 0; i < 4; i++) {
        servo[probe_index].move(probe.z_servo_angle[0]); // deploy
        printer.safe_delay(500);
        deploy_state = digitalRead(PROBE_TEST_PIN);
        servo[probe_index].move(probe.z_servo_angle[1]); // stow
        printer.safe_delay(500);
        stow_state = digitalRead(PROBE_TEST_PIN);
      }
      if (probe_inverting != deploy_state) SERIAL_EM("WARNING - INVERTING setting probably backwards");

      refresh_cmd_timeout();

      if (deploy_state != stow_state) {
        SERIAL_EM("BLTouch clone detected");
        if (deploy_state) {
          SERIAL_EM(".  DEPLOYED state: HIGH (logic 1)");
          SERIAL_EM(".  STOWED (triggered) state: LOW (logic 0)");
        }
        else {
          SERIAL_EM(".  DEPLOYED state: LOW (logic 0)");
          SERIAL_EM(".  STOWED (triggered) state: HIGH (logic 1)");
        }
        #if ENABLED(BLTOUCH)
          SERIAL_EM("ERROR: BLTOUCH enabled - set this device up as a Z Servo Probe with inverting as true.");
        #endif

      }
      else {                                              // measure active signal length
        servo[probe_index].move(probe.z_servo_angle[0]);  // deploy
        printer.safe_delay(500);
        SERIAL_EM("please trigger probe");
        uint16_t probe_counter = 0;

        // Allow 30 seconds max for operator to trigger probe
        for (uint16_t j = 0; j < 500 * 30 && probe_counter == 0 ; j++) {

          printer.safe_delay(2);

          if (0 == j % (500 * 1)) // keep cmd_timeout happy
            refresh_cmd_timeout();

          if (deploy_state != digitalRead(PROBE_TEST_PIN)) { // probe triggered

            for (probe_counter = 1; probe_counter < 50 && (deploy_state != digitalRead(PROBE_TEST_PIN)); probe_counter ++)
              printer.safe_delay(2);

            if (probe_counter == 50)
              SERIAL_EM("Z Servo Probe detected");   // >= 100mS active time
            else if (probe_counter >= 2 )
              SERIAL_EMV("BLTouch compatible probe detected - pulse width (+/- 4mS): ", probe_counter * 2 );   // allow 4 - 100mS pulse
            else
              SERIAL_EM("noise detected - please re-run test");   // less than 2mS pulse

            servo[probe_index].move(probe.z_servo_angle[1]); // stow

          } // pulse detected

        } // for loop waiting for trigger

        if (probe_counter == 0) SERIAL_EM("trigger not detected");

      } // measure active signal length

    #endif

  } // servo_probe_test

  /**
   * M43: Pin debug - report pin state, watch pins, toggle pins and servo probe test/report
   *
   *  M43         - report name and state of pin(s)
   *                  P<pin>  Pin to read or watch. If omitted, reads all pins.
   *                  I       Flag to ignore MK4duo's pin protection.
   *
   *  M43 W       - Watch pins -reporting changes- until reset, click, or M108.
   *                  P<pin>  Pin to read or watch. If omitted, read/watch all pins.
   *                  I       Flag to ignore MK4duo's pin protection.
   *
   *  M43 E<bool> - Enable / disable background endstop monitoring
   *                  - Machine continues to operate
   *                  - Reports changes to endstops
   *                  - Toggles LED when an endstop changes
   *                  - Can not reliably catch the 5mS pulse from BLTouch type probes
   *
   *  M43 T       - Toggle pin(s) and report which pin is being toggled
   *                  S<pin>  - Start Pin number.   If not given, will default to 0
   *                  L<pin>  - End Pin number.   If not given, will default to last pin defined for this board
   *                  I       - Flag to ignore MK4duo's pin protection.   Use with caution!!!!
   *                  R       - Repeat pulses on each pin this number of times before continueing to next pin
   *                  W       - Wait time (in miliseconds) between pulses.  If not given will default to 500
   *
   *  M43 S       - Servo probe test
   *                  P<index> - Probe index (optional - defaults to 0)
   */
  inline void gcode_M43() {

    if (parser.seen('T')) {   // must be first ot else it's "S" and "E" parameters will execute endstop or servo test
      toggle_pins();
      return;
    }

    // Enable or disable endstop monitoring
    if (parser.seen('E')) {
      endstop_monitor_flag = parser.value_bool();
      SERIAL_MSG("endstop monitor ");
      SERIAL_TXT(endstop_monitor_flag ? "en" : "dis");
      SERIAL_EM("abled");
      return;
    }

    if (parser.seen('S')) {
      servo_probe_test();
      return;
    }

    // Get the range of pins to test or watch
    const uint8_t first_pin = parser.seen('P') ? parser.value_byte() : 0,
                  last_pin = parser.seen('P') ? first_pin : LAST_PIN - 1;

    if (first_pin > last_pin) return;

    const bool ignore_protection = parser.seen('I') && parser.value_bool();

    // Watch until click, M108, or reset
    if (parser.seen('W') && parser.value_bool()) { // watch digital pins
      SERIAL_EM("Watching pins");
      byte pin_state[last_pin - first_pin + 1];
      for (int8_t pin = first_pin; pin <= last_pin; pin++) {
        if (printer.pin_is_protected(pin) && !ignore_protection) continue;
        pinMode(pin, INPUT_PULLUP);
        // if (IS_ANALOG(pin))
        //   pin_state[pin - first_pin] = analogRead(pin - analogInputToDigitalPin(0)); // int16_t pin_state[...]
        // else
          pin_state[pin - first_pin] = digitalRead(pin);
      }

      #if HAS(RESUME_CONTINUE)
        wait_for_user = true;
        KEEPALIVE_STATE(PAUSED_FOR_USER);
      #endif

      for(;;) {
        for (int8_t pin = first_pin; pin <= last_pin; pin++) {
          if (printer.pin_is_protected(pin)) continue;
          byte val;
          // if (IS_ANALOG(pin))
          //   val = analogRead(pin - analogInputToDigitalPin(0)); // int16_t val
          // else
            val = digitalRead(pin);
          if (val != pin_state[pin - first_pin]) {
            report_pin_state(pin);
            pin_state[pin - first_pin] = val;
          }
        }

        #if HAS(RESUME_CONTINUE)
          if (!wait_for_user) {
            KEEPALIVE_STATE(IN_HANDLER);
            break;
          }
        #endif

        printer.safe_delay(500);
      }
      return;
    }

    // Report current state of selected pin(s)
    for (uint8_t pin = first_pin; pin <= last_pin; pin++)
      report_pin_state_extended(pin, ignore_protection);
  }

#endif // PINS_DEBUGGING

#if ENABLED(Z_MIN_PROBE_REPEATABILITY_TEST)

  /**
   * M48: Z-Probe repeatability measurement function.
   *
   * Usage:
   *   M48 <P#> <X#> <Y#> <V#> <E> <L#> <S>
   *     P = Number of sampled points (4-50, default 10)
   *     X = Sample X position
   *     Y = Sample Y position
   *     V = Verbose level (0-4, default=1)
   *     E = Engage probe for each reading
   *     L = Number of legs of movement before probe
   *     S = Schizoid (Or Star if you prefer)
   *
   * This function assumes the bed has been homed.  Specifically, that a G28 command
   * as been issued prior to invoking the M48 Z-Probe repeatability measurement function.
   * Any information generated by a prior G29 Bed leveling command will be lost and need to be
   * regenerated.
   */
  inline void gcode_M48() {

    if (mechanics.axis_unhomed_error()) return;

    int8_t verbose_level = parser.seen('V') ? parser.value_byte() : 1;
    if (!WITHIN(verbose_level, 0, 4)) {
      SERIAL_LM(ER, "?Verbose Level not plausible (0-4).");
      return;
    }

    if (verbose_level > 0)
      SERIAL_EM("M48 Z-Probe Repeatability Test");

    int8_t n_samples = parser.seen('P') ? parser.value_byte() : 10;
    if (!WITHIN(n_samples, 4, 50)) {
      SERIAL_LM(ER, "?Sample size not plausible (4-50).");
      return;
    }

    float X_current = mechanics.current_position[X_AXIS],
          Y_current = mechanics.current_position[Y_AXIS];

    bool stow_probe_after_each = parser.seen('E');

    float X_probe_location = parser.seen('X') ? parser.value_linear_units() : X_current + X_PROBE_OFFSET_FROM_NOZZLE;
    float Y_probe_location = parser.seen('Y') ? parser.value_linear_units() : Y_current + Y_PROBE_OFFSET_FROM_NOZZLE;

    #if NOMECH(DELTA)
      if (!WITHIN(X_probe_location, LOGICAL_X_POSITION(MIN_PROBE_X), LOGICAL_X_POSITION(MAX_PROBE_X))) {
        out_of_range_error(PSTR("X"));
        return;
      }
      if (!WITHIN(Y_probe_location, LOGICAL_Y_POSITION(MIN_PROBE_Y), LOGICAL_Y_POSITION(MAX_PROBE_Y))) {
        out_of_range_error(PSTR("Y"));
        return;
      }
    #else
      if (!mechanics.position_is_reachable_by_probe_xy(X_probe_location, Y_probe_location)) {
        SERIAL_LM(ER, "? (X,Y) location outside of probeable radius.");
        return;
      }
    #endif

    bool seen_L = parser.seen('L');
    uint8_t n_legs = seen_L ? parser.value_byte() : 0;
    if (n_legs > 15) {
      SERIAL_LM(ER, "?Number of legs in movement not plausible (0-15).");
      return;
    }
    if (n_legs == 1) n_legs = 2;

    bool schizoid_flag = parser.seen('S');
    if (schizoid_flag && !seen_L) n_legs = 7;

    /**
     * Now get everything to the specified probe point So we can safely do a
     * probe to get us close to the bed.  If the Z-Axis is far from the bed,
     * we don't want to use that as a starting point for each probe.
     */
    if (verbose_level > 2)
      SERIAL_EM("Positioning the probe...");

    // Disable bed level correction in M48 because we want the raw data when we probe
    #if HAS_LEVELING
      const bool was_enabled = bedlevel.leveling_is_active();
      bedlevel.set_bed_leveling_enabled(false);
    #endif

    setup_for_endstop_or_probe_move();

    // Move to the first point, deploy, and probe
    const float t = probe.check_pt(X_probe_location, Y_probe_location, stow_probe_after_each, verbose_level);
    if (isnan(t)) return;

    randomSeed(millis());

    double mean = 0.0, sigma = 0.0, min = 99999.9, max = -99999.9, sample_set[n_samples];

    for (uint8_t n = 0; n < n_samples; n++) {
      if (n_legs) {
        int dir = (random(0, 10) > 5.0) ? -1 : 1;  // clockwise or counter clockwise
        float angle = random(0.0, 360.0),
              radius = random(
                #if MECH(DELTA)
                  mechanics.delta_probe_radius / 8, mechanics.delta_probe_radius / 3
                #else
                  5, X_MAX_LENGTH / 8
                #endif
              );

        if (verbose_level > 3) {
          SERIAL_MV("Starting radius: ", radius);
          SERIAL_MV("   angle: ", angle);
          SERIAL_MSG(" Direction: ");
          if (dir > 0) SERIAL_MSG("Counter-");
          SERIAL_EM("Clockwise");
        }

        for (uint8_t l = 0; l < n_legs - 1; l++) {
          double delta_angle;

          if (schizoid_flag)
            // The points of a 5 point star are 72 degrees apart.  We need to
            // skip a point and go to the next one on the star.
            delta_angle = dir * 2.0 * 72.0;

          else
            // If we do this line, we are just trying to move further
            // around the circle.
            delta_angle = dir * (float) random(25, 45);

          angle += delta_angle;

          while (angle > 360.0)   // We probably do not need to keep the angle between 0 and 2*PI, but the
            angle -= 360.0;       // Arduino documentation says the trig functions should not be given values
          while (angle < 0.0)     // outside of this range.   It looks like they behave correctly with
            angle += 360.0;       // numbers outside of the range, but just to be safe we clamp them.

          X_current = X_probe_location - (X_PROBE_OFFSET_FROM_NOZZLE) + cos(RADIANS(angle)) * radius;
          Y_current = Y_probe_location - (Y_PROBE_OFFSET_FROM_NOZZLE) + sin(RADIANS(angle)) * radius;

          #if MECH(DELTA)
            // If we have gone out too far, we can do a simple fix and scale the numbers
            // back in closer to the origin.
            while (!mechanics.position_is_reachable_by_probe_xy(X_current, Y_current)) {
              X_current *= 0.8;
              Y_current *= 0.8;
              if (verbose_level > 3) {
                SERIAL_MV("Pulling point towards center:", X_current);
                SERIAL_EMV(", ", Y_current);
              }
            }
          #else
            X_current = constrain(X_current, X_MIN_POS, X_MAX_POS);
            Y_current = constrain(Y_current, Y_MIN_POS, Y_MAX_POS);
          #endif
          if (verbose_level > 3) {
            SERIAL_MSG("Going to:");
            SERIAL_MV(" x: ", X_current);
            SERIAL_MV(" y: ", Y_current);
            SERIAL_EMV("  z: ", mechanics.current_position[Z_AXIS]);
          }
          mechanics.do_blocking_move_to_xy(X_current, Y_current);
        } // n_legs loop
      } // n_legs

      // Probe a single point
      sample_set[n] = probe.check_pt(X_probe_location, Y_probe_location, stow_probe_after_each, 0);

      /**
       * Get the current mean for the data points we have so far
       */
      double sum = 0.0;
      for (uint8_t j = 0; j <= n; j++) sum += sample_set[j];
      mean = sum / (n + 1);

      NOMORE(min, sample_set[n]);
      NOLESS(max, sample_set[n]);

      /**
       * Now, use that mean to calculate the standard deviation for the
       * data points we have so far
       */
      sum = 0.0;
      for (uint8_t j = 0; j <= n; j++)
        sum += sq(sample_set[j] - mean);

      sigma = SQRT(sum / (n + 1));
      if (verbose_level > 0) {
        if (verbose_level > 1) {
          SERIAL_VAL(n + 1);
          SERIAL_MV(" of ", (int)n_samples);
          SERIAL_MV(": z: ", sample_set[n], 3);
          if (verbose_level > 2) {
            SERIAL_MV(" mean: ", mean, 4);
            SERIAL_MV(" sigma: ", sigma, 6);
            SERIAL_MV(" min: ", min, 3);
            SERIAL_MV(" max: ", max, 3);
            SERIAL_MV(" range: ", max - min, 3);
          }
          SERIAL_EOL();
        }
      }

    }  // End of probe loop

    if (probe.set_deployed(false)) return;

    SERIAL_EM("Finished!");

    if (verbose_level > 0) {
      SERIAL_MV("Mean: ", mean, 6);
      SERIAL_MV(" Min: ", min, 3);
      SERIAL_MV(" Max: ", max, 3);
      SERIAL_MV(" Range: ", max - min, 3);
      SERIAL_EOL();
    }

    SERIAL_EMV("Standard Deviation: ", sigma, 6);
    SERIAL_EOL();

    clean_up_after_endstop_or_probe_move();

    // Re-enable bed level correction if it had been on
    #if HAS_LEVELING
      bedlevel.set_bed_leveling_enabled(was_enabled);
    #endif

    mechanics.report_current_position();
  }

#endif // Z_MIN_PROBE_REPEATABILITY_TEST

#if HAS_POWER_CONSUMPTION_SENSOR
  /**
   * M70 - Power consumption sensor calibration
   *
   * Z - Calibrate zero current offset
   * A - Isert readed DC Current value (Ampere)
   * W - Insert readed AC Wattage value (Watt)
   */
  inline void gcode_M70() {
    if(parser.seen('Z')) {
      SERIAL_EMV("Actual POWER_ZERO:", POWER_ZERO, 7);
      SERIAL_EMV("New POWER_ZERO:", raw_analog2voltage(), 7);
      SERIAL_EM("Insert new calculated values into the FW and call \"M70 A\" for the next calibration step.");
    }
    else if(parser.seen('A')) {
      SERIAL_EMV("Actual POWER_ERROR:", POWER_ERROR, 7);
      SERIAL_EMV("New POWER_ERROR:", analog2error(parser.value_float()), 7);
      SERIAL_EM("Insert new calculated values into the FW and call \"M70 W\" for the last calibration step.");
    }
    else if(parser.seen('W')) {
      SERIAL_EMV("Actual POWER_EFFICIENCY:", POWER_EFFICIENCY, 7);
      SERIAL_EMV("New POWER_EFFICIENCY:", analog2efficiency(parser.value_float()), 7);
      SERIAL_EM("Insert new calculated values into the FW and then ACS712 it should be calibrated correctly.");
    }
  }
#endif

/**
 * M75: Start print timer
 */
inline void gcode_M75() { printer.print_job_counter.start(); }

/**
 * M76: Pause print timer
 */
inline void gcode_M76() { printer.print_job_counter.pause(); }

/**
 * M77: Stop print timer
 */
inline void gcode_M77() { printer.print_job_counter.stop(); }

/**
 * M78: Show print statistics
 */
inline void gcode_M78() {
  // "M78 S78" will reset the statistics
  if (parser.seen('S') && parser.value_int() == 78)
    printer.print_job_counter.initStats();
  else printer.print_job_counter.showStats();
}

#if HAS_POWER_SWITCH
  /**
   * M80   : Turn on Power Supply
   * M80 S : Report the current state and exit
   */
  inline void gcode_M80() {

    // S: Report the current power supply state and exit
    if (parser.seen('S')) {
      SERIAL_PS(powerManager.powersupply_on ? PSTR("PS:1\n") : PSTR("PS:0\n"));
      return;
    }

    powerManager.power_on();

    // If you have a switch on suicide pin, this is useful
    // if you want to start another print with suicide feature after
    // a print without suicide...
    #if HAS(SUICIDE)
      OUT_WRITE(SUICIDE_PIN, HIGH);
    #endif

    #if ENABLED(HAVE_TMC2130)
      delay(100);
      tmc2130_init(); // Settings only stick when the driver has power
    #endif

    LCD_MESSAGEPGM(WELCOME_MSG);

    #if ENABLED(LASER) && ENABLED(LASER_PERIPHERALS)
      laser.peripherals_on();
      laser.wait_for_peripherals();
    #endif
  }
#endif // HAS_POWER_SWITCH

/**
 * M81: Turn off Power, including Power Supply, if there is one.
 *
 *      This code should ALWAYS be available for EMERGENCY SHUTDOWN!
 */
inline void gcode_M81() {
  thermalManager.disable_all_heaters();
  thermalManager.disable_all_coolers();
  stepper.finish_and_disable();

  #if FAN_COUNT > 0
    LOOP_FAN() printer.fanSpeeds[f] = 0;
    #if ENABLED(PROBING_FANS_OFF)
      printer.fans_paused = false;
      ZERO(printer.paused_fanSpeeds);
    #endif
  #endif

  #if ENABLED(LASER)
    laser.extinguish();
    #if ENABLED(LASER_PERIPHERALS)
      laser.peripherals_off();
    #endif
  #endif

  #if ENABLED(CNCROUTER)
    disable_cncrouter();
  #endif

  printer.safe_delay(1000); // Wait 1 second before switching off

  #if HAS(SUICIDE)
    stepper.synchronize();
    suicide();
  #elif HAS_POWER_SWITCH
    powerManager.power_off();
  #endif

  LCD_MESSAGEPGM(MACHINE_NAME " " MSG_OFF ".");

}

/**
 * M82: Set E codes absolute (default)
 */
inline void gcode_M82() { printer.axis_relative_modes[E_AXIS] = false; }

/**
 * M83: Set E codes relative while in Absolute Coordinates (G90) mode
 */
inline void gcode_M83() { printer.axis_relative_modes[E_AXIS] = true; }

/**
 * M18, M84: Disable stepper motors
 */
inline void gcode_M18_M84() {
  if (parser.seenval('S')) {
    stepper.stepper_inactive_time = parser.value_millis_from_seconds();
  }
  else {
    bool all_axis = !(parser.seen_axis());
    if (all_axis) {
      stepper.finish_and_disable();
    }
    else {
      stepper.synchronize();
      if (parser.seen('X')) disable_X();
      if (parser.seen('Y')) disable_Y();
      if (parser.seen('Z')) disable_Z();
      #if E0_ENABLE_PIN != X_ENABLE_PIN && E1_ENABLE_PIN != Y_ENABLE_PIN // Only enable on boards that have seperate ENABLE_PINS
        if (parser.seen('E')) stepper.disable_e_steppers();
      #endif
    }
  }
}

/**
 * M85: Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
 */
inline void gcode_M85() {
  if (parser.seenval('S')) printer.max_inactive_time = parser.value_millis_from_seconds();
}

/**
 * M92: Set axis steps-per-unit for one or more axes, X, Y, Z, and E.
 *      (Follows the same syntax as G92)
 *
 *      With multiple extruders use T to specify which one.
 */
inline void gcode_M92() {

  GET_TARGET_EXTRUDER(92);

  LOOP_XYZE(i) {
    if (parser.seen(axis_codes[i])) {
      if (i == E_AXIS) {
        const float value = parser.value_per_axis_unit((AxisEnum)(E_AXIS + TARGET_EXTRUDER));
        if (value < 20.0) {
          float factor = mechanics.axis_steps_per_mm[E_AXIS + TARGET_EXTRUDER] / value; // increase e constants if M92 E14 is given for netfab.
          mechanics.max_jerk[E_AXIS + TARGET_EXTRUDER] *= factor;
          mechanics.max_feedrate_mm_s[E_AXIS + TARGET_EXTRUDER] *= factor;
          mechanics.max_acceleration_steps_per_s2[E_AXIS + TARGET_EXTRUDER] *= factor;
        }
        mechanics.axis_steps_per_mm[E_AXIS + TARGET_EXTRUDER] = value;
      }
      else {
        mechanics.axis_steps_per_mm[i] = parser.value_per_axis_unit((AxisEnum)i);
      }
    }
  }
  mechanics.refresh_positioning();
}

#if ENABLED(ZWOBBLE)
  /**
   * M96: Print ZWobble value
   */
  inline void gcode_M96() {
    mechanics.report_zwobble();
  }

  /**
   * M97: Set ZWobble value
   */
  inline void gcode_M97() {
    float zVal = -1.0, hVal = -1.0, lVal = -1.0;

    if (parser.seen('A')) mechanics.set_zwobble_amplitude(parser.value_float());
    if (parser.seen('W')) mechanics.set_zwobble_period(parser.value_float());
    if (parser.seen('P')) mechanics.set_zwobble_phase(parser.value_float());
    if (parser.seen('Z')) zVal = parser.value_float();
    if (parser.seen('H')) hVal = parser.value_float();
    if (parser.seen('L')) lVal = parser.value_float();
    if (zVal >= 0 && hVal >= 0) mechanics.set_zwobble_sample(zVal, hVal);
    if (zVal >= 0 && lVal >= 0) mechanics.set_zwobble_scaledsample(zVal, lVal);
    if (lVal >  0 && hVal >  0) mechanics.set_zwobble_scalingfactor(hVal / lVal);
  }
#endif // ZWOBBLE

#if ENABLED(HYSTERESIS)

  /**
   * M98: Print Hysteresis value
   */
  inline void gcode_M98() {
    mechanics.report_hysteresis();
  }

  /**
   * M99: Set Hysteresis value
   */
  inline void gcode_M99() {
    LOOP_XYZE(i) {
      if (parser.seen(axis_codes[i]))
        mechanics.set_hysteresis_axis(i, parser.value_float());
    }
  }
#endif // HYSTERESIS

#if HAS_TEMP_HOTEND

  /**
   * M104: Set hotend temperature
   */
  inline void gcode_M104() {

    GET_TARGET_EXTRUDER(104);

    if (DEBUGGING(DRYRUN)) return;

    #if ENABLED(SINGLENOZZLE)
      if (TARGET_EXTRUDER != printer.active_extruder) return;
    #endif

    if (parser.seenval('S')) {
      const int16_t temp = parser.value_celsius();
      thermalManager.setTargetHotend(temp, TARGET_EXTRUDER);

      #if ENABLED(DUAL_X_CARRIAGE)
        if (mechanics.dual_x_carriage_mode == DXC_DUPLICATION_MODE && TARGET_EXTRUDER == 0)
          thermalManager.setTargetHotend(temp ? temp + mechanics.duplicate_hotend_temp_offset : 0, 1);
      #endif

      if (temp > thermalManager.degHotend(TARGET_EXTRUDER))
        lcd_status_printf_P(0, PSTR("H%i %s"), TARGET_EXTRUDER, MSG_HEATING);
    }

    #if ENABLED(AUTOTEMP)
      planner.autotemp_M104_M109();
    #endif

  }

#endif

/**
 * M105: Read hot end and bed temperature
 */
inline void gcode_M105() {

  GET_TARGET_HOTEND(105);

  #if HAS_TEMP_HOTEND || HAS_TEMP_BED || HAS_TEMP_CHAMBER || HAS_TEMP_COOLER || ENABLED(FLOWMETER_SENSOR) || (ENABLED(CNCROUTER) && ENABLED(FAST_PWM_CNCROUTER))
    SERIAL_STR(OK);
    #if HAS_TEMP_HOTEND || HAS_TEMP_BED
      thermalManager.print_heaterstates();
    #endif
    #if HAS_TEMP_CHAMBER
      thermalManager.print_chamberstate();
    #endif
    #if HAS_TEMP_COOLER
      thermalManager.print_coolerstate();
    #endif
    #if ENABLED(FLOWMETER_SENSOR)
      print_flowratestate();
    #endif
    #if ENABLED(CNCROUTER) && ENABLED(FAST_PWM_CNCROUTER)
      print_cncspeed();
    #endif
    #if ENABLED(ARDUINO_ARCH_SAM) && !MB(RADDS)
      thermalManager.print_MCUstate();
    #endif
  #else // HASNT(TEMP_0) && HASNT(TEMP_BED)
    SERIAL_LM(ER, MSG_ERR_NO_THERMISTORS);
  #endif

  SERIAL_EOL();
}

#if FAN_COUNT > 0

  #if ENABLED(FAN_MIN_PWM)
    #define CALC_FAN_SPEED() (speed ? ( FAN_MIN_PWM + (speed * (255 - FAN_MIN_PWM)) / 255 ) : 0)
  #else
    #define CALC_FAN_SPEED() speed
  #endif

  /**
   * M106: Set Fan Speed
   *
   *  S<int>   Speed between 0-255
   *  P<index> Fan index, if more than one fan
   */
  inline void gcode_M106() {
    uint8_t speed = parser.seen('S') ? parser.value_ushort() : 255,
            fan   = parser.seen('P') ? parser.value_ushort() : 0;

    if (fan >= FAN_COUNT || printer.fanSpeeds[fan] == speed)
      return;

    #if ENABLED(FAN_KICKSTART_TIME)
      if (printer.fanKickstart == 0 && speed > printer.fanSpeeds[fan] && speed < 85) {
        if (printer.fanSpeeds[fan]) printer.fanKickstart = FAN_KICKSTART_TIME / 100;
        else                printer.fanKickstart = FAN_KICKSTART_TIME / 25;
      }
    #endif

    printer.fanSpeeds[fan] = CALC_FAN_SPEED();
  }

  /**
   * M107: Fan Off
   */
  inline void gcode_M107() {
    uint16_t p = parser.seen('P') ? parser.value_ushort() : 0;
    if (p < FAN_COUNT) printer.fanSpeeds[p] = 0;
  }

#endif // FAN_COUNT > 0

#if DISABLED(EMERGENCY_PARSER)
  /**
   * M108: Cancel heatup and wait for the hotend and bed, this G-code is asynchronously handled in the get_serial_commands() parser
   */
  inline void gcode_M108() { thermalManager.wait_for_heatup = false; }
#endif

#if HAS_TEMP_HOTEND

  /**
   * M109: Sxxx Wait for hotend(s) to reach temperature. Waits only when heating.
   *       Rxxx Wait for hotend(s) to reach temperature. Waits when heating and cooling.
   */
  inline void gcode_M109() {

    GET_TARGET_EXTRUDER(109);
    if (DEBUGGING(DRYRUN)) return;

    #if ENABLED(SINGLENOZZLE)
      if (TARGET_EXTRUDER != printer.active_extruder) return;
    #endif

    const bool no_wait_for_cooling = parser.seenval('S');
    if (no_wait_for_cooling || parser.seenval('R')) {
      const int16_t temp = parser.value_celsius();
      thermalManager.setTargetHotend(temp, TARGET_EXTRUDER);

      #if ENABLED(DUAL_X_CARRIAGE)
        if (mechanics.dual_x_carriage_mode == DXC_DUPLICATION_MODE && TARGET_EXTRUDER == 0)
          thermalManager.setTargetHotend(temp ? temp + mechanics.duplicate_hotend_temp_offset : 0, 1);
      #endif

      if (thermalManager.isHeatingHotend(TARGET_EXTRUDER))
        lcd_status_printf_P(0, PSTR("H%i %s"), TARGET_EXTRUDER, MSG_HEATING);
    }

    #if ENABLED(AUTOTEMP)
      planner.autotemp_M104_M109();
    #endif

    printer.wait_heater(no_wait_for_cooling);
  }

#endif

/**
 * M110: Set Current Line Number
 */
inline void gcode_M110() {
  if (parser.seenval('N')) commands.gcode_LastN = parser.value_long();
}

/**
 * M111: Debug mode Repetier Host compatibile
 */
inline void gcode_M111() {
  commands.mk_debug_flags = parser.byteval('S', (uint8_t)DEBUG_NONE);

  const static char str_debug_1[]   PROGMEM = MSG_DEBUG_ECHO;
  const static char str_debug_2[]   PROGMEM = MSG_DEBUG_INFO;
  const static char str_debug_4[]   PROGMEM = MSG_DEBUG_ERRORS;
  const static char str_debug_8[]   PROGMEM = MSG_DEBUG_DRYRUN;
  const static char str_debug_16[]  PROGMEM = MSG_DEBUG_COMMUNICATION;
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    const static char str_debug_32[] PROGMEM = MSG_DEBUG_LEVELING;
  #endif

  const static char* const debug_strings[] PROGMEM = {
    str_debug_1, str_debug_2, str_debug_4, str_debug_8, str_debug_16,
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      str_debug_32
    #endif
  };

  SERIAL_SM(ECHO, MSG_DEBUG_PREFIX);
  if (commands.mk_debug_flags) {
    uint8_t comma = 0;
    for (uint8_t i = 0; i < COUNT(debug_strings); i++) {
      if (TEST(commands.mk_debug_flags, i)) {
        if (comma++) SERIAL_CHR(',');
        SERIAL_PS((char*)pgm_read_word(&(debug_strings[i])));
      }
    }
  }
  else {
    SERIAL_MSG(MSG_DEBUG_OFF);
  }
  SERIAL_EOL();
}

#if DISABLED(EMERGENCY_PARSER)
  /**
   * M112: Emergency Stop
   */
  inline void gcode_M112() { printer.kill(PSTR(MSG_KILLED)); }
#endif

#if ENABLED(HOST_KEEPALIVE_FEATURE)
  /**
   * M113: Get or set Host Keepalive interval (0 to disable)
   *
   *   S<seconds> Optional. Set the keepalive interval.
   */
  inline void gcode_M113() {
    if (parser.seenval('S')) {
      printer.host_keepalive_interval = parser.value_byte();
      NOMORE(printer.host_keepalive_interval, 60);
    }
    else {
      SERIAL_EMV("M113 S", (unsigned long)printer.host_keepalive_interval);
    }
  }
#endif

/**
 * M114: Report current position to host
 */
inline void gcode_M114() {

  if (parser.seen('D')) {
    mechanics.report_current_position_detail();
    return;
  }

  stepper.synchronize();
  mechanics.report_current_position();
}

/**
 * M115: Capabilities string
 */
inline void gcode_M115() {
  SERIAL_EM(MSG_M115_REPORT);

  #if ENABLED(EXTENDED_CAPABILITIES_REPORT)

    // EEPROM (M500, M501)
    #if ENABLED(EEPROM_SETTINGS)
      SERIAL_LM(CAP, "EEPROM:1");
    #else
      SERIAL_LM(CAP, "EEPROM:0");
    #endif

    // AUTOREPORT_TEMP (M155)
    #if ENABLED(AUTO_REPORT_TEMPERATURES)
      SERIAL_LM(CAP, "AUTOREPORT_TEMP:1");
    #else
      SERIAL_LM(CAP, "AUTOREPORT_TEMP:0");
    #endif

    // PROGRESS (M530 S L, M531 <file>, M532 X L)
    SERIAL_LM(CAP, "PROGRESS:1");

    // Print Job timer M75, M76, M77
    SERIAL_LM(CAP, "PRINT_JOB:1");

    // AUTOLEVEL (G29)
    #if HAS_ABL
      SERIAL_LM(CAP, "AUTOLEVEL:1");
    #else
      SERIAL_LM(CAP, "AUTOLEVEL:0");
    #endif

    // Z_PROBE (G30)
    #if HAS_BED_PROBE
      SERIAL_LM(CAP, "Z_PROBE:1");
    #else
      SERIAL_LM(CAP, "Z_PROBE:0");
    #endif

    // MESH_REPORT (M320 V, M420 V)
    #if HAS_LEVELING
      SERIAL_LM(CAP, "LEVELING_DATA:1");
    #else
      SERIAL_LM(CAP, "LEVELING_DATA:0");
    #endif

    // SOFTWARE_POWER (M80, M81)
    #if HAS_POWER_SWITCH
      SERIAL_LM(CAP, "SOFTWARE_POWER:1");
    #else
      SERIAL_LM(CAP, "SOFTWARE_POWER:0");
    #endif

    // CASE LIGHTS (M355)
    #if HAS_CASE_LIGHT
      SERIAL_LM(CAP, "TOGGLE_LIGHTS:1");
    #else
      SERIAL_LM(CAP, "TOGGLE_LIGHTS:0");
    #endif

    // EMERGENCY_PARSER (M108, M112, M410)
    #if ENABLED(EMERGENCY_PARSER)
      SERIAL_LM(CAP, "EMERGENCY_PARSER:1");
    #else
      SERIAL_LM(CAP, "EMERGENCY_PARSER:0");
    #endif

  #endif // EXTENDED_CAPABILITIES_REPORT
}

/**
 * M117: Set LCD Status Message
 */
inline void gcode_M117() { lcd_setstatus(parser.string_arg); }

/**
 * M118: Display a message in the host console.
 *
 *  A  Append '// ' for an action command, as in OctoPrint
 *  E  Have the host 'echo:' the text
 */
inline void gcode_M118() {
  if (parser.boolval('E')) SERIAL_STR(ECHO);
  if (parser.boolval('A')) SERIAL_MSG("// ");
  SERIAL_ET(parser.string_arg);
}

/**
 * M119: Output endstop states to serial output
 */
inline void gcode_M119() { endstops.M119(); }

/**
 * M120: Enable endstops and set non-homing endstop state to "enabled"
 */
inline void gcode_M120() { endstops.enable_globally(true); }

/**
 * M121: Disable endstops and set non-homing endstop state to "disabled"
 */
inline void gcode_M121() { endstops.enable_globally(false); }

/**
 * M122: Enable, Disable, and/or Report software endstops
 *
 * Usage: M122 S1 to enable, M122 S0 to disable, M122 alone for report
 */
inline void gcode_M122() {
  #if HAS_SOFTWARE_ENDSTOPS
    if (parser.seen('S')) endstops.soft_endstops_enabled = parser.value_bool();
    SERIAL_SM(ECHO, MSG_SOFT_ENDSTOPS);
    SERIAL_PS(endstops.soft_endstops_enabled ? PSTR(MSG_ON) : PSTR(MSG_OFF));
  #else
    SERIAL_MSG(MSG_SOFT_ENDSTOPS);
    SERIAL_MSG(MSG_OFF);
  #endif
  SERIAL_MSG(MSG_SOFT_MIN);
  SERIAL_MV(    MSG_X, endstops.soft_endstop_min[X_AXIS]);
  SERIAL_MV(" " MSG_Y, endstops.soft_endstop_min[Y_AXIS]);
  SERIAL_MV(" " MSG_Z, endstops.soft_endstop_min[Z_AXIS]);
  SERIAL_MSG(MSG_SOFT_MAX);
  SERIAL_MV(    MSG_X, endstops.soft_endstop_max[X_AXIS]);
  SERIAL_MV(" " MSG_Y, endstops.soft_endstop_max[Y_AXIS]);
  SERIAL_MV(" " MSG_Z, endstops.soft_endstop_max[Z_AXIS]);
  SERIAL_EOL();
}

#if ENABLED(PARK_HEAD_ON_PAUSE)

  /**
   * M125: Store current position and move to pause park position.
   *       Called on pause (by M25) to prevent material leaking onto the
   *       object. On resume (M24) the head will be moved back and the
   *       print will resume.
   *
   *       If MK4duo is compiled without SD Card support, M125 can be
   *       used directly to pause the print and move to park position,
   *       resuming with a button click or M108.
   *
   *    L = override retract length
   *    X = override X
   *    Y = override Y
   *    Z = override Z raise
   */
  inline void gcode_M125() {

    // Initial retract before move to pause park position
    const float retract = parser.seen('L') ? parser.value_axis_units(E_AXIS) : 0
      #if ENABLED(PAUSE_PARK_RETRACT_LENGTH) && PAUSE_PARK_RETRACT_LENGTH > 0
        - (PAUSE_PARK_RETRACT_LENGTH)
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

    // Move XY axes to pause park position or given position
    const float x_pos = parser.seen('X') ? parser.value_linear_units() : 0
      #ifdef PAUSE_PARK_X_POS
        + PAUSE_PARK_X_POS
      #endif
    ;
    const float y_pos = parser.seen('Y') ? parser.value_linear_units() : 0
      #ifdef PAUSE_PARK_Y_POS
        + PAUSE_PARK_Y_POS
      #endif
    ;

    #if HOTENDS > 1 && DISABLED(DUAL_X_CARRIAGE)
      if (printer.active_extruder > 0) {
        if (!parser.seen('X')) x_pos += printer.hotend_offset[X_AXIS][printer.active_extruder];
        if (!parser.seen('Y')) y_pos += printer.hotend_offset[Y_AXIS][printer.active_extruder];
      }
    #endif

    const bool job_running = printer.print_job_counter.isRunning();

    if (printer.pause_print(retract, 0, z_lift, x_pos, y_pos)) {
      if (!IS_SD_PRINTING) {
        // Wait for lcd click or M108
        printer.wait_for_filament_reload();

        // Return to print position and continue
        printer.resume_print();

        if (job_running) printer.print_job_counter.start();
      }
    }
  }

#endif // PARK_HEAD_ON_PAUSE

#if ENABLED(BARICUDA)
  #if HAS_HEATER_1
    /**
     * M126: Heater 1 valve open
     */
    inline void gcode_M126() { baricuda_valve_pressure = parser.byteval('S', 255); }
    /**
     * M127: Heater 1 valve close
     */
    inline void gcode_M127() { baricuda_valve_pressure = 0; }
  #endif

  #if HAS_HEATER_2
    /**
     * M128: Heater 2 valve open
     */
    inline void gcode_M128() { baricuda_e_to_p_pressure = parser.byteval('S', 255); }
    /**
     * M129: Heater 2 valve close
     */
    inline void gcode_M129() { baricuda_e_to_p_pressure = 0; }
  #endif
#endif // BARICUDA

#if HAS_TEMP_BED
  /**
   * M140: Set Bed temperature
   */
  inline void gcode_M140() {
    if (DEBUGGING(DRYRUN)) return;
    if (parser.seenval('S')) thermalManager.setTargetBed(parser.value_celsius());
  }
#endif

#if HAS_TEMP_CHAMBER
  /**
   * M141: Set Chamber temperature
   */
  inline void gcode_M141() {
    if (DEBUGGING(DRYRUN)) return;
    if (parser.seenval('S')) thermalManager.setTargetChamber(parser.value_celsius());
  }
#endif

#if HAS_TEMP_COOLER
  /**
   * M142: Set Cooler temperature
   */
  inline void gcode_M142() {
    if (DEBUGGING(DRYRUN)) return;
    if (parser.seenval('S')) thermalManager.setTargetCooler(parser.value_celsius());
  }
#endif

#if ENABLED(ULTIPANEL) && HAS_TEMP_0

  /**
   * M145: Set the heatup state for a material in the LCD menu
   *   S<material> (0=PLA, 1=ABS, 2=GUM)
   *   H<hotend temp>
   *   B<bed temp>
   *   F<fan speed>
   */
  inline void gcode_M145() {
    uint8_t material = (uint8_t)parser.intval('S');
    if (material >= COUNT(lcd_preheat_hotend_temp)) {
      SERIAL_LM(ER, MSG_ERR_MATERIAL_INDEX);
    }
    else {
      int v;
      if (parser.seenval('H')) {
        v = parser.value_int();
        #if HEATER_0_MAXTEMP
          lcd_preheat_hotend_temp[material] = constrain(v, HEATER_0_MINTEMP, HEATER_0_MAXTEMP - 15);
        #endif
      }
      if (parser.seenval('F')) {
        v = parser.value_int();
        lcd_preheat_fan_speed[material] = constrain(v, 0, 255);
      }
      #if HAS_TEMP_BED
        if (parser.seenval('B')) {
          v = parser.value_int();
          lcd_preheat_bed_temp[material] = constrain(v, BED_MINTEMP, BED_MAXTEMP - 15);
        }
      #endif
    }
  }

#endif

#if ENABLED(TEMPERATURE_UNITS_SUPPORT)
  /**
   * M149: Set temperature units
   */
  inline void gcode_M149() {
         if (parser.seenval('C')) set_input_temp_units(TEMPUNIT_C);
    else if (parser.seenval('K')) set_input_temp_units(TEMPUNIT_K);
    else if (parser.seenval('F')) set_input_temp_units(TEMPUNIT_F);
  }
#endif

#if HAS_COLOR_LEDS

  /**
   * M150: Set Status LED Color - Use R-U-B-W for R-G-B-W
   *
   * Always sets all 3 or 4 components. If a component is left out, set to 0.
   *
   * Examples:
   *
   *   M150 R255       ; Turn LED red
   *   M150 R255 U127  ; Turn LED orange (PWM only)
   *   M150            ; Turn LED off
   *   M150 R U B      ; Turn LED white
   *   M150 W          ; Turn LED white using a white LED
   *
   */
  inline void gcode_M150() {
    printer.set_led_color(
      parser.byteval('R'),
      parser.byteval('U'),
      parser.byteval('B')
      #if ENABLED(RGBW_LED)
        , parser.byteval('W')
      #endif
    );
  }

#endif // HAS_COLOR_LEDS

#if ENABLED(AUTO_REPORT_TEMPERATURES) && (HAS_TEMP_HOTEND || HAS_TEMP_BED)

  /**
   * M155: Set temperature auto-report interval. M155 S<seconds>
   */
  inline void gcode_M155() {
    if (parser.seenval('S')) {
      thermalManager.auto_report_temp_interval = parser.value_byte();
      NOMORE(thermalManager.auto_report_temp_interval, 60);
      thermalManager.next_temp_report_ms = millis() + 1000UL * thermalManager.auto_report_temp_interval;
    }
  }

#endif // AUTO_REPORT_TEMPERATURES

#if ENABLED(COLOR_MIXING_EXTRUDER)
  /**
   * M163: Set a single mix factor for a mixing extruder
   *       This is called "weight" by some systems.
   *
   *   S[index]   The channel index to set
   *   P[float]   The mix value
   *
   */
  inline void gcode_M163() {
    int mix_index = parser.seen('S') ? parser.value_int() : 0;
    if (mix_index < MIXING_STEPPERS) {
      float mix_value = parser.seen('P') ? parser.value_float() : 0.0;
      NOLESS(mix_value, 0.0);
      mixing_factor[mix_index] = RECIPROCAL(mix_value);
    }
  }

  #if MIXING_VIRTUAL_TOOLS  > 1
    /**
     * M164: Store the current mix factors as a virtual tools.
     *
     *   S[index]   The virtual tools to store
     *
     */
    inline void gcode_M164() {
      int tool_index = parser.seen('S') ? parser.value_int() : 0;
      if (tool_index < MIXING_VIRTUAL_TOOLS) {
        normalize_mix();
        for (uint8_t i = 0; i < MIXING_STEPPERS; i++) {
          mixing_virtual_tool_mix[tool_index][i] = mixing_factor[i];
        }
      }
    }
  #endif

  /**
   * M165: Set multiple mix factors for a mixing extruder.
   *       Factors that are left out will be set to 0.
   *       All factors together must add up to 1.0.
   *
   *   A[factor] Mix factor for extruder stepper 1
   *   B[factor] Mix factor for extruder stepper 2
   *   C[factor] Mix factor for extruder stepper 3
   *   D[factor] Mix factor for extruder stepper 4
   *   H[factor] Mix factor for extruder stepper 5
   *   I[factor] Mix factor for extruder stepper 6
   *
   */
  inline void gcode_M165() { gcode_get_mix(); }
#endif  // COLOR_MIXING_EXTRUDER

#if HAS_TEMP_BED
  /**
   * M190: Sxxx Wait for bed current temp to reach target temp. Waits only when heating
   *       Rxxx Wait for bed current temp to reach target temp. Waits when heating and cooling
   */
  inline void gcode_M190() {
    if (DEBUGGING(DRYRUN)) return;

    LCD_MESSAGEPGM(MSG_BED_HEATING);
    const bool no_wait_for_cooling = parser.seen('S');
    if (no_wait_for_cooling || parser.seen('R'))
      thermalManager.setTargetBed(parser.value_celsius());

    printer.wait_bed(no_wait_for_cooling);
  }
#endif // HAS_TEMP_BED

#if HAS_TEMP_CHAMBER
  /**
   * M191: Sxxx Wait for chamber current temp to reach target temp. Waits only when heating
   *       Rxxx Wait for chamber current temp to reach target temp. Waits when heating and cooling
   */
  inline void gcode_M191() {
    if (DEBUGGING(DRYRUN)) return;

    LCD_MESSAGEPGM(MSG_CHAMBER_HEATING);
    bool no_wait_for_cooling = parser.seen('S');
    if (no_wait_for_cooling || parser.seen('R')) thermalManager.setTargetChamber(parser.value_celsius());

    printer.wait_chamber(no_wait_for_cooling);
  }
#endif // HAS_TEMP_CHAMBER

#if HAS_TEMP_COOLER
  /**
   * M192: Sxxx Wait for cooler current temp to reach target temp. Waits only when heating
   *       Rxxx Wait for cooler current temp to reach target temp. Waits when heating and cooling
   */
  inline void gcode_M192() {
    if (DEBUGGING(DRYRUN)) return;

    LCD_MESSAGEPGM(MSG_COOLER_COOLING);
    bool no_wait_for_heating = parser.seen('S');
    if (no_wait_for_heating || parser.seen('R')) thermalManager.setTargetCooler(parser.value_celsius());

    printer.wait_cooler(no_wait_for_heating);
  }
#endif

/**
 * M200: Set filament diameter and set E axis units to cubic units
 *
 *    T<extruder> - Optional extruder number. Current extruder if omitted.
 *    D<linear> - Diameter of the filament. Use "D0" to switch back to linear units on the E axis.
 */
inline void gcode_M200() {

  GET_TARGET_EXTRUDER(200);

  if (parser.seen('D')) {
    // setting any extruder filament size disables volumetric on the assumption that
    // slicers either generate in extruder values as cubic mm or as as filament feeds
    // for all extruders
    printer.volumetric_enabled = (parser.value_linear_units() != 0.0);
    if (printer.volumetric_enabled) {
      printer.filament_size[TARGET_EXTRUDER] = parser.value_linear_units();
      // make sure all extruders have some sane value for the filament size
      for (int i = 0; i < EXTRUDERS; i++)
        if (!printer.filament_size[i]) printer.filament_size[i] = DEFAULT_NOMINAL_FILAMENT_DIA;
    }
  }
  else {
    // reserved for setting filament diameter via UFID or filament measuring device
    return;
  }

  printer.calculate_volumetric_multipliers();
}

/**
 * M201: Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
 *
 *       With multiple extruders use T to specify which one.
 */
inline void gcode_M201() {

  GET_TARGET_EXTRUDER(201);

  LOOP_XYZE(i) {
    if (parser.seen(axis_codes[i])) {
      const uint8_t a = i + (i == E_AXIS ? TARGET_EXTRUDER : 0);
      mechanics.max_acceleration_mm_per_s2[a] = parser.value_axis_units((AxisEnum)a);
    }
  }
  // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
  mechanics.reset_acceleration_rates();
}

#if 0 // Not used for Sprinter/grbl gen6
  inline void gcode_M202() {
    LOOP_XYZE(i) {
      if(parser.seen(axis_codes[i])) axis_travel_steps_per_sqr_second[i] = parser.value_axis_units((AxisEnum)i) * mechanics.axis_steps_per_mm[i];
    }
  }
#endif

/**
 * M203: Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in units/sec
 *
 *       With multiple extruders use T to specify which one.
 */
inline void gcode_M203() {

  GET_TARGET_EXTRUDER(203);

  LOOP_XYZE(i) {
    if (parser.seen(axis_codes[i])) {
      const uint8_t a = i + (i == E_AXIS ? TARGET_EXTRUDER : 0);
      mechanics.max_feedrate_mm_s[a] = parser.value_axis_units((AxisEnum)a);
    }
  }
}

/**
 * M204: Set planner.accelerations in units/sec^2 (M204 P1200 T0 R3000 V3000)
 *
 *    P     = Printing moves
 *    T* R  = Retract only (no X, Y, Z) moves
 *    V     = Travel (non printing) moves
 *
 *  Also sets minimum segment time in ms (B20000) to prevent buffer under-runs and M20 minimum mechanics.feedrate_mm_s
 */
inline void gcode_M204() {

  GET_TARGET_EXTRUDER(204);

  if (parser.seen('S')) {  // Kept for legacy compatibility. Should NOT BE USED for new developments.
    mechanics.travel_acceleration = mechanics.acceleration = parser.value_linear_units();
    SERIAL_EMV("Setting Print and Travel acceleration: ", mechanics.acceleration );
  }
  if (parser.seen('P')) {
    mechanics.acceleration = parser.value_linear_units();
    SERIAL_EMV("Setting Print acceleration: ", mechanics.acceleration );
  }
  if (parser.seen('R')) {
    mechanics.retract_acceleration[TARGET_EXTRUDER] = parser.value_linear_units();
    SERIAL_EMV("Setting Retract acceleration: ", mechanics.retract_acceleration[TARGET_EXTRUDER]);
  }
  if (parser.seen('V')) {
    mechanics.travel_acceleration = parser.value_linear_units();
    SERIAL_EMV("Setting Travel acceleration: ", mechanics.travel_acceleration );
  }
}

/**
 * M205: Set Advanced Settings
 *
 *    S = Min Feed Rate (units/s)
 *    V = Min Travel Feed Rate (units/s)
 *    B = Min Segment Time (µs)
 *    X = Max X Jerk (units/sec^2)
 *    Y = Max Y Jerk (units/sec^2)
 *    Z = Max Z Jerk (units/sec^2)
 *    E = Max E Jerk (units/sec^2)
 */
inline void gcode_M205() {

  GET_TARGET_EXTRUDER(205);

  if (parser.seen('S')) mechanics.min_feedrate_mm_s = parser.value_linear_units();
  if (parser.seen('V')) mechanics.min_travel_feedrate_mm_s = parser.value_linear_units();
  if (parser.seen('B')) mechanics.min_segment_time = parser.value_millis();
  if (parser.seen('X')) mechanics.max_jerk[X_AXIS] = parser.value_linear_units();
  if (parser.seen('Y')) mechanics.max_jerk[Y_AXIS] = parser.value_linear_units();
  if (parser.seen('Z')) mechanics.max_jerk[Z_AXIS] = parser.value_linear_units();
  if (parser.seen('E')) mechanics.max_jerk[E_AXIS + TARGET_EXTRUDER] = parser.value_linear_units();
}

#if ENABLED(WORKSPACE_OFFSETS)

  /**
   * M206: Set Additional Homing Offset (X Y Z). SCARA aliases T=X, P=Y
   */
  inline void gcode_M206() {
    LOOP_XYZ(i) {
      if (parser.seen(axis_codes[i])) {
        set_home_offset((AxisEnum)i, parser.value_linear_units());
      }
    }
    #if MECH(MORGAN_SCARA)
      if (parser.seen('T')) set_home_offset(X_AXIS, parser.value_linear_units()); // Theta
      if (parser.seen('P')) set_home_offset(Y_AXIS, parser.value_linear_units()); // Psi
    #endif

    mechanics.sync_plan_position();
    mechanics.report_current_position();
  }

#endif // ENABLED(WORKSPACE_OFFSETS)

#if ENABLED(FWRETRACT)
  /**
   * M207: Set firmware retraction values
   *
   *   S[+units]    retract_length
   *   W[+units]    retract_length_swap (multi-extruder)
   *   F[units/min] retract_feedrate_mm_s
   *   Z[units]     retract_zlift
   */
  inline void gcode_M207() {
    if (parser.seenval('S')) retract_length = parser.value_axis_units(E_AXIS);
    if (parser.seenval('F')) retract_feedrate_mm_s = MMM_TO_MMS(parser.value_axis_units(E_AXIS));
    if (parser.seenval('Z')) retract_zlift = parser.value_linear_units();
    #if EXTRUDERS > 1
      if (parser.seenval('W')) retract_length_swap = parser.value_axis_units(E_AXIS);
    #endif
  }

  /**
   * M208: Set firmware un-retraction values
   *
   *   S[+units]    retract_recover_length (in addition to M207 S*)
   *   W[+units]    retract_recover_length_swap (multi-extruder)
   *   F[units/min] retract_recover_feedrate_mm_s
   */
  inline void gcode_M208() {
    if (parser.seenval('S')) retract_recover_length = parser.value_axis_units(E_AXIS);
    if (parser.seenval('F')) retract_recover_feedrate_mm_s = MMM_TO_MMS(parser.value_axis_units(E_AXIS));
    #if EXTRUDERS > 1
      if (parser.seenval('W')) retract_recover_length_swap = parser.value_axis_units(E_AXIS);
    #endif
  }

  /**
   * M209: Enable automatic retract (M209 S1)
   *   For slicers that don't support G10/11, reversed extrude-only
   *   moves will be classified as retraction.
   */
  inline void gcode_M209() {
    if (parser.seenval('S')) {
      autoretract_enabled = parser.value_bool();
      for (int i = 0; i < EXTRUDERS; i++) retracted[i] = false;
    }
  }
#endif // FWRETRACT

/**
 * M218 - set hotend offset (in linear units)
 *
 *   T<tool>
 *   X<xoffset>
 *   Y<yoffset>
 *   Z<zoffset>
 */
inline void gcode_M218() {

  GET_TARGET_HOTEND(218);
  if (TARGET_EXTRUDER == 0) return;

  if (parser.seenval('X')) printer.hotend_offset[X_AXIS][TARGET_EXTRUDER] = parser.value_linear_units();
  if (parser.seenval('Y')) printer.hotend_offset[Y_AXIS][TARGET_EXTRUDER] = parser.value_linear_units();
  if (parser.seenval('Z')) printer.hotend_offset[Z_AXIS][TARGET_EXTRUDER] = parser.value_linear_units();

  SERIAL_SM(ECHO, MSG_HOTEND_OFFSET);
  LOOP_HOTEND() {
    SERIAL_MV(" ", printer.hotend_offset[X_AXIS][h]);
    SERIAL_MV(",", printer.hotend_offset[Y_AXIS][h]);
    SERIAL_MV(",", printer.hotend_offset[Z_AXIS][h]);
  }
  SERIAL_EOL();
}

/**
 * M220: Set speed percentage factor, aka "Feed Rate" (M220 S95)
 */
inline void gcode_M220() {
  if (parser.seenval('S')) mechanics.feedrate_percentage = parser.value_int();
}

/**
 * M221: Set extrusion percentage (M221 T0 S95)
 */
inline void gcode_M221() {

  GET_TARGET_EXTRUDER(221);
  if (parser.seenval('S')) printer.flow_percentage[TARGET_EXTRUDER] = parser.value_int();
}

/**
 * M222: Set density extrusion percentage (M222 T0 S95)
 */
inline void gcode_M222() {

  GET_TARGET_EXTRUDER(222);

  if (parser.seenval('S')) {
    printer.density_percentage[TARGET_EXTRUDER] = parser.value_int();
    #if ENABLED(RFID_MODULE)
      RFID522.RfidData[TARGET_EXTRUDER].data.density = printer.density_percentage[TARGET_EXTRUDER];
    #endif
  }
}

/**
 * M226: Wait until the specified pin reaches the state required (M226 P<pin> S<state>)
 */
inline void gcode_M226() {
  if (parser.seenval('P')) {
    const int pin_number = parser.value_int(),
              pin_state = parser.intval('S', -1); // required pin state - default is inverted

    if (WITHIN(pin_state, -1, 1) && pin_number > -1 && !printer.pin_is_protected(pin_number)) {

      int target = LOW;

      stepper.synchronize();

      HAL::pinMode(pin_number, INPUT);
      switch(pin_state) {
        case 1:
          target = HIGH;
          break;
        case 0:
          target = LOW;
          break;
        case -1:
          target = !HAL::digitalRead(pin_number);
          break;
      }

      while (HAL::digitalRead(pin_number) != target) printer.idle();

    } // pin_state -1 0 1 && pin_number > -1
  } // parser.seen('P')
}

#if HAS_CHDK || HAS_PHOTOGRAPH
  /**
   * M240: Trigger a camera
   */
  inline void gcode_M240() {
    #if HAS_CHDK
       OUT_WRITE(CHDK_PIN, HIGH);
       chdkHigh = millis();
       chdkActive = true;
    #elif HAS_PHOTOGRAPH
      const uint8_t NUM_PULSES = 16;
      const float PULSE_LENGTH = 0.01524;
      for (int i = 0; i < NUM_PULSES; i++) {
        WRITE(PHOTOGRAPH_PIN, HIGH);
        HAL::delayMilliseconds(PULSE_LENGTH);
        WRITE(PHOTOGRAPH_PIN, LOW);
        HAL::delayMilliseconds(PULSE_LENGTH);
      }
      HAL::delayMilliseconds(7.33);
      for (int i = 0; i < NUM_PULSES; i++) {
        WRITE(PHOTOGRAPH_PIN, HIGH);
        HAL::delayMilliseconds(PULSE_LENGTH);
        WRITE(PHOTOGRAPH_PIN, LOW);
        HAL::delayMilliseconds(PULSE_LENGTH);
      }
    #endif // HASNT(CHDK) && HAS_PHOTOGRAPH
  }
#endif // HAS_CHDK || PHOTOGRAPH_PIN

#if HAS(LCD_CONTRAST)
  /**
   * M250: Read and optionally set the LCD contrast
   */
  inline void gcode_M250() {
    if (parser.seenval('C')) set_lcd_contrast(parser.value_int());
    SERIAL_EMV("lcd contrast value: ", lcd_contrast);
  }

#endif // DOGLCD

#if HAS_SERVOS
  /**
   * M280: Get or set servo position. P<index> S<angle>
   */
  inline void gcode_M280() {
    if (!parser.seen('P')) return;
    const int servo_index = parser.value_int();

    #if HAS_DONDOLO
      int servo_position = 0;
      if (parser.seenval('S')) {
        servo_position = parser.value_int();
        if (servo_index >= 0 && servo_index < NUM_SERVOS && servo_index != DONDOLO_SERVO_INDEX) {
          MOVE_SERVO(servo_index, servo_position);
        }
        else if (servo_index == DONDOLO_SERVO_INDEX) {
          Servo *srv = &servo[servo_index];
          srv->attach(0);
          srv->write(servo_position);
          #if (DONDOLO_SERVO_DELAY > 0)
            printer.safe_delay(DONDOLO_SERVO_DELAY);
            srv->detach();
          #endif
        }
        else {
          SERIAL_SMV(ER, "Servo ", servo_index);
          SERIAL_EM(" out of range");
        }
      }
    #else
      if (WITHIN(servo_index, 0, NUM_SERVOS - 1)) {
        if (parser.seenval('S'))
          MOVE_SERVO(servo_index, parser.value_int());
        else {
          SERIAL_SMV(ECHO, " Servo ", servo_index);
          SERIAL_EMV(": ", servo[servo_index].read());
        }
      }
      else {
        SERIAL_SMV(ER, "Servo ", servo_index);
        SERIAL_EM(" out of range");
      }
    #endif
  }
#endif // NUM_SERVOS > 0

#if HAS_BUZZER
  /**
   * M300: Play beep sound S<frequency Hz> P<duration ms>
   */
  inline void gcode_M300() {
    uint16_t const frequency = parser.seen('S') ? parser.value_ushort() : 260;
    uint16_t duration = parser.seen('P') ? parser.value_ushort() : 1000;

    // Limits the tone duration to 0-5 seconds.
    NOMORE(duration, 5000);

    BUZZ(duration, frequency);
  }
#endif // HAS_BUZZER

#if ENABLED(PIDTEMP)
  /**
   * M301: Set PID parameters P I D (and optionally C, L)
   *
   *   P[float] Kp term
   *   I[float] Ki term (unscaled)
   *   D[float] Kd term (unscaled)
   *
   * With PID_ADD_EXTRUSION_RATE:
   *
   *   C[float] Kc term
   *   L[float] LPQ length
   */
  inline void gcode_M301() {

    // multi-hotend PID patch: M301 updates or prints a single hotend's PID values
    // default behaviour (omitting E parameter) is to update for hotend 0 only
    int h = parser.seen('H') ? parser.value_int() : 0; // hotend being updated

    if (h < HOTENDS) { // catch bad input value
      if (parser.seen('P')) PID_PARAM(Kp, h) = parser.value_float();
      if (parser.seen('I')) PID_PARAM(Ki, h) = parser.value_float();
      if (parser.seen('D')) PID_PARAM(Kd, h) = parser.value_float();
      #if ENABLED(PID_ADD_EXTRUSION_RATE)
        if (parser.seen('C')) PID_PARAM(Kc, h) = parser.value_float();
        if (parser.seen('L')) lpq_len = parser.value_float();
        NOMORE(lpq_len, LPQ_MAX_LEN);
      #endif

      thermalManager.updatePID();
      SERIAL_SMV(ECHO, "H", h);
      SERIAL_MV(" P:", PID_PARAM(Kp, h));
      SERIAL_MV(" I:", PID_PARAM(Ki, h));
      SERIAL_MV(" D:", PID_PARAM(Kd, h));
      #if ENABLED(PID_ADD_EXTRUSION_RATE)
        SERIAL_MV(" C:", PID_PARAM(Kc, h));
      #endif
      SERIAL_EOL();
    }
    else {
      SERIAL_LM(ER, MSG_INVALID_EXTRUDER);
    }
  }
#endif // PIDTEMP

#if HAS_EXTRUDERS && ENABLED(PREVENT_COLD_EXTRUSION)
  /**
   * M302: Allow cold extrudes, or set the minimum extrude temperature
   *
   *       S<temperature> sets the minimum extrude temperature
   *       P<bool> enables (1) or disables (0) cold extrusion
   *
   *  Examples:
   *
   *       M302         ; report current cold extrusion state
   *       M302 P0      ; enable cold extrusion checking
   *       M302 P1      ; disables cold extrusion checking
   *       M302 S0      ; always allow extrusion (disables checking)
   *       M302 S170    ; only allow extrusion above 170
   *       M302 S170 P1 ; set min extrude temp to 170 but leave disabled
   */
  inline void gcode_M302() {
    bool seen_S = parser.seen('S');
    if (seen_S) {
      thermalManager.extrude_min_temp = parser.value_celsius();
      thermalManager.allow_cold_extrude = (thermalManager.extrude_min_temp == 0);
    }

    if (parser.seen('P'))
      thermalManager.allow_cold_extrude = (thermalManager.extrude_min_temp == 0) || parser.value_bool();
    else if (!seen_S) {
      // Report current state
      SERIAL_MV("Cold extrudes are ", (thermalManager.allow_cold_extrude ? "en" : "dis"));
      SERIAL_MV("abled (min temp ", thermalManager.extrude_min_temp);
      SERIAL_EM("C)");
    }
  }
#endif // PREVENT_COLD_EXTRUSION

/**
 * M303: PID relay autotune
 *
 *       S<temperature> sets the target temperature. (default target temperature = 150C)
 *       H<hotend> (-1 for the bed, -2 for chamber, -3 for cooler) (default 0)
 *       C<cycles>
 *       U<bool> with a non-zero value will apply the result to current settings
 */
inline void gcode_M303() {

  #if HAS_PID_HEATING || HAS_PID_COOLING
    const int   h = parser.seen('H') ? parser.value_int() : 0,
                c = parser.seen('C') ? parser.value_int() : 5;
    const bool  u = parser.seen('U') && parser.value_bool() != 0;

    int16_t temp = parser.seen('S') ? parser.value_celsius() : (h < 0 ? 70 : 200);

    if (WITHIN(h, 0, HOTENDS - 1)) printer.target_extruder = h;

    #if DISABLED(BUSY_WHILE_HEATING)
      KEEPALIVE_STATE(NOT_BUSY);
    #endif

    thermalManager.PID_autotune(temp, h, c, u);

    #if DISABLED(BUSY_WHILE_HEATING)
      KEEPALIVE_STATE(IN_HANDLER);
    #endif

  #else

    SERIAL_LM(ER, MSG_ERR_M303_DISABLED);

  #endif

}

#if ENABLED(PIDTEMPBED)

  // M304: Set bed PID parameters P I and D
  inline void gcode_M304() {
    if (parser.seen('P')) thermalManager.bedKp = parser.value_float();
    if (parser.seen('I')) thermalManager.bedKi = parser.value_float();
    if (parser.seen('D')) thermalManager.bedKd = parser.value_float();

    thermalManager.updatePID();
    SERIAL_SMV(ECHO, " p:", thermalManager.bedKp);
    SERIAL_MV(" i:", thermalManager.bedKi);
    SERIAL_EMV(" d:", thermalManager.bedKd);
  }

#endif // PIDTEMPBED

#if ENABLED(PIDTEMPCHAMBER)

  // M305: Set chamber PID parameters P I and D
  inline void gcode_M305() {
    if (parser.seen('P')) thermalManager.chamberKp = parser.value_float();
    if (parser.seen('I')) thermalManager.chamberKi = parser.value_float();
    if (parser.seen('D')) thermalManager.chamberKd = parser.value_float();

    thermalManager.updatePID();
    SERIAL_SMV(OK, " p:", thermalManager.chamberKp);
    SERIAL_MV(" i:", thermalManager.chamberKi);
    SERIAL_EMV(" d:", thermalManager.chamberKd);
  }

#endif // PIDTEMPCHAMBER

#if ENABLED(PIDTEMPCOOLER)

  // M306: Set cooler PID parameters P I and D
  inline void gcode_M306() {
    if (parser.seen('P')) thermalManager.coolerKp = parser.value_float();
    if (parser.seen('I')) thermalManager.coolerKi = parser.value_float();
    if (parser.seen('D')) thermalManager.coolerKd = parser.value_float();

    thermalManager.updatePID();
    SERIAL_SMV(OK, " p:", thermalManager.coolerKp);
    SERIAL_MV(" i:", thermalManager.coolerKi);
    SERIAL_EMV(" d:", thermalManager.coolerKd);
  }

#endif // PIDTEMPCOOLER

#if HAS_ABL

  /**
   * M320: Enable/Disable Bed Leveling and/or set the Z fade height.
   *
   *       S[bool]   Turns leveling on or off
   *       Z[height] Sets the Z fade height (0 or none to disable)
   *       V[bool]   Verbose - Print the leveling grid
   */
  inline void gcode_M320() {

    // V to print the matrix
    if (parser.seen('V')) {
      #if ABL_PLANAR
        bedlevel.bed_level_matrix.debug("Bed Level Correction Matrix:");
      #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
        if (bedlevel.leveling_is_valid()) {
          bedlevel.print_bilinear_leveling_grid();
          #if ENABLED(ABL_BILINEAR_SUBDIVISION)
            bedlevel.bed_level_virt_print();
          #endif
        }
      #endif
    }

    bool to_enable = false;
    if (parser.seen('S')) {
      to_enable = parser.value_bool();
      bedlevel.set_bed_leveling_enabled(to_enable);
    }

    #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
      if (parser.seen('Z')) {
        bedlevel.set_z_fade_height(parser.value_linear_units());
        SERIAL_LMV(ECHO, "ABL Fade Height = ", parser.value_linear_units(), 2);
      }
    #endif

    const bool new_status = bedlevel.leveling_is_active();
    if (to_enable && !new_status)
      SERIAL_LM(ER, MSG_ERR_M320_M420_FAILED);

    SERIAL_LMV(ECHO, "ABL: ", new_status ? MSG_ON : MSG_OFF);
  }

  #if ENABLED(AUTO_BED_LEVELING_BILINEAR)

    /**
     * M321: Set Level bilinear manual
     *
     * Usage:
     *   M321 I<xindex> J<yindex> Z<linear>
     *   M321 I<xindex> J<yindex> Q<offset>
     */
    inline void gcode_M321() {
      const bool hasI = parser.seen('I');
      const int8_t ix = hasI ? parser.value_int() : -1;
      const bool hasJ = parser.seen('J');
      const int8_t iy = hasJ ? parser.value_int() : -1;
      const bool hasZ = parser.seen('Z'), hasQ = !hasZ && parser.seen('Q');

      if (!hasI || !hasJ || !(hasZ || hasQ)) {
        SERIAL_LM(ER, MSG_ERR_M321_PARAMETERS);
      }
        else if (!WITHIN(ix, 0, GRID_MAX_POINTS_X - 1) || !WITHIN(iy, 0, GRID_MAX_POINTS_Y - 1)) {
        SERIAL_LM(ER, MSG_ERR_MESH_XY);
      }

      if (hasI && hasJ && !(hasZ || hasQ)) {
        SERIAL_MV("Level value in ix", ix);
        SERIAL_MV(" iy", iy);
        SERIAL_EMV(" Z", bedlevel.z_values[ix][iy]);
        return;
      }
      else {
        bedlevel.z_values[ix][iy] = parser.value_linear_units() + (hasQ ? bedlevel.z_values[ix][iy] : 0);
        #if ENABLED(ABL_BILINEAR_SUBDIVISION)
          bedlevel.bed_level_virt_interpolate();
        #endif
      }
    }

  #endif

  // M322: Reset auto leveling matrix
  inline void gcode_M322() {
    bedlevel.reset_bed_level();
    if (parser.seen('S') && parser.value_bool())
      eeprom.Store_Settings();
  }

#endif

#if HAS_MICROSTEPS

  // M350 Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
  inline void gcode_M350() {
    if (parser.seen('S')) for (int i = 0; i <= 4; i++) stepper.microstep_mode(i, parser.value_byte());
    LOOP_XYZE(i) if (parser.seen(axis_codes[i])) stepper.microstep_mode(i, parser.value_byte());
    if (parser.seen('B')) stepper.microstep_mode(4, parser.value_byte());
    stepper.microstep_readings();
  }

  /**
   * M351: Toggle MS1 MS2 pins directly with axis codes X Y Z E B
   *       S# determines MS1 or MS2, X# sets the pin high/low.
   */
  inline void gcode_M351() {
    if (parser.seen('S')) switch(parser.value_byte()) {
      case 1:
        LOOP_XYZE(i) if (parser.seen(axis_codes[i])) stepper.microstep_ms(i, parser.value_byte(), -1);
        if (parser.seen('B')) stepper.microstep_ms(4, parser.value_byte(), -1);
        break;
      case 2:
        LOOP_XYZE(i) if (parser.seen(axis_codes[i])) stepper.microstep_ms(i, -1, parser.value_byte());
        if (parser.seen('B')) stepper.microstep_ms(4, -1, parser.value_byte());
        break;
    }
    stepper.microstep_readings();
  }

#endif // HAS_MICROSTEPS

#if HAS_CASE_LIGHT

  int case_light_brightness;
  bool case_light_on;

  void update_case_light() {
    pinMode(CASE_LIGHT_PIN, OUTPUT);
    uint8_t case_light_bright = (uint8_t)case_light_brightness;
    if (case_light_on) {
      HAL::analogWrite(CASE_LIGHT_PIN, INVERT_CASE_LIGHT ? 255 - case_light_brightness : case_light_brightness );
      WRITE(CASE_LIGHT_PIN, INVERT_CASE_LIGHT ? LOW : HIGH);
    }
    else WRITE(CASE_LIGHT_PIN, INVERT_CASE_LIGHT ? HIGH : LOW);
  }

  /**
   * M355: Turn case light on/off and set brightness
   *
   *   P<byte>  Set case light brightness (PWM pin required - ignored otherwise)
   *
   *   S<bool>  Set case light on/off
   *
   *   When S turns on the light on a PWM pin then the current brightness level is used/restored
   *
   *   M355 P200 S0 turns off the light & sets the brightness level
   *   M355 S1 turns on the light with a brightness of 200 (assuming a PWM pin)
   */
  inline void gcode_M355() {
    uint8_t args = 0;
    if (parser.seen('P')) ++args, case_light_brightness = parser.value_byte();
    if (parser.seen('S')) ++args, case_light_on = parser.value_bool();
    if (args) update_case_light();

    // always report case light status
    SERIAL_STR(ECHO);
    if (!case_light_on)
      SERIAL_EM("Case light: off");
    else
      SERIAL_MV("Case light: ", case_light_brightness);
  }

#endif // HAS_CASE_LIGHT

#if MECH(MORGAN_SCARA)

  bool SCARA_move_to_cal(uint8_t delta_a, uint8_t delta_b) {
    if (IsRunning()) {
      forward_kinematics_SCARA(delta_a, delta_b);
      mechanics.destination[X_AXIS] = LOGICAL_X_POSITION(mechanics.cartesian_position[X_AXIS]);
      mechanics.destination[Y_AXIS] = LOGICAL_Y_POSITION(mechanics.cartesian_position[Y_AXIS]);
      mechanics.destination[Z_AXIS] = mechanics.current_position[Z_AXIS];
      mechanics.prepare_move_to_destination();
      return true;
    }
    return false;
  }

  /**
   * M360: SCARA calibration: Move to cal-position ThetaA (0 deg calibration)
   */
  inline bool gcode_M360() {
    SERIAL_LM(ECHO, " Cal: Theta 0");
    return SCARA_move_to_cal(0, 120);
  }

  /**
   * M361: SCARA calibration: Move to cal-position ThetaB (90 deg calibration - steps per degree)
   */
  inline bool gcode_M361() {
    SERIAL_LM(ECHO, " Cal: Theta 90");
    return SCARA_move_to_cal(90, 130);
  }

  /**
   * M362: SCARA calibration: Move to cal-position PsiA (0 deg calibration)
   */
  inline bool gcode_M362() {
    SERIAL_LM(ECHO, " Cal: Psi 0");
    return SCARA_move_to_cal(60, 180);
  }

  /**
   * M363: SCARA calibration: Move to cal-position PsiB (90 deg calibration - steps per degree)
   */
  inline bool gcode_M363() {
    SERIAL_LM(ECHO, " Cal: Psi 90");
    return SCARA_move_to_cal(50, 90);
  }

  /**
   * M364: SCARA calibration: Move to cal-position PsiC (90 deg to Theta calibration position)
   */
  inline bool gcode_M364() {
    SERIAL_LM(ECHO, " Cal: Theta-Psi 90");
    return SCARA_move_to_cal(45, 135);
  }

#endif // MORGAN_SCARA

#if ENABLED(EXT_SOLENOID)

  void enable_solenoid(uint8_t num) {
    switch(num) {
      case 0:
        OUT_WRITE(SOL0_PIN, HIGH);
        break;
        #if HAS(SOLENOID_1) && EXTRUDERS > 1
          case 1:
            OUT_WRITE(SOL1_PIN, HIGH);
            break;
        #endif
        #if HAS(SOLENOID_2) && EXTRUDERS > 2
          case 2:
            OUT_WRITE(SOL2_PIN, HIGH);
            break;
        #endif
        #if HAS(SOLENOID_3) && EXTRUDERS > 3
          case 3:
            OUT_WRITE(SOL3_PIN, HIGH);
            break;
        #endif
        #if HAS(SOLENOID_4) && EXTRUDERS > 4
          case 4:
            OUT_WRITE(SOL4_PIN, HIGH);
            break;
        #endif
      default:
        SERIAL_LM(ER, MSG_INVALID_SOLENOID);
        break;
    }
  }

  void enable_solenoid_on_active_extruder() { enable_solenoid(printer.active_extruder); }

  void disable_all_solenoids() {
    OUT_WRITE(SOL0_PIN, LOW);
    #if HAS(SOLENOID_1) && EXTRUDERS > 1
      OUT_WRITE(SOL1_PIN, LOW);
    #endif
    #if HAS(SOLENOID_2) && EXTRUDERS > 2
      OUT_WRITE(SOL2_PIN, LOW);
    #endif
    #if HAS(SOLENOID_3) && EXTRUDERS > 3
      OUT_WRITE(SOL3_PIN, LOW);
    #endif
    #if HAS(SOLENOID_4) && EXTRUDERS > 4
      OUT_WRITE(SOL4_PIN, LOW);
    #endif
  }

  /**
   * M380: Enable solenoid on the active extruder
   */
  inline void gcode_M380() { enable_solenoid_on_active_extruder(); }

  /**
   * M381: Disable all solenoids
   */
  inline void gcode_M381() { disable_all_solenoids(); }

#endif // EXT_SOLENOID

/**
 * M400: Finish all moves
 */
inline void gcode_M400() { stepper.synchronize(); }

#if HAS_BED_PROBE

  /**
   * M401: Engage Z Servo endstop if available
   */
  inline void gcode_M401() { probe.set_deployed(true); }

  /**
   * M402: Retract Z Servo endstop if enabled
   */
  inline void gcode_M402() { probe.set_deployed(false); }

#endif // (ENABLED(AUTO_BED_LEVELING_FEATURE) && DISABLED(Z_PROBE_SLED) && HAS_Z_SERVO_PROBE)

#if ENABLED(FILAMENT_SENSOR)

  /**
   * M404: Display or set (in current units) the nominal filament width (3mm, 1.75mm ) W<3.0>
   */
  inline void gcode_M404() {
    if (parser.seen('W')) {
      filament_width_nominal = parser.value_linear_units();
    }
    else {
      SERIAL_EMV("Filament dia (nominal mm):", filament_width_nominal);
    }
  }

  /**
   * M405: Turn on filament sensor for control
   */
  inline void gcode_M405() {
    // This is technically a linear measurement, but since it's quantized to centimeters and is a different unit than
    // everything else, it uses parser.value_int() instead of parser.value_linear_units().
    if (parser.seen('D')) meas_delay_cm = parser.value_byte();
    NOMORE(meas_delay_cm, MAX_MEASUREMENT_DELAY);

    if (filwidth_delay_index[1] == -1) { // Initialize the ring buffer if not done since startup
      const uint8_t temp_ratio = thermalManager.widthFil_to_size_ratio() - 100; // -100 to scale within a signed byte

      for (uint8_t i = 0; i < COUNT(measurement_delay); ++i)
        measurement_delay[i] = temp_ratio;

      filwidth_delay_index[0] = filwidth_delay_index[1] = 0;
    }

    filament_sensor = true;

    //SERIAL_MV("Filament dia (measured mm):", filament_width_meas);
    //SERIAL_EMV("Extrusion ratio(%):", printer.flow_percentage[printer.active_extruder]);
  }

  /**
   * M406: Turn off filament sensor for control
   */
  inline void gcode_M406() { filament_sensor = false; }

  /**
   * M407: Get measured filament diameter on serial output
   */
  inline void gcode_M407() {
    SERIAL_EMV("Filament dia (measured mm):", filament_width_meas);
  }

#endif // FILAMENT_SENSOR

#if ENABLED(JSON_OUTPUT)
  /**
   * M408: JSON STATUS OUTPUT
   */
  inline void gcode_M408() {
    bool firstOccurrence;
    uint8_t type = 0;

    if (parser.seen('S')) type = parser.value_byte();

    SERIAL_MSG("{\"status\":\"");
    #if HAS_SDSUPPORT
      if (!printer.print_job_counter.isRunning() && !card.sdprinting) SERIAL_CHR('I'); // IDLING
      else if (card.sdprinting) SERIAL_CHR('P');          // SD PRINTING
      else SERIAL_MSG("B");                               // SOMETHING ELSE, BUT SOMETHIG
    #else
      if (!printer.print_job_counter.isRunning()) SERIAL_CHR('I');                     // IDLING
      else SERIAL_CHR('B');                               // SOMETHING ELSE, BUT SOMETHIG
    #endif

    SERIAL_MSG("\",\"coords\": {");
    SERIAL_MSG("\"axesHomed\":[");
    if (mechanics.axis_homed[X_AXIS] && mechanics.axis_homed[Y_AXIS] && mechanics.axis_homed[Z_AXIS])
      SERIAL_MSG("1, 1, 1");
    else
      SERIAL_MSG("0, 0, 0");

    SERIAL_MV("],\"extr\":[", mechanics.current_position[E_AXIS]);
    SERIAL_MV("],\"xyz\":[", mechanics.current_position[X_AXIS]); // X AXIS
    SERIAL_MV(",", mechanics.current_position[Y_AXIS]);           // Y AXIS
    SERIAL_MV(",", mechanics.current_position[Z_AXIS]);           // Z AXIS

    SERIAL_MV("]},\"currentTool\":", printer.active_extruder);

    #if HAS_POWER_SWITCH
      SERIAL_MSG(",\"params\": {\"atxPower\":");
      SERIAL_CHR(powerManager.powersupply_on ? '1' : '0');
    #else
      SERIAL_MSG(",\"params\": {\"NormPower\":");
    #endif

    SERIAL_MSG(",\"fanPercent\":[");
    SERIAL_VAL(printer.fanSpeeds[0]);

    SERIAL_MV("],\"speedFactor\":", mechanics.feedrate_percentage);

    SERIAL_MSG(",\"extrFactors\":[");
    firstOccurrence = true;
    for (uint8_t i = 0; i < EXTRUDERS; i++) {
      if (!firstOccurrence) SERIAL_CHR(',');
      SERIAL_VAL(printer.flow_percentage[i]); // Really *100? 100 is normal
      firstOccurrence = false;
    }
    SERIAL_EM("]},");

    SERIAL_MSG("\"temps\": {");
    #if HAS_TEMP_BED
      SERIAL_MV("\"bed\": {\"current\":", thermalManager.degBed(), 1);
      SERIAL_MV(",\"active\":", thermalManager.degTargetBed());
      SERIAL_MSG(",\"state\":");
      SERIAL_CHR(thermalManager.degTargetBed() > 0 ? '2' : '1');
      SERIAL_MSG("},");
    #endif
    SERIAL_MSG("\"heads\": {\"current\":[");
    firstOccurrence = true;
    for (int8_t h = 0; h < HOTENDS; h++) {
      if (!firstOccurrence) SERIAL_CHR(',');
      SERIAL_VAL(thermalManager.degHotend(h), 1);
      firstOccurrence = false;
    }
    SERIAL_MSG("],\"active\":[");
    firstOccurrence = true;
    for (int8_t h = 0; h < HOTENDS; h++) {
      if (!firstOccurrence) SERIAL_CHR(',');
      SERIAL_VAL(thermalManager.degTargetHotend(h));
      firstOccurrence = false;
    }
    SERIAL_MSG("],\"state\":[");
    firstOccurrence = true;
    for (int8_t h = 0; h < HOTENDS; h++) {
      if (!firstOccurrence) SERIAL_CHR(',');
      SERIAL_CHR(thermalManager.degTargetHotend(h) > HOTEND_AUTO_FAN_TEMPERATURE ? '2' : '1');
      firstOccurrence = false;
    }

    SERIAL_MV("]}},\"time\":", HAL::timeInMilliseconds());

    switch (type) {
      case 0:
      case 1:
        break;
      case 2:
        SERIAL_EM(",");
        SERIAL_MSG("\"coldExtrudeTemp\":0,\"coldRetractTemp\":0.0,\"geometry\":\"");
        #if MECH(CARTESIAN)
          SERIAL_MSG("cartesian");
        #elif MECH(COREXY)
          SERIAL_MSG("corexy");
        #elif MECH(COREYX)
          SERIAL_MSG("coreyx");
        #elif MECH(COREXZ)
          SERIAL_MSG("corexz");
        #elif MECH(COREZX)
          SERIAL_MSG("corezx");
        #elif MECH(DELTA)
          SERIAL_MSG("delta");
        #endif
        SERIAL_MSG("\",\"name\":\"");
        SERIAL_MSG(CUSTOM_MACHINE_NAME);
        SERIAL_MSG("\",\"tools\":[");
        firstOccurrence = true;
        for (uint8_t i = 0; i < EXTRUDERS; i++) {
          if (!firstOccurrence) SERIAL_CHR(',');
          SERIAL_MV("{\"number\":", i + 1);
          #if HOTENDS > 1
            SERIAL_MV(",\"heaters\":[", i + 1);
            SERIAL_MSG("],");
          #else
            SERIAL_MSG(",\"heaters\":[1],");
          #endif
          #if DRIVER_EXTRUDERS > 1
            SERIAL_MV("\"drives\":[", i);
            SERIAL_MSG("]");
          #else
            SERIAL_MSG("\"drives\":[0]");
          #endif
          SERIAL_MSG("}");
          firstOccurrence = false;
        }
        break;
      case 3:
        SERIAL_EM(",");
        SERIAL_MSG("\"printer.currentLayer\":");
        #if HAS_SDSUPPORT
          if (card.sdprinting && card.layerHeight > 0) { // ONLY CAN TELL WHEN SD IS PRINTING
            SERIAL_VAL((int) (mechanics.current_position[Z_AXIS] / card.layerHeight));
          }
          else SERIAL_VAL(0);
        #else
          SERIAL_VAL(-1);
        #endif
        SERIAL_MSG(",\"extrRaw\":[");
        firstOccurrence = true;
        for (uint8_t i = 0; i < EXTRUDERS; i++) {
          if (!firstOccurrence) SERIAL_CHR(',');
          SERIAL_VAL(mechanics.current_position[E_AXIS] * printer.flow_percentage[i]);
          firstOccurrence = false;
        }
        SERIAL_MSG("],");
        #if HAS_SDSUPPORT
          if (card.sdprinting) {
            SERIAL_MSG("\"fractionPrinted\":");
            float fractionprinted;
            if (card.fileSize < 2000000) {
              fractionprinted = (float)card.sdpos / (float)card.fileSize;
            }
            else fractionprinted = (float)(card.sdpos >> 8) / (float)(card.fileSize >> 8);
            SERIAL_VAL((float) floorf(fractionprinted * 1000) / 1000);
            SERIAL_CHR(',');
          }
        #endif
        SERIAL_MSG("\"firstLayerHeight\":");
        #if HAS_SDSUPPORT
          if (card.sdprinting) SERIAL_VAL(card.firstlayerHeight);
          else SERIAL_MSG("0");
        #else
          SERIAL_MSG("0");
        #endif
        break;
      case 4:
      case 5:
        SERIAL_EM(",");
        SERIAL_MSG("\"axisMins\":[");
        SERIAL_VAL((int) X_MIN_POS);
        SERIAL_CHR(',');
        SERIAL_VAL((int) Y_MIN_POS);
        SERIAL_CHR(',');
        SERIAL_VAL((int) Z_MIN_POS);
        SERIAL_MSG("],\"axisMaxes\":[");
        SERIAL_VAL((int) X_MAX_POS);
        SERIAL_CHR(',');
        SERIAL_VAL((int) Y_MAX_POS);
        SERIAL_CHR(',');
        SERIAL_VAL((int) Z_MAX_POS);
        SERIAL_MSG("],\"planner.accelerations\":[");
        SERIAL_VAL(mechanics.max_acceleration_mm_per_s2[X_AXIS]);
        SERIAL_CHR(',');
        SERIAL_VAL(mechanics.max_acceleration_mm_per_s2[Y_AXIS]);
        SERIAL_CHR(',');
        SERIAL_VAL(mechanics.max_acceleration_mm_per_s2[Z_AXIS]);
        for (uint8_t i = 0; i < EXTRUDERS; i++) {
          SERIAL_CHR(',');
          SERIAL_VAL(mechanics.max_acceleration_mm_per_s2[E_AXIS + i]);
        }
        SERIAL_MSG("],");

        #if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
          SERIAL_MSG("\"currents\":[");
          SERIAL_VAL(printer.motor_current[X_AXIS]);
          SERIAL_CHR(',');
          SERIAL_VAL(printer.motor_current[Y_AXIS]);
          SERIAL_CHR(',');
          SERIAL_VAL(printer.motor_current[Z_AXIS]);
          for (uint8_t i = 0; i < DRIVER_EXTRUDERS; i++) {
            SERIAL_CHR(',');
            SERIAL_VAL(printer.motor_current[E_AXIS + i]);
          }
          SERIAL_EM("],");
        #endif

        SERIAL_MSG("\"firmwareElectronics\":\"");
        #if MB(RAMPS_13_HFB) || MB(RAMPS_13_HHB) || MB(RAMPS_13_HFF) || MB(RAMPS_13_HHF) || MB(RAMPS_13_HHH)
          SERIAL_MSG("RAMPS");
        #elif MB(ALLIGATOR)
          SERIAL_MSG("ALLIGATOR");
        #elif MB(ALLIGATOR_V3)
          SERIAL_MSG("ALLIGATOR_V3");
        #elif MB(RADDS) || MB(RAMPS_FD_V1) || MB(RAMPS_FD_V2) || MB(SMART_RAMPS) || MB(RAMPS4DUE)
          SERIAL_MSG("Arduino due");
        #elif MB(ULTRATRONICS)
          SERIAL_MSG("ULTRATRONICS");
        #else
          SERIAL_MSG("AVR");
        #endif
        SERIAL_MSG("\",\"firmwareName\":\"");
        SERIAL_MSG(FIRMWARE_NAME);
        SERIAL_MSG(",\"firmwareVersion\":\"");
        SERIAL_MSG(SHORT_BUILD_VERSION);
        SERIAL_MSG("\",\"firmwareDate\":\"");
        SERIAL_MSG(STRING_DISTRIBUTION_DATE);

        SERIAL_MSG("\",\"minFeedrates\":[0,0,0");
        for (uint8_t i = 0; i < EXTRUDERS; i++) {
          SERIAL_MSG(",0");
        }
        SERIAL_MSG("],\"maxFeedrates\":[");
        SERIAL_VAL(mechanics.max_feedrate_mm_s[X_AXIS]);
        SERIAL_CHR(',');
        SERIAL_VAL(mechanics.max_feedrate_mm_s[Y_AXIS]);
        SERIAL_CHR(',');
        SERIAL_VAL(mechanics.max_feedrate_mm_s[Z_AXIS]);
        for (uint8_t i = 0; i < EXTRUDERS; i++) {
          SERIAL_CHR(',');
          SERIAL_VAL(mechanics.max_feedrate_mm_s[E_AXIS + i]);
        }
        SERIAL_CHR(']');
        break;
    }
    SERIAL_CHR('}');
    SERIAL_EOL();
  }
#endif // JSON_OUTPUT

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
    if (mechanics.axis_unhomed_error()) home_all_axes();

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
