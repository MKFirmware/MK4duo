/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 Alberto Cotronei @MagoKimbra
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
 * mcode
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#if ENABLED(PINS_DEBUGGING)

  #include "../../../../utility/pinsdebug.h"

  #define CODE_M43

  inline void toggle_pins() {
    const bool  I_flag  = parser.boolval('I');
    const int   repeat  = parser.intval('R', 1),
                start   = parser.byteval('S'),
                end     = parser.byteval('E', NUMBER_PINS_TOTAL - 1),
                wait    = parser.intval('W', 500);

    for (uint8_t i = start; i <= end; i++) {
      pin_t pin = GET_PIN_MAP_PIN(i);
      //report_pin_state_extended(pin, I_flag, false);
      if (!VALID_PIN(pin)) continue;
      if (!I_flag && printer.pin_is_protected(pin)) {
        report_pin_state_extended(pin, I_flag, true, "Untouched ");
        SERIAL_EOL();
      }
      else {
        report_pin_state_extended(pin, I_flag, true, "Pulsing   ");
        HAL::pinMode(pin, OUTPUT);
        for (int16_t j = 0; j < repeat; j++) {
          HAL::digitalWrite(pin, 0); printer.safe_delay(wait);
          HAL::digitalWrite(pin, 1); printer.safe_delay(wait);
          HAL::digitalWrite(pin, 0); printer.safe_delay(wait);
        }
      }
      SERIAL_EOL();
    }
    SERIAL_EM("Done.");

  } // toggle_pins

  inline void servo_probe_test(){
    #if !(NUM_SERVOS >= 1 && HAS_SERVO_0)

      SERIAL_LM(ER, "SERVO not setup");

    #elif !HAS_Z_SERVO_PROBE

      SERIAL_LM(ER, "Z_PROBE_SERVO_NR not setup");

    #else // HAS_Z_SERVO_PROBE

      const uint8_t probe_index = parser.seen('P') ? parser.value_byte() : Z_PROBE_SERVO_NR;

      SERIAL_EM("Servo probe test");
      SERIAL_EMV(".  Using index:  ", probe_index);
      SERIAL_EMV(".  Deploy angle: ", probe.z_servo_angle[0]);
      SERIAL_EMV(".  Stow angle:   ", probe.z_servo_angle[1]);

      bool probe_logic;

      #if HAS_Z_PROBE_PIN

        #define PROBE_TEST_PIN Z_PROBE_PIN

        SERIAL_EMV("Probe uses Z_MIN_PROBE_PIN: ", PROBE_TEST_PIN);
        SERIAL_EM(".  Uses Z_PROBE_ENDSTOP_LOGIC (ignores Z_MIN_ENDSTOP_LOGIC)");
        SERIAL_MSG(".  Z_PROBE_ENDSTOP_LOGIC: ");

        if (endstops.isLogic(Z_PROBE))
          SERIAL_EM("true");
        else
          SERIAL_EM("false");

        probe_logic = endstops.isLogic(Z_PROBE);

      #elif HAS_Z_MIN

        #define PROBE_TEST_PIN Z_MIN_PIN

        SERIAL_EMV("Probe uses Z_MIN pin: ", PROBE_TEST_PIN);
        SERIAL_EM(".  Uses Z_MIN_ENDSTOP_LOGIC (ignores Z_PROBE_ENDSTOP_LOGIC)");
        SERIAL_MSG(".  Z_MIN_ENDSTOP_LOGIC: ");

        if (endstops.isLogic(Z_MIN))
          SERIAL_EM("true");
        else
          SERIAL_EM("false");

        probe_logic = endstops.isLogic(Z_MIN);

      #endif

      SERIAL_EM("Deploy & stow 4 times");
      SET_INPUT_PULLUP(PROBE_TEST_PIN);
      uint8_t i = 0;
      bool deploy_state, stow_state;
      do {
        MOVE_SERVO(probe_index, probe.z_servo_angle[0]); //deploy
        printer.safe_delay(500);
        deploy_state = HAL::digitalRead(PROBE_TEST_PIN);
        MOVE_SERVO(probe_index, probe.z_servo_angle[1]); //stow
        printer.safe_delay(500);
        stow_state = HAL::digitalRead(PROBE_TEST_PIN);
      } while (++i < 4);
      if (probe_logic != deploy_state) SERIAL_EM("WARNING - INVERTING setting probably backwards");

      printer.move_watch.start();

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
      else {                                        // measure active signal length
        MOVE_SERVO(probe_index, probe.z_servo_angle[0]);  // deploy
        printer.safe_delay(500);
        SERIAL_EM("please trigger probe");
        uint16_t probe_counter = 0;

        // Allow 30 seconds max for operator to trigger probe
        for (uint16_t j = 0; j < 500 * 30 && probe_counter == 0 ; j++) {

          printer.safe_delay(2);

          if (0 == j % (500 * 1)) // keep cmd_timeout happy
            printer.move_watch.start();

          if (deploy_state != HAL::digitalRead(PROBE_TEST_PIN)) { // probe triggered

            for (probe_counter = 1; probe_counter < 50 && (deploy_state != HAL::digitalRead(PROBE_TEST_PIN)); probe_counter ++)
              printer.safe_delay(2);

            if (probe_counter == 50)
              SERIAL_EM("Z Servo Probe detected");   // >= 100mS active time
            else if (probe_counter >= 2 )
              SERIAL_EMV("BLTouch compatible probe detected - pulse width (+/- 4mS): ", probe_counter * 2 );   // allow 4 - 100mS pulse
            else
              SERIAL_EM("noise detected - please re-run test");   // less than 2mS pulse

            MOVE_SERVO(probe_index, probe.z_servo_angle[1]); //stow

          } // pulse detected

        } // for loop waiting for trigger

        if (probe_counter == 0) SERIAL_EM("trigger not detected");

      } // measure active signal length

    #endif // HAS_Z_SERVO_PROBE

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
  inline void gcode_M43(void) {

    if (parser.seen('T')) {   // must be first ot else it's "S" and "E" parameters will execute endstop or servo test
      toggle_pins();
      return;
    }

    // Enable or disable endstop monitoring
    if (parser.seen('E')) {
      endstops.setMonitorEnabled(parser.value_bool());
      SERIAL_MSG("endstop monitor ");
      SERIAL_TXT(endstops.isMonitorEnabled() ? "en" : "dis");
      SERIAL_EM("abled");
      return;
    }

    if (parser.seen('S')) {
      servo_probe_test();
      return;
    }

    // Get the range of pins to test or watch
    const uint8_t first_pin = PARSED_PIN_INDEX('P', 0),
                  last_pin = parser.seenval('P') ? first_pin : NUM_DIGITAL_PINS - 1;

    if (first_pin > last_pin) return;

    const bool ignore_protection = parser.boolval('I');

    // Watch until click, M108, or reset
    if (parser.boolval('W')) {
      SERIAL_EM("Watching pins");
      uint8_t pin_state[last_pin - first_pin + 1];
      for (uint8_t i = first_pin; i <= last_pin; i++) {
        pin_t pin = GET_PIN_MAP_PIN(i);
        if (!VALID_PIN(pin)) continue;
        if (printer.pin_is_protected(pin) && !ignore_protection) continue;
        HAL::pinMode(pin, INPUT_PULLUP);
        delay(1);
        // if (IS_ANALOG(pin))
        //   pin_state[pin - first_pin] = analogRead(pin - analogInputToDigitalPin(0)); // int16_t pin_state[...]
        // else
          pin_state[pin - first_pin] = HAL::digitalRead(pin);
      }

      #if HAS_RESUME_CONTINUE
        printer.setWaitForUser(true);
        printer.keepalive(PausedforUser);
      #endif

      for(;;) {
        for (uint8_t i = first_pin; i <= last_pin; i++) {
          pin_t pin = GET_PIN_MAP_PIN(i);
          if (!VALID_PIN(pin)) continue;
          if (printer.pin_is_protected(pin)) continue;
          byte val;
          // if (IS_ANALOG(pin))
          //   val = analogRead(pin - analogInputToDigitalPin(0)); // int16_t val
          // else
            val = HAL::digitalRead(pin);
          if (val != pin_state[pin - first_pin]) {
            report_pin_state_extended(pin, ignore_protection, false);
            pin_state[pin - first_pin] = val;
          }
        }

        #if HAS_RESUME_CONTINUE
          if (!printer.isWaitForUser()) {
            printer.keepalive(InHandler);
            break;
          }
        #endif

        printer.safe_delay(500);
      }
      return;
    }

    // Report current state of selected pin(s)
    for (uint8_t i = first_pin; i <= last_pin; i++) {
      pin_t pin = GET_PIN_MAP_PIN(i);
      if (VALID_PIN(pin)) report_pin_state_extended(pin, ignore_protection, true);
    }
  }

#endif // ENABLED(PINS_DEBUGGING)
