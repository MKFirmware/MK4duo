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
 *
 */

/**
 * mcode
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#if ENABLED(PINS_DEBUGGING)

#include "../../../platform/common/pinsdebug.h"

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
        HAL::digitalWrite(pin, 0); HAL::delayMilliseconds(wait);
        HAL::digitalWrite(pin, 1); HAL::delayMilliseconds(wait);
        HAL::digitalWrite(pin, 0); HAL::delayMilliseconds(wait);
      }
    }
    SERIAL_EOL();
  }
  SERIAL_EM("Done.");

} // toggle_pins

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
 *  M43 S       - Servo probe test or BLTouch test
 *                  P<index> - Probe index (optional - defaults to 0)
 */
inline void gcode_M43() {

  // 'T' must be first. It uses 'S' and 'E' differently.
  if (parser.seen('T')) return toggle_pins();

  // 'E' Enable or disable endstop monitoring and return
  if (parser.seen('E')) {
    endstops.setMonitorEnabled(parser.value_bool());
    SERIAL_MSG("endstop monitor ");
    SERIAL_TXT(endstops.isMonitorEnabled() ? "en" : "dis");
    SERIAL_EM("abled");
    return;
  }

  // 'S' Run servo probe test and return
  if (parser.seen('S')) {
    #if HAS_BLTOUCH
      return bltouch.test();
    #else
      return probe.servo_test();
    #endif
  }

  // 'P' Get the range of pins to test or watch
  uint8_t first_pin = PARSED_PIN_INDEX('P', 0),
          last_pin = parser.seenval('P') ? first_pin : NUM_DIGITAL_PINS - 1;

  if (first_pin > last_pin) return;

  // 'I' to ignore protected pins
  const bool ignore_protection = parser.boolval('I');

  // 'W' Watch until click, M108, or reset
  if (parser.boolval('W')) {
    SERIAL_EM("Watching pins");
    #if ENABLED(ARDUINO_ARCH_SAM)
      NOLESS(first_pin, 2); // Don't hijack the UART pins
    #endif
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
      PRINTER_KEEPALIVE(PausedforUser);
      printer.setWaitForUser(true);
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
        if (!printer.isWaitForUser()) break;
      #endif

      HAL::delayMilliseconds(500);
    }
  }
  else {
    // Report current state of selected pin(s)
    for (uint8_t i = first_pin; i <= last_pin; i++) {
      pin_t pin = GET_PIN_MAP_PIN(i);
      if (VALID_PIN(pin)) report_pin_state_extended(pin, ignore_protection, true);
    }
  }
}

#else // DISABLED(PINS_DEBUGGING)

#define CODE_M43

/**
 * M43: Servo probe test/report
 *
 *  M43 S - Servo probe test or BLTouch test
 */
inline void gcode_M43() {
  if (parser.seen('S')) {
    #if HAS_BLTOUCH
      return bltouch.test();
    #else
      return probe.servo_test();
    #endif
  }
}

#endif // DISABLED(PINS_DEBUGGING)
