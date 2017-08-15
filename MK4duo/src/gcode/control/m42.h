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

#define CODE_M42

/**
 * M42: Change pin status via GCode
 *
 *  P<pin>  Pin number (LED if omitted)
 *  S<byte> Pin status from 0 - 255
 */
inline void gcode_M42(void) {
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
    LOOP_FAN() {
      if (fans.pin[f] = pin_number) {
        fans.Speed[f] = pin_status;
      }
    }
  #endif
}
