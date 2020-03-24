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

#define CODE_M42

/**
 * M42: Change pin status via GCode
 *
 *  P<pin>  Pin number (LED if omitted)
 *  S<byte> Pin status from 0 - 255
 *
 *  M<mode> Pin mode: 0=INPUT  1=OUTPUT  2=INPUT_PULLUP
 */
inline void gcode_M42() {

  const pin_t pin_number = parser.pinval('P', LED_PIN);
  if (pin_number == NoPin) return;

  if (printer.pin_is_protected(pin_number)) {
    SERIAL_LM(ER, MSG_HOST_ERR_PROTECTED_PIN);
    return;
  }

  if (parser.seenval('M')) {
    switch (parser.value_byte()) {
      case 0: pinMode(pin_number, INPUT);         break;
      case 1: pinMode(pin_number, OUTPUT);        break;
      case 2: pinMode(pin_number, INPUT_PULLUP);  break;
      default: SERIAL_EM("Invalid Pin Mode");
    }
    return;
  }

  if (!parser.seenval('S')) return;
  const byte pin_status = parser.value_byte();

  pinMode(pin_number, OUTPUT);
  digitalWrite(pin_number, pin_status);
  analogWrite(pin_number, pin_status);

}
