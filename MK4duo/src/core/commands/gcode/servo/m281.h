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

#if HAS_SERVOS

  #define CODE_M281

  /**
   * M281: Set servo min|max position
   *  P<index>
   *  L<min>
   *  U<max>
   */
  inline void gcode_M281(void) {

    if (!parser.seen('P')) return;
    const int servo_index = parser.value_int();

    if (WITHIN(servo_index, 0, NUM_SERVOS - 1)) {
      if (parser.seen('L')) servo[servo_index].angle[0] = parser.value_int();
      if (parser.seen('U')) servo[servo_index].angle[1] = parser.value_int();
      servo[servo_index].print_parameters();
    }
    else {
      SERIAL_SMV(ER, "Servo ", servo_index);
      SERIAL_EM(" out of range");
    }

  }

#endif // NUM_SERVOS > 0
