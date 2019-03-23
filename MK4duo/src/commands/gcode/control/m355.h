/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
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
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
 */

#if HAS_CASE_LIGHT

  #define CODE_M355

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
  inline void gcode_M355(void) {

    if (parser.seen('P')) caselight.brightness = parser.value_byte();

    if (parser.seen('S')) caselight.status = parser.value_bool();

    if (parser.seen('P') || parser.seen('S')) caselight.update();

    // Always report case light status
    caselight.report();
  }

#endif // HAS_CASE_LIGHT
