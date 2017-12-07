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

#if ENABLED(ZWOBBLE)

  #define CODE_M96
  #define CODE_M97

  /**
   * M96: Print ZWobble value
   */
  inline void gcode_M96(void) { mechanics.report_zwobble(); }

  /**
   * M97: Set ZWobble value
   */
  inline void gcode_M97(void) {
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
