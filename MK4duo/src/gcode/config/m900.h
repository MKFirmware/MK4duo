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
 * mcode
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#if ENABLED(LIN_ADVANCE)

  #define CODE_M900

  /**
   * M900: Set and/or Get advance K factor and WH/D ratio
   *
   *  K<factor>                  Set advance K factor
   *  R<ratio>                   Set ratio directly (overrides WH/D)
   *  W<width> H<height> D<diam> Set ratio from WH/D
   */
  inline void gcode_M900(void) {
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

#endif // ENABLED(LIN_ADVANCE)
