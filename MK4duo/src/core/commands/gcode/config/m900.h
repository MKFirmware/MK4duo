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

#if ENABLED(LIN_ADVANCE)

  #define CODE_M900

  /**
   * M900: Set Linear Advance K-factor
   *
   *  K<factor>   Set advance K factor
   */
  inline void gcode_M900(void) {
    if (parser.seenval('K')) {
      const float newK = parser.floatval('K');
      if (WITHIN(newK, 0, 10)) {
        stepper.synchronize();
        planner.extruder_advance_K = newK;
      }
      else
        SERIAL_EM("?K value out of range (0-10).");
    }
    else
      SERIAL_LMV(ECHO, "Advance K=", planner.extruder_advance_K);
  }

#endif // ENABLED(LIN_ADVANCE)
