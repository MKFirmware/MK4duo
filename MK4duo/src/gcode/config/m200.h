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

#if EXTRUDERS > 0

  #define CODE_M200

  /**
   * M200: Set filament diameter and set E axis units to cubic units
   *
   *    T<extruder> - Optional extruder number. Current extruder if omitted.
   *    D<linear> - Diameter of the filament. Use "D0" to switch back to linear units on the E axis.
   */
  inline void gcode_M200(void) {

    GET_TARGET_EXTRUDER(200);

    if (parser.seen('D')) {
      // setting any extruder filament size disables volumetric on the assumption that
      // slicers either generate in extruder values as cubic mm or as as filament feeds
      // for all extruders
      tools.volumetric_enabled = (parser.value_linear_units() != 0.0);
      if (tools.volumetric_enabled) {
        tools.filament_size[TARGET_EXTRUDER] = parser.value_linear_units();
        // make sure all extruders have some sane value for the filament size
        for (int e = 0; e < EXTRUDERS; e++)
          if (!tools.filament_size[e]) tools.filament_size[e] = DEFAULT_NOMINAL_FILAMENT_DIA;
      }
    }
    else {
      // reserved for setting filament diameter via UFID or filament measuring device
      return;
    }

    printer.calculate_volumetric_multipliers();
  }

#endif // EXTRUDERS > 0
