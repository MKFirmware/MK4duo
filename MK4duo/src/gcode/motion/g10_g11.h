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
 * gcode.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#if ENABLED(FWRETRACT)

  #define CODE_G10
  #define CODE_G11

  /**
   * G10 - Retract filament according to settings of M207
   * G11 - Recover filament according to settings of M208
   */
  void gcode_G10_G11(bool doRetract = false) {
    #if EXTRUDERS > 1
      if (doRetract) {
        tools.retracted_swap[tools.active_extruder] = (parser.seen('S') && parser.value_bool()); // checks for swap retract argument
      }
    #endif
    printer.retract(doRetract
     #if EXTRUDERS > 1
      , tools.retracted_swap[tools.active_extruder]
     #endif
    );
  }

  inline void gcode_G10(void) { gcode_G10_G11(true); }
  inline void gcode_G11(void) { gcode_G10_G11(false); }

#endif // FWRETRACT
