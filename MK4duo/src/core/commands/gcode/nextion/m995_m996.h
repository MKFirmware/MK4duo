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

#if ENABLED(NEXTION) && ENABLED(NEXTION_GFX)

  #define CODE_M995
  #define CODE_M996

  /**
   * M995: Nextion Origin
   */
  inline void gcode_M995(void) { gfx_origin(parser.linearval('X'), parser.linearval('Y'), parser.linearval('Z')); }

  /**
   * M996: Nextion Scale
   */
  inline void gcode_M996() { if (parser.seenval('S')) gfx_scale(parser.value_float()); }

#endif // ENABLED(NEXTION) && ENABLED(NEXTION_GFX)
