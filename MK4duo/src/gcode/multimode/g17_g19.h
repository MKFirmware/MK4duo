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

#if ENABLED(CNC_WORKSPACE_PLANES)

  #define CODE_G17
  #define CODE_G18
  #define CODE_G19

  void report_workspace_plane() {
    SERIAL_SM(ECHO, "Workspace Plane ");
    SERIAL_PS(mechanics.workspace_plane == PLANE_YZ ? PSTR("YZ\n") : mechanics.workspace_plane == PLANE_ZX ? PSTR("ZX\n") : PSTR("XY\n"));
  }

  /**
   * G17: Select Plane XY
   * G18: Select Plane ZX
   * G19: Select Plane YZ
   */
  inline void gcode_G17(void) { mechanics.workspace_plane = PLANE_XY; }
  inline void gcode_G18(void) { mechanics.workspace_plane = PLANE_ZX; }
  inline void gcode_G19(void) { mechanics.workspace_plane = PLANE_YZ; }

#endif // CNC_WORKSPACE_PLANES
