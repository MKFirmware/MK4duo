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

#if ENABLED(ADVANCED_PAUSE_FEATURE)

  #define CODE_M603

  /**
   * M603: Configure filament change
   *
   *  T[toolhead] - Select extruder to configure, active extruder if not specified
   *  U[distance] - Retract distance for removal, for the specified extruder
   *  L[distance] - Extrude distance for insertion, for the specified extruder
   *
   */
  inline void gcode_M603() {

    if (commands.get_target_tool(603)) return;

    // Unload length
    if (parser.seen('U')) {
      extruders[toolManager.extruder.target]->data.unload_length = ABS(parser.value_axis_units(E_AXIS));
      #if ENABLED(PREVENT_LENGTHY_EXTRUDE)
        NOMORE(extruders[toolManager.extruder.target]->data.unload_length, EXTRUDE_MAXLENGTH);
      #endif
    }

    // Load length
    if (parser.seen('L')) {
      extruders[toolManager.extruder.target]->data.load_length = ABS(parser.value_axis_units(E_AXIS));
      #if ENABLED(PREVENT_LENGTHY_EXTRUDE)
        NOMORE(extruders[toolManager.extruder.target]->data.load_length, EXTRUDE_MAXLENGTH);
      #endif
    }
  }

#endif // ADVANCED_PAUSE_FEATURE