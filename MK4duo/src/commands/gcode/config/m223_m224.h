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

#if HAS_FIL_RUNOUT_0

  #define CODE_M223
  #define CODE_M224

  /**
   * M223: Set Filrunout Logic
   *
   *  T<tools>  - Set Extruder
   *  S<bool>   - Set false or true
   *
   */
  inline void gcode_M223() {
    if (commands.get_target_tool(223)) return;
    filamentrunout.sensor.setLogic((FilRunoutEnum)toolManager.extruder.target, parser.value_bool());
    filamentrunout.sensor.report();
  }

  /**
   * M224: Set Filrunout Pullup
   *
   *  T<tools>  - Set Extruder
   *  S<bool>   - Set false or true
   *
   */
  inline void gcode_M224() {
    if (commands.get_target_tool(224)) return;
    filamentrunout.sensor.setPullup((FilRunoutEnum)toolManager.extruder.target, parser.value_bool());
    filamentrunout.sensor.setup_pullup();
    filamentrunout.sensor.report();
  }

#endif // HAS_FIL_RUNOUT_0
