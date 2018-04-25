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

#if HAS_TEMP_HOTEND

  #define CODE_M104

  /**
   * M104: Set hotend temperature
   */
  inline void gcode_M104(void) {

    GET_TARGET_EXTRUDER(104);

    if (printer.debugDryrun() || printer.debugSimulation()) return;

    #if ENABLED(SINGLENOZZLE)
      if (TARGET_EXTRUDER != tools.active_extruder) return;
    #endif

    if (parser.seenval('S')) {
      const int16_t temp = parser.value_celsius();
      heaters[TRG_EXTRUDER_IDX].setTarget(temp);

      #if ENABLED(DUAL_X_CARRIAGE)
        if (mechanics.dual_x_carriage_mode == DXC_DUPLICATION_MODE && TARGET_EXTRUDER == 0)
          heaters[1].setTarget(temp ? temp + mechanics.duplicate_hotend_temp_offset : 0);
      #endif

      if (temp > heaters[TRG_EXTRUDER_IDX].current_temperature) {
        #if HOTENDS > 1
          lcd_status_printf_P(0, PSTR("H%i " MSG_HEATING), TRG_EXTRUDER_IDX);
        #else
          LCD_MESSAGEPGM("H " MSG_HEATING);
        #endif
      }
    }

    #if ENABLED(AUTOTEMP)
      planner.autotemp_M104_M109();
    #endif

  }

#endif
