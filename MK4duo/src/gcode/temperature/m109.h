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

  #define CODE_M109

  /**
   * M109: Sxxx Wait for hotend(s) to reach temperature. Waits only when heating.
   *       Rxxx Wait for hotend(s) to reach temperature. Waits when heating and cooling.
   */
  inline void gcode_M109(void) {

    GET_TARGET_EXTRUDER(109);
    if (DEBUGGING(DRYRUN)) return;

    #if ENABLED(SINGLENOZZLE)
      if (TARGET_EXTRUDER != tools.active_extruder) return;
    #endif

    const bool no_wait_for_cooling = parser.seenval('S');
    if (no_wait_for_cooling || parser.seenval('R')) {
      const int16_t temp = parser.value_celsius();
      heaters[EXTRUDER_IDX].target_temperature = temp;

      #if ENABLED(DUAL_X_CARRIAGE)
        if (mechanics.dual_x_carriage_mode == DXC_DUPLICATION_MODE && TARGET_EXTRUDER == 0)
          heaters[1].target_temperature = (temp ? temp + mechanics.duplicate_hotend_temp_offset : 0);
      #endif

      if (heaters[EXTRUDER_IDX].isHeating())
        lcd_status_printf_P(0, PSTR("H%i %s"), TARGET_EXTRUDER, MSG_HEATING);
    }

    #if ENABLED(AUTOTEMP)
      planner.autotemp_M104_M109();
    #endif

    thermalManager.wait_heater(EXTRUDER_IDX, no_wait_for_cooling);
  }

#endif // HAS_TEMP_HOTEND
