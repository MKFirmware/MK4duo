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

#if HAS_TEMP_HOTEND

#define CODE_M109

/**
 * M109: Sxxx Wait for hotend(s) to reach temperature. Waits only when heating.
 *       Rxxx Wait for hotend(s) to reach temperature. Waits when heating and cooling.
 */
inline void gcode_M109() {

  if (commands.get_target_tool(109)) return;

  if (printer.debugDryrun() || printer.debugSimulation()) return;

  const bool no_wait_for_cooling = parser.seenval('S');
  if (no_wait_for_cooling || parser.seenval('R')) {
    const int16_t temp = parser.value_celsius();
    if (tempManager.heater.hotends == 1) {
      extruders[toolManager.extruder.target]->singlenozzle_temp = temp;
      if (toolManager.extruder.target != toolManager.extruder.active) return;
    }
    hotends[toolManager.target_hotend()]->set_target_temp(temp);

    #if ENABLED(DUAL_X_CARRIAGE)
      if (mechanics.dxc_is_duplicating() && toolManager.extruder.target == 0)
        hotends[1]->set_target_temp(temp ? temp + mechanics.duplicate_extruder_temp_offset : 0);
    #endif

    #if HAS_LCD
      if (hotends[toolManager.target_hotend()]->isHeating() || !no_wait_for_cooling)
        nozzle.set_heating_message();
    #endif

  }
  else return;

  #if ENABLED(AUTOTEMP)
    planner.autotemp_M104_M109();
  #endif

  hotends[toolManager.target_hotend()]->wait_for_target(no_wait_for_cooling);
}

#endif // HAS_TEMP_HOTEND
