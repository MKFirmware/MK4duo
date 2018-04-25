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

#if ENABLED(FILAMENT_LOAD_UNLOAD_GCODES)

  #define CODE_M701
  #define CODE_M702
  
  /**
   * M701: Load filament
   *
   *  T[toolhead] - Optional toolhead number. Current toolhead if omitted.
   *  Z[distance] - Move the Z axis by this distance
   *  L[distance] - Extrude distance for insertion (positive value) (manual reload)
   *
   *  Default values are used for omitted arguments.
   */
  inline void gcode_M701(void) {

    point_t park_point = NOZZLE_PARK_POINT;

    // Only raise Z if the machine is homed
    if (mechanics.axis_unhomed_error()) park_point.z = 0;

    if (commands.get_target_tool(701)) return;

    // Z axis lift
    if (parser.seenval('Z')) park_point.z = parser.linearval('Z');

    // Load filament
    const float load_length = FABS(parser.seen('L') ? parser.value_axis_units(E_AXIS) :
                                                      filament_change_load_length[tools.target_extruder]);

    // Show initial "wait for load" message
    #if HAS_LCD
      lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_LOAD, ADVANCED_PAUSE_MODE_LOAD_FILAMENT, tools.target_extruder);
    #endif

    #if EXTRUDERS > 1
      // Change toolhead if specified
      uint8_t active_extruder_before_filament_change = tools.active_extruder;
      if (tools.active_extruder != tools.target_extruder)
        tools.change(tools.target_extruder, 0, true);
    #endif

    // Lift Z axis
    if (park_point.z > 0)
      mechanics.do_blocking_move_to_z(min(mechanics.current_position[Z_AXIS] + park_point.z, Z_MAX_POS), NOZZLE_PARK_Z_FEEDRATE);

    load_filament(load_length, PAUSE_PARK_EXTRUDE_LENGTH, PAUSE_PARK_NUMBER_OF_ALERT_BEEPS, true,
                  heaters[TARGET_EXTRUDER].wait_for_heating(), ADVANCED_PAUSE_MODE_LOAD_FILAMENT);

    // Restore Z axis
    if (park_point.z > 0)
      mechanics.do_blocking_move_to_z(max(mechanics.current_position[Z_AXIS] - park_point.z, 0), NOZZLE_PARK_Z_FEEDRATE);

    #if EXTRUDERS > 1
      // Restore toolhead if it was changed
      if (active_extruder_before_filament_change != tools.active_extruder)
        tools.change(active_extruder_before_filament_change, 0, true);
    #endif

    // Show status screen
    #if HAS_LCD
      lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_STATUS);
    #endif
  }

  /**
   * M702: Unload filament
   *
   *  T[toolhead] - Optional toolhead number. If omitted, current toolhead
   *                (or ALL extruders with FILAMENT_UNLOAD_ALL_EXTRUDERS).
   *  Z[distance] - Move the Z axis by this distance
   *  U[distance] - Retract distance for removal (manual reload)
   *
   *  Default values are used for omitted arguments.
   */
  inline void gcode_M702(void) {

    point_t park_point = NOZZLE_PARK_POINT;

    // Only raise Z if the machine is homed
    if (mechanics.axis_unhomed_error()) park_point.z = 0;

    if (commands.get_target_tool(702)) return;

    // Z axis lift
    if (parser.seenval('Z')) park_point.z = parser.linearval('Z');

    // Show initial "wait for unload" message
    #if HAS_LCD
      lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_UNLOAD, ADVANCED_PAUSE_MODE_UNLOAD_FILAMENT, tools.target_extruder);
    #endif

    #if EXTRUDERS > 1
      // Change toolhead if specified
      uint8_t active_extruder_before_filament_change = tools.active_extruder;
      if (tools.active_extruder != tools.target_extruder)
        tools.change(tools.target_extruder, 0, true);
    #endif

    // Lift Z axis
    if (park_point.z > 0)
      mechanics.do_blocking_move_to_z(min(mechanics.current_position[Z_AXIS] + park_point.z, Z_MAX_POS), NOZZLE_PARK_Z_FEEDRATE);

    // Unload filament
    #if EXTRUDERS > 1 && ENABLED(FILAMENT_UNLOAD_ALL_EXTRUDERS)
      if (!parser.seenval('T')) {
        HOTEND_LOOP() {
          if (h != tools.active_extruder) tools.change(h, 0, true);
          unload_filament(-filament_change_unload_length[h], true, ADVANCED_PAUSE_MODE_UNLOAD_FILAMENT);
        }
      }
      else
    #endif
    {
      // Unload length
      const float unload_length = -FABS(parser.seen('U') ? parser.value_axis_units(E_AXIS) :
                                                          filament_change_unload_length[tools.target_extruder]);

      unload_filament(unload_length, true, ADVANCED_PAUSE_MODE_UNLOAD_FILAMENT);
    }

    // Restore Z axis
    if (park_point.z > 0)
      mechanics.do_blocking_move_to_z(max(mechanics.current_position[Z_AXIS] - park_point.z, 0), NOZZLE_PARK_Z_FEEDRATE);

    #if EXTRUDERS > 1
      // Restore toolhead if it was changed
      if (active_extruder_before_filament_change != tools.active_extruder)
        tools.change(active_extruder_before_filament_change, 0, true);
    #endif

    // Show status screen
    #if HAS_LCD
      lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_STATUS);
    #endif
  }

#endif // ADVANCED_PAUSE_FEATURE
