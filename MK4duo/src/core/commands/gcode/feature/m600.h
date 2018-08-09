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

#if ENABLED(ADVANCED_PAUSE_FEATURE)

  #define CODE_M600

  /**
   * M600: Pause Park and filament change
   *
   *  E[distance] - Retract the filament this far
   *  Z[distance] - Move the Z axis by this distance
   *  X[position] - Move to this X position
   *  Y[position] - Move to this Y position
   *  U[distance] - Retract distance for removal (manual reload)
   *  L[distance] - Extrude distance for insertion (manual reload)
   *  S[temp]     - New temperature for new filament
   *  B[count]    - Number of times to beep, -1 for indefinite (if equipped with a buzzer)
   *  T[toolhead] - Select extruder for filament change
   *
   *  Default values are used for omitted arguments.
   *
   */
  inline void gcode_M600(void) {

    point_t park_point = NOZZLE_PARK_POINT;

    if (commands.get_target_tool(600)) return;

    // Show initial "wait for start" message
    #if HAS_LCD
      lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_INIT, ADVANCED_PAUSE_MODE_PAUSE_PRINT, TARGET_HOTEND);
    #endif

    #if ENABLED(HOME_BEFORE_FILAMENT_CHANGE)
      // Don't allow filament change without homing first
      if (mechanics.axis_unhomed_error()) mechanics.home();
    #endif

    #if EXTRUDERS > 1
      // Change toolhead if specified
      uint8_t active_extruder_before_filament_change = tools.active_extruder;
      if (tools.active_extruder != tools.target_extruder)
        tools.change(tools.target_extruder, 0, true);
    #endif

    // Initial retract before move to pause park position
    const float retract = -ABS(parser.seen('E') ? parser.value_axis_units(E_AXIS) : 0)
      #if ENABLED(PAUSE_PARK_RETRACT_LENGTH) && PAUSE_PARK_RETRACT_LENGTH > 0
        + (PAUSE_PARK_RETRACT_LENGTH)
      #endif
    ;

    // Lift Z axis
    if (parser.seenval('Z')) park_point.z = parser.linearval('Z');

    // Move XY axes to filament change position or given position
    if (parser.seenval('X')) park_point.x = parser.linearval('X');
    if (parser.seenval('Y')) park_point.y = parser.linearval('Y');

    #if HOTENDS > 1 && DISABLED(DUAL_X_CARRIAGE) && !MECH(DELTA)
      park_point.x += (tools.active_extruder ? tools.hotend_offset[X_AXIS][tools.active_extruder] : 0);
      park_point.y += (tools.active_extruder ? tools.hotend_offset[Y_AXIS][tools.active_extruder] : 0);
    #endif

    // Unload filament
    const float unload_length = -ABS(parser.seen('U') ? parser.value_axis_units(E_AXIS)
                                                       : filament_change_unload_length[tools.active_extruder]);

    // Slow load filament
    constexpr float slow_load_length = PAUSE_PARK_SLOW_LOAD_LENGTH;

    // Load filament
    const float fast_load_length = ABS(parser.seen('L') ? parser.value_axis_units(E_AXIS)
                                                          : filament_change_load_length[tools.active_extruder]);

    if (parser.seenval('S')) heaters[ACTIVE_HOTEND].setTarget(parser.value_celsius());

    const int beep_count = parser.intval('B',
      #if ENABLED(PAUSE_PARK_NUMBER_OF_ALERT_BEEPS)
        PAUSE_PARK_NUMBER_OF_ALERT_BEEPS
      #else
        -1
      #endif
    );

    const bool job_running = print_job_counter.isRunning();

    if (pause_print(retract, park_point, unload_length, true)) {
      wait_for_filament_reload(beep_count);
      resume_print(slow_load_length, fast_load_length, PAUSE_PARK_EXTRUDE_LENGTH, beep_count);
    }

    #if EXTRUDERS > 1
    // Restore toolhead if it was changed
      if (active_extruder_before_filament_change != tools.active_extruder)
        tools.change(active_extruder_before_filament_change, 0, true);
    #endif

    // Resume the print job timer if it was running
    if (job_running) print_job_counter.start();

  }

#endif // ENABLED(ADVANCED_PAUSE_FEATURE)
