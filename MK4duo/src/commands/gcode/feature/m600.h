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

  #define CODE_M600

  /**
   * M600: Pause for filament change
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
  inline void gcode_M600() {

    xyz_pos_t park_point = nozzle.data.park_point;

    if (commands.get_target_tool(600)) return;

    #if ENABLED(DUAL_X_CARRIAGE)
      int8_t DXC_ext = toolManager.extruder.target;
      if (!parser.seen('T')) {  // If no tool index is specified, M600 was (probably) sent in response to filament runout.
                                // In this case, for duplicating modes set DXC_ext to the extruder that ran out.
        #if HAS_FILAMENT_SENSOR && PIN_EXISTS(FIL_RUNOUT1)
          if (mechanics.dxc_is_duplicating())
            DXC_ext = (READ(FIL_RUNOUT_1_PIN) == endstops.isLogic(FIL_RUNOUT) ? 1 : 0;
        #else
          DXC_ext = toolManager.extruder.active;
        #endif
      }
    #endif

    // Show initial "wait for start" message
    #if HAS_LCD_MENU && DISABLED(PRUSA_MMU2)
      lcd_pause_show_message(PAUSE_MESSAGE_CHANGING, PAUSE_MODE_PAUSE_PRINT, toolManager.target_hotend());
    #endif

    #if ENABLED(HOME_BEFORE_FILAMENT_CHANGE)
      // Don't allow filament change without homing first
      if (mechanics.axis_unhomed_error()) mechanics.home();
    #endif

    #if MAX_EXTRUDER > 1
      // Change toolhead if specified
      uint8_t active_extruder_before_filament_change = toolManager.extruder.active;
      if (toolManager.extruder.active != toolManager.extruder.target
        #if ENABLED(DUAL_X_CARRIAGE)
          && mechanics.dual_x_carriage_mode != DXC_DUPLICATION_MODE && mechanics.dual_x_carriage_mode != DXC_MIRRORED_MODE
        #endif
      ) toolManager.change(toolManager.extruder.target);
    #endif

    // Initial retract before move to pause park position
    const float retract = -ABS(parser.seen('E') ? parser.value_axis_units(E_AXIS) : 0
      #if ENABLED(PAUSE_PARK_RETRACT_LENGTH) && PAUSE_PARK_RETRACT_LENGTH > 0
        + (PAUSE_PARK_RETRACT_LENGTH)
      #endif
    );

    // Lift Z axis
    if (parser.seenval('Z')) park_point.z = parser.linearval('Z');

    // Move XY axes to filament change position or given position
    if (parser.seenval('X')) park_point.x = parser.linearval('X');
    if (parser.seenval('Y')) park_point.y = parser.linearval('Y');

    #if DISABLED(DUAL_X_CARRIAGE) && NOMECH(DELTA)
      if (tempManager.heater.hotends > 1) park_point += nozzle.data.hotend_offset[toolManager.active_hotend()];
    #endif

    #if HAS_MMU2
      // For MMU2 reset retract and load/unload values so they don't mess with MMU filament handling
      constexpr float unload_length     = 0.5f,
                      slow_load_length  = 0.0f,
                      fast_load_length  = 0.0f;
    #else
      // Unload filament
      const float unload_length = -ABS(parser.seen('U') ? parser.value_axis_units(E_AXIS)
                                                        : extruders[toolManager.extruder.active]->data.unload_length);

      // Slow load filament
      constexpr float slow_load_length = PAUSE_PARK_SLOW_LOAD_LENGTH;

      // Load filament
      const float fast_load_length = ABS(parser.seen('L') ? parser.value_axis_units(E_AXIS)
                                                          : extruders[toolManager.extruder.active]->data.load_length);
    #endif

    if (parser.seenval('S')) hotends[toolManager.active_hotend()]->set_target_temp(parser.value_celsius());

    const int beep_count = parser.intval('B',
      #if ENABLED(PAUSE_PARK_NUMBER_OF_ALERT_BEEPS)
        PAUSE_PARK_NUMBER_OF_ALERT_BEEPS
      #else
        -1
      #endif
    );

    if (advancedpause.pause_print(retract, park_point, unload_length, true DXC_PASS)) {
      #if HAS_MMU2
        mmu2_M600();
        advancedpause.resume_print(slow_load_length, fast_load_length, 0, beep_count DXC_PASS);
      #else
        advancedpause.wait_for_confirmation(true, beep_count DXC_PASS);
        advancedpause.resume_print(slow_load_length, fast_load_length, PAUSE_PARK_PURGE_LENGTH, beep_count DXC_PASS);
      #endif
    }

    #if MAX_EXTRUDER > 1
    // Restore toolhead if it was changed
      if (active_extruder_before_filament_change != toolManager.extruder.active)
        toolManager.change(active_extruder_before_filament_change);
    #endif

  }

#endif // ENABLED(ADVANCED_PAUSE_FEATURE)
