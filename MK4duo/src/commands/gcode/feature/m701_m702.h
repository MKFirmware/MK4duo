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
inline void gcode_M701() {

  xyz_pos_t park_point = nozzle.data.park_point;

  // Only raise Z if the machine is homed
  if (mechanics.axis_need_homing()) park_point.z = 0;

  if (commands.get_target_tool(701)) return;

  // Z axis lift
  if (parser.seenval('Z')) park_point.z = parser.linearval('Z');

  // Show initial "wait for load" message
  #if HAS_LCD_MENU
    lcd_pause_show_message(PAUSE_MESSAGE_LOAD, PAUSE_MODE_LOAD_FILAMENT, toolManager.target_hotend());
  #endif

  #if DISABLED(PRUSA_MMU2)
    // Change toolhead if specified
    uint8_t active_extruder_before_filament_change = toolManager.extruder.active;
    if (toolManager.extruder.active != toolManager.extruder.target)
      toolManager.change(toolManager.extruder.target);
  #endif

  // Lift Z axis
  if (park_point.z > 0)
    mechanics.do_blocking_move_to_z(MIN(mechanics.position.z + park_point.z, Z_MAX_BED), NOZZLE_PARK_Z_FEEDRATE);

  // Load filament
  #if HAS_MMU2
    mmu2.load_filament_to_nozzle(toolManager.extruder.target);
  #else
    constexpr float     purge_length = PAUSE_PARK_PURGE_LENGTH,
                    slow_load_length = PAUSE_PARK_SLOW_LOAD_LENGTH;
    const float     fast_load_length = ABS(parser.seen('L') ? parser.value_axis_units(E_AXIS) :
                                                              extruders[toolManager.extruder.target]->data.load_length);

    advancedpause.load_filament(slow_load_length, fast_load_length, purge_length,
                                PAUSE_PARK_NUMBER_OF_ALERT_BEEPS,
                                true,
                                hotends[toolManager.target_hotend()]->wait_for_heating(),
                                PAUSE_MODE_LOAD_FILAMENT
                                #if ENABLED(DUAL_X_CARRIAGE)
                                  , toolManager.extruder.target
                                #endif
    );
  #endif

  // Restore Z axis
  if (park_point.z > 0)
    mechanics.do_blocking_move_to_z(MAX(mechanics.position.z - park_point.z, 0), NOZZLE_PARK_Z_FEEDRATE);

  #if DISABLED(PRUSA_MMU2)
    // Restore toolhead if it was changed
    if (active_extruder_before_filament_change != toolManager.extruder.active)
      toolManager.change(active_extruder_before_filament_change);
  #endif

  // Show status screen
  #if HAS_LCD_MENU
    lcd_pause_show_message(PAUSE_MESSAGE_STATUS);
  #endif
}

/**
 * M702: Unload filament
 *
 *  T[extruder] - Extruder number. Required for mixing extruder.
 *                For non-mixing, current extruder if omitted.
 *  Z[distance] - Move the Z axis by this distance
 *  U[distance] - Retract distance for removal (manual reload)
 *
 *  Default values are used for omitted arguments.
 */
inline void gcode_M702() {

  if (commands.get_target_tool(702)) return;

  xyz_pos_t park_point = nozzle.data.park_point;

  // Only raise Z if the machine is homed
  if (mechanics.axis_need_homing()) park_point.z = 0;

  // Z axis lift
  if (parser.seenval('Z')) park_point.z = parser.linearval('Z');

  // Show initial "wait for unload" message
  #if HAS_LCD_MENU
    lcd_pause_show_message(PAUSE_MESSAGE_UNLOAD, PAUSE_MODE_UNLOAD_FILAMENT, toolManager.target_hotend());
  #endif

  #if DISABLED(PRUSA_MMU2)
    // Change toolhead if specified
    uint8_t active_extruder_before_filament_change = toolManager.extruder.active;
    if (toolManager.extruder.active != toolManager.extruder.target)
      toolManager.change(toolManager.extruder.target);
  #endif

  // Lift Z axis
  if (park_point.z > 0)
    mechanics.do_blocking_move_to_z(MIN(mechanics.position.z + park_point.z, Z_MAX_BED), NOZZLE_PARK_Z_FEEDRATE);

  // Unload filament
  #if HAS_MMU2
    mmu2.unload();
  #else
    #if ENABLED(FILAMENT_UNLOAD_ALL_EXTRUDERS)
      if (!parser.seenval('T')) {
        LOOP_EXTRUDER() {
          if (e != toolManager.extruder.active) toolManager.change(e);
          advancedpause.unload_filament(filament_change_unload_length[e], true, PAUSE_MODE_UNLOAD_FILAMENT);
        }
      }
      else
    #endif
    {
      // Unload length
      const float unload_length = -ABS(parser.seen('U') ? parser.value_axis_units(E_AXIS)
                                                        : extruders[toolManager.extruder.target]->data.unload_length);

      advancedpause.unload_filament(unload_length, true, PAUSE_MODE_UNLOAD_FILAMENT);
    }
  #endif

  // Restore Z axis
  if (park_point.z > 0)
    mechanics.do_blocking_move_to_z(MAX(mechanics.position.z - park_point.z, 0), NOZZLE_PARK_Z_FEEDRATE);

  #if DISABLED(PRUSA_MMU2)
    // Restore toolhead if it was changed
    if (active_extruder_before_filament_change != toolManager.extruder.active)
      toolManager.change(active_extruder_before_filament_change);
  #endif

  // Show status screen
  #if HAS_LCD_MENU
    lcd_pause_show_message(PAUSE_MESSAGE_STATUS);
  #endif
}

#endif // ADVANCED_PAUSE_FEATURE
