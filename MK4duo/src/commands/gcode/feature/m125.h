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

#if ENABLED(PARK_HEAD_ON_PAUSE)

#define CODE_M125

/**
 * M125: Store current position and move to parking position.
 *       Called on pause (by M25) to prevent material leaking onto the
 *       object. On resume (M24) the head will be moved back and the
 *       print will resume.
 *
 *       When not actively SD printing, M125 simply moves to the park
 *       position and waits, resuming with a button click or M108.
 *       Without PARK_HEAD_ON_PAUSE the M125 command does nothing.
 *
 *    L = override retract length
 *    X = override X
 *    Y = override Y
 *    Z = override Z raise
 */
inline void gcode_M125() {

  // Initial retract before move to pause park position
  const float retract = -ABS(parser.seen('L') ? parser.value_axis_units(E_AXIS) : 0
    #if ENABLED(PAUSE_PARK_RETRACT_LENGTH) && PAUSE_PARK_RETRACT_LENGTH > 0
      + (PAUSE_PARK_RETRACT_LENGTH)
    #endif
  );

  xyz_pos_t park_point = nozzle.data.park_point;

  // Move XY axes to filament change position or given position
  if (parser.seenval('X')) park_point.x = NATIVE_X_POSITION(parser.linearval('X'));
  if (parser.seenval('Y')) park_point.y = NATIVE_Y_POSITION(parser.linearval('Y'));

  // Lift Z axis
  if (parser.seenval('Z')) park_point.z = parser.linearval('Z');

  #if DISABLED(DUAL_X_CARRIAGE) && NOMECH(DELTA)
    if (tempManager.heater.hotends > 1) park_point += nozzle.data.hotend_offset[toolManager.active_hotend()];
  #endif

  #if HAS_LCD_MENU
    lcd_pause_show_message(PAUSE_MESSAGE_PAUSING, PAUSE_MODE_PAUSE_PRINT);
    const bool show_lcd = parser.seenval('P');
  #else
    constexpr bool show_lcd = false;
  #endif

  if (advancedpause.pause_print(retract, park_point, 0, show_lcd)) {
    #if HAS_SD_RESTART
      if (restart.enabled && IS_SD_PRINTING()) restart.save_job();
    #endif
    if (!IS_SD_PRINTING() || show_lcd ) {
      advancedpause.wait_for_confirmation(false, 0);
      advancedpause.resume_print(0, 0, PAUSE_PARK_RETRACT_LENGTH, 0);
    }
  }
}

#endif // ENABLED(PARK_HEAD_ON_PAUSE)
