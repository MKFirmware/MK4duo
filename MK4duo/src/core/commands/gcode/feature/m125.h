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

#if ENABLED(PARK_HEAD_ON_PAUSE)

  #define CODE_M125

  /**
   * M125: Store current position and move to pause park position.
   *       Called on pause (by M25) to prevent material leaking onto the
   *       object. On resume (M24) the head will be moved back and the
   *       print will resume.
   *
   *       If MK4duo is compiled without SD Card support, M125 can be
   *       used directly to pause the print and move to park position,
   *       resuming with a button click or M108.
   *
   *    L = override retract length
   *    X = override X
   *    Y = override Y
   *    Z = override Z raise
   */
  inline void gcode_M125(void) {

    // Initial retract before move to pause park position
    const float retract = ABS(parser.seen('L') ? parser.value_axis_units(E_AXIS) : 0)
      #if ENABLED(PAUSE_PARK_RETRACT_LENGTH) && PAUSE_PARK_RETRACT_LENGTH > 0
        + (PAUSE_PARK_RETRACT_LENGTH)
      #endif
    ;

    point_t park_point = NOZZLE_PARK_POINT;

    // Move XY axes to filament change position or given position
    if (parser.seenval('X')) park_point.x = parser.linearval('X');
    if (parser.seenval('Y')) park_point.y = parser.linearval('Y');

    // Lift Z axis
    if (parser.seenval('Z')) park_point.z = parser.linearval('Z');

    #if HOTENDS > 1 && DISABLED(DUAL_X_CARRIAGE)
      park_point.x += (tools.active_extruder ? tools.hotend_offset[X_AXIS][tools.active_extruder] : 0);
      park_point.y += (tools.active_extruder ? tools.hotend_offset[Y_AXIS][tools.active_extruder] : 0);
    #endif

    #if DISABLED(SDSUPPORT)
      const bool job_running = print_job_counter.isRunning();
    #endif

    if (pause_print(retract, park_point)) {
      #if DISABLED(SDSUPPORT)
        // Wait for lcd click or M108
        wait_for_filament_reload();

        // Return to print position and continue
        resume_print();

        if (job_running) print_job_counter.start();
      #endif
    }
  }

#endif // ENABLED(PARK_HEAD_ON_PAUSE)
