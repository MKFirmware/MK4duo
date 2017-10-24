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
 * gcode.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#if HAS_BED_PROBE

  #define CODE_G30

  /**
   * G30: Do a single Z probe
   * Usage:
   *    G30 <X#> <Y#> <S#> <Z#> <P#>
   *      X   Probe X position (default=current probe position)
   *      Y   Probe Y position (default=current probe position)
   *      E   Engage the probe for each probe
   *      Z   <bool> with a non-zero value will apply the result to current delta_height (ONLY DELTA)
   *      P   <bool> with a non-zero value will apply the result to current offset[Z_AXIS] (ONLY DELTA)
   */
  inline void gcode_G30(void) {

    const float xpos = parser.linearval('X', mechanics.current_position[X_AXIS] + probe.offset[X_AXIS]),
                ypos = parser.linearval('Y', mechanics.current_position[Y_AXIS] + probe.offset[Y_AXIS]);

    // Don't allow G30 without homing first
    if (mechanics.axis_unhomed_error()) return;

    if (!mechanics.position_is_reachable_by_probe_xy(xpos, ypos)) return;

    // Disable leveling so the planner won't mess with us
    #if HAS_LEVELING
      bedlevel.set_bed_leveling_enabled(false);
    #endif

    printer.setup_for_endstop_or_probe_move();

    const float measured_z = probe.check_pt(xpos, ypos, !parser.boolval('E'), 1);

    if (!isnan(measured_z)) {
      SERIAL_MV(MSG_BED_LEVELING_Z, FIXFLOAT(measured_z), 3);
      SERIAL_MV(MSG_BED_LEVELING_X, FIXFLOAT(xpos), 3);
      SERIAL_MV(MSG_BED_LEVELING_Y, FIXFLOAT(ypos), 3);
    }

    #if IS_DELTA
      if (parser.boolval('Z')) {
        mechanics.delta_height -= measured_z;
        mechanics.recalc_delta_settings();
        SERIAL_MV("  New delta height = ", mechanics.delta_height, 3);
      }
      else if (parser.boolval('P')) {
        probe.offset[Z_AXIS] += endstops.soft_endstop_min[Z_AXIS] - measured_z;
        SERIAL_MV("  New Z probe offset = ", probe.offset[Z_AXIS], 3);
      }
    #endif

    SERIAL_EOL();

    printer.clean_up_after_endstop_or_probe_move();

    mechanics.report_current_position();
  }

#endif // HAS_BED_PROBE
