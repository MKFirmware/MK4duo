/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
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
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
 */

#if HAS_TRINAMIC

#define CODE_M922

/**
 * M922: Debug TMC drivers
 */
inline void gcode_M922(void) {

  bool print_axis[XYZE] = { false, false, false, false },
       print_all = true;

  LOOP_XYZE(axis) if (parser.seen(axis_codes[axis])) { print_axis[axis] = true; print_all = false; }

  if (print_all) LOOP_XYZE(axis) print_axis[axis] = true;

  #if ENABLED(TMC_DEBUG)
    #if ENABLED(MONITOR_DRIVER_STATUS)
      const bool sflag = parser.seen('S'), s0 = sflag && !parser.value_bool();
      if (sflag) tmc.set_report_interval(s0 ? 0 : MONITOR_DRIVER_STATUS_INTERVAL_MS);
      if (!s0 && parser.seenval('P')) tmc.set_report_interval(MIN(parser.value_ushort(), MONITOR_DRIVER_STATUS_INTERVAL_MS));
    #endif

    if (parser.seen('V'))
      tmc.get_registers(print_axis[X_AXIS], print_axis[Y_AXIS], print_axis[Z_AXIS], print_axis[E_AXIS]);
    else
      tmc.report_all(print_axis[X_AXIS], print_axis[Y_AXIS], print_axis[Z_AXIS], print_axis[E_AXIS]);
  #endif

  tmc.test_connection(print_axis[X_AXIS], print_axis[Y_AXIS], print_axis[Z_AXIS], print_axis[E_AXIS]);

}

#endif // HAS_TRINAMIC
