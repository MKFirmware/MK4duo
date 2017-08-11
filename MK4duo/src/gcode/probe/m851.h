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

#if HAS_BED_PROBE

  #define CODE_M851

  inline void gcode_M851(void) {

    SERIAL_SM(ECHO, MSG_PROBE_OFFSET);

    probe.offset[X_AXIS] = parser.linearval('X', probe.offset[X_AXIS]);
    probe.offset[Y_AXIS] = parser.linearval('Y', probe.offset[Y_AXIS]);

    if (parser.seen('Z')) {
      const float value = parser.value_linear_units();
      if (WITHIN(value, Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX)) {
        probe.offset[Z_AXIS] = value;
        probe.refresh_offset();
      }
      else {
        SERIAL_MT(" " MSG_Z_MIN, Z_PROBE_OFFSET_RANGE_MIN);
        SERIAL_CHR(' ');
        SERIAL_MT(MSG_Z_MAX, Z_PROBE_OFFSET_RANGE_MAX);
      }
    }

    SERIAL_MV(" X:", probe.offset[X_AXIS], 3);
    SERIAL_MV(" Y:", probe.offset[Y_AXIS], 3);
    SERIAL_MV(" Z:", probe.offset[Z_AXIS], 3);
    SERIAL_EOL();
  }

#endif
