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

#define CODE_M851

inline void gcode_M851() {

  #if DISABLED(DISABLE_M503)
    // No arguments? Show M851 report.
    if (parser.seen_any()) {
      probe.print_M851();
      return;
    }
  #endif

  probe.data.offset.x = parser.linearval('X', probe.data.offset.x);
  probe.data.offset.y = parser.linearval('Y', probe.data.offset.y);

  if (parser.seen('Z')) {
    const float value = parser.value_linear_units();
    if (WITHIN(value, Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX)) {
      probe.data.offset.z = value;
    }
    else {
      SERIAL_LM(ER, "?Z out of range (" STRINGIFY(Z_PROBE_OFFSET_RANGE_MIN) " to " STRINGIFY(Z_PROBE_OFFSET_RANGE_MAX));
      return;
    }
  }

  probe.data.speed_fast   = parser.ushortval('F', probe.data.speed_fast);
  probe.data.speed_slow   = parser.ushortval('S', probe.data.speed_slow);
  probe.data.repetitions  = parser.byteval('R', probe.data.repetitions);

  NOLESS(probe.data.speed_fast, 120);
  NOLESS(probe.data.speed_slow, 60);
  NOLESS(probe.data.repetitions, 1);

}
