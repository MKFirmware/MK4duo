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
 * mcode.h
 *
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
 */

#if ENABLED(BABYSTEPPING)

  #define CODE_M290

  #if ENABLED(BABYSTEP_ZPROBE_OFFSET)
    FORCE_INLINE void mod_probe_zoffset(const float &offs) {
      probe.data.offset[Z_AXIS] += offs;
      SERIAL_LMV(ECHO, MSG_PROBE_Z_OFFSET ": ", probe.data.offset[Z_AXIS]);
    }
  #endif

  /**
   * M290: Babystepping
   */
  inline void gcode_M290(void) {
    #if ENABLED(BABYSTEP_XY)
      for (uint8_t a = X_AXIS; a <= Z_AXIS; a++)
        if (parser.seenval(axis_codes[a]) || (a == Z_AXIS && parser.seenval('S'))) {
          const float offs = constrain(parser.value_axis_units((AxisEnum)a), -2, 2);
          mechanics.babystep_axis((AxisEnum)a, offs * mechanics.data.axis_steps_per_mm[a]);
          #if ENABLED(BABYSTEP_ZPROBE_OFFSET)
            if (a == Z_AXIS && parser.boolval('P')) mod_probe_zoffset(offs);
          #endif
        }
    #else
      if (parser.seenval('Z') || parser.seenval('S')) {
        const float offs = constrain(parser.value_axis_units(Z_AXIS), -2, 2);
        mechanics.babystep_axis(Z_AXIS, offs * mechanics.data.axis_steps_per_mm[Z_AXIS]);
        #if ENABLED(BABYSTEP_ZPROBE_OFFSET)
          if (parser.boolval('P')) mod_probe_zoffset(offs);
        #endif
      }
    #endif
  }

#endif // BABYSTEPPING
