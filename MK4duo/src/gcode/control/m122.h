/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2017 Alberto Cotronei @MagoKimbra
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

#define CODE_M122

/**
 * M122: Enable, Disable, and/or Report software endstops
 *
 * Usage: M122 S1 to enable, M122 S0 to disable, M122 alone for report
 */
inline void gcode_M122(void) {

  #if HAS_SOFTWARE_ENDSTOPS
    if (parser.seen('S')) endstops.soft_endstops_enabled = parser.value_bool();
    SERIAL_SM(ECHO, MSG_SOFT_ENDSTOPS);
    SERIAL_PS(endstops.soft_endstops_enabled ? PSTR(MSG_ON) : PSTR(MSG_OFF));
  #else
    SERIAL_MSG(MSG_SOFT_ENDSTOPS);
    SERIAL_MSG(MSG_OFF);
  #endif

  SERIAL_MSG(MSG_SOFT_MIN);
  SERIAL_MV(    MSG_X, endstops.soft_endstop_min[X_AXIS]);
  SERIAL_MV(" " MSG_Y, endstops.soft_endstop_min[Y_AXIS]);
  SERIAL_MV(" " MSG_Z, endstops.soft_endstop_min[Z_AXIS]);
  SERIAL_MSG(MSG_SOFT_MAX);
  SERIAL_MV(    MSG_X, endstops.soft_endstop_max[X_AXIS]);
  SERIAL_MV(" " MSG_Y, endstops.soft_endstop_max[Y_AXIS]);
  SERIAL_MV(" " MSG_Z, endstops.soft_endstop_max[Z_AXIS]);
  SERIAL_EOL();
}
