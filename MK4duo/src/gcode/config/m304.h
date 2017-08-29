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

#if (PIDTEMPBED)

  #define CODE_M304

  // M304: Set bed PID parameters P I and D
  inline void gcode_M304(void) {
    if (parser.seen('P')) heaters[BED_INDEX].Kp = parser.value_float();
    if (parser.seen('I')) heaters[BED_INDEX].Ki = parser.value_float();
    if (parser.seen('D')) heaters[BED_INDEX].Kd = parser.value_float();

    thermalManager.updatePID();
    SERIAL_SMV(ECHO, " p:", heaters[BED_INDEX].Kp);
    SERIAL_MV(" i:", heaters[BED_INDEX].Ki);
    SERIAL_EMV(" d:", heaters[BED_INDEX].Kd);
  }

#endif // (PIDTEMPBED)
