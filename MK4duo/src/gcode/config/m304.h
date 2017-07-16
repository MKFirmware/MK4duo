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

#if ENABLED(PIDTEMPBED)

  #define CODE_M304

  // M304: Set bed PID parameters P I and D
  inline void gcode_M304(void) {
    if (parser.seen('P')) thermalManager.bedKp = parser.value_float();
    if (parser.seen('I')) thermalManager.bedKi = parser.value_float();
    if (parser.seen('D')) thermalManager.bedKd = parser.value_float();

    thermalManager.updatePID();
    SERIAL_SMV(ECHO, " p:", thermalManager.bedKp);
    SERIAL_MV(" i:", thermalManager.bedKi);
    SERIAL_EMV(" d:", thermalManager.bedKd);
  }

#endif // ENABLED(PIDTEMPBED)
