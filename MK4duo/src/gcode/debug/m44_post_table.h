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

/**
 * M44: Codes debug - report codes available (and how many of them there are).
 *
 *  M44         - Report complete statistics of available gcodes.
 *                  I   Flag to show G-code statistics.
 *                  J   Flag to show M-code statistics.
 *
 */
inline void gcode_M44(void) {

  if (parser.seen('I')) {
    SERIAL_EMV("Number of G-codes available: ", (int)(COUNT(GCode_Table) + 2));
    SERIAL_MV("G-code table static memory consumption: ", (int)sizeof(GCode_Table));
    SERIAL_EM(" bytes.");

    SERIAL_EM("Complete list of G-codes available for this machine:");
    SERIAL_EM("G0");
    SERIAL_EM("G1");
    for (G_CODE_TYPE index = 0; index < COUNT(GCode_Table); index++) {
      SERIAL_EMV("G", GCode_Table[index].code);
    }
  }
  
  if (parser.seen('J')) {
    SERIAL_EMV("Number of M-codes available: ", (int)COUNT(MCode_Table));
    SERIAL_MV("M-code table static memory consumption: ", (int)sizeof(MCode_Table));
    SERIAL_EM(" bytes.");

    SERIAL_EM("Complete list of M-codes available for this machine:");
    for (M_CODE_TYPE index = 0; index < (COUNT(MCode_Table) - 1); index++) {
      SERIAL_EMV("M", MCode_Table[index].code);
    }
  }

}
