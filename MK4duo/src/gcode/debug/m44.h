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

#if ENABLED(CODES_DEBUGGING)

  #define CODE_M44

   /**
   * M44: Codes debug - report codes available (and how many of them there are).
   *
   *  M44         - Report complete statistics of available gcodes.
   *                  G   Flag to show G-code statistics.
   *                  M   Flag to show M-code statistics.
   *
   */
  inline void gcode_M44(void) {

    if (parser.seen('G')) {
      SERIAL_EMV("Number of G-codes available: ", COUNT(GCode_Table));
      SERIAL_EM("Complete list of G-codes available for this machine:");

      G_CODE_TYPE i;
      for(i = 0; i < COUNT(GCode_Table) ; i++){
        SERIAL_EMV("G", GCode_Table[i].code);
      }

    }else if (parser.seen('M') {
      SERIAL_EMV("Number of M-codes available: ", COUNT(MCode_Table));
      SERIAL_EM("Complete list of M-codes available for this machine:");

      M_CODE_TYPE i;
      for(i = 0; i < COUNT(MCode_Table) ; i++){
        SERIAL_EMV("M", MCode_Table[i].code);
      }

    }else {
      SERIAL_EM("Invalid parameter.");
      SERIAL_EM("Please use the G or the M flag to choose between G-codes and M-codes.");
    }

  }
#endif // GCODES_DEBUGGING
