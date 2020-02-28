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

#if HAS_SD_SUPPORT

#define CODE_M32

/**
 * M32: Select file and start SD print
 */
inline void gcode_M32() {
  if (IS_SD_PRINTING()) planner.synchronize();

  if (card.isMounted()) {
    card.closeFile();

    char* namestartpos = parser.string_arg ; // default name position

    SERIAL_MV("Open file: ", namestartpos);
    SERIAL_EM(" and start print.");
    card.selectFile(namestartpos);
    if (parser.seenval('S')) card.setIndex(parser.value_long());

    mechanics.feedrate_mm_s       = 20.0; // 20 units/sec
    mechanics.feedrate_percentage = 100;  // 100% mechanics.feedrate_mm_s
    card.startFilePrint();
    print_job_counter.start();
  }
}

#endif // HAS_SD_SUPPORT
