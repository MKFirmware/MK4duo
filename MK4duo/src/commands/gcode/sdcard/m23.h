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

#define CODE_M23

/**
 * M23: Select a file
 */
inline void gcode_M23() {
  // Simplify3D includes the size, so zero out all spaces (#7227)
  // Questa funzione blocca il nome al primo spazio quindi file con spazio nei nomi non funziona da rivedere
  //for (char *fn = parser.string_arg; *fn; ++fn) if (*fn == ' ') *fn = '\0';
  card.selectFile(parser.string_arg);
  lcdui.set_status(card.fileName);
}

#endif // HAS_SD_SUPPORT
