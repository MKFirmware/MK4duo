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
 * mcode
 *
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
 */

#define CODE_M73
#define CODE_M532

void gcode_M73_M532() {
  if (parser.seen('P') || parser.seen('X')) {
    printer.progress = parser.value_byte();
    NOMORE(printer.progress, 100);
  }
  if (parser.seen('L')) {
    printer.currentLayer = parser.value_long();
    char text[30] = "";
    sprintf_P(text, PSTR("%s  %i of %i"), printer.printName, int(printer.currentLayer), int(printer.maxLayer));
    lcdui.set_status(text);
  }
}

/**
 * M73: Set percentage complete (compatibility with Marlin)
 *
 * Example:
 *   M73 P25 ; Set progress to 25%
 *
 */
inline void gcode_M73(void) { gcode_M73_M532(); }

/**
 * M532: X<percent> L<curLayer> - update current print state progress (X=0..100) and layer L
 */
inline void gcode_M532(void) { gcode_M73_M532(); }
