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

#if HAS_NEXTION_LCD && HAS_SD_SUPPORT

#define CODE_M36

/**
 * M36: Print pause, resume and stop from Nextion
 *    S - Stop print
 *    P - Play or resume print
 */
inline void gcode_M36(void) {
  if (parser.seen('S')) {
    #if HAS_LCD_MENU
      lcdui.goto_screen(menu_stop_print);
    #else
      lcdui.stop_print();
    #endif
  }
  else if (parser.seen('P')) {
    if (printer.isPrinting())     lcdui.pause_print();
    else if (printer.isPaused())  lcdui.resume_print();
  }
}

#endif // HAS_NEXTION_LCD && HAS_SD_SUPPORT
