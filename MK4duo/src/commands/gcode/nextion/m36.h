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

#if HAS_NEXTION_LCD && HAS_SD_SUPPORT

#define CODE_M36

/**
 * M36: Print pause, resume and stop from Nextion
 *    S - Stop print
 *    P - Play or resume print
 */
inline void gcode_M36() {
  if (parser.seen('S')) {
    #if HAS_LCD_MENU
      lcdui.goto_screen([]{
        MenuItem_confirm::confirm_screen(
          lcdui.stop_print, lcdui.goto_previous_screen,
          GET_TEXT(MSG_ARE_YOU_SURE), (PGM_P)nullptr, PSTR("?")
        );
      });
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
