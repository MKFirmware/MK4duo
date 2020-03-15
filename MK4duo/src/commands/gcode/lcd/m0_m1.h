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

#if HAS_RESUME_CONTINUE

#define CODE_M0
#define CODE_M1

/**
 * M0: Unconditional stop - Wait for user button press on LCD
 * M1: Same as M0
 */
inline void gcode_M0_M1() {

  millis_l ms = 0;

  if (parser.seenval('P')) ms = parser.value_millis();              // Milliseconds to wait
  if (parser.seenval('S')) ms = parser.value_millis_from_seconds(); // Seconds to wait

  const bool seenQ = parser.seen('Q');

  planner.synchronize();

  #if HAS_LEDS_OFF_FLAG
    if (seenQ) ledevents.onPrintCompleted();                        // Change LED color for Print Completed
  #endif

  #if HAS_LCD_MENU

    if (parser.string_arg)
      lcdui.set_status(parser.string_arg, true);
    else if (!seenQ) {
      LCD_MESSAGEPGM(MSG_USERWAIT);
      #if ENABLED(LCD_PROGRESS_BAR) && PROGRESS_MSG_EXPIRE > 0
        lcdui.reset_progress_bar_timeout();
      #endif
    }

  #else

    if (parser.string_arg) SERIAL_LT(ECHO, parser.string_arg);

  #endif

  PRINTER_KEEPALIVE(PausedforUser);
  printer.setWaitForUser(true);

  #if HAS_NEXTION_LCD
    lcdui.goto_screen(menu_m0);
  #endif

  if (ms > 0) ms += millis();   // wait until this time for a click
  while (printer.isWaitForUser() && (ms == 0 || PENDING(millis(), ms))) printer.idle();

  #if HAS_NEXTION_LCD
    if (!seenQ) lcdui.return_to_status();
  #endif

  #if HAS_LCD_MENU
    if (!seenQ) lcdui.reset_status();
  #endif

  printer.setWaitForUser(false);
}

#endif // HAS_RESUME_CONTINUE
