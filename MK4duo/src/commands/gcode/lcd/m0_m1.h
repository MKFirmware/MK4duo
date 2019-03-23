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

#if HAS_RESUME_CONTINUE

  #define CODE_M0
  #define CODE_M1

  /**
   * M0: Unconditional stop - Wait for user button press on LCD
   * M1: Same as M0
   */
  inline void gcode_M0_M1(void) {

    PGM_P const args = parser.string_arg;
    millis_t ms = 0;
    bool hasP = false, hasS = false;

    if (parser.seenval('P')) {
      ms = parser.value_millis(); // milliseconds to wait
      hasP = ms > 0;
    }

    if (parser.seenval('S')) {
      ms = parser.value_millis_from_seconds(); // seconds to wait
      hasS = ms > 0;
    }

    planner.synchronize();

    #if ENABLED(ULTIPANEL)

      if (!hasP && !hasS && args && *args)
        lcdui.set_status(args, true);
      else {
        LCD_MESSAGEPGM(MSG_USERWAIT);
        #if ENABLED(LCD_PROGRESS_BAR) && PROGRESS_MSG_EXPIRE > 0
          dontExpireStatus();
        #endif
      }

    #else

      if (!hasP && !hasS && args && *args)
        SERIAL_LT(ECHO, args);

    #endif

    printer.keepalive(PausedforUser);
    printer.setWaitForUser(true);

    #if HAS_NEXTION_LCD
      lcdui.goto_screen(menu_m0);
    #endif

    if (ms > 0) {
      ms += millis();
      while (PENDING(millis(), ms) && printer.isWaitForUser()) printer.idle();
    }
    else
      while (printer.isWaitForUser()) printer.idle();

    #if HAS_NEXTION_LCD
      lcdui.return_to_status();
    #endif

    #if ENABLED(ULTIPANEL)
      lcdui.reset_status();
    #endif

    printer.setWaitForUser(false);
    printer.keepalive(InHandler);
  }

#endif // HAS_RESUME_CONTINUE
