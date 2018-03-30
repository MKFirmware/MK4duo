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

#if HAS_RESUME_CONTINUE

  #define CODE_M0
  #define CODE_M1

  /**
   * M0: Unconditional stop - Wait for user button press on LCD
   * M1: Same as M0
   */
  inline void gcode_M0_M1(void) {
    const char * const args = parser.string_arg;

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

    #if ENABLED(ULTIPANEL)

      if (!hasP && !hasS && args && *args)
        lcd_setstatus(args, true);
      else {
        LCD_MESSAGEPGM(MSG_USERWAIT);
        #if ENABLED(LCD_PROGRESS_BAR) && PROGRESS_MSG_EXPIRE > 0
          dontExpireStatus();
        #endif
      }

    #elif ENABLED(NEXTION)

      if (!hasP && !hasS && args && *args)
        lcd_yesno(args, "", MSG_USERWAIT);
      else
        lcd_yesno(MSG_USERWAIT);

    #else

      if (!hasP && !hasS && args && *args)
        SERIAL_LT(ECHO, args);

    #endif

    printer.setWaitForUser(true);
    printer.keepalive(PausedforUser);

    stepper.synchronize();
    commands.refresh_cmd_timeout();

    if (ms > 0) {
      ms += millis();  // wait until this time for a click
      while (PENDING(millis(), ms) && printer.isWaitForUser()) printer.idle();
    }
    else {
      #if ENABLED(ULTIPANEL)
        if (lcd_detected())
      #endif
        while (printer.isWaitForUser()) printer.idle();
    }

    IS_SD_PRINTING ? LCD_MESSAGEPGM(MSG_RESUMING) : LCD_MESSAGEPGM(WELCOME_MSG);

    printer.setWaitForUser(false);
    printer.keepalive(InHandler);
  }

#endif // HAS_RESUME_CONTINUE
