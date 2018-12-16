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

#if HAS_SD_RESTART

  #define CODE_M413

  /**
   * M413: Enable / Disable Restart Job
   *
   * Parameters
   *  S[bool] - Flag to enable / disable.
   */
  inline void gcode_M413(void) {

    if (parser.seen('S'))
      restart.enable(parser.value_bool());
    else {
      SERIAL_STR(ECHO);
      SERIAL_ONOFF(" SD Restart Job", restart.enabled);
      SERIAL_EOL();
    }

    #if ENABLED(DEBUG_RESTART)
      if (parser.seen('R') || parser.seen('L')) restart.load_job();
      if (parser.seen('W')) restart.save_job(true);
      if (parser.seen('P')) restart.purge_job();
      if (parser.seen('E')) SERIAL_PGM(restart.exists() ? PSTR("restart Exists\n") : PSTR("No restart\n"));
      if (parser.seen('V')) SERIAL_PGM(restart.valid() ? PSTR("Valid\n") : PSTR("Invalid\n"));
    #endif

  }

#endif // HAS_SD_RESTART
