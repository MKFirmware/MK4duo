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

#if HAS_SD_RESTART

#define CODE_M413

/**
 * M413: Enable / Disable Restart Job
 *
 * Parameters
 *  S[bool] - Flag to enable / disable.
 */
inline void gcode_M413() {

  if (parser.seen('S')) restart.enable(parser.value_bool());

  SERIAL_STR(ECHO);
  SERIAL_ONOFF(" SD Restart Job", restart.enabled);
  SERIAL_EOL();

  if (parser.seen('R') || parser.seen('L')) restart.load_job();
  if (parser.seen('W')) restart.save_job();
  if (parser.seen('P')) restart.purge_job();
  if (parser.seen('E')) (void)restart.exists();
  if (parser.seen('V')) DEBUG_LS(DEB, restart.valid() ? PSTR(" Valid") : PSTR(" Invalid"));

}

#endif // HAS_SD_RESTART
