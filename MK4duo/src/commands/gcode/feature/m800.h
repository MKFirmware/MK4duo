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

#define CODE_M800

#if HAS_LCD_MENU
  void menu_sdcard_restart();
  void lcd_sdcard_restart_cancel();
#endif

/**
 * M800: Resume from Restart Job
 *   - With 'S' go to the Restart/Cancel menu
 *   - With no parameters run restart commands
 */
inline void gcode_M800() {
  if (restart.valid()) {
    if (parser.seen('S')) {
      #if HAS_LCD_MENU
        lcdui.goto_screen(menu_sdcard_restart);
      #else
        SERIAL_EM("Restart requires LCD.");
      #endif
    }
    else if (parser.seen('C')) {
      #if HAS_LCD_MENU
        lcd_sdcard_restart_cancel();
      #else
        restart.purge_job();
      #endif
    }
    else
      restart.resume_job();
  }
  else
    DEBUG_LSM(DEB, restart.job_info.valid_head ? PSTR("No") : PSTR("Invalid"), " Restart Job Data");
}

#endif // HAS_SD_RESTART
