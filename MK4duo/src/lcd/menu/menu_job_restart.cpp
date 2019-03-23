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

//
// Job Restart Menu
//

#include "../../../MK4duo.h"

#if HAS_LCD_MENU && HAS_SD_RESTART

static void lcd_sdcard_restart_resume() {
  // Return to status now
  lcdui.return_to_status();
  commands.enqueue_and_echo_P(PSTR("M800"));
}

static void lcd_sdcard_restart_cancel() {
  restart.purge_job();
  card.autostart_index = 0;
  lcdui.return_to_status();
}

void menu_sdcard_restart() {
  lcdui.defer_status_screen(true);
  START_MENU();
  STATIC_ITEM(MSG_RESTART);
  MENU_ITEM(function, MSG_RESUME_PRINT, lcd_sdcard_restart_resume);
  MENU_ITEM(function, MSG_STOP_PRINT, lcd_sdcard_restart_cancel);
  END_MENU();
}

#endif // HAS_LCD_MENU && HAS_SD_RESTART
