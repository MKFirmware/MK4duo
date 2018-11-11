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

//
// Job Restart Menu
//

#include "../../../MK4duo.h"

#if HAS_LCD_MENU && HAS_SD_RESTART

static void lcd_sdcard_restart_job() {
  // Return to status now
  lcd_return_to_status();
  restart.start_job();
}

static void lcd_sdcard_restart_stop() {
  card.printingHasFinished();
  lcd_return_to_status();
}

void menu_sdcard_restart() {
  set_defer_return_to_status(true);
  START_MENU();
  MENU_ITEM(function, MSG_RESTART_PRINT, lcd_sdcard_restart_job);
  MENU_ITEM(function, MSG_STOP_PRINT, lcd_sdcard_restart_stop);
  END_MENU();
}

#endif // HAS_LCD_MENU && HAS_SD_RESTART
