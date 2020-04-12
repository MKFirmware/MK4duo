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
 * lcdui.cpp
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */
 
#include "../../MK4duo.h"
#include "sanitycheck.h"

LcdUI lcdui;

#if LCD_HAS_WAIT_FOR_MOVE
  bool LcdUI::wait_for_move = false;
#endif

#if HAS_LCD
  uint8_t LcdUI::alert_level = 0,
          LcdUI::lang = 0;

  short_timer_t LcdUI::next_lcd_update_timer(millis());
#endif

#if HAS_SD_SUPPORT

void LcdUI::sd_changed(const uint8_t old_status, const uint8_t status) {

  if (old_status == status) return;

  if (status) {
    if (old_status < 2) set_status_P(GET_TEXT(MSG_SD_INSERTED));
  }
  else {
    if (old_status < 2) {
      #if PIN_EXISTS(SD_DETECT)
        set_status_P(GET_TEXT(MSG_SD_REMOVED));
        #if HAS_LCD_MENU
          return_to_status();
        #endif
      #endif
    }
  }

  #if PIN_EXISTS(SD_DETECT) && HAS_SPI_LCD
    init_lcd();                                                   // Revive a noisy shared SPI LCD
  #endif

  refresh();

  #if HAS_LCD
    next_lcd_update_timer.start(millis() + LCD_UPDATE_INTERVAL);  // Delay LCD update for SD activity
  #endif

}

#endif // HAS_SD_SUPPORT

#if !HAS_LCD

#define MAX_MESSAGE_LENGTH 50

void LcdUI::set_status(const char * const message, const bool)        { host_action.action_notify(message); }
void LcdUI::set_status_P(PGM_P const message, const int8_t)           { host_action.action_notify_P(message); }
void LcdUI::status_printf_P(const uint8_t, PGM_P const message, ...)  { host_action.action_notify_P(message); }

#endif
