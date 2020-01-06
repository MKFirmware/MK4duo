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

//
// Custom User Menu
//

#include "../../../MK4duo.h"

#if HAS_LCD_MENU && NUM_LANGUAGES > 1

void menu_language() {
  START_MENU();
  BACK_ITEM(MSG_CONFIGURATION);
  ACTION_ITEM_P(GET_LANGUAGE_NAME(0), []() { lcdui.lang = 0; lcdui.goto_previous_screen(); });
  ACTION_ITEM_P(GET_LANGUAGE_NAME(1), []() { lcdui.lang = 1; lcdui.goto_previous_screen(); });
  #if NUM_LANGUAGES > 2
    ACTION_ITEM_P(GET_LANGUAGE_NAME(2), []() { lcdui.lang = 2; lcdui.goto_previous_screen(); });
  #endif
  #if NUM_LANGUAGES > 3
    ACTION_ITEM_P(GET_LANGUAGE_NAME(3), []() { lcdui.lang = 3; lcdui.goto_previous_screen(); });
  #endif
  #if NUM_LANGUAGES > 4
    ACTION_ITEM_P(GET_LANGUAGE_NAME(4), []() { lcdui.lang = 4; lcdui.goto_previous_screen(); });
  #endif
  END_MENU();
}

#endif // HAS_LCD_MENU && NUM_LANGUAGES > 1
