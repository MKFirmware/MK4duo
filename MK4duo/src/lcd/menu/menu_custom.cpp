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
// Custom User Menu
//

#include "../../../MK4duo.h"

#if HAS_LCD_MENU && ENABLED(CUSTOM_USER_MENUS)

#ifdef USER_SCRIPT_DONE
  #define _DONE_SCRIPT "\n" USER_SCRIPT_DONE
#else
  #define _DONE_SCRIPT ""
#endif

void _lcd_user_gcode(PGM_P const cmd) {
  commands.enqueue_and_echo_P(cmd);
  #if ENABLED(USER_SCRIPT_AUDIBLE_FEEDBACK)
    sound.feedback();
  #endif
  #if ENABLED(USER_SCRIPT_RETURN)
    lcdui.return_to_status();
  #endif
}

#if ENABLED(USER_DESC_1) && ENABLED(USER_GCODE_1)
  void lcd_user_gcode_1() { _lcd_user_gcode(PSTR(USER_GCODE_1 _DONE_SCRIPT)); }
#endif
#if ENABLED(USER_DESC_2) && ENABLED(USER_GCODE_2)
  void lcd_user_gcode_2() { _lcd_user_gcode(PSTR(USER_GCODE_2 _DONE_SCRIPT)); }
#endif
#if ENABLED(USER_DESC_3) && ENABLED(USER_GCODE_3)
  void lcd_user_gcode_3() { _lcd_user_gcode(PSTR(USER_GCODE_3 _DONE_SCRIPT)); }
#endif
#if ENABLED(USER_DESC_4) && ENABLED(USER_GCODE_4)
  void lcd_user_gcode_4() { _lcd_user_gcode(PSTR(USER_GCODE_4 _DONE_SCRIPT)); }
#endif
#if ENABLED(USER_DESC_5) && ENABLED(USER_GCODE_5)
  void lcd_user_gcode_5() { _lcd_user_gcode(PSTR(USER_GCODE_5 _DONE_SCRIPT)); }
#endif

void menu_user() {
  START_MENU();
  MENU_BACK(MSG_MAIN);
  #if ENABLED(USER_DESC_1) && ENABLED(USER_GCODE_1)
    MENU_ITEM(function, USER_DESC_1, lcd_user_gcode_1);
  #endif
  #if ENABLED(USER_DESC_2) && ENABLED(USER_GCODE_2)
    MENU_ITEM(function, USER_DESC_2, lcd_user_gcode_2);
  #endif
  #if ENABLED(USER_DESC_3) && ENABLED(USER_GCODE_3)
    MENU_ITEM(function, USER_DESC_3, lcd_user_gcode_3);
  #endif
  #if ENABLED(USER_DESC_4) && ENABLED(USER_GCODE_4)
    MENU_ITEM(function, USER_DESC_4, lcd_user_gcode_4);
  #endif
  #if ENABLED(USER_DESC_5) && ENABLED(USER_GCODE_5)
    MENU_ITEM(function, USER_DESC_5, lcd_user_gcode_5);
  #endif
  END_MENU();
}

#endif // HAS_LCD_MENU && CUSTOM_USER_MENUS
