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

#if HAS_LCD_MENU && ENABLED(CUSTOM_USER_MENUS)

#ifdef USER_SCRIPT_DONE
  #define _DONE_SCRIPT "\n" USER_SCRIPT_DONE
#else
  #define _DONE_SCRIPT ""
#endif

void _lcd_user_gcode(PGM_P const cmd) {
  commands.inject_P(cmd);
  #if ENABLED(USER_SCRIPT_AUDIBLE_FEEDBACK)
    sound.feedback();
  #endif
  #if ENABLED(USER_SCRIPT_RETURN)
    lcdui.return_to_status();
  #endif
}

#if ENABLED(CUSTOM_USER_MENUS)

  #define HAS_USER_ITEM(N)  (defined(USER_DESC_##N) && defined(USER_GCODE_##N))
  #define USER_ITEM(N)      ACTION_ITEM_P(PSTR(USER_DESC_##N), []{ _lcd_user_gcode(PSTR(USER_GCODE_##N _DONE_SCRIPT)); });
  
  void menu_user() {
    START_MENU();
    BACK_ITEM(MSG_MAIN);
    #if HAS_USER_ITEM(1)
      USER_ITEM(1);
    #endif
    #if HAS_USER_ITEM(2)
      USER_ITEM(2);
    #endif
    #if HAS_USER_ITEM(3)
      USER_ITEM(3);
    #endif
    #if HAS_USER_ITEM(4)
      USER_ITEM(4);
    #endif
    #if HAS_USER_ITEM(5)
      USER_ITEM(5);
    #endif
    #if HAS_USER_ITEM(6)
      USER_ITEM(6);
    #endif
    #if HAS_USER_ITEM(7)
      USER_ITEM(7);
    #endif
    #if HAS_USER_ITEM(8)
      USER_ITEM(8);
    #endif
    #if HAS_USER_ITEM(9)
      USER_ITEM(9);
    #endif
    #if HAS_USER_ITEM(10)
      USER_ITEM(10);
    #endif
    #if HAS_USER_ITEM(11)
      USER_ITEM(11);
    #endif
    #if HAS_USER_ITEM(12)
      USER_ITEM(12);
    #endif
    #if HAS_USER_ITEM(13)
      USER_ITEM(13);
    #endif
    #if HAS_USER_ITEM(14)
      USER_ITEM(14);
    #endif
    #if HAS_USER_ITEM(15)
      USER_ITEM(15);
    #endif
    #if HAS_USER_ITEM(16)
      USER_ITEM(16);
    #endif
    #if HAS_USER_ITEM(17)
      USER_ITEM(17);
    #endif
    #if HAS_USER_ITEM(18)
      USER_ITEM(18);
    #endif
    #if HAS_USER_ITEM(19)
      USER_ITEM(19);
    #endif
    #if HAS_USER_ITEM(20)
      USER_ITEM(20);
    #endif
    #if HAS_USER_ITEM(21)
      USER_ITEM(21);
    #endif
    #if HAS_USER_ITEM(22)
      USER_ITEM(22);
    #endif
    #if HAS_USER_ITEM(23)
      USER_ITEM(23);
    #endif
    #if HAS_USER_ITEM(24)
      USER_ITEM(24);
    #endif
    #if HAS_USER_ITEM(25)
      USER_ITEM(25);
    #endif
    #if HAS_USER_ITEM(26)
      USER_ITEM(26);
    #endif
    #if HAS_USER_ITEM(27)
      USER_ITEM(27);
    #endif
    #if HAS_USER_ITEM(28)
      USER_ITEM(28);
    #endif
    #if HAS_USER_ITEM(29)
      USER_ITEM(29);
    #endif
    #if HAS_USER_ITEM(30)
      USER_ITEM(30);
    #endif
    END_MENU();
  }

#endif // CUSTOM_USER_MENUS

#endif // HAS_LCD_MENU && CUSTOM_USER_MENUS
