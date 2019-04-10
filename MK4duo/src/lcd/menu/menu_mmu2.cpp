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

#include "../../../MK4duo.h"

#if HAS_LCD_MENU && HAS_MMU2

#include "menu.h"

uint8_t currentTool;
bool mmuMenuWait;

//
// Load Filament
//

void _mmu2_load_filamentToNozzle(uint8_t index) {
  lcdui.reset_status();
  lcdui.return_to_status();
  lcdui.status_printf_P(0,  PSTR(MSG_MMU2_LOADING_FILAMENT), int(index + 1));
  if (mmu2.load_filament_to_nozzle(index)) lcdui.reset_status();
}

inline void action_mmu2_load_filament_to_nozzle(const uint8_t tool) {
  _mmu2_load_filamentToNozzle(tool);
  lcdui.return_to_status();
}
inline void action_mmu2_load_filament_to_nozzle_0() { action_mmu2_load_filament_to_nozzle(0); }
inline void action_mmu2_load_filament_to_nozzle_1() { action_mmu2_load_filament_to_nozzle(1); }
inline void action_mmu2_load_filament_to_nozzle_2() { action_mmu2_load_filament_to_nozzle(2); }
inline void action_mmu2_load_filament_to_nozzle_3() { action_mmu2_load_filament_to_nozzle(3); }
inline void action_mmu2_load_filament_to_nozzle_4() { action_mmu2_load_filament_to_nozzle(4); }

void _mmu2_load_filament(uint8_t index) {
  lcdui.return_to_status();
  lcdui.status_printf_P(0, PSTR(MSG_MMU2_LOADING_FILAMENT), int(index + 1));
  mmu2.load_filament(index);
  lcdui.reset_status();
}
void action_mmu2_load_all() {
  for (uint8_t i = 0; i < EXTRUDERS; i++)
    _mmu2_load_filament(i);
  lcdui.return_to_status();
}
inline void action_mmu2_load_filament_0() { _mmu2_load_filament(0); }
inline void action_mmu2_load_filament_1() { _mmu2_load_filament(1); }
inline void action_mmu2_load_filament_2() { _mmu2_load_filament(2); }
inline void action_mmu2_load_filament_3() { _mmu2_load_filament(3); }
inline void action_mmu2_load_filament_4() { _mmu2_load_filament(4); }

void menu_mmu2_load_filament() {
  START_MENU();
  MENU_BACK(MSG_MMU2_MENU);
  MENU_ITEM(function, MSG_MMU2_ALL, action_mmu2_load_all);
  MENU_ITEM(function, MSG_MMU2_FILAMENT0, action_mmu2_load_filament_0);
  MENU_ITEM(function, MSG_MMU2_FILAMENT1, action_mmu2_load_filament_1);
  MENU_ITEM(function, MSG_MMU2_FILAMENT2, action_mmu2_load_filament_2);
  MENU_ITEM(function, MSG_MMU2_FILAMENT3, action_mmu2_load_filament_3);
  MENU_ITEM(function, MSG_MMU2_FILAMENT4, action_mmu2_load_filament_4);
  END_MENU();
}

void menu_mmu2_load_to_nozzle() {
  START_MENU();
  MENU_BACK(MSG_MMU2_MENU);
  MENU_ITEM(function, MSG_MMU2_FILAMENT0, action_mmu2_load_filament_to_nozzle_0);
  MENU_ITEM(function, MSG_MMU2_FILAMENT1, action_mmu2_load_filament_to_nozzle_1);
  MENU_ITEM(function, MSG_MMU2_FILAMENT2, action_mmu2_load_filament_to_nozzle_2);
  MENU_ITEM(function, MSG_MMU2_FILAMENT3, action_mmu2_load_filament_to_nozzle_3);
  MENU_ITEM(function, MSG_MMU2_FILAMENT4, action_mmu2_load_filament_to_nozzle_4);
  END_MENU();
}

//
// Eject Filament
//

void _mmu2_eject_filament(uint8_t index) {
  lcdui.reset_status();
  lcdui.return_to_status();
  lcdui.status_printf_P(0, PSTR(MSG_MMU2_EJECTING_FILAMENT), int(index + 1));
  if (mmu2.eject_filament(index, true)) lcdui.reset_status();
}
inline void action_mmu2_eject_filament_0() { _mmu2_eject_filament(0); }
inline void action_mmu2_eject_filament_1() { _mmu2_eject_filament(1); }
inline void action_mmu2_eject_filament_2() { _mmu2_eject_filament(2); }
inline void action_mmu2_eject_filament_3() { _mmu2_eject_filament(3); }
inline void action_mmu2_eject_filament_4() { _mmu2_eject_filament(4); }

void action_mmu2_unload_filament() {
  lcdui.reset_status();
  lcdui.return_to_status();
  LCD_MESSAGEPGM(MSG_MMU2_UNLOADING_FILAMENT);
  printer.idle();
  if (mmu2.unload()) lcdui.reset_status();
}

void menu_mmu2_eject_filament() {
  START_MENU();
  MENU_BACK(MSG_MMU2_MENU);
  MENU_ITEM(function, MSG_MMU2_FILAMENT0, action_mmu2_eject_filament_0);
  MENU_ITEM(function, MSG_MMU2_FILAMENT1, action_mmu2_eject_filament_1);
  MENU_ITEM(function, MSG_MMU2_FILAMENT2, action_mmu2_eject_filament_2);
  MENU_ITEM(function, MSG_MMU2_FILAMENT3, action_mmu2_eject_filament_3);
  MENU_ITEM(function, MSG_MMU2_FILAMENT4, action_mmu2_eject_filament_4);
  END_MENU();
}

//
// MMU2 Menu
//

void action_mmu2_reset() {
  mmu2.init();
  lcdui.reset_status();
}

void menu_mmu2() {
  START_MENU();
  MENU_BACK(MSG_MAIN);
  MENU_ITEM(submenu, MSG_MMU2_LOAD_FILAMENT, menu_mmu2_load_filament);
  MENU_ITEM(submenu, MSG_MMU2_LOAD_TO_NOZZLE, menu_mmu2_load_to_nozzle);
  MENU_ITEM(submenu, MSG_MMU2_EJECT_FILAMENT, menu_mmu2_eject_filament);
  MENU_ITEM(function, MSG_MMU2_UNLOAD_FILAMENT, action_mmu2_unload_filament);
  MENU_ITEM(function, MSG_MMU2_RESET, action_mmu2_reset);
  END_MENU();
}

//
// T* Choose Filament
//

inline void action_mmu2_choose(const uint8_t tool) {
  currentTool = tool;
  mmuMenuWait = false;
}
inline void action_mmu2_choose_0() { action_mmu2_choose(0); }
inline void action_mmu2_choose_1() { action_mmu2_choose(1); }
inline void action_mmu2_choose_2() { action_mmu2_choose(2); }
inline void action_mmu2_choose_3() { action_mmu2_choose(3); }
inline void action_mmu2_choose_4() { action_mmu2_choose(4); }

void menu_mmu2_choose_filament() {
  START_MENU();
  #if LCD_HEIGHT > 2
    STATIC_ITEM(MSG_MMU2_CHOOSE_FILAMENT_HEADER, true, true);
  #endif
  MENU_ITEM(function, MSG_MMU2_FILAMENT0, action_mmu2_choose_0);
  MENU_ITEM(function, MSG_MMU2_FILAMENT1, action_mmu2_choose_1);
  MENU_ITEM(function, MSG_MMU2_FILAMENT2, action_mmu2_choose_2);
  MENU_ITEM(function, MSG_MMU2_FILAMENT3, action_mmu2_choose_3);
  MENU_ITEM(function, MSG_MMU2_FILAMENT4, action_mmu2_choose_4);
  END_MENU();
}

//
// MMU2 Filament Runout
//

inline void action_mmu2_M600_load_current_filament()            { mmu2.load_filament(currentTool); }
inline void action_mmu2_M600_load_current_filament_to_nozzle()  { mmu2.load_filament_to_nozzle(currentTool); }
inline void action_mmu2_M600_unload_filament()                  { mmu2.unload(); }
inline void action_mmu2_M600_resume()                           { mmuMenuWait = false; }

void menu_mmu2_advanced_pause() {
  currentTool = mmu2.get_current_tool();
  START_MENU();
  #if LCD_HEIGHT > 2
    STATIC_ITEM(MSG_MMU2_FILAMENT_CHANGE_HEADER, true, true);
  #endif
  MENU_ITEM(function, MSG_MMU2_RESUME, action_mmu2_M600_resume);
  MENU_ITEM(function, MSG_MMU2_UNLOAD_FILAMENT, action_mmu2_M600_unload_filament);
  MENU_ITEM(function, MSG_MMU2_LOAD_FILAMENT, action_mmu2_M600_load_current_filament);
  MENU_ITEM(function, MSG_MMU2_LOAD_TO_NOZZLE, action_mmu2_M600_load_current_filament_to_nozzle);
  END_MENU();
}

void mmu2_M600() {
  lcdui.defer_status_screen();
  lcdui.goto_screen(menu_mmu2_advanced_pause);
  mmuMenuWait = true;
  while (mmuMenuWait) printer.idle();
}

uint8_t mmu2_choose_filament() {
  lcdui.defer_status_screen();
  lcdui.goto_screen(menu_mmu2_choose_filament);
  mmuMenuWait = true;
  while (mmuMenuWait) printer.idle();
  lcdui.return_to_status();
  return currentTool;
}

#endif // HAS_LCD_MENU && HAS_MMU2
