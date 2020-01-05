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
// Temperature Menu
//

#include "../../../MK4duo.h"

#if HAS_LCD_MENU

// Initialized by settings.load()
#if HAS_HOTENDS
  int16_t LcdUI::preheat_hotend_temp[3];
#endif
#if HAS_BEDS
  int16_t LcdUI::preheat_bed_temp[3];
#endif
#if HAS_CHAMBERS
  int16_t LcdUI::preheat_chamber_temp[3];
#endif
#if HAS_FAN
  uint8_t LcdUI::preheat_fan_speed[3];
#endif

//
// "Temperature" submenu items
//

void _lcd_preheat(const int16_t hotend, const uint8_t memory, const bool only_hotend) {
  #if HAS_HOTENDS
    hotends[hotend]->set_target_temp(MIN(tempManager.hotend_maxtemp_all(), lcdui.preheat_hotend_temp[memory]));
  #else
    UNUSED(hotend);
  #endif
  if (!only_hotend) {
    #if HAS_BEDS
      LOOP_BED() beds[h]->set_target_temp(lcdui.preheat_bed_temp[memory]);
    #endif
    #if HAS_CHAMBERS
      LOOP_CHAMBER() chambers[h]->set_target_temp(lcdui.preheat_chamber_temp[memory]);
    #endif
  }
  #if HAS_FAN
    #if FAN_COUNT > 1
      fans[toolManager.extruder.active < FAN_COUNT ? toolManager.extruder.active : 0]->speed = lcdui.preheat_fan_speed[memory];
    #else
      fans[0]->speed = lcdui.preheat_fan_speed[memory];
    #endif
  #endif
  lcdui.return_to_status();
}

#if MAX_HOTEND >0 || HAS_BEDS

  void menu_preheat_m1() {
    START_MENU();
    BACK_ITEM(MSG_TEMPERATURE);
    LOOP_HOTEND() {
      ACTION_ITEM_N(h, MSG_PREHEAT_1_H,   []{ _lcd_preheat(MenuItemBase::itemIndex, 0, false);  });
      ACTION_ITEM_N(h, MSG_PREHEAT_1_END, []{ _lcd_preheat(MenuItemBase::itemIndex, 0, true);   });
    }
    ACTION_ITEM(MSG_PREHEAT_1_ALL,        []{ LOOP_HOTEND() _lcd_preheat(h, 0, false);          });
    END_MENU();
  }

  void menu_preheat_m2() {
    START_MENU();
    BACK_ITEM(MSG_TEMPERATURE);
    LOOP_HOTEND() {
      ACTION_ITEM_N(h, MSG_PREHEAT_2_H,   []{ _lcd_preheat(MenuItemBase::itemIndex, 1, false);  });
      ACTION_ITEM_N(h, MSG_PREHEAT_2_END, []{ _lcd_preheat(MenuItemBase::itemIndex, 1, true);   });
    }
    ACTION_ITEM(MSG_PREHEAT_2_ALL,        []{ LOOP_HOTEND() _lcd_preheat(h, 1, false);          });
    END_MENU();
  }

  void menu_preheat_m3() {
    START_MENU();
    BACK_ITEM(MSG_TEMPERATURE);
    LOOP_HOTEND() {
      ACTION_ITEM_N(h, MSG_PREHEAT_2_H,   []{ _lcd_preheat(MenuItemBase::itemIndex, 2, false);  });
      ACTION_ITEM_N(h, MSG_PREHEAT_3_END, []{ _lcd_preheat(MenuItemBase::itemIndex, 2, true);   });
    }
    ACTION_ITEM(MSG_PREHEAT_3_ALL,        []{ LOOP_HOTEND() _lcd_preheat(h, 2, false);          });
    END_MENU();
  }

  void lcd_cooldown() {
    tempManager.disable_all_heaters();
    printer.zero_fan_speed();
    lcdui.return_to_status();
  }

#endif // HAS_TEMP_HOTEND || HAS_TEMP_BED0

void menu_temperature() {
  START_MENU();
  BACK_ITEM(MSG_MAIN);

  //
  // Nozzle:
  //
  #if HAS_HOTENDS
    LOOP_HOTEND()
      EDIT_ITEM_FAST_N(int3, h, MSG_NOZZLE, &hotends[h]->target_temperature, 0, hotends[h]->data.temp.max - 10, watch_temp_callback_hotend);
  #endif

  //
  // Bed:
  //
  #if HAS_BEDS
    LOOP_BED()
      EDIT_ITEM_FAST_N(int3, h, MSG_BED, &beds[h]->target_temperature, 0, beds[h]->data.temp.max - 10, watch_temp_callback_bed);
  #endif

  //
  // Chamber:
  //
  #if HAS_CHAMBERS
    LOOP_CHAMBER()
      EDIT_ITEM_FAST_N(int3, h, MSG_CHAMBER, &chambers[h]->target_temperature, 0, chambers[h]->data.temp.max - 10, watch_temp_callback_chamber);
  #endif

  //
  // Cooler:
  //
  #if HAS_COOLERS
    EDIT_ITEM_FAST(int3, MSG_COOLER, &coolers[0]->target_temperature, 0, coolers[0]->data.temp.max - 10, watch_temp_callback_cooler);
  #endif

  //
  // Fan Speed:
  //
  #if HAS_FAN
    LOOP_FAN()
      EDIT_ITEM_FAST_N(percent, f, MSG_FAN_SPEED, &fans[f]->speed, 0, 255);
  #endif

  #if HAS_HOTENDS

    //
    // Preheat for Material 1, 2 and 3
    //
    SUBMENU(MSG_PREHEAT_1, menu_preheat_m1);
    SUBMENU(MSG_PREHEAT_2, menu_preheat_m2);
    SUBMENU(MSG_PREHEAT_3, menu_preheat_m3);

    //
    // Cooldown
    //
    bool has_heat = false;
    LOOP_HOTEND() if (hotends[h]->deg_target()) { has_heat = true; break; }
    #if HAS_BEDS
      LOOP_BED() if (beds[h]->deg_target()) { has_heat = true; break; }
    #endif
    if (has_heat) ACTION_ITEM(MSG_COOLDOWN, lcd_cooldown);

  #endif // HAS_HOTENDS

  END_MENU();
}

#endif // HAS_LCD_MENU
