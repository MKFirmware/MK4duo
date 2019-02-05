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
// Temperature Menu
//

#include "../../../MK4duo.h"

#if HAS_LCD_MENU

// Initialized by settings.load()
int16_t LcdUI::preheat_hotend_temp[3], LcdUI::preheat_bed_temp[3], LcdUI::preheat_fan_speed[3];

//
// "Temperature" submenu items
//

void _lcd_preheat(const int16_t endnum, const int16_t temph, const int16_t tempb, const uint8_t fan) {
  #if HAS_TEMP_HOTEND
    if (temph > 0) heaters[endnum].setTarget(MIN(heaters[endnum].data.maxtemp - 15, temph));
  #endif
  #if HAS_TEMP_BED
    if (tempb >= 0) heaters[BED_INDEX].setTarget(tempb);
  #else
    UNUSED(tempb);
  #endif
  #if FAN_COUNT > 0
    #if FAN_COUNT > 1
      fans[tools.active_extruder < FAN_COUNT ? tools.active_extruder : 0].Speed = fan;
    #else
      fans[0].Speed = fan;
    #endif
  #else
    UNUSED(fan);
  #endif
  lcdui.return_to_status();
}

#if HOTENDS > 1
  void lcd_preheat_m1_h1_only() { _lcd_preheat(1, lcdui.preheat_hotend_temp[0], -1, lcdui.preheat_fan_speed[0]); }
  void lcd_preheat_m2_h1_only() { _lcd_preheat(1, lcdui.preheat_hotend_temp[1], -1, lcdui.preheat_fan_speed[1]); }
  void lcd_preheat_m3_h1_only() { _lcd_preheat(1, lcdui.preheat_hotend_temp[2], -1, lcdui.preheat_fan_speed[2]); }
  #if HAS_TEMP_BED
    void lcd_preheat_m1_h1() { _lcd_preheat(1, lcdui.preheat_hotend_temp[0], lcdui.preheat_bed_temp[0], lcdui.preheat_fan_speed[0]); }
    void lcd_preheat_m2_h1() { _lcd_preheat(1, lcdui.preheat_hotend_temp[1], lcdui.preheat_bed_temp[1], lcdui.preheat_fan_speed[1]); }
    void lcd_preheat_m3_h1() { _lcd_preheat(1, lcdui.preheat_hotend_temp[2], lcdui.preheat_bed_temp[2], lcdui.preheat_fan_speed[2]); }
  #endif
  #if HOTENDS > 2
    void lcd_preheat_m1_h2_only() { _lcd_preheat(2, lcdui.preheat_hotend_temp[0], -1, lcdui.preheat_fan_speed[0]); }
    void lcd_preheat_m2_h2_only() { _lcd_preheat(2, lcdui.preheat_hotend_temp[1], -1, lcdui.preheat_fan_speed[1]); }
    void lcd_preheat_m3_h2_only() { _lcd_preheat(2, lcdui.preheat_hotend_temp[2], -1, lcdui.preheat_fan_speed[2]); }
    #if HAS_TEMP_BED
      void lcd_preheat_m1_h2() { _lcd_preheat(2, lcdui.preheat_hotend_temp[0], lcdui.preheat_bed_temp[0], lcdui.preheat_fan_speed[0]); }
      void lcd_preheat_m2_h2() { _lcd_preheat(2, lcdui.preheat_hotend_temp[1], lcdui.preheat_bed_temp[1], lcdui.preheat_fan_speed[1]); }
      void lcd_preheat_m3_h2() { _lcd_preheat(2, lcdui.preheat_hotend_temp[2], lcdui.preheat_bed_temp[2], lcdui.preheat_fan_speed[2]); }
    #endif
    #if HOTENDS > 3
      void lcd_preheat_m1_h3_only() { _lcd_preheat(3, lcdui.preheat_hotend_temp[0], -1, lcdui.preheat_fan_speed[0]); }
      void lcd_preheat_m2_h3_only() { _lcd_preheat(3, lcdui.preheat_hotend_temp[1], -1, lcdui.preheat_fan_speed[1]); }
      void lcd_preheat_m3_h3_only() { _lcd_preheat(3, lcdui.preheat_hotend_temp[2], -1, lcdui.preheat_fan_speed[2]); }
      #if HAS_TEMP_BED
        void lcd_preheat_m1_h3() { _lcd_preheat(3, lcdui.preheat_hotend_temp[0], lcdui.preheat_bed_temp[0], lcdui.preheat_fan_speed[0]); }
        void lcd_preheat_m2_h3() { _lcd_preheat(3, lcdui.preheat_hotend_temp[1], lcdui.preheat_bed_temp[1], lcdui.preheat_fan_speed[1]); }
        void lcd_preheat_m3_h3() { _lcd_preheat(3, lcdui.preheat_hotend_temp[2], lcdui.preheat_bed_temp[2], lcdui.preheat_fan_speed[2]); }
      #endif
    #endif
  #endif

  #if HAS_TEMP_BED
    void lcd_preheat_m1_h0();
    void lcd_preheat_m2_h0();
    void lcd_preheat_m3_h0();
  #else
    void lcd_preheat_m1_h0_only();
    void lcd_preheat_m2_h0_only();
    void lcd_preheat_m3_h0_only();
  #endif

  void lcd_preheat_m1_all() {
    LOOP_HOTEND() heaters[h].setTarget(lcdui.preheat_hotend_temp[0]);
    #if HAS_TEMP_BED
      lcd_preheat_m1_h0();
    #else
      lcd_preheat_m1_h0_only();
    #endif
  }
  void lcd_preheat_m2_all() {
    LOOP_HOTEND() heaters[h].setTarget(lcdui.preheat_hotend_temp[1]);
    #if HAS_TEMP_BED
      lcd_preheat_m2_h0();
    #else
      lcd_preheat_m2_h0_only();
    #endif
  }
  void lcd_preheat_m3_all() {
    LOOP_HOTEND() heaters[h].setTarget(lcdui.preheat_hotend_temp[2]);
    #if HAS_TEMP_BED
      lcd_preheat_m3_h0();
    #else
      lcd_preheat_m3_h0_only();
    #endif
  }

#endif // HOTENDS > 1

#if HAS_TEMP_HOTEND || HAS_TEMP_BED

  void lcd_preheat_m1_h0_only() { _lcd_preheat(0, lcdui.preheat_hotend_temp[0], -1, lcdui.preheat_fan_speed[0]); }
  void lcd_preheat_m2_h0_only() { _lcd_preheat(0, lcdui.preheat_hotend_temp[1], -1, lcdui.preheat_fan_speed[1]); }
  void lcd_preheat_m3_h0_only() { _lcd_preheat(0, lcdui.preheat_hotend_temp[2], -1, lcdui.preheat_fan_speed[2]); }

  #if HAS_TEMP_BED
    void lcd_preheat_m1_h0() { _lcd_preheat(0, lcdui.preheat_hotend_temp[0], lcdui.preheat_bed_temp[0], lcdui.preheat_fan_speed[0]); }
    void lcd_preheat_m2_h0() { _lcd_preheat(0, lcdui.preheat_hotend_temp[1], lcdui.preheat_bed_temp[1], lcdui.preheat_fan_speed[1]); }
    void lcd_preheat_m3_h0() { _lcd_preheat(0, lcdui.preheat_hotend_temp[2], lcdui.preheat_bed_temp[2], lcdui.preheat_fan_speed[2]); }
    void lcd_preheat_m1_bedonly() { _lcd_preheat(0, 0, lcdui.preheat_bed_temp[0], lcdui.preheat_fan_speed[0]); }
    void lcd_preheat_m2_bedonly() { _lcd_preheat(0, 0, lcdui.preheat_bed_temp[1], lcdui.preheat_fan_speed[1]); }
    void lcd_preheat_m3_bedonly() { _lcd_preheat(0, 0, lcdui.preheat_bed_temp[2], lcdui.preheat_fan_speed[2]); }
  #endif

  void menu_preheat_m1() {
    START_MENU();
    MENU_BACK(MSG_TEMPERATURE);
    #if HOTENDS == 1
      #if HAS_TEMP_BED
        MENU_ITEM(function, MSG_PREHEAT_1, lcd_preheat_m1_h0);
        MENU_ITEM(function, MSG_PREHEAT_1_END, lcd_preheat_m1_h0_only);
      #else
        MENU_ITEM(function, MSG_PREHEAT_1, lcd_preheat_m1_h0_only);
      #endif
    #elif HOTENDS > 1
      #if HAS_TEMP_BED
        MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H1, lcd_preheat_m1_h0);
        MENU_ITEM(function, MSG_PREHEAT_1_END " " MSG_H1, lcd_preheat_m1_h0_only);
        MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H2, lcd_preheat_m1_h1);
        MENU_ITEM(function, MSG_PREHEAT_1_END " " MSG_H2, lcd_preheat_m1_h1_only);
      #else
        MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H1, lcd_preheat_m1_h0_only);
        MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H2, lcd_preheat_m1_h1_only);
      #endif
      #if HOTENDS > 2
        #if HAS_TEMP_BED
          MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H3, lcd_preheat_m1_h2);
          MENU_ITEM(function, MSG_PREHEAT_1_END " " MSG_H3, lcd_preheat_m1_h2_only);
        #else
          MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H3, lcd_preheat_m1_h2_only);
        #endif
        #if HOTENDS > 3
          #if HAS_TEMP_BED
            MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H4, lcd_preheat_m1_h3);
            MENU_ITEM(function, MSG_PREHEAT_1_END " " MSG_H4, lcd_preheat_m1_h3_only);
          #else
            MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H4, lcd_preheat_m1_h3_only);
          #endif
        #endif
      #endif
      MENU_ITEM(function, MSG_PREHEAT_1_ALL, lcd_preheat_m1_all);
    #endif
    #if HAS_TEMP_BED
      MENU_ITEM(function, MSG_PREHEAT_1_BEDONLY, lcd_preheat_m1_bedonly);
    #endif
    END_MENU();
  }

  void menu_preheat_m2() {
    START_MENU();
    MENU_BACK(MSG_TEMPERATURE);
    #if HOTENDS == 1
      #if HAS_TEMP_BED
        MENU_ITEM(function, MSG_PREHEAT_2, lcd_preheat_m2_h0);
        MENU_ITEM(function, MSG_PREHEAT_2_END, lcd_preheat_m2_h0_only);
      #else
        MENU_ITEM(function, MSG_PREHEAT_2, lcd_preheat_m2_h0_only);
      #endif
    #elif HOTENDS > 1
      #if HAS_TEMP_BED
        MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H1, lcd_preheat_m2_h0);
        MENU_ITEM(function, MSG_PREHEAT_2_END " " MSG_H1, lcd_preheat_m2_h0_only);
        MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H2, lcd_preheat_m2_h1);
        MENU_ITEM(function, MSG_PREHEAT_2_END " " MSG_H2, lcd_preheat_m2_h1_only);
      #else
        MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H1, lcd_preheat_m2_h0_only);
        MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H2, lcd_preheat_m2_h1_only);
      #endif
      #if HOTENDS > 2
        #if HAS_TEMP_BED
          MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H3, lcd_preheat_m2_h2);
          MENU_ITEM(function, MSG_PREHEAT_2_END " " MSG_H3, lcd_preheat_m2_h2_only);
        #else
          MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H3, lcd_preheat_m2_h2_only);
        #endif
        #if HOTENDS > 3
          #if HAS_TEMP_BED
            MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H4, lcd_preheat_m2_h3);
            MENU_ITEM(function, MSG_PREHEAT_2_END " " MSG_H4, lcd_preheat_m2_h3_only);
          #else
            MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H4, lcd_preheat_m2_h3_only);
          #endif
        #endif
      #endif
      MENU_ITEM(function, MSG_PREHEAT_2_ALL, lcd_preheat_m2_all);
    #endif
    #if HAS_TEMP_BED
      MENU_ITEM(function, MSG_PREHEAT_2_BEDONLY, lcd_preheat_m2_bedonly);
    #endif
    END_MENU();
  }

  void menu_preheat_m3() {
    START_MENU();
    MENU_BACK(MSG_TEMPERATURE);
    #if HOTENDS == 1
      #if HAS_TEMP_BED
        MENU_ITEM(function, MSG_PREHEAT_3, lcd_preheat_m3_h0);
        MENU_ITEM(function, MSG_PREHEAT_3_END, lcd_preheat_m3_h0_only);
      #else
        MENU_ITEM(function, MSG_PREHEAT_3, lcd_preheat_m3_h0_only);
      #endif
    #elif HOTENDS > 1
      #if HAS_TEMP_BED
        MENU_ITEM(function, MSG_PREHEAT_3_N MSG_H1, lcd_preheat_m3_h0);
        MENU_ITEM(function, MSG_PREHEAT_3_END " " MSG_H1, lcd_preheat_m3_h0_only);
        MENU_ITEM(function, MSG_PREHEAT_3_N MSG_H2, lcd_preheat_m3_h1);
        MENU_ITEM(function, MSG_PREHEAT_3_END " " MSG_H2, lcd_preheat_m3_h1_only);
      #else
        MENU_ITEM(function, MSG_PREHEAT_3_N MSG_H1, lcd_preheat_m3_h0_only);
        MENU_ITEM(function, MSG_PREHEAT_3_N MSG_H2, lcd_preheat_m3_h1_only);
      #endif
      #if HOTENDS > 2
        #if HAS_TEMP_BED
          MENU_ITEM(function, MSG_PREHEAT_3_N MSG_H3, lcd_preheat_m3_h2);
          MENU_ITEM(function, MSG_PREHEAT_3_END " " MSG_H3, lcd_preheat_m3_h2_only);
        #else
          MENU_ITEM(function, MSG_PREHEAT_3_N MSG_H3, lcd_preheat_m3_h2_only);
        #endif
        #if HOTENDS > 3
          #if HAS_TEMP_BED
            MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H4, lcd_preheat_m3_h3);
            MENU_ITEM(function, MSG_PREHEAT_2_END " " MSG_H4, lcd_preheat_m3_h3_only);
          #else
            MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H4, lcd_preheat_m3_h3_only);
          #endif
        #endif
      #endif
      MENU_ITEM(function, MSG_PREHEAT_3_ALL, lcd_preheat_m3_all);
    #endif
    #if HAS_TEMP_BED
      MENU_ITEM(function, MSG_PREHEAT_3_BEDONLY, lcd_preheat_m3_bedonly);
    #endif
    END_MENU();
  }

  void lcd_cooldown() {
    thermalManager.disable_all_heaters();
    printer.zero_fan_speed();
    lcdui.return_to_status();
  }

#endif // HAS_TEMP_HOTEND || HAS_TEMP_BED

void menu_temperature() {
  START_MENU();
  MENU_BACK(MSG_MAIN);

  //
  // Nozzle:
  // Nozzle [1-4]:
  //
  #if HOTENDS == 1
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE, &heaters[0].target_temperature, 0, heaters[0].data.maxtemp - 15, watch_temp_callback_E0);
  #elif HOTENDS > 1
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N1, &heaters[0].target_temperature, 0, heaters[0].data.maxtemp - 15, watch_temp_callback_E0);
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N2, &heaters[1].target_temperature, 0, heaters[1].data.maxtemp - 15, watch_temp_callback_E1);
    #if HOTENDS > 2
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N3, &heaters[2].target_temperature, 0, heaters[2].data.maxtemp - 15, watch_temp_callback_E2);
      #if HOTENDS > 3
        MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N4, &heaters[3].target_temperature, 0, heaters[3].data.maxtemp - 15, watch_temp_callback_E3);
      #endif // HOTENDS > 3
    #endif // HOTENDS > 2
  #endif // HOTENDS > 1

  //
  // Bed:
  //
  #if HAS_TEMP_BED
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_BED, &heaters[BED_INDEX].target_temperature, 0, heaters[BED_INDEX].data.maxtemp - 15, watch_temp_callback_bed);
  #endif

  //
  // Chamber:
  //
  #if HAS_TEMP_CHAMBER
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_CHAMBER, &heaters[CHAMBER_INDEX].target_temperature, 0, heaters[CHAMBER_INDEX].data.maxtemp - 15, watch_temp_callback_chamber);
  #endif

  //
  // Cooler:
  //
  #if HAS_TEMP_COOLER
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_COOLER, &heaters[COOLER_INDEX].target_temperature, 0, heaters[COOLER_INDEX].data.maxtemp - 15, watch_temp_callback_cooler);
  #endif

  //
  // Fan Speed:
  //
  #if FAN_COUNT > 0
    #if HAS_FAN0
      MENU_MULTIPLIER_ITEM_EDIT(uint8, MSG_FAN_SPEED " 0", &fans[0].Speed, 0, 255);
    #endif
    #if HAS_FAN1
      MENU_MULTIPLIER_ITEM_EDIT(uint8, MSG_FAN_SPEED " 1", &fans[1].Speed, 0, 255);
    #endif
    #if HAS_FAN2
      MENU_MULTIPLIER_ITEM_EDIT(uint8, MSG_FAN_SPEED " 2", &fans[2].Speed, 0, 255);
    #endif
    #if HAS_FAN3
      MENU_MULTIPLIER_ITEM_EDIT(uint8, MSG_FAN_SPEED " 3", &fans[3].Speed, 0, 255);
    #endif
    #if HAS_FAN4
      MENU_MULTIPLIER_ITEM_EDIT(uint8, MSG_FAN_SPEED " 4", &fans[4].Speed, 0, 255);
    #endif
    #if HAS_FAN5
      MENU_MULTIPLIER_ITEM_EDIT(uint8, MSG_FAN_SPEED " 5", &fans[5].Speed, 0, 255);
    #endif
  #endif // FAN_COUNT > 0

  if (printer.mode == PRINTER_MODE_FFF) {

    #if HAS_TEMP_0

      //
      // Cooldown
      //
      bool has_heat = false;
      #if HEATER_COUNT > 0
        LOOP_HEATER() if (heaters[h].target_temperature) { has_heat = true; break; }
      #endif
      if (has_heat) MENU_ITEM(function, MSG_COOLDOWN, lcd_cooldown);

      //
      // Preheat for Material 1, 2 and 3
      //
      #if HAS_TEMP_1 || HAS_TEMP_2 || HAS_TEMP_3 || HAS_TEMP_BED
        MENU_ITEM(submenu, MSG_PREHEAT_1, menu_preheat_m1);
        MENU_ITEM(submenu, MSG_PREHEAT_2, menu_preheat_m2);
        MENU_ITEM(submenu, MSG_PREHEAT_3, menu_preheat_m3);
      #else
        MENU_ITEM(function, MSG_PREHEAT_1, lcd_preheat_m1_h0_only);
        MENU_ITEM(function, MSG_PREHEAT_2, lcd_preheat_m2_h0_only);
        MENU_ITEM(function, MSG_PREHEAT_3, lcd_preheat_m3_h0_only);
      #endif

    #endif // HAS_TEMP_0

  } // printer mode FFF

  END_MENU();
}

#endif // HAS_LCD_MENU
