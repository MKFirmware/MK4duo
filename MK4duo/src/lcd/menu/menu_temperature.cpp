/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2019 Alberto Cotronei @MagoKimbra
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
#if MAX_HOTEND > 0
  int16_t LcdUI::preheat_hotend_temp[3];
#endif
#if MAX_BED > 0
  int16_t LcdUI::preheat_bed_temp[3];
#endif
#if MAX_CHAMBER > 0
  int16_t LcdUI::preheat_chamber_temp[3];
#endif
#if MAX_FAN > 0
  int16_t LcdUI::preheat_fan_speed[3];
#endif

//
// "Temperature" submenu items
//

void _lcd_preheat(const int16_t hotend, const uint8_t memory, const bool only_hotend) {
  #if MAX_HOTEND > 0
    hotends[hotend]->set_target_temp(MIN(thermalManager.hotend_maxtemp_all(), lcdui.preheat_hotend_temp[memory]));
  #else
    UNUSED(hotend);
  #endif
  if (!only_hotend) {
    #if MAX_BED > 0
      LOOP_BED() beds[h]->set_target_temp(lcdui.preheat_bed_temp[memory]);
    #endif
    #if MAX_CHAMBER > 0
      LOOP_CHAMBER() chambers[h]->set_target_temp(lcdui.preheat_chamber_temp[memory]);
    #endif
  }
  #if MAX_FAN > 0
    #if FAN_COUNT > 1
      fans[tools.data.extruder.active < FAN_COUNT ? tools.data.extruder.active : 0]->speed = lcdui.preheat_fan_speed[memory];
    #else
      fans[0]->speed = lcdui.preheat_fan_speed[memory];
    #endif
  #endif
  lcdui.return_to_status();
}

#if HOTENDS > 1
  void lcd_preheat_m1_h1_only() { _lcd_preheat(1, 0, true); }
  void lcd_preheat_m2_h1_only() { _lcd_preheat(1, 1, true); }
  void lcd_preheat_m3_h1_only() { _lcd_preheat(1, 2, true); }
  #if MAX_BED > 0 || MAX_CHAMBER > 0
    void lcd_preheat_m1_h1() { _lcd_preheat(1, 0, false); }
    void lcd_preheat_m2_h1() { _lcd_preheat(1, 1, false); }
    void lcd_preheat_m3_h1() { _lcd_preheat(1, 2, false); }
  #endif
  #if HOTENDS > 2
    void lcd_preheat_m1_h2_only() { _lcd_preheat(2, 0, true); }
    void lcd_preheat_m2_h2_only() { _lcd_preheat(2, 1, true); }
    void lcd_preheat_m3_h2_only() { _lcd_preheat(2, 2, true); }
    #if MAX_BED > 0 || MAX_CHAMBER > 0
      void lcd_preheat_m1_h2() { _lcd_preheat(2, 0, false); }
      void lcd_preheat_m2_h2() { _lcd_preheat(2, 1, false); }
      void lcd_preheat_m3_h2() { _lcd_preheat(2, 2, false); }
    #endif
    #if HOTENDS > 3
      void lcd_preheat_m1_h3_only() { _lcd_preheat(3, 0, true); }
      void lcd_preheat_m2_h3_only() { _lcd_preheat(3, 1, true); }
      void lcd_preheat_m3_h3_only() { _lcd_preheat(3, 2, true); }
      #if MAX_BED > 0 || MAX_CHAMBER > 0
        void lcd_preheat_m1_h3() { _lcd_preheat(3, 0, false); }
        void lcd_preheat_m2_h3() { _lcd_preheat(3, 1, false); }
        void lcd_preheat_m3_h3() { _lcd_preheat(3, 2, false); }
      #endif
      #if HOTENDS > 4
        void lcd_preheat_m1_h4_only() { _lcd_preheat(4, 0, true); }
        void lcd_preheat_m2_h4_only() { _lcd_preheat(4, 1, true); }
        void lcd_preheat_m3_h4_only() { _lcd_preheat(4, 2, true); }
        #if MAX_BED > 0 || MAX_CHAMBER > 0
          void lcd_preheat_m1_h4() { _lcd_preheat(4, 0, false); }
          void lcd_preheat_m2_h4() { _lcd_preheat(4, 1, false); }
          void lcd_preheat_m3_h4() { _lcd_preheat(4, 2, false); }
        #endif
        #if HOTENDS > 5
          void lcd_preheat_m1_h5_only() { _lcd_preheat(5, 0, true); }
          void lcd_preheat_m2_h5_only() { _lcd_preheat(5, 1, true); }
          void lcd_preheat_m3_h5_only() { _lcd_preheat(5, 2, true); }
          #if MAX_BED > 0 || MAX_CHAMBER > 0
            void lcd_preheat_m1_h5() { _lcd_preheat(5, 0, false); }
            void lcd_preheat_m2_h5() { _lcd_preheat(5, 1, false); }
            void lcd_preheat_m3_h5() { _lcd_preheat(5, 2, false); }
          #endif
        #endif // HOTENDS > 5
      #endif // HOTENDS > 4
    #endif // HOTENDS > 3
  #endif // HOTENDS > 2

  #if MAX_BED > 0 || MAX_CHAMBER > 0
    void lcd_preheat_m1_h0();
    void lcd_preheat_m2_h0();
    void lcd_preheat_m3_h0();
  #else
    void lcd_preheat_m1_h0_only();
    void lcd_preheat_m2_h0_only();
    void lcd_preheat_m3_h0_only();
  #endif

  void lcd_preheat_m1_all() {
    #if HOTENDS > 1
      LOOP_HOTEND() hotends[h]->set_target_temp(lcdui.preheat_hotend_temp[0]);
    #endif
    #if MAX_BED > 0 || MAX_CHAMBER > 0
      lcd_preheat_m1_h0();
    #else
      lcd_preheat_m1_h0_only();
    #endif
  }
  void lcd_preheat_m2_all() {
    #if HOTENDS > 1
      LOOP_HOTEND() hotends[h]->set_target_temp(lcdui.preheat_hotend_temp[1]);
    #endif
    #if MAX_BED > 0 || MAX_CHAMBER > 0
      lcd_preheat_m2_h0();
    #else
      lcd_preheat_m2_h0_only();
    #endif
  }
  void lcd_preheat_m3_all() {
    #if HOTENDS > 1
      LOOP_HOTEND() hotends[h]->set_target_temp(lcdui.preheat_hotend_temp[2]);
    #endif
    #if MAX_BED > 0 || MAX_CHAMBER > 0
      lcd_preheat_m3_h0();
    #else
      lcd_preheat_m3_h0_only();
    #endif
  }

#endif // HOTENDS > 1

#if HOTENDS >0 || MAX_BED > 0

  void lcd_preheat_m1_h0_only() { _lcd_preheat(0, 0, true); }
  void lcd_preheat_m2_h0_only() { _lcd_preheat(0, 1, true); }
  void lcd_preheat_m3_h0_only() { _lcd_preheat(0, 2, true); }

  #if MAX_BED > 0 || MAX_CHAMBER > 0
    void lcd_preheat_m1_h0() { _lcd_preheat(0, 0, false); }
    void lcd_preheat_m2_h0() { _lcd_preheat(0, 1, false); }
    void lcd_preheat_m3_h0() { _lcd_preheat(0, 2, false); }
  #endif

  void menu_preheat_m1() {
    START_MENU();
    BACK_ITEM(MSG_TEMPERATURE);
    #if HOTENDS == 1
      #if MAX_BED > 0 || MAX_CHAMBER > 0
        ACTION_ITEM(MSG_PREHEAT_1, lcd_preheat_m1_h0);
        ACTION_ITEM(MSG_PREHEAT_1_END, lcd_preheat_m1_h0_only);
      #else
        ACTION_ITEM(MSG_PREHEAT_1, lcd_preheat_m1_h0_only);
      #endif
    #elif HOTENDS > 1
      #if MAX_BED > 0 || MAX_CHAMBER > 0
        ACTION_ITEM(MSG_PREHEAT_1_N MSG_H1, lcd_preheat_m1_h0);
        ACTION_ITEM(MSG_PREHEAT_1_END " " MSG_H1, lcd_preheat_m1_h0_only);
        ACTION_ITEM(MSG_PREHEAT_1_N MSG_H2, lcd_preheat_m1_h1);
        ACTION_ITEM(MSG_PREHEAT_1_END " " MSG_H2, lcd_preheat_m1_h1_only);
      #else
        ACTION_ITEM(MSG_PREHEAT_1_N MSG_H1, lcd_preheat_m1_h0_only);
        ACTION_ITEM(MSG_PREHEAT_1_N MSG_H2, lcd_preheat_m1_h1_only);
      #endif
      #if HOTENDS > 2
        #if MAX_BED > 0 || MAX_CHAMBER > 0
          ACTION_ITEM(MSG_PREHEAT_1_N MSG_H3, lcd_preheat_m1_h2);
          ACTION_ITEM(MSG_PREHEAT_1_END " " MSG_H3, lcd_preheat_m1_h2_only);
        #else
          ACTION_ITEM(MSG_PREHEAT_1_N MSG_H3, lcd_preheat_m1_h2_only);
        #endif
        #if HOTENDS > 3
          #if MAX_BED > 0 || MAX_CHAMBER > 0
            ACTION_ITEM(MSG_PREHEAT_1_N MSG_H4, lcd_preheat_m1_h3);
            ACTION_ITEM(MSG_PREHEAT_1_END " " MSG_H4, lcd_preheat_m1_h3_only);
          #else
            ACTION_ITEM(MSG_PREHEAT_1_N MSG_H4, lcd_preheat_m1_h3_only);
          #endif
          #if HOTENDS > 4
            #if MAX_BED > 0 || MAX_CHAMBER > 0
              ACTION_ITEM(MSG_PREHEAT_1_N MSG_H5, lcd_preheat_m1_h4);
              ACTION_ITEM(MSG_PREHEAT_1_END " " MSG_H5, lcd_preheat_m1_h4_only);
            #else
              ACTION_ITEM(MSG_PREHEAT_1_N MSG_H5, lcd_preheat_m1_h4_only);
            #endif
            #if HOTENDS > 5
              #if MAX_BED > 0 || MAX_CHAMBER > 0
                ACTION_ITEM(MSG_PREHEAT_1_N MSG_H6, lcd_preheat_m1_h5);
                ACTION_ITEM(MSG_PREHEAT_1_END " " MSG_H6, lcd_preheat_m1_h5_only);
              #else
                ACTION_ITEM(MSG_PREHEAT_1_N MSG_H6, lcd_preheat_m1_h5_only);
              #endif
            #endif // HOTENDS > 5
          #endif // HOTENDS > 4
        #endif // HOTENDS > 3
      #endif // HOTENDS > 2
      ACTION_ITEM(MSG_PREHEAT_1_ALL, lcd_preheat_m1_all);
    #endif // HOTENDS > 1
    END_MENU();
  }

  void menu_preheat_m2() {
    START_MENU();
    BACK_ITEM(MSG_TEMPERATURE);
    #if HOTENDS == 1
      #if MAX_BED > 0 || MAX_CHAMBER > 0
        ACTION_ITEM(MSG_PREHEAT_2, lcd_preheat_m2_h0);
        ACTION_ITEM(MSG_PREHEAT_2_END, lcd_preheat_m2_h0_only);
      #else
        ACTION_ITEM(MSG_PREHEAT_2, lcd_preheat_m2_h0_only);
      #endif
    #elif HOTENDS > 1
      #if MAX_BED > 0 || MAX_CHAMBER > 0
        ACTION_ITEM(MSG_PREHEAT_2_N MSG_H1, lcd_preheat_m2_h0);
        ACTION_ITEM(MSG_PREHEAT_2_END " " MSG_H1, lcd_preheat_m2_h0_only);
        ACTION_ITEM(MSG_PREHEAT_2_N MSG_H2, lcd_preheat_m2_h1);
        ACTION_ITEM(MSG_PREHEAT_2_END " " MSG_H2, lcd_preheat_m2_h1_only);
      #else
        ACTION_ITEM(MSG_PREHEAT_2_N MSG_H1, lcd_preheat_m2_h0_only);
        ACTION_ITEM(MSG_PREHEAT_2_N MSG_H2, lcd_preheat_m2_h1_only);
      #endif
      #if HOTENDS > 2
        #if MAX_BED > 0 || MAX_CHAMBER > 0
          ACTION_ITEM(MSG_PREHEAT_2_N MSG_H3, lcd_preheat_m2_h2);
          ACTION_ITEM(MSG_PREHEAT_2_END " " MSG_H3, lcd_preheat_m2_h2_only);
        #else
          ACTION_ITEM(MSG_PREHEAT_2_N MSG_H3, lcd_preheat_m2_h2_only);
        #endif
        #if HOTENDS > 3
          #if MAX_BED > 0 || MAX_CHAMBER > 0
            ACTION_ITEM(MSG_PREHEAT_2_N MSG_H4, lcd_preheat_m2_h3);
            ACTION_ITEM(MSG_PREHEAT_2_END " " MSG_H4, lcd_preheat_m2_h3_only);
          #else
            ACTION_ITEM(MSG_PREHEAT_2_N MSG_H4, lcd_preheat_m2_h3_only);
          #endif
          #if HOTENDS > 4
            #if MAX_BED > 0 || MAX_CHAMBER > 0
              ACTION_ITEM(MSG_PREHEAT_2_N MSG_H5, lcd_preheat_m2_h4);
              ACTION_ITEM(MSG_PREHEAT_2_END " " MSG_H5, lcd_preheat_m2_h4_only);
            #else
              ACTION_ITEM(MSG_PREHEAT_2_N MSG_H5, lcd_preheat_m2_h4_only);
            #endif
            #if HOTENDS > 5
              #if MAX_BED > 0 || MAX_CHAMBER > 0
                ACTION_ITEM(MSG_PREHEAT_2_N MSG_H6, lcd_preheat_m2_h5);
                ACTION_ITEM(MSG_PREHEAT_2_END " " MSG_H6, lcd_preheat_m2_h5_only);
              #else
                ACTION_ITEM(MSG_PREHEAT_2_N MSG_H6, lcd_preheat_m2_h5_only);
              #endif
            #endif // HOTENDS > 5
          #endif // HOTENDS > 4
        #endif // HOTENDS > 3
      #endif // HOTENDS > 2
      ACTION_ITEM(MSG_PREHEAT_2_ALL, lcd_preheat_m2_all);
    #endif
    END_MENU();
  }

  void menu_preheat_m3() {
    START_MENU();
    BACK_ITEM(MSG_TEMPERATURE);
    #if HOTENDS == 1
      #if MAX_BED > 0 || MAX_CHAMBER > 0
        ACTION_ITEM(MSG_PREHEAT_3, lcd_preheat_m3_h0);
        ACTION_ITEM(MSG_PREHEAT_3_END, lcd_preheat_m3_h0_only);
      #else
        ACTION_ITEM(MSG_PREHEAT_3, lcd_preheat_m3_h0_only);
      #endif
    #elif HOTENDS > 1
      #if MAX_BED > 0 || MAX_CHAMBER > 0
        ACTION_ITEM(MSG_PREHEAT_3_N MSG_H1, lcd_preheat_m3_h0);
        ACTION_ITEM(MSG_PREHEAT_3_END " " MSG_H1, lcd_preheat_m3_h0_only);
        ACTION_ITEM(MSG_PREHEAT_3_N MSG_H2, lcd_preheat_m3_h1);
        ACTION_ITEM(MSG_PREHEAT_3_END " " MSG_H2, lcd_preheat_m3_h1_only);
      #else
        ACTION_ITEM(MSG_PREHEAT_3_N MSG_H1, lcd_preheat_m3_h0_only);
        ACTION_ITEM(MSG_PREHEAT_3_N MSG_H2, lcd_preheat_m3_h1_only);
      #endif
      #if HOTENDS > 2
        #if MAX_BED > 0 || MAX_CHAMBER > 0
          ACTION_ITEM(MSG_PREHEAT_3_N MSG_H3, lcd_preheat_m3_h2);
          ACTION_ITEM(MSG_PREHEAT_3_END " " MSG_H3, lcd_preheat_m3_h2_only);
        #else
          ACTION_ITEM(MSG_PREHEAT_3_N MSG_H3, lcd_preheat_m3_h2_only);
        #endif
        #if HOTENDS > 3
          #if MAX_BED > 0 || MAX_CHAMBER > 0
            ACTION_ITEM(MSG_PREHEAT_2_N MSG_H4, lcd_preheat_m3_h3);
            ACTION_ITEM(MSG_PREHEAT_2_END " " MSG_H4, lcd_preheat_m3_h3_only);
          #else
            ACTION_ITEM(MSG_PREHEAT_2_N MSG_H4, lcd_preheat_m3_h3_only);
          #endif
          #if HOTENDS > 4
            #if MAX_BED > 0 || MAX_CHAMBER > 0
              ACTION_ITEM(MSG_PREHEAT_2_N MSG_H5, lcd_preheat_m3_h4);
              ACTION_ITEM(MSG_PREHEAT_2_END " " MSG_H5, lcd_preheat_m3_h4_only);
            #else
              ACTION_ITEM(MSG_PREHEAT_2_N MSG_H5, lcd_preheat_m3_h4_only);
            #endif
            #if HOTENDS > 5
              #if MAX_BED > 0 || MAX_CHAMBER > 0
                ACTION_ITEM(MSG_PREHEAT_2_N MSG_H6, lcd_preheat_m3_h5);
                ACTION_ITEM(MSG_PREHEAT_2_END " " MSG_H6, lcd_preheat_m3_h5_only);
              #else
                ACTION_ITEM(MSG_PREHEAT_2_N MSG_H6, lcd_preheat_m3_h5_only);
              #endif
            #endif // HOTENDS > 5
          #endif // HOTENDS > 4
        #endif // HOTENDS > 3
      #endif // HOTENDS > 2
      ACTION_ITEM(MSG_PREHEAT_3_ALL, lcd_preheat_m3_all);
    #endif
    END_MENU();
  }

  void lcd_cooldown() {
    thermalManager.disable_all_heaters();
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
  #if MAX_HOTEND > 0
    LOOP_HOTEND()
      EDIT_ITEM_FAST_INDEX(int3, MSG_NOZZLE, h, &hotends[h]->target_temperature, 0, hotends[h]->data.temp.max - 10, watch_temp_callback_hotend);
  #endif

  //
  // Bed:
  //
  #if MAX_BED > 0
    LOOP_BED()
      EDIT_ITEM_FAST_INDEX(int3, MSG_BED, h, &beds[h]->target_temperature, 0, beds[h]->data.temp.max - 10, watch_temp_callback_bed);
  #endif

  //
  // Chamber:
  //
  #if MAX_CHAMBER > 0
    LOOP_CHAMBER()
      EDIT_ITEM_FAST_INDEX(int3, MSG_CHAMBER, h, &chambers[h]->target_temperature, 0, chambers[h]->data.temp.max - 10, watch_temp_callback_chamber);
  #endif

  //
  // Cooler:
  //
  #if MAX_COOLER > 0
    EDIT_ITEM_FAST_INDEX(int3, MSG_COOLER, NULL, &coolers[0]->target_temperature, 0, coolers[0]->data.temp.max - 10, watch_temp_callback_cooler);
  #endif

  //
  // Fan Speed:
  //
  #if MAX_FAN > 0
    LOOP_FAN()
      EDIT_ITEM_FAST_INDEX(percent, MSG_FAN_SPEED, f, &fans[f]->speed, 0, 255);
  #endif

  #if MAX_HOTEND > 0

    //
    // Preheat for Material 1, 2 and 3
    //
    #if MAX_BED > 0
      SUBMENU(MSG_PREHEAT_1, menu_preheat_m1);
      SUBMENU(MSG_PREHEAT_2, menu_preheat_m2);
      SUBMENU(MSG_PREHEAT_3, menu_preheat_m3);
    #else
      ACTION_ITEM(MSG_PREHEAT_1, lcd_preheat_m1_h0_only);
      ACTION_ITEM(MSG_PREHEAT_2, lcd_preheat_m2_h0_only);
      ACTION_ITEM(MSG_PREHEAT_3, lcd_preheat_m3_h0_only);
    #endif

    //
    // Cooldown
    //
    bool has_heat = false;
    LOOP_HOTEND() if (hotends[h]->deg_target()) { has_heat = true; break; }
    #if MAX_BED > 0
      LOOP_BED() if (beds[h]->deg_target()) { has_heat = true; break; }
    #endif
    if (has_heat) ACTION_ITEM(MSG_COOLDOWN, lcd_cooldown);

  #endif // MAX_HOTEND > 0

  END_MENU();
}

#endif // HAS_LCD_MENU
