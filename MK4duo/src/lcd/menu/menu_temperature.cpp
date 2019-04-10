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
// Temperature Menu
//

#include "../../../MK4duo.h"

#if HAS_LCD_MENU

// Initialized by settings.load()
#if HOTENDS > 0
  int16_t LcdUI::preheat_hotend_temp[3];
#endif
#if BEDS > 0
  int16_t LcdUI::preheat_bed_temp[3];
#endif
#if CHAMBERS > 0
  int16_t LcdUI::preheat_chamber_temp[3];
#endif
#if FAN_COUNT > 0
  int16_t LcdUI::preheat_fan_speed[3];
#endif

//
// "Temperature" submenu items
//

void _lcd_preheat(const int16_t hotend, const uint8_t memory, const bool only_hotend) {
  #if HOTENDS > 0
    hotends[hotend].setTarget(MIN(thermalManager.hotend_maxtemp_all(), lcdui.preheat_hotend_temp[memory]));
  #else
    UNUSED(hotend);
  #endif
  if (!only_hotend) {
    #if BEDS > 0
      LOOP_BED() beds[h].setTarget(lcdui.preheat_bed_temp[memory]);
    #endif
    #if CHAMBERS > 0
      LOOP_CHAMBER() chambers[h].setTarget(lcdui.preheat_chamber_temp[memory]);
    #endif
  }
  #if FAN_COUNT > 0
    #if FAN_COUNT > 1
      fans[tools.active_extruder < FAN_COUNT ? tools.active_extruder : 0].speed = lcdui.preheat_fan_speed[memory];
    #else
      fans[0].speed = lcdui.preheat_fan_speed[memory];
    #endif
  #endif
  lcdui.return_to_status();
}

#if HOTENDS > 1
  void lcd_preheat_m1_h1_only() { _lcd_preheat(1, 0, true); }
  void lcd_preheat_m2_h1_only() { _lcd_preheat(1, 1, true); }
  void lcd_preheat_m3_h1_only() { _lcd_preheat(1, 2, true); }
  #if BEDS > 0 || CHAMBERS > 0
    void lcd_preheat_m1_h1() { _lcd_preheat(1, 0, false); }
    void lcd_preheat_m2_h1() { _lcd_preheat(1, 1, false); }
    void lcd_preheat_m3_h1() { _lcd_preheat(1, 2, false); }
  #endif
  #if HOTENDS > 2
    void lcd_preheat_m1_h2_only() { _lcd_preheat(2, 0, true); }
    void lcd_preheat_m2_h2_only() { _lcd_preheat(2, 1, true); }
    void lcd_preheat_m3_h2_only() { _lcd_preheat(2, 2, true); }
    #if BEDS > 0 || CHAMBERS > 0
      void lcd_preheat_m1_h2() { _lcd_preheat(2, 0, false); }
      void lcd_preheat_m2_h2() { _lcd_preheat(2, 1, false); }
      void lcd_preheat_m3_h2() { _lcd_preheat(2, 2, false); }
    #endif
    #if HOTENDS > 3
      void lcd_preheat_m1_h3_only() { _lcd_preheat(3, 0, true); }
      void lcd_preheat_m2_h3_only() { _lcd_preheat(3, 1, true); }
      void lcd_preheat_m3_h3_only() { _lcd_preheat(3, 2, true); }
      #if BEDS > 0 || CHAMBERS > 0
        void lcd_preheat_m1_h3() { _lcd_preheat(3, 0, false); }
        void lcd_preheat_m2_h3() { _lcd_preheat(3, 1, false); }
        void lcd_preheat_m3_h3() { _lcd_preheat(3, 2, false); }
      #endif
      #if HOTENDS > 4
        void lcd_preheat_m1_h4_only() { _lcd_preheat(4, 0, true); }
        void lcd_preheat_m2_h4_only() { _lcd_preheat(4, 1, true); }
        void lcd_preheat_m3_h4_only() { _lcd_preheat(4, 2, true); }
        #if BEDS > 0 || CHAMBERS > 0
          void lcd_preheat_m1_h4() { _lcd_preheat(4, 0, false); }
          void lcd_preheat_m2_h4() { _lcd_preheat(4, 1, false); }
          void lcd_preheat_m3_h4() { _lcd_preheat(4, 2, false); }
        #endif
        #if HOTENDS > 5
          void lcd_preheat_m1_h5_only() { _lcd_preheat(5, 0, true); }
          void lcd_preheat_m2_h5_only() { _lcd_preheat(5, 1, true); }
          void lcd_preheat_m3_h5_only() { _lcd_preheat(5, 2, true); }
          #if BEDS > 0 || CHAMBERS > 0
            void lcd_preheat_m1_h5() { _lcd_preheat(5, 0, false); }
            void lcd_preheat_m2_h5() { _lcd_preheat(5, 1, false); }
            void lcd_preheat_m3_h5() { _lcd_preheat(5, 2, false); }
          #endif
        #endif // HOTENDS > 5
      #endif // HOTENDS > 4
    #endif // HOTENDS > 3
  #endif // HOTENDS > 2

  #if BEDS > 0 || CHAMBERS > 0
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
      LOOP_HOTEND() hotends[h].setTarget(lcdui.preheat_hotend_temp[0]);
    #endif
    #if BEDS > 0 || CHAMBERS > 0
      lcd_preheat_m1_h0();
    #else
      lcd_preheat_m1_h0_only();
    #endif
  }
  void lcd_preheat_m2_all() {
    #if HOTENDS > 1
      LOOP_HOTEND() hotends[h].setTarget(lcdui.preheat_hotend_temp[1]);
    #endif
    #if BEDS > 0 || CHAMBERS > 0
      lcd_preheat_m2_h0();
    #else
      lcd_preheat_m2_h0_only();
    #endif
  }
  void lcd_preheat_m3_all() {
    #if HOTENDS > 1
      LOOP_HOTEND() hotends[h].setTarget(lcdui.preheat_hotend_temp[2]);
    #endif
    #if BEDS > 0 || CHAMBERS > 0
      lcd_preheat_m3_h0();
    #else
      lcd_preheat_m3_h0_only();
    #endif
  }

#endif // HOTENDS > 1

#if HOTENDS >0 || BEDS > 0

  void lcd_preheat_m1_h0_only() { _lcd_preheat(0, 0, true); }
  void lcd_preheat_m2_h0_only() { _lcd_preheat(0, 1, true); }
  void lcd_preheat_m3_h0_only() { _lcd_preheat(0, 2, true); }

  #if BEDS > 0 || CHAMBERS > 0
    void lcd_preheat_m1_h0() { _lcd_preheat(0, 0, false); }
    void lcd_preheat_m2_h0() { _lcd_preheat(0, 1, false); }
    void lcd_preheat_m3_h0() { _lcd_preheat(0, 2, false); }
  #endif

  void menu_preheat_m1() {
    START_MENU();
    MENU_BACK(MSG_TEMPERATURE);
    #if HOTENDS == 1
      #if BEDS > 0 || CHAMBERS > 0
        MENU_ITEM(function, MSG_PREHEAT_1, lcd_preheat_m1_h0);
        MENU_ITEM(function, MSG_PREHEAT_1_END, lcd_preheat_m1_h0_only);
      #else
        MENU_ITEM(function, MSG_PREHEAT_1, lcd_preheat_m1_h0_only);
      #endif
    #elif HOTENDS > 1
      #if BEDS > 0 || CHAMBERS > 0
        MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H1, lcd_preheat_m1_h0);
        MENU_ITEM(function, MSG_PREHEAT_1_END " " MSG_H1, lcd_preheat_m1_h0_only);
        MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H2, lcd_preheat_m1_h1);
        MENU_ITEM(function, MSG_PREHEAT_1_END " " MSG_H2, lcd_preheat_m1_h1_only);
      #else
        MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H1, lcd_preheat_m1_h0_only);
        MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H2, lcd_preheat_m1_h1_only);
      #endif
      #if HOTENDS > 2
        #if BEDS > 0 || CHAMBERS > 0
          MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H3, lcd_preheat_m1_h2);
          MENU_ITEM(function, MSG_PREHEAT_1_END " " MSG_H3, lcd_preheat_m1_h2_only);
        #else
          MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H3, lcd_preheat_m1_h2_only);
        #endif
        #if HOTENDS > 3
          #if BEDS > 0 || CHAMBERS > 0
            MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H4, lcd_preheat_m1_h3);
            MENU_ITEM(function, MSG_PREHEAT_1_END " " MSG_H4, lcd_preheat_m1_h3_only);
          #else
            MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H4, lcd_preheat_m1_h3_only);
          #endif
          #if HOTENDS > 4
            #if BEDS > 0 || CHAMBERS > 0
              MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H5, lcd_preheat_m1_h4);
              MENU_ITEM(function, MSG_PREHEAT_1_END " " MSG_H5, lcd_preheat_m1_h4_only);
            #else
              MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H5, lcd_preheat_m1_h4_only);
            #endif
            #if HOTENDS > 5
              #if BEDS > 0 || CHAMBERS > 0
                MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H6, lcd_preheat_m1_h5);
                MENU_ITEM(function, MSG_PREHEAT_1_END " " MSG_H6, lcd_preheat_m1_h5_only);
              #else
                MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H6, lcd_preheat_m1_h5_only);
              #endif
            #endif // HOTENDS > 5
          #endif // HOTENDS > 4
        #endif // HOTENDS > 3
      #endif // HOTENDS > 2
      MENU_ITEM(function, MSG_PREHEAT_1_ALL, lcd_preheat_m1_all);
    #endif // HOTENDS > 1
    END_MENU();
  }

  void menu_preheat_m2() {
    START_MENU();
    MENU_BACK(MSG_TEMPERATURE);
    #if HOTENDS == 1
      #if BEDS > 0 || CHAMBERS > 0
        MENU_ITEM(function, MSG_PREHEAT_2, lcd_preheat_m2_h0);
        MENU_ITEM(function, MSG_PREHEAT_2_END, lcd_preheat_m2_h0_only);
      #else
        MENU_ITEM(function, MSG_PREHEAT_2, lcd_preheat_m2_h0_only);
      #endif
    #elif HOTENDS > 1
      #if BEDS > 0 || CHAMBERS > 0
        MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H1, lcd_preheat_m2_h0);
        MENU_ITEM(function, MSG_PREHEAT_2_END " " MSG_H1, lcd_preheat_m2_h0_only);
        MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H2, lcd_preheat_m2_h1);
        MENU_ITEM(function, MSG_PREHEAT_2_END " " MSG_H2, lcd_preheat_m2_h1_only);
      #else
        MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H1, lcd_preheat_m2_h0_only);
        MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H2, lcd_preheat_m2_h1_only);
      #endif
      #if HOTENDS > 2
        #if BEDS > 0 || CHAMBERS > 0
          MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H3, lcd_preheat_m2_h2);
          MENU_ITEM(function, MSG_PREHEAT_2_END " " MSG_H3, lcd_preheat_m2_h2_only);
        #else
          MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H3, lcd_preheat_m2_h2_only);
        #endif
        #if HOTENDS > 3
          #if BEDS > 0 || CHAMBERS > 0
            MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H4, lcd_preheat_m2_h3);
            MENU_ITEM(function, MSG_PREHEAT_2_END " " MSG_H4, lcd_preheat_m2_h3_only);
          #else
            MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H4, lcd_preheat_m2_h3_only);
          #endif
          #if HOTENDS > 4
            #if BEDS > 0 || CHAMBERS > 0
              MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H5, lcd_preheat_m2_h4);
              MENU_ITEM(function, MSG_PREHEAT_2_END " " MSG_H5, lcd_preheat_m2_h4_only);
            #else
              MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H5, lcd_preheat_m2_h4_only);
            #endif
            #if HOTENDS > 5
              #if BEDS > 0 || CHAMBERS > 0
                MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H6, lcd_preheat_m2_h5);
                MENU_ITEM(function, MSG_PREHEAT_2_END " " MSG_H6, lcd_preheat_m2_h5_only);
              #else
                MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H6, lcd_preheat_m2_h5_only);
              #endif
            #endif // HOTENDS > 5
          #endif // HOTENDS > 4
        #endif // HOTENDS > 3
      #endif // HOTENDS > 2
      MENU_ITEM(function, MSG_PREHEAT_2_ALL, lcd_preheat_m2_all);
    #endif
    END_MENU();
  }

  void menu_preheat_m3() {
    START_MENU();
    MENU_BACK(MSG_TEMPERATURE);
    #if HOTENDS == 1
      #if BEDS > 0 || CHAMBERS > 0
        MENU_ITEM(function, MSG_PREHEAT_3, lcd_preheat_m3_h0);
        MENU_ITEM(function, MSG_PREHEAT_3_END, lcd_preheat_m3_h0_only);
      #else
        MENU_ITEM(function, MSG_PREHEAT_3, lcd_preheat_m3_h0_only);
      #endif
    #elif HOTENDS > 1
      #if BEDS > 0 || CHAMBERS > 0
        MENU_ITEM(function, MSG_PREHEAT_3_N MSG_H1, lcd_preheat_m3_h0);
        MENU_ITEM(function, MSG_PREHEAT_3_END " " MSG_H1, lcd_preheat_m3_h0_only);
        MENU_ITEM(function, MSG_PREHEAT_3_N MSG_H2, lcd_preheat_m3_h1);
        MENU_ITEM(function, MSG_PREHEAT_3_END " " MSG_H2, lcd_preheat_m3_h1_only);
      #else
        MENU_ITEM(function, MSG_PREHEAT_3_N MSG_H1, lcd_preheat_m3_h0_only);
        MENU_ITEM(function, MSG_PREHEAT_3_N MSG_H2, lcd_preheat_m3_h1_only);
      #endif
      #if HOTENDS > 2
        #if BEDS > 0 || CHAMBERS > 0
          MENU_ITEM(function, MSG_PREHEAT_3_N MSG_H3, lcd_preheat_m3_h2);
          MENU_ITEM(function, MSG_PREHEAT_3_END " " MSG_H3, lcd_preheat_m3_h2_only);
        #else
          MENU_ITEM(function, MSG_PREHEAT_3_N MSG_H3, lcd_preheat_m3_h2_only);
        #endif
        #if HOTENDS > 3
          #if BEDS > 0 || CHAMBERS > 0
            MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H4, lcd_preheat_m3_h3);
            MENU_ITEM(function, MSG_PREHEAT_2_END " " MSG_H4, lcd_preheat_m3_h3_only);
          #else
            MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H4, lcd_preheat_m3_h3_only);
          #endif
          #if HOTENDS > 4
            #if BEDS > 0 || CHAMBERS > 0
              MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H5, lcd_preheat_m3_h4);
              MENU_ITEM(function, MSG_PREHEAT_2_END " " MSG_H5, lcd_preheat_m3_h4_only);
            #else
              MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H5, lcd_preheat_m3_h4_only);
            #endif
            #if HOTENDS > 5
              #if BEDS > 0 || CHAMBERS > 0
                MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H6, lcd_preheat_m3_h5);
                MENU_ITEM(function, MSG_PREHEAT_2_END " " MSG_H6, lcd_preheat_m3_h5_only);
              #else
                MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H6, lcd_preheat_m3_h5_only);
              #endif
            #endif // HOTENDS > 5
          #endif // HOTENDS > 4
        #endif // HOTENDS > 3
      #endif // HOTENDS > 2
      MENU_ITEM(function, MSG_PREHEAT_3_ALL, lcd_preheat_m3_all);
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
  MENU_BACK(MSG_MAIN);

  //
  // Nozzle:
  // Nozzle [1-4]:
  //
  #if HOTENDS == 1
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE, &hotends[0].target_temperature, 0, hotends[0].data.maxtemp - 10, watch_temp_callback_H0);
  #elif HOTENDS > 1
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N0, &hotends[0].target_temperature, 0, hotends[0].data.maxtemp - 10, watch_temp_callback_H0);
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N1, &hotends[1].target_temperature, 0, hotends[1].data.maxtemp - 10, watch_temp_callback_H1);
    #if HOTENDS > 2
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N2, &hotends[2].target_temperature, 0, hotends[2].data.maxtemp - 10, watch_temp_callback_H2);
      #if HOTENDS > 3
        MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N3, &hotends[3].target_temperature, 0, hotends[3].data.maxtemp - 10, watch_temp_callback_H3);
        #if HOTENDS > 4
          MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N4, &hotends[4].target_temperature, 0, hotends[4].data.maxtemp - 10, watch_temp_callback_E4);
          #if HOTENDS > 5
            MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N5, &hotends[5].target_temperature, 0, hotends[5].data.maxtemp - 10, watch_temp_callback_E5);
          #endif // HOTENDS > 5
        #endif // HOTENDS > 4
      #endif // HOTENDS > 3
    #endif // HOTENDS > 2
  #endif // HOTENDS > 1

  //
  // Bed:
  //
  #if BEDS == 1
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_BED, &beds[0].target_temperature, 0, beds[0].data.maxtemp - 10, watch_temp_callback_bed0);
  #elif BEDS > 1
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_BED MSG_N0, &beds[0].target_temperature, 0, beds[0].data.maxtemp - 10, watch_temp_callback_bed0);
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_BED MSG_N1, &beds[1].target_temperature, 0, beds[1].data.maxtemp - 10, watch_temp_callback_bed1);
    #if BEDS > 2
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_BED MSG_N2, &beds[2].target_temperature, 0, beds[2].data.maxtemp - 10, watch_temp_callback_bed2);
      #if BEDS > 3
        MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_BED MSG_N2, &beds[3].target_temperature, 0, beds[3].data.maxtemp - 10, watch_temp_callback_bed3);
      #endif // BEDS > 3
    #endif // BEDS > 2
  #endif // BEDS > 1

  //
  // Chamber:
  //
  #if CHAMBERS == 1
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_CHAMBER, &chambers[0].target_temperature, 0, chambers[0].data.maxtemp - 10, watch_temp_callback_chamber0);
  #elif CHAMBERS > 1
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_CHAMBER MSG_N0, &chambers[0].target_temperature, 0, chambers[0].data.maxtemp - 10, watch_temp_callback_chamber0);
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_CHAMBER MSG_N1, &chambers[1].target_temperature, 0, chambers[1].data.maxtemp - 10, watch_temp_callback_chamber1);
    #if CHAMBERS > 2
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_CHAMBER MSG_N2, &chambers[2].target_temperature, 0, chambers[2].data.maxtemp - 10, watch_temp_callback_chamber2);
      #if CHAMBERS > 3
        MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_CHAMBER MSG_N3, &chambers[3].target_temperature, 0, chambers[3].data.maxtemp - 10, watch_temp_callback_chamber3);
      #endif // CHAMBERS > 3
    #endif // CHAMBERS > 2
  #endif // CHAMBERS > 1

  //
  // Cooler:
  //
  #if COOLERS == 1
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_COOLER, &coolers[0].target_temperature, 0, coolers[0].data.maxtemp - 10, watch_temp_callback_cooler0);
  #endif

  //
  // Fan Speed:
  //
  #if FAN_COUNT > 0
    #if HAS_FAN0
      MENU_MULTIPLIER_ITEM_EDIT(percent, MSG_FAN_SPEED " 0", &fans[0].speed, 0, 255);
    #endif
    #if HAS_FAN1
      MENU_MULTIPLIER_ITEM_EDIT(percent, MSG_FAN_SPEED " 1", &fans[1].speed, 0, 255);
    #endif
    #if HAS_FAN2
      MENU_MULTIPLIER_ITEM_EDIT(percent, MSG_FAN_SPEED " 2", &fans[2].speed, 0, 255);
    #endif
    #if HAS_FAN3
      MENU_MULTIPLIER_ITEM_EDIT(percent, MSG_FAN_SPEED " 3", &fans[3].speed, 0, 255);
    #endif
    #if HAS_FAN4
      MENU_MULTIPLIER_ITEM_EDIT(percent, MSG_FAN_SPEED " 4", &fans[4].speed, 0, 255);
    #endif
    #if HAS_FAN5
      MENU_MULTIPLIER_ITEM_EDIT(percent, MSG_FAN_SPEED " 5", &fans[5].speed, 0, 255);
    #endif
  #endif // FAN_COUNT > 0

  if (printer.mode == PRINTER_MODE_FFF) {

    #if HAS_TEMP_HE0

      //
      // Cooldown
      //
      bool has_heat = false;
      #if HOTENDS > 0
        LOOP_HOTEND() if (hotends[h].target_temperature) { has_heat = true; break; }
      #endif
      if (has_heat) MENU_ITEM(function, MSG_COOLDOWN, lcd_cooldown);

      //
      // Preheat for Material 1, 2 and 3
      //
      #if HAS_TEMP_HE1 || HAS_TEMP_HE2 || HAS_TEMP_HE3 || HAS_TEMP_BED0
        MENU_ITEM(submenu, MSG_PREHEAT_1, menu_preheat_m1);
        MENU_ITEM(submenu, MSG_PREHEAT_2, menu_preheat_m2);
        MENU_ITEM(submenu, MSG_PREHEAT_3, menu_preheat_m3);
      #else
        MENU_ITEM(function, MSG_PREHEAT_1, lcd_preheat_m1_h0_only);
        MENU_ITEM(function, MSG_PREHEAT_2, lcd_preheat_m2_h0_only);
        MENU_ITEM(function, MSG_PREHEAT_3, lcd_preheat_m3_h0_only);
      #endif

    #endif // HAS_TEMP_HE0

  } // printer mode FFF

  END_MENU();
}

#endif // HAS_LCD_MENU
