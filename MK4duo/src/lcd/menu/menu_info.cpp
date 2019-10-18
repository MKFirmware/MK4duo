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
// Info Menu
//

#include "../../../MK4duo.h"

#if HAS_LCD_MENU && ENABLED(LCD_INFO_MENU)

#define STATIC_PAIR(MSG, VALUE, STYL)     do{ strcpy_P(buffer, PSTR(": ")); strcpy(buffer + 2, VALUE);          STATIC_ITEM(MSG, STYL, buffer); }while(0)
#define STATIC_PAIR_P(MSG, PVALUE, STYL)  do{ strcpy_P(buffer, PSTR(": ")); strcpy_P(buffer + 2, PSTR(PVALUE)); STATIC_ITEM(MSG, STYL, buffer); }while(0)

void menu_info_stats() {
  if (lcdui.use_click()) return lcdui.goto_previous_screen();

  char buffer[21];  // for STATIC_PAIR_P

  printStatistics stats = print_job_counter.getStats();

  START_SCREEN();
  STATIC_PAIR(MSG_INFO_PRINT_COUNT, i16tostr3left(stats.totalPrints), SS_LEFT);
  STATIC_PAIR(MSG_INFO_COMPLETED_PRINTS, i16tostr3left(stats.finishedPrints), SS_LEFT);

  STATIC_PAIR_P(MSG_INFO_PRINT_TIME, "", SS_LEFT);  
  STATIC_ITEM(">", SS_LEFT, duration_t(stats.timePrint).toString(buffer));

  STATIC_PAIR_P(MSG_INFO_PRINT_LONGEST, "", SS_LEFT);
  STATIC_ITEM(">", SS_LEFT, duration_t(stats.longestPrint).toString(buffer));

  STATIC_PAIR_P(MSG_INFO_POWER_ON, "", SS_LEFT);
  STATIC_ITEM(">", SS_LEFT, duration_t(stats.timePowerOn).toString(buffer));

  ftostrlength(buffer, stats.filamentUsed);
  STATIC_PAIR_P(MSG_INFO_PRINT_FILAMENT, "", SS_LEFT); 
  STATIC_ITEM(">", SS_LEFT, buffer);

  #if HAS_POWER_CONSUMPTION_SENSOR
    sprintf_P(buffer, PSTR("%uWh"), stats.consumptionHour);
    STATIC_PAIR_P(MSG_INFO_PWRCONSUMED, "", SS_LEFT); 
    STATIC_ITEM(">", SS_LEFT, buffer);
  #endif

  #if ENABLED(SERVICE_TIME_1)
    STATIC_ITEM(SERVICE_NAME_1 " in: ", SS_LEFT);
    STATIC_ITEM(">", SS_LEFT, duration_t(stats.ServiceTime1).toString(buffer));
  #endif

  #if ENABLED(SERVICE_TIME_2)
    STATIC_ITEM(SERVICE_NAME_2 " in: ", SS_LEFT);
    STATIC_ITEM("> ", SS_LEFT, duration_t(stats.ServiceTime2).toString(buffer));
  #endif

  #if ENABLED(SERVICE_TIME_3)
    STATIC_ITEM(SERVICE_NAME_3 " in: ", SS_LEFT);
    STATIC_ITEM("> ", SS_LEFT, duration_t(stats.ServiceTime3).toString(buffer));
  #endif

  END_SCREEN();
}

/**
 *
 * About Printer > Thermistors
 *
 */
void menu_info_thermistors() {
  if (lcdui.use_click()) return lcdui.goto_previous_screen();

  char buffer[21];  // for STATIC_PAIR_P

  START_SCREEN();

  #if MAX_HOTEND > 0
    LOOP_HOTEND() {
      STATIC_ITEM_I("Hotend", h, SS_INVERT);
      STATIC_PAIR(MSG_INFO_MIN_TEMP, i16tostr3left(hotends[h]->data.temp.min), SS_LEFT);
      STATIC_PAIR(MSG_INFO_MAX_TEMP, i16tostr3left(hotends[h]->data.temp.max), SS_LEFT);
    }
  #endif

  #if MAX_BED > 0
    LOOP_BED() {
      STATIC_ITEM_I("Bed", h, SS_INVERT);
      STATIC_PAIR(MSG_INFO_MIN_TEMP, i16tostr3left(beds[h]->data.temp.min), SS_LEFT);
      STATIC_PAIR(MSG_INFO_MAX_TEMP, i16tostr3left(beds[h]->data.temp.max), SS_LEFT);
    }
  #endif

  #if MAX_CHAMBER > 0
    LOOP_CHAMBER() {
      STATIC_ITEM_I("Chamber", h, SS_INVERT);
      STATIC_PAIR(MSG_INFO_MIN_TEMP, i16tostr3left(chambers[h]->data.temp.min), SS_LEFT);
      STATIC_PAIR(MSG_INFO_MAX_TEMP, i16tostr3left(chambers[h]->data.temp.max), SS_LEFT);
    }
  #endif

  END_SCREEN();
}

/**
 *
 * About Printer > Board Info
 *
 */
void menu_info_board() {
  if (lcdui.use_click()) return lcdui.goto_previous_screen();

  char buffer[21];  // for STATIC_PAIR_P

  START_SCREEN();
  STATIC_ITEM(BOARD_NAME, SS_CENTER|SS_INVERT);                       // Board
  STATIC_PAIR_P(MSG_INFO_BAUDRATE, STRINGIFY(BAUDRATE_1), SS_CENTER); // Baud: 250000
  STATIC_PAIR_P(MSG_INFO_PROTOCOL, PROTOCOL_VERSION, SS_CENTER);      // Protocol: 2.0
  STATIC_PAIR_P(MSG_INFO_PSU, POWER_NAME, SS_CENTER);                 // Power Supply: Normal
  END_SCREEN();
}

/**
 *
 * About Printer > Firmware Info
 *
 */
void menu_info_firmware() {
  if (lcdui.use_click()) return lcdui.goto_previous_screen();
  START_SCREEN();
  STATIC_ITEM(FIRMWARE_NAME, SS_CENTER|SS_INVERT);
  STATIC_ITEM(SHORT_BUILD_VERSION);
  STATIC_ITEM(STRING_REVISION_DATE);
  STATIC_ITEM(MACHINE_NAME);
  STATIC_ITEM(MK4DUO_FIRMWARE_URL);
  STATIC_ITEM(MSG_INFO_EXTRUDERS ":", tools.data.extruders);
  STATIC_ITEM(MSG_INFO_HOTENDS ":", tools.data.hotends);
  STATIC_ITEM(MSG_INFO_BEDS ":", tools.data.beds);
  STATIC_ITEM(MSG_INFO_CHAMBERS ":", tools.data.chambers);
  #if ENABLED(AUTO_BED_LEVELING_3POINT)
    STATIC_ITEM(MSG_3POINT_LEVELING);     // 3-Point Leveling
  #elif ENABLED(AUTO_BED_LEVELING_LINEAR)
    STATIC_ITEM(MSG_LINEAR_LEVELING);     // Linear Leveling
  #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
    STATIC_ITEM(MSG_BILINEAR_LEVELING);   // Bi-linear Leveling
  #elif ENABLED(AUTO_BED_LEVELING_UBL)
    STATIC_ITEM(MSG_UBL_LEVELING);        // Unified Bed Leveling
  #elif ENABLED(MESH_BED_LEVELING)
    STATIC_ITEM(MSG_MESH_LEVELING);       // Mesh Leveling
  #endif
  END_SCREEN();
}

/**
 *
 * "About Printer" submenu
 *
 */
void menu_info() {
  START_MENU();
  BACK_ITEM(MSG_MAIN);
  SUBMENU(MSG_INFO_FIRMWARE_MENU, menu_info_firmware);       // Printer Info >
  SUBMENU(MSG_INFO_BOARD_MENU, menu_info_board);             // Board Info >
  SUBMENU(MSG_INFO_THERMISTOR_MENU, menu_info_thermistors);  // Thermistors >
  SUBMENU(MSG_INFO_STATS_MENU, menu_info_stats);             // Printer Statistics >
  END_MENU();
}

#endif // HAS_LCD_MENU && LCD_INFO_MENU
