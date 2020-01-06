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
// Info Menu
//

#include "../../../MK4duo.h"

#if HAS_LCD_MENU && ENABLED(LCD_INFO_MENU)

#if HAS_DHT
  void menu_dht();
#endif

#define VALUE_ITEM(MSG, VALUE, STYL)    do{ strcpy_P(buffer, PSTR(": ")); strcpy(buffer + 2, VALUE);          STATIC_ITEM(MSG, STYL, buffer); }while(0)
#define VALUE_ITEM_P(MSG, PVALUE, STYL) do{ strcpy_P(buffer, PSTR(": ")); strcpy_P(buffer + 2, PSTR(PVALUE)); STATIC_ITEM(MSG, STYL, buffer); }while(0)

void menu_info_stats() {
  if (lcdui.use_click()) return lcdui.goto_previous_screen();

  char buffer[21];

  printStatistics stats = print_job_counter.getStats();

  START_SCREEN();
  VALUE_ITEM(MSG_INFO_PRINT_COUNT, i16tostr3left(stats.totalPrints), SS_LEFT);
  VALUE_ITEM(MSG_INFO_COMPLETED_PRINTS, i16tostr3left(stats.finishedPrints), SS_LEFT);

  STATIC_ITEM(MSG_INFO_PRINT_TIME, SS_LEFT);  
  STATIC_ITEM_P(">", SS_LEFT, duration_t(stats.timePrint).toString(buffer));

  STATIC_ITEM(MSG_INFO_PRINT_LONGEST, SS_LEFT);
  STATIC_ITEM_P(">", SS_LEFT, duration_t(stats.longestPrint).toString(buffer));

  STATIC_ITEM(MSG_INFO_POWER_ON, SS_LEFT);
  STATIC_ITEM_P(">", SS_LEFT, duration_t(stats.timePowerOn).toString(buffer));

  STATIC_ITEM(MSG_INFO_PRINT_FILAMENT, SS_LEFT);
  sprintf_P(buffer, PSTR("%ld.%im"), long(stats.filamentUsed / 1000), int16_t(stats.filamentUsed / 100) % 10);
  STATIC_ITEM_P(">", SS_LEFT, buffer);

  #if HAS_POWER_CONSUMPTION_SENSOR
    sprintf_P(buffer, PSTR("%uWh"), stats.consumptionHour);
    STATIC_ITEM(MSG_INFO_PWRCONSUMED, SS_LEFT); 
    STATIC_ITEM_P(">", SS_LEFT, buffer);
  #endif

  #if SERVICE_INTERVAL_1 > 0 || SERVICE_INTERVAL_2 > 0 || SERVICE_INTERVAL_3 > 0
    strcpy_P(buffer, GET_TEXT(MSG_SERVICE_IN));
  #endif

  #if ENABLED(SERVICE_TIME_1)
    STATIC_ITEM_P(PSTR(SERVICE_NAME_1 " "), SS_LEFT, buffer);
    STATIC_ITEM_P(PSTR("> "), SS_LEFT, duration_t(stats.ServiceTime1).toString(buffer));
  #endif

  #if ENABLED(SERVICE_TIME_2)
    STATIC_ITEM_P(PSTR(SERVICE_NAME_2 " "), SS_LEFT, buffer);
    STATIC_ITEM_P(PSTR("> "), SS_LEFT, duration_t(stats.ServiceTime2).toString(buffer));
  #endif

  #if ENABLED(SERVICE_TIME_3)
    STATIC_ITEM_P(PSTR(SERVICE_NAME_3 " "), SS_LEFT, buffer);
    STATIC_ITEM_P(PSTR("> "), SS_LEFT, duration_t(stats.ServiceTime3).toString(buffer));
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

  char buffer[21];  // for VALUE_ITEM_P

  START_SCREEN();

  #if HAS_HOTENDS
    LOOP_HOTEND() {
      STATIC_ITEM_P(PSTR("Hotend"), SS_INVERT);
      VALUE_ITEM(MSG_INFO_MIN_TEMP, i16tostr3left(hotends[h]->data.temp.min), SS_LEFT);
      VALUE_ITEM(MSG_INFO_MAX_TEMP, i16tostr3left(hotends[h]->data.temp.max), SS_LEFT);
    }
  #endif

  #if HAS_BEDS
    LOOP_BED() {
      STATIC_ITEM_P(PSTR("Bed"), SS_INVERT);
      VALUE_ITEM(MSG_INFO_MIN_TEMP, i16tostr3left(beds[h]->data.temp.min), SS_LEFT);
      VALUE_ITEM(MSG_INFO_MAX_TEMP, i16tostr3left(beds[h]->data.temp.max), SS_LEFT);
    }
  #endif

  #if HAS_CHAMBERS
    LOOP_CHAMBER() {
      STATIC_ITEM_P(PSTR("Chamber"), SS_INVERT);
      VALUE_ITEM(MSG_INFO_MIN_TEMP, i16tostr3left(chambers[h]->data.temp.min), SS_LEFT);
      VALUE_ITEM(MSG_INFO_MAX_TEMP, i16tostr3left(chambers[h]->data.temp.max), SS_LEFT);
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

  char buffer[21];  // for VALUE_ITEM

  START_SCREEN();
  STATIC_ITEM_P(PSTR(BOARD_NAME), SS_CENTER|SS_INVERT);             // Board
  VALUE_ITEM(MSG_INFO_BAUDRATE, STRINGIFY(BAUDRATE_1), SS_CENTER);  // Baud: 250000
  VALUE_ITEM(MSG_INFO_PROTOCOL, PROTOCOL_VERSION, SS_CENTER);       // Protocol: 2.0
  VALUE_ITEM(MSG_INFO_PSU, POWER_NAME, SS_CENTER);                  // Power Supply: Normal
  END_SCREEN();
}

/**
 *
 * About Printer > Firmware Info
 *
 */
void menu_info_firmware() {
  if (lcdui.use_click()) return lcdui.goto_previous_screen();

  char buffer[21];  // for VALUE_ITEM

  START_SCREEN();
  STATIC_ITEM_P(PSTR(FIRMWARE_NAME), SS_CENTER|SS_INVERT);
  STATIC_ITEM_P(PSTR(SHORT_BUILD_VERSION));
  STATIC_ITEM_P(PSTR(STRING_REVISION_DATE));
  STATIC_ITEM_P(PSTR(MACHINE_NAME));
  STATIC_ITEM_P(PSTR(MK4DUO_FIRMWARE_URL));
  VALUE_ITEM(MSG_INFO_EXTRUDERS, ui8tostr3(toolManager.extruder.total), SS_CENTER);
  VALUE_ITEM(MSG_INFO_HOTENDS, ui8tostr3(tempManager.heater.hotends), SS_CENTER);
  VALUE_ITEM(MSG_INFO_BEDS, ui8tostr3(tempManager.heater.beds), SS_CENTER);
  VALUE_ITEM(MSG_INFO_CHAMBERS, ui8tostr3(tempManager.heater.chambers), SS_CENTER);
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
  SUBMENU(MSG_INFO_FIRMWARE_MENU, menu_info_firmware);      // Printer Info >
  SUBMENU(MSG_INFO_BOARD_MENU, menu_info_board);            // Board Info >
  SUBMENU(MSG_INFO_THERMISTOR_MENU, menu_info_thermistors); // Thermistors >
  #if HAS_DHT
    SUBMENU(MSG_DHT, menu_dht);                             // DHT >
  #endif
  SUBMENU(MSG_INFO_STATS_MENU, menu_info_stats);            // Printer Statistics >
  END_MENU();
}

#endif // HAS_LCD_MENU && LCD_INFO_MENU
