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
// Info Menu
//

#include "../../../MK4duo.h"

#if HAS_LCD_MENU && ENABLED(LCD_INFO_MENU)

void menu_info_stats() {
  if (lcdui.use_click()) return lcdui.goto_previous_screen();

  uint16_t day, hours, minutes, kmeter, meter, centimeter;
  millis_t t;
  char lifeTime[20];
  char printTime[20];
  char Filamentlung[20];

  t       = print_job_counter.data.printer_usage / 60;
  day     = t / 60 / 24;
  hours   = (t / 60) % 24;
  minutes = t % 60;
  sprintf_P(lifeTime, PSTR("%ud %uh %um"), day, hours, minutes);

  t       = print_job_counter.data.printTime / 60;
  day     = t / 60 / 24;
  hours   = (t / 60) % 24;
  minutes = t % 60;
  sprintf_P(printTime, PSTR("%ud %uh %um"), day, hours, minutes);

  kmeter      = (long)print_job_counter.data.filamentUsed / 1000 / 1000;
  meter       = ((long)print_job_counter.data.filamentUsed / 1000) % 1000;
  centimeter  = ((long)print_job_counter.data.filamentUsed / 10) % 100;
  sprintf_P(Filamentlung, PSTR("%uKm %um %ucm"), kmeter, meter, centimeter);

  #if HAS_POWER_CONSUMPTION_SENSOR
    char Power[10];
    sprintf_P(Power, PSTR("%uWh"), powerManager.consumption_hour);
  #endif

  START_SCREEN();
  STATIC_ITEM(MSG_INFO_TOTAL_PRINTS ": ", false, false, itostr3left(print_job_counter.data.totalPrints));
  STATIC_ITEM(MSG_INFO_FINISHED_PRINTS ": ",  false, false, itostr3left(print_job_counter.data.finishedPrints));
  STATIC_ITEM(MSG_INFO_ON_TIME ": ",  false, false, lifeTime);
  STATIC_ITEM(MSG_INFO_PRINT_TIME ": ",  false, false, printTime);
  STATIC_ITEM(MSG_INFO_FILAMENT_USAGE ": ",  false, false, Filamentlung);
  #if HAS_POWER_CONSUMPTION_SENSOR
    STATIC_ITEM(MSG_INFO_PWRCONSUMED ": ",  false, false, Power);
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
  START_SCREEN();

  #if HAS_TEMP_0
    STATIC_ITEM("T0: " HOT0_NAME, false, true);
    STATIC_ITEM(MSG_INFO_MIN_TEMP ": " STRINGIFY(HEATER_0_MINTEMP), false);
    STATIC_ITEM(MSG_INFO_MAX_TEMP ": " STRINGIFY(HEATER_0_MAXTEMP), false);
  #endif

  #if HAS_TEMP_1
    STATIC_ITEM("T1: " HOT1_NAME, false, true);
    STATIC_ITEM(MSG_INFO_MIN_TEMP ": " STRINGIFY(HEATER_1_MINTEMP), false);
    STATIC_ITEM(MSG_INFO_MAX_TEMP ": " STRINGIFY(HEATER_1_MAXTEMP), false);
  #endif

  #if HAS_TEMP_2
    STATIC_ITEM("T2: " HOT2_NAME, false, true);
    STATIC_ITEM(MSG_INFO_MIN_TEMP ": " STRINGIFY(HEATER_2_MINTEMP), false);
    STATIC_ITEM(MSG_INFO_MAX_TEMP ": " STRINGIFY(HEATER_2_MAXTEMP), false);
  #endif

  #if HAS_TEMP_3
    STATIC_ITEM("T3: " HOT3_NAME, false, true);
    STATIC_ITEM(MSG_INFO_MIN_TEMP ": " STRINGIFY(HEATER_3_MINTEMP), false);
    STATIC_ITEM(MSG_INFO_MAX_TEMP ": " STRINGIFY(HEATER_3_MAXTEMP), false);
  #endif

  #if HAS_TEMP_BED
    STATIC_ITEM("TBed:" BED_NAME, false, true);
    STATIC_ITEM(MSG_INFO_MIN_TEMP ": " STRINGIFY(BED_MINTEMP), false);
    STATIC_ITEM(MSG_INFO_MAX_TEMP ": " STRINGIFY(BED_MAXTEMP), false);
  #endif

  #if HAS_TEMP_CHAMBER
    STATIC_ITEM("TChamber:" CHAMBER_NAME, false, true);
    STATIC_ITEM(MSG_INFO_MIN_TEMP ": " STRINGIFY(CHAMBER_MINTEMP), false);
    STATIC_ITEM(MSG_INFO_MAX_TEMP ": " STRINGIFY(CHAMBER_MAXTEMP), false);
  #endif

  #if HAS_TEMP_COOLER
    STATIC_ITEM("TCooler:" COOLER_NAME, false, true);
    STATIC_ITEM(MSG_INFO_MIN_TEMP ": " STRINGIFY(COOLER_MINTEMP), false);
    STATIC_ITEM(MSG_INFO_MAX_TEMP ": " STRINGIFY(COOLER_MAXTEMP), false);
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
  START_SCREEN();
  STATIC_ITEM(BOARD_NAME, true, true);                              // Board
  STATIC_ITEM(MSG_INFO_BAUDRATE ": " STRINGIFY(BAUDRATE_1), true);  // Baud: 250000
  STATIC_ITEM(MSG_INFO_PROTOCOL ": " PROTOCOL_VERSION, true);       // Protocol: 2.0
  #if (POWER_SUPPLY == 0)
    STATIC_ITEM(MSG_INFO_PSU ": Normal", true); // Power Supply: Normal
  #elif (POWER_SUPPLY == 1)
    STATIC_ITEM(MSG_INFO_PSU ": ATX", true);    // Power Supply: ATX
  #elif (POWER_SUPPLY == 2)
    STATIC_ITEM(MSG_INFO_PSU ": XBox", true);   // Power Supply: XBox
  #endif
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
  STATIC_ITEM(FIRMWARE_NAME, true, true);
  STATIC_ITEM("V" SHORT_BUILD_VERSION " R" FIRMWARE_REVISION, true);
  STATIC_ITEM(STRING_DISTRIBUTION_DATE, true);
  STATIC_ITEM(MACHINE_NAME, true);
  STATIC_ITEM(FIRMWARE_URL, true);
  STATIC_ITEM(MSG_INFO_EXTRUDERS ": " STRINGIFY(EXTRUDERS), true);
  STATIC_ITEM(MSG_INFO_HOTENDS ": " STRINGIFY(HOTENDS), true);
  #if ENABLED(AUTO_BED_LEVELING_3POINT)
    STATIC_ITEM(MSG_3POINT_LEVELING, true);     // 3-Point Leveling
  #elif ENABLED(AUTO_BED_LEVELING_LINEAR)
    STATIC_ITEM(MSG_LINEAR_LEVELING, true);     // Linear Leveling
  #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
    STATIC_ITEM(MSG_BILINEAR_LEVELING, true);   // Bi-linear Leveling
  #elif ENABLED(AUTO_BED_LEVELING_UBL)
    STATIC_ITEM(MSG_UBL_LEVELING, true);        // Unified Bed Leveling
  #elif ENABLED(MESH_BED_LEVELING)
    STATIC_ITEM(MSG_MESH_LEVELING, true);       // Mesh Leveling
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
  MENU_BACK(MSG_MAIN);
  MENU_ITEM(submenu, MSG_INFO_FIRMWARE_MENU, menu_info_firmware);       // Printer Info >
  MENU_ITEM(submenu, MSG_INFO_BOARD_MENU, menu_info_board);             // Board Info >
  MENU_ITEM(submenu, MSG_INFO_THERMISTOR_MENU, menu_info_thermistors);  // Thermistors >
  MENU_ITEM(submenu, MSG_INFO_STATS_MENU, menu_info_stats);             // Printer Statistics >
  END_MENU();
}

#endif // HAS_LCD_MENU && LCD_INFO_MENU
