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
// Main Menu
//

#include "../../../MK4duo.h"

#if HAS_LCD_MENU

#if HAS_SD_SUPPORT

  void lcd_sdcard_pause() {
    #if HAS_SD_RESTART
      if (restart.enabled) restart.save_job(true, false);
    #endif
    card.pauseSDPrint();
    print_job_counter.pause();
    #if ENABLED(PARK_HEAD_ON_PAUSE)
      commands.enqueue_and_echo_P(PSTR("M125"));
    #endif
    lcdui.reset_status();
  }

  void lcd_sdcard_resume() {
    #if ENABLED(PARK_HEAD_ON_PAUSE)
      commands.enqueue_and_echo_P(PSTR("M24"));
    #else
      card.startFileprint();
      print_job_counter.start();
    #endif
    lcdui.reset_status();
  }

  void lcd_sdcard_stop() {
    printer.setWaitForHeatUp(false);
    printer.setWaitForUser(false);
    card.setAbortSDprinting(true);
    lcdui.setstatusPGM(PSTR(MSG_PRINT_ABORTED), -1);
    lcdui.return_to_status();
  }

  void menu_stop_print() {
    lcdui.encoder_direction_menus();
    START_MENU();
    MENU_BACK(MSG_MAIN);
    STATIC_ITEM(MSG_DO_YOU_ARE_SHURE);
    MENU_ITEM(function, MSG_YES, lcd_sdcard_stop);
    MENU_ITEM(function, MSG_NO, lcdui.return_to_status);
    END_MENU();
  }

#endif // HAS_SD_SUPPORT

#if HAS_EEPROM
  void menu_eeprom() {
    lcdui.defer_status_screen(true);
    if (lcdui.use_click()) return lcdui.return_to_status();
    START_SCREEN();
    STATIC_ITEM(MSG_EEPROM_CHANGED_ALLERT_1);
    STATIC_ITEM(MSG_EEPROM_CHANGED_ALLERT_2);
    STATIC_ITEM(MSG_EEPROM_CHANGED_ALLERT_3);
    STATIC_ITEM(MSG_EEPROM_CHANGED_ALLERT_4);
    END_SCREEN();
  }
#endif

#if HAS_NEXTION_LCD

  void menu_nextion() {
    lcdui.defer_status_screen(true);
    if (lcdui.use_click()) return lcdui.return_to_status();
    START_SCREEN();
    STATIC_ITEM(MSG_NEXTION_CHANGED_ALLERT_1);
    STATIC_ITEM(MSG_NEXTION_CHANGED_ALLERT_2);
    STATIC_ITEM(MSG_NEXTION_CHANGED_ALLERT_3);
    STATIC_ITEM(MSG_NEXTION_CHANGED_ALLERT_4);
    STATIC_ITEM(MSG_NEXTION_CHANGED_ALLERT_5);
    END_SCREEN();
  }

  void menu_firmware() {
    lcdui.encoderPosition = 2 * ENCODER_STEPS_PER_MENU_ITEM;
    START_MENU();
    MENU_BACK(MSG_MAIN);
    STATIC_ITEM(MSG_DO_YOU_ARE_SHURE);
    MENU_ITEM(function, MSG_YES, UploadNewFirmware);
    MENU_ITEM(submenu, MSG_NO, menu_main);
    END_MENU();
  }

#endif

void menu_tune();
void menu_motion();
void menu_temperature();
void menu_configuration();
void menu_user();
void menu_temp_e0_filament_change();
void menu_change_filament();
void menu_info();
void menu_led();

void menu_main() {
  START_MENU();
  MENU_BACK(MSG_WATCH);

  #if HAS_SD_SUPPORT
    if (card.isOK()) {
      if (card.isFileOpen()) {
        if (IS_SD_PRINTING())
          MENU_ITEM(function, MSG_PAUSE_PRINT, lcd_sdcard_pause);
        else
          MENU_ITEM(function, MSG_RESUME_PRINT, lcd_sdcard_resume);
        MENU_ITEM(submenu, MSG_STOP_PRINT, menu_stop_print);
      }
      else {
        MENU_ITEM(submenu, MSG_CARD_MENU, menu_sdcard);
        #if !PIN_EXISTS(SD_DETECT)
          MENU_ITEM(gcode, MSG_CHANGE_SDCARD, PSTR("M21"));  // SD-card changed by user
        #endif
      }
    }
    else {
      MENU_ITEM(submenu, MSG_NO_CARD, menu_sdcard);
      #if !PIN_EXISTS(SD_DETECT)
        MENU_ITEM(gcode, MSG_INIT_SDCARD, PSTR("M21")); // Manually initialize the SD-card via user interface
      #endif
    }
  #endif // SDSUPPORT

  const bool busy = printer.isPrinting();
  if (busy)
    MENU_ITEM(submenu, MSG_TUNE, menu_tune);
  else {
    #if !HAS_NEXTION_LCD
      MENU_ITEM(submenu, MSG_MOTION, menu_motion);
      if (printer.mode == PRINTER_MODE_FFF)
        MENU_ITEM(submenu, MSG_TEMPERATURE, menu_temperature);
    #endif
  }

  MENU_ITEM(submenu, MSG_CONFIGURATION, menu_configuration);

  #if ENABLED(CUSTOM_USER_MENUS)
    MENU_ITEM(submenu, MSG_USER_MENU, menu_user);
  #endif

  if (printer.mode == PRINTER_MODE_FFF) {
    #if ENABLED(ADVANCED_PAUSE_FEATURE)
      #if DRIVER_EXTRUDERS == 1 && DISABLED(FILAMENT_LOAD_UNLOAD_GCODES)
        if (thermalManager.targetHotEnoughToExtrude(tools.active_extruder))
          MENU_ITEM(gcode, MSG_FILAMENTCHANGE, PSTR("M600 B0"));
        else
          MENU_ITEM(submenu, MSG_FILAMENTCHANGE, menu_temp_e0_filament_change);
      #else
        MENU_ITEM(submenu, MSG_FILAMENTCHANGE, menu_change_filament);
      #endif
    #endif
  }

  #if ENABLED(LCD_INFO_MENU)
    MENU_ITEM(submenu, MSG_INFO_MENU, menu_info);
  #endif

  #if ENABLED(LED_CONTROL_MENU)
    MENU_ITEM(submenu, MSG_LED_CONTROL, menu_led);
  #endif

  //
  // Switch power on/off
  //
  #if HAS_POWER_SWITCH
    if (powerManager.is_on())
      MENU_ITEM(gcode, MSG_SWITCH_PS_OFF, PSTR("M81"));
    else
      MENU_ITEM(gcode, MSG_SWITCH_PS_ON, PSTR("M80"));
  #endif

  //
  // Autostart
  //
  #if ENABLED(SDSUPPORT) && ENABLED(MENU_ADDAUTOSTART)
    if (!busy)
      MENU_ITEM(function, MSG_AUTOSTART, card.beginautostart);
  #endif

  END_MENU();
}

#endif // HAS_LCD_MENU
