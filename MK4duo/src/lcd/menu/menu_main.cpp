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
// Main Menu
//

#include "../../../MK4duo.h"

#if HAS_LCD_MENU

#if HAS_GAMES
  #include "game/game.h"
#endif

bool stop_print_file;

void menu_stop_print() {
  do_select_screen_yn(lcdui.stop_print, lcdui.goto_previous_screen, PSTR(MSG_ARE_YOU_SURE), nullptr, PSTR("?"));
}

#if HAS_EEPROM
  void menu_eeprom() {
    lcdui.defer_status_screen();
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
    lcdui.defer_status_screen();
    if (lcdui.use_click()) return lcdui.return_to_status();
    START_SCREEN();
    STATIC_ITEM(MSG_NEXTION_CHANGED_ALLERT_1);
    STATIC_ITEM(MSG_NEXTION_CHANGED_ALLERT_2);
    STATIC_ITEM(MSG_NEXTION_CHANGED_ALLERT_3);
    STATIC_ITEM(MSG_NEXTION_CHANGED_ALLERT_4);
    STATIC_ITEM(MSG_NEXTION_CHANGED_ALLERT_5);
    END_SCREEN();
  }

  void menu_m0() {
    lcdui.defer_status_screen();
    if (lcdui.use_click()) {
      printer.setWaitForUser(false);
      return;
    }
    START_SCREEN();
    STATIC_ITEM(MSG_NEXTION_M0_M1_1);
    STATIC_ITEM(MSG_NEXTION_M0_M1_2);
    END_SCREEN();
  }

  #if HAS_SD_SUPPORT

    void menu_firmware() {
      do_select_screen_yn(nexlcd.UploadNewFirmware, lcdui.goto_previous_screen, PSTR(MSG_ARE_YOU_SURE), nullptr, PSTR("?"));
    }

  #endif

#endif // HAS_NEXTION_LCD

void menu_tune();
void menu_motion();
void menu_temperature();
void menu_configuration();
void menu_user();
void menu_change_filament();
void menu_info();
void menu_led();

#if ENABLED(COLOR_MIXING_EXTRUDER) && DISABLED(NEXTION)
  void menu_mixer();
#endif

#if HAS_DHT
  void menu_dht();
#endif

#if ENABLED(SERVICE_TIME_1)
  void menu_service1();
#endif
#if ENABLED(SERVICE_TIME_2)
  void menu_service2();
#endif
#if ENABLED(SERVICE_TIME_3)
  void menu_service3();
#endif

void menu_main() {
  START_MENU();
  BACK_ITEM(MSG_WATCH);

  const bool busy = printer.isPrinting();

  #if HAS_SD_SUPPORT
    const bool  card_mounted  = card.isMounted(),
                card_open     = card_mounted && card.isFileOpen();
  #endif

  if (busy) {
    ACTION_ITEM(MSG_PAUSE_PRINT, lcdui.pause_print);
    SUBMENU(MSG_STOP_PRINT, menu_stop_print);
    SUBMENU(MSG_TUNE, menu_tune);
  }
  else {
    #if !HAS_ENCODER_WHEEL && HAS_SD_SUPPORT
      //
      // Autostart
      //
      #if ENABLED(MENU_ADDAUTOSTART)
        if (!busy) ACTION_ITEM(MSG_AUTOSTART, card.beginautostart);
      #endif

      if (card_mounted) {
        if (!card_open) {
          #if PIN_EXISTS(SD_DETECT)
            GCODES_ITEM(MSG_CHANGE_SDCARD, PSTR("M21"));
          #else
            GCODES_ITEM(MSG_RELEASE_SDCARD, PSTR("M22"));
          #endif
          SUBMENU(MSG_CARD_MENU, menu_sdcard);
        }
      }
      else {
        #if PIN_EXISTS(SD_DETECT)
          ACTION_ITEM(MSG_NO_CARD, nullptr);
        #else
          GCODES_ITEM(MSG_INIT_SDCARD, PSTR("M21"));
          ACTION_ITEM(MSG_SD_RELEASED, nullptr);
        #endif
        
      }
    #endif // !HAS_ENCODER_WHEEL && HAS_SD_SUPPORT

    if (printer.isPaused())
      ACTION_ITEM(MSG_RESUME_PRINT, lcdui.resume_print);

    SUBMENU(MSG_MOTION, menu_motion);
  }

  if (printer.mode == PRINTER_MODE_FFF) {
    SUBMENU(MSG_TEMPERATURE, menu_temperature);
    #if ENABLED(COLOR_MIXING_EXTRUDER) && DISABLED(NEXTION)
      SUBMENU(MSG_MIXER, menu_mixer);
    #endif
    #if HAS_MMU2
      if (!busy) SUBMENU(MSG_MMU2_MENU, menu_mmu2);
    #endif
  }

  #if HAS_DHT
    SUBMENU(MSG_DHT, menu_dht);
  #endif

  SUBMENU(MSG_CONFIGURATION, menu_configuration);

  #if ENABLED(CUSTOM_USER_MENUS)
    SUBMENU(MSG_USER_MENU, menu_user);
  #endif

  if (printer.mode == PRINTER_MODE_FFF) {
    #if ENABLED(ADVANCED_PAUSE_FEATURE)
      SUBMENU(MSG_FILAMENTCHANGE, menu_change_filament);
    #endif
  }

  #if ENABLED(LCD_INFO_MENU)
    SUBMENU(MSG_INFO_MENU, menu_info);
  #endif

  #if ENABLED(LED_CONTROL_MENU)
    SUBMENU(MSG_LED_CONTROL, menu_led);
  #endif

  //
  // Switch power on/off
  //
  #if HAS_POWER_SWITCH
    if (powerManager.is_on())
      GCODES_ITEM(MSG_SWITCH_PS_OFF, PSTR("M81"));
    else
      GCODES_ITEM(MSG_SWITCH_PS_ON, PSTR("M80"));
  #endif

  #if HAS_ENCODER_WHEEL && HAS_SD_SUPPORT
    //
    // Autostart
    //
    #if ENABLED(MENU_ADDAUTOSTART)
      if (!busy) ACTION_ITEM(MSG_AUTOSTART, card.beginautostart);
    #endif

    if (card_mounted) {
      if (!card_open) {
        SUBMENU(MSG_CARD_MENU, menu_sdcard);
        #if !PIN_EXISTS(SD_DETECT)
          GCODES_ITEM(MSG_CHANGE_SDCARD, PSTR("M21"));  // SD-card changed by user
        #endif
      }
    }
    else {
      #if !PIN_EXISTS(SD_DETECT)
        GCODES_ITEM(MSG_INIT_SDCARD, PSTR("M21")); // Manually initialize the SD-card via user interface
      #endif
      ACTION_ITEM(MSG_NO_CARD, nullptr);
    }
  #endif // HAS_ENCODER_WHEEL && HAS_SD_SUPPORT

  #if ENABLED(SERVICE_TIME_1)
    SUBMENU(SERVICE_NAME_1, menu_service1);
  #endif
  #if ENABLED(SERVICE_TIME_2)
    SUBMENU(SERVICE_NAME_2, menu_service2);
  #endif
  #if ENABLED(SERVICE_TIME_3)
    SUBMENU(SERVICE_NAME_3, menu_service3);
  #endif

  #if HAS_GAMES
    SUBMENU("Game", (
      #if HAS_GAME_MENU
        menu_game
      #elif ENABLED(GAME_BRICKOUT)
        brickout.enter_game
      #elif ENABLED(GAME_INVADERS)
        invaders.enter_game
      #elif ENABLED(GAME_MAZE)
        maze.enter_game
      #elif ENABLED(GAME_SNAKE)
        snake.enter_game
      #endif
    ));
  #endif

  END_MENU();
}

#endif // HAS_LCD_MENU
