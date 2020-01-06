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
// Main Menu
//

#include "../../../MK4duo.h"

#if HAS_LCD_MENU

#if HAS_GAMES
  #include "game/game.h"
#endif

bool stop_print_file;

#if HAS_EEPROM || HAS_NEXTION_LCD
  void menu_allert(PGM_P const msg) {
    PGM_P const msg1 = msg;
    PGM_P const msg2 = msg1 + strlen_P(msg1) + 1;
    PGM_P const msg3 = msg2 + strlen_P(msg2) + 1;
    PGM_P const msg4 = msg3 + strlen_P(msg3) + 1;
    const bool  has2 = msg2[0], has3 = msg3[0], has4 = msg4[0];
    lcdui.defer_status_screen();
    if (lcdui.use_click()) return lcdui.return_to_status();
    START_SCREEN();
    STATIC_ITEM_P(msg1);
    if (has2) STATIC_ITEM_P(msg2);
    if (has3) STATIC_ITEM_P(msg3);
    if (has4) STATIC_ITEM_P(msg4);
    END_SCREEN();
  }
#endif

#if HAS_EEPROM
  void lcd_eeprom_allert() { menu_allert(GET_TEXT(MSG_EEPROM_ALLERT)); }
#endif

#if HAS_NEXTION_LCD

  void lcd_nextion_allert() { menu_allert(GET_TEXT(MSG_NEXTION_ALLERT)); }

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
      MenuItem_confirm::confirm_screen(
        nexlcd.UploadNewFirmware, lcdui.goto_previous_screen,
        GET_TEXT(MSG_ARE_YOU_SURE), (PGM_P)nullptr, PSTR("?")
      );
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

void menu_main() {
  START_MENU();
  BACK_ITEM(MSG_INFO_SCREEN);

  const bool busy = printer.isPrinting();

  #if HAS_SD_SUPPORT
    const bool  card_mounted  = card.isMounted(),
                card_open     = card_mounted && card.isFileOpen();
  #endif

  if (busy) {
    ACTION_ITEM(MSG_PAUSE_PRINT, lcdui.pause_print);
    SUBMENU(MSG_STOP_PRINT, []{
      MenuItem_confirm::confirm_screen(
        lcdui.stop_print, lcdui.goto_previous_screen,
        GET_TEXT(MSG_ARE_YOU_SURE), (PGM_P)nullptr, PSTR("?")
      );
    });
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
          SUBMENU(MSG_CARD_MENU, menu_sdcard);
          #if PIN_EXISTS(SD_DETECT)
            GCODES_ITEM(MSG_CHANGE_SDCARD, M21_CMD);
          #else
            GCODES_ITEM(MSG_RELEASE_SDCARD, M22_CMD);
          #endif
        }
      }
      else {
        #if PIN_EXISTS(SD_DETECT)
          STATIC_ITEM(MSG_NO_CARD);
        #else
          GCODES_ITEM(MSG_INIT_SDCARD, M21_CMD);
          STATIC_ITEM(MSG_SD_RELEASED);
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

  SUBMENU(MSG_CONFIGURATION, menu_configuration);

  #if ENABLED(CUSTOM_USER_MENUS)
    SUBMENU(MSG_USER_MENU, menu_user);
  #endif

  if (printer.mode == PRINTER_MODE_FFF) {
    #if ENABLED(ADVANCED_PAUSE_FEATURE) && DISABLED(FILAMENT_LOAD_UNLOAD_GCODES)
      GCODES_ITEM(MSG_FILAMENTCHANGE, PSTR("M600 B0"));
    #elif ENABLED(FILAMENT_LOAD_UNLOAD_GCODES)
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
        #if PIN_EXISTS(SD_DETECT)
          GCODES_ITEM(MSG_CHANGE_SDCARD, M21_CMD);
        #else
          GCODES_ITEM(MSG_RELEASE_SDCARD, M22_CMD);
        #endif
        SUBMENU(MSG_CARD_MENU, menu_sdcard);
      }
    }
    else {
      #if PIN_EXISTS(SD_DETECT)
        STATIC_ITEM(MSG_NO_CARD);
      #else
        GCODES_ITEM(MSG_INIT_SDCARD, M21_CMD);
        STATIC_ITEM(MSG_SD_RELEASED);
      #endif
    }
  #endif // HAS_ENCODER_WHEEL && HAS_SD_SUPPORT

  #if HAS_SERVICE_TIMES
    static auto _service_reset = [](const int index) {
      print_job_counter.resetServiceTime(index);
      #if HAS_BUZZER
        sound.playtone(200, 404);
      #endif
      lcdui.reset_status();
      lcdui.return_to_status();
    };
    #if ENABLED(SERVICE_TIME_1)
      CONFIRM_ITEM_P(PSTR(SERVICE_NAME_1),
        MSG_BUTTON_RESET, MSG_BUTTON_CANCEL,
        []{ _service_reset(1); }, lcdui.goto_previous_screen,
        GET_TEXT(MSG_SERVICE_RESET), PSTR(SERVICE_NAME_1), PSTR("?")
      );
    #endif
    #if ENABLED(SERVICE_TIME_2)
      CONFIRM_ITEM_P(PSTR(SERVICE_NAME_2),
        MSG_BUTTON_RESET, MSG_BUTTON_CANCEL,
        []{ _service_reset(2); }, lcdui.goto_previous_screen,
        GET_TEXT(MSG_SERVICE_RESET), PSTR(SERVICE_NAME_2), PSTR("?")
      );
    #endif
    #if ENABLED(SERVICE_TIME_3)
      CONFIRM_ITEM_P(PSTR(SERVICE_NAME_3),
        MSG_BUTTON_RESET, MSG_BUTTON_CANCEL,
        []{ _service_reset(3); }, lcdui.goto_previous_screen,
        GET_TEXT(MSG_SERVICE_RESET), PSTR(SERVICE_NAME_3), PSTR("?")
      );
    #endif
  #endif // HAS_SERVICE_TIMES

  #if HAS_GAMES
    SUBMENU(MSG_GAMES, (
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
