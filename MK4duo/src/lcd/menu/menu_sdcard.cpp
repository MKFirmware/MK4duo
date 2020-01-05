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
// SD Card Menu
//

#include "../../../MK4duo.h"

#if HAS_LCD_MENU && HAS_SD_SUPPORT

#if ENABLED(SD_REPRINT_LAST_SELECTED_FILE)

  uint16_t  sd_encoder_position = 0xFFFF;
  int8_t    sd_top_line, sd_items;

  void LcdUI::reselect_last_file() {
    if (sd_encoder_position == 0xFFFF) return;
    //#if HAS_GRAPHICAL_LCD
    //  // This is a hack to force a screen update.
    //  lcdui.refresh(LCDVIEW_CALL_REDRAW_NEXT);
    //  lcdui.synchronize();
    //  HAL::delayMilliseconds(50);
    //  lcdui.synchronize();
    //  lcdui.refresh(LCDVIEW_CALL_REDRAW_NEXT);
    //  lcdui.drawing_screen = lcdui.screen_changed = true;
    //#endif

    goto_screen(menu_sdcard, sd_encoder_position, sd_top_line, sd_items);
    sd_encoder_position = 0xFFFF;

    defer_status_screen();

    //#if HAS_GRAPHICAL_LCD
    //  update();
    //#endif
  }
#endif

inline void sdcard_start_selected_file() {
  card.openAndPrintFile(card.fileName);
  lcdui.return_to_status();
  lcdui.reset_status();
}

class MenuItem_sdfile : public MenuItem_sdbase {
  public:
    static inline void draw(const bool sel, const uint8_t row, PGM_P const pstr, SDCard &theCard) {
      MenuItem_sdbase::draw(sel, row, pstr, theCard, false);
    }
    static void action(PGM_P const pstr, SDCard &) {
      #if ENABLED(SD_REPRINT_LAST_SELECTED_FILE)
        // Save which file was selected for later use
        sd_encoder_position = lcdui.encoderPosition;
        sd_top_line = encoderTopLine;
        sd_items = screen_items;
      #endif
      MenuItem_submenu::action(pstr, []{
        char * const longest = card.fileName;
        char buffer[strlen(longest) + 2];
        buffer[0] = ' ';
        strcpy(buffer + 1, longest);
        MenuItem_confirm::select_screen(
          GET_TEXT(MSG_BUTTON_PRINT), GET_TEXT(MSG_BUTTON_CANCEL),
          sdcard_start_selected_file, lcdui.goto_previous_screen,
          GET_TEXT(MSG_START_PRINT), buffer, PSTR("?")
        );
      });
    }
};

class MenuItem_sdfolder : public MenuItem_sdbase {
  public:
    static inline void draw(const bool sel, const uint8_t row, PGM_P const pstr, SDCard &theCard) {
      MenuItem_sdbase::draw(sel, row, pstr, theCard, true);
    }
    static void action(PGM_P const, SDCard &theCard) {
      card.chdir(theCard.fileName);
      encoderTopLine = 0;
      lcdui.encoderPosition = 2 * (ENCODER_STEPS_PER_MENU_ITEM);
      lcdui.screen_changed = true;
      #if HAS_GRAPHICAL_LCD
        lcdui.drawing_screen = false;
      #elif HAS_NEXTION_LCD
        lcdui.clear_lcd();
      #endif
      lcdui.refresh();
    }
};

void menu_sdcard() {
  lcdui.encoder_direction_menus();

  #if HAS_GRAPHICAL_LCD
    static uint16_t fileCnt;
    if (lcdui.first_page) fileCnt = card.get_num_Files();
  #else
    const uint16_t fileCnt = card.get_num_Files();
  #endif

  START_MENU();
  BACK_ITEM(MSG_MAIN);
  if (card.flag.WorkdirIsRoot) {
    #if !PIN_EXISTS(SD_DETECT)
      ACTION_ITEM(MSG_REFRESH, []{
        encoderTopLine = 0;
        card.unmount(); card.mount();
        if (card.isMounted()) card.ls();
      });
    #endif
  }
  else if (card.isMounted()) {
    ACTION_ITEM_P(PSTR(LCD_STR_FOLDER ".."), []{
      lcdui.encoderPosition = card.updir() ? ENCODER_STEPS_PER_MENU_ITEM : 0;
      encoderTopLine = 0;
      lcdui.screen_changed = true;
      lcdui.refresh();
    });
  }

  for (uint16_t i = 0; i < fileCnt; i++) {
    if (_menuLineNr == _thisItemNr) {
      const uint16_t nr =
        #if DISABLED(SDCARD_SORT_ALPHA)
          fileCnt - 1 -
        #endif
      i;

      card.getfilename_sorted(nr);

      if (card.isFilenameIsDir())
        MENU_ITEM(sdfolder, MSG_CARD_MENU, card);
      else
        MENU_ITEM(sdfile, MSG_CARD_MENU, card);
    }
    else {
      SKIP_ITEM();
    }
  }
  END_MENU();
}

#endif // HAS_LCD_MENU && HAS_SD_SUPPORT
