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
// SD Card Menu
//

#include "../../../MK4duo.h"

#if HAS_LCD_MENU && HAS_SD_SUPPORT

#if !PIN_EXISTS(SD_DETECT)
  void lcd_sd_refresh() {
    encoderTopLine = 0;
    card.mount();
  }
#endif

void lcd_sd_updir() {
  lcdui.encoderPosition = card.updir() ? ENCODER_STEPS_PER_MENU_ITEM : 0;
  encoderTopLine = 0;
  screen_changed = true;
  lcdui.refresh();
}

#if ENABLED(SD_REPRINT_LAST_SELECTED_FILE)

  uint32_t last_sdfile_encoderPosition = 0xFFFF;

  void LcdUI::reselect_last_file() {
    if (last_sdfile_encoderPosition == 0xFFFF) return;
    //#if HAS_GRAPHICAL_LCD
    //  // This is a hack to force a screen update.
    //  lcdui.refresh(LCDVIEW_CALL_REDRAW_NEXT);
    //  lcdui.synchronize();
    //  safe_delay(50);
    //  lcdui.synchronize();
    //  lcdui.refresh(LCDVIEW_CALL_REDRAW_NEXT);
    //  lcdui.drawing_screen = screen_changed = true;
    //#endif

    goto_screen(menu_sdcard, last_sdfile_encoderPosition);
    last_sdfile_encoderPosition = 0xFFFF;

    defer_status_screen(true);

    //#if HAS_GRAPHICAL_LCD
    //  update();
    //#endif
  }
#endif

class MenuItem_sdfile {
  public:
    static void action(SDCard &theCard) {
      #if ENABLED(SD_REPRINT_LAST_SELECTED_FILE)
        last_sdfile_encoderPosition = lcdui.encoderPosition;  // Save which file was selected for later use
      #endif
      card.openAndPrintFile(theCard.fileName);
      lcdui.return_to_status();
      lcdui.reset_status();
    }
};

class MenuItem_sdfolder {
  public:
    static void action(SDCard &theCard) {
      card.chdir(theCard.fileName);
      encoderTopLine = 0;
      lcdui.encoderPosition = 2 * ENCODER_STEPS_PER_MENU_ITEM;
      screen_changed = true;
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

  const uint16_t fileCnt = card.get_num_Files();

  START_MENU();
  MENU_BACK(MSG_MAIN);
  card.getWorkDirName();
  if (card.fileName[0] == '/') {
    #if !PIN_EXISTS(SD_DETECT)
      MENU_ITEM(function, LCD_STR_REFRESH MSG_REFRESH, lcd_sd_refresh);
    #endif
  }
  else if (card.isDetected()) {
    MENU_ITEM(function, LCD_STR_FOLDER "..", lcd_sd_updir);
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
      MENU_ITEM_DUMMY();
    }
  }
  END_MENU();
}

#endif // HAS_LCD_MENU && HAS_SD_SUPPORT
