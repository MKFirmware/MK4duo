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

/**
 * Nextion_lcd.cpp
 *
 * Copyright (c) 2014-2016 Alberto Cotronei @MagoKimbra
 *
 * Grbl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Grbl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Grbl. If not, see <http://www.gnu.org/licenses/>.
 */

#include "../../../base.h"

#if ENABLED(NEXTION)

  #include "Nextion_lcd.h"
  #include "Nextion_gfx.h"
  #include "nextion_lib/Nextion.h"

  bool  NextionON                     = false,
        show_Wave                     = true,
        lcdDrawUpdate                 = false,
        lcd_clicked                   = false;
  uint8_t PageID                      = 0,
          lcd_status_message_level    = 0;
  uint16_t slidermaxval               = 20;
  char buffer[100]                    = { 0 };
  char lcd_status_message[30]         = WELCOME_MSG;
  const float manual_feedrate_mm_m[]  = MANUAL_FEEDRATE;

  #if HAS_SDSUPPORT
    uint8_t SDstatus    = 0; // 0 card not present, 1 SD not insert, 2 SD insert, 3 SD printing
    NexUpload Firmware(NEXTION_FIRMWARE_FILE, 57600);
  #endif

  #if ENABLED(NEXTION_GFX)
    GFX gfx = GFX(1, 1, 1, 1);
  #endif

  /**
   *******************************************************************
   * Nextion component all page
   *******************************************************************
   */
  NexObject Pstart        = NexObject(0,  0,  "start");
  NexObject Pmenu         = NexObject(1,  0,  "menu");
  NexObject Pprinter      = NexObject(2,  0,  "printer");
  NexObject Psdcard       = NexObject(3,  0,  "sdcard");
  NexObject Psetup        = NexObject(4,  0,  "setup");
  NexObject Pmove         = NexObject(5,  0,  "move");
  NexObject Pspeed        = NexObject(6,  0,  "speed");
  NexObject Pgcode        = NexObject(7,  0,  "gcode");
  NexObject Prfid         = NexObject(8,  0,  "rfid");
  NexObject Pbrightness   = NexObject(9,  0,  "brightness");
  NexObject Ptemp         = NexObject(10, 0,  "temp");
  NexObject Pinfo         = NexObject(11, 0,  "info");
  NexObject Pyesno        = NexObject(12, 0,  "yesno");
  NexObject Pfilament     = NexObject(13, 0,  "filament");
  NexObject Pselect       = NexObject(14, 0,  "select");
  NexObject PBedLevel     = NexObject(15, 0,  "bedlevel");
  NexObject Poptions      = NexObject(16, 0,  "options");
  NexObject Ptime         = NexObject(17, 0,  "time");
  NexObject Pusertemp     = NexObject(18, 0,  "usertemp");

  /**
   *******************************************************************
   * Nextion component for page:start
   *******************************************************************
   */
  NexObject startimer     = NexObject(0,  1,  "tm0");

  /**
   *******************************************************************
   * Nextion component for page:menu
   *******************************************************************
   */

  /**
   *******************************************************************
   * Nextion component for page:printer
   *******************************************************************
   */
  NexObject Hotend00    = NexObject(2,  2,  "he00");
  NexObject Hotend01    = NexObject(2,  3,  "he01");
  NexObject Hotend10    = NexObject(2,  4,  "he10");
  NexObject Hotend11    = NexObject(2,  5,  "he11");
  NexObject Bed0        = NexObject(2,  6,  "bed0");
  NexObject Bed1        = NexObject(2,  7,  "bed1");
  NexObject Chamber0    = NexObject(2,  8,  "cha0");
  NexObject Chamber1    = NexObject(2,  9,  "cha1");
  NexObject Extruders   = NexObject(2,  10, "extruder");
  NexObject Fan         = NexObject(2,  11, "fan");
  NexObject SD          = NexObject(2,  12, "sd");
  NexObject RFID        = NexObject(2,  13, "rfid");
  NexObject Language    = NexObject(2,  14, "lang");
  NexObject VSpeed      = NexObject(2,  15, "vspeed");
  NexObject Fantimer    = NexObject(2,  16, "tm0");
  NexObject Fanpic      = NexObject(2,  19, "p0");
  NexObject NStop       = NexObject(2,  20, "p1");
  NexObject NPlay       = NexObject(2,  21, "p2");
  NexObject NSStop      = NexObject(2,  22, "p3");
  NexObject LcdStatus   = NexObject(2,  24, "t0");
  NexObject LcdCommand  = NexObject(2,  25, "t1");
  NexObject Hotend0     = NexObject(2,  26, "t2");
  NexObject Hotend1     = NexObject(2,  27, "t3");
  NexObject Hotend2     = NexObject(2,  28, "t4");
  NexObject Fanspeed    = NexObject(2,  29, "t5");
  NexObject Wavetemp    = NexObject(2,  30, "s0");
  NexObject sdbar       = NexObject(2,  31, "j0");
  NexObject LcdX        = NexObject(2,  32, "t6");
  NexObject LcdY        = NexObject(2,  33, "t7");
  NexObject LcdZ        = NexObject(2,  34, "t8");
  NexObject LcdTime     = NexObject(2,  35, "t9");

  /**
   *******************************************************************
   * Nextion component for page:SDCard
   *******************************************************************
   */
  NexObject sdlist      = NexObject(3,  1,  "h0");
  NexObject sdrow0      = NexObject(3,  2,  "t0");
  NexObject sdrow1      = NexObject(3,  3,  "t1");
  NexObject sdrow2      = NexObject(3,  4,  "t2");
  NexObject sdrow3      = NexObject(3,  5,  "t3");
  NexObject sdrow4      = NexObject(3,  6,  "t4");
  NexObject sdrow5      = NexObject(3,  7,  "t5");
  NexObject Folder0     = NexObject(3,  8,  "p0");
  NexObject Folder1     = NexObject(3,  9,  "p1");
  NexObject Folder2     = NexObject(3,  10, "p2");
  NexObject Folder3     = NexObject(3,  11, "p3");
  NexObject Folder4     = NexObject(3,  12, "p4");
  NexObject Folder5     = NexObject(3,  13, "p5");
  NexObject Folderup    = NexObject(3,  14, "p6");
  NexObject sdfolder    = NexObject(3,  16, "t6");
  NexObject ScrollUp    = NexObject(3,  18, "p7");
  NexObject ScrollDown  = NexObject(3,  19, "p8");
  NexObject sd_mount    = NexObject(3,  22, "p12");
  NexObject sd_dismount = NexObject(3,  23, "p13");

  /**
   *******************************************************************
   * Nextion component for page:Setup
   *******************************************************************
   */

  /**
   *******************************************************************
   * Nextion component for page:Move
   *******************************************************************
   */
  NexObject XYHome      = NexObject(5,  2,  "p4");
  NexObject XYUp        = NexObject(5,  3,  "p5");
  NexObject XYRight     = NexObject(5,  4,  "p6");
  NexObject XYDown      = NexObject(5,  5,  "p7");
  NexObject XYLeft      = NexObject(5,  6,  "p8");
  NexObject ZHome       = NexObject(5,  7,  "p9");
  NexObject ZUp         = NexObject(5,  8,  "p10");
  NexObject ZDown       = NexObject(5,  9,  "p11");
  NexObject movecmd     = NexObject(5,  11, "vacmd");
  NexObject LedCoord5   = NexObject(5,  12, "t0");
  NexObject MotorOff    = NexObject(5,  17, "p0");
  NexObject ext         = NexObject(5,  19, "va0");
  NexObject Extrude     = NexObject(5,  20, "p12");
  NexObject Retract     = NexObject(5,  22, "p14");
  NexObject SpeedX      = NexObject(5,  23, "vafrx");
  NexObject SpeedY      = NexObject(5,  24, "vafry");
  NexObject SpeedZ      = NexObject(5,  25, "vafrz");
  NexObject SpeedE      = NexObject(5,  26, "vafre");

  /**
   *******************************************************************
   * Nextion component for page:Speed
   *******************************************************************
   */
  NexObject Speed       = NexObject(6,  7,  "h0");

  /**
   *******************************************************************
   * Nextion component for page:GCode
   *******************************************************************
   */
  NexObject Tgcode      = NexObject(7,  1,  "tgcode");
  NexObject Send        = NexObject(7,  27, "bsend");

  /**
   *******************************************************************
   * Nextion component for page:Rfid
   *******************************************************************
   */
  NexObject Rfid0       = NexObject(8,  2,  "b0");
  NexObject Rfid1       = NexObject(8,  3,  "b1");
  NexObject Rfid2       = NexObject(8,  4,  "b2");
  NexObject Rfid3       = NexObject(8,  5,  "b3");
  NexObject Rfid4       = NexObject(8,  6,  "b4");
  NexObject Rfid5       = NexObject(8,  7,  "b5");
  NexObject RfidText    = NexObject(8,  8,  "t0");
  NexObject RfidR       = NexObject(8,  9,  "bt0");

  /**
   *******************************************************************
   * Nextion component for page:Brightness
   *******************************************************************
   */

  /**
   *******************************************************************
   * Nextion component for page:Temp
   *******************************************************************
   */
  NexObject tset        = NexObject(10, 1,  "t0");
  NexObject theater     = NexObject(10, 2,  "va0");
  NexObject tenter      = NexObject(10, 3,  "p5");
  NexObject tup         = NexObject(10, 6,  "p8");
  NexObject tdown       = NexObject(10, 7,  "p9");

  /**
   *******************************************************************
   * Nextion component for page:Info
   *******************************************************************
   */
  NexObject InfoText    = NexObject(11, 2,  "t0");
  NexObject ScrollText  = NexObject(11, 3,  "g0");

  /**
   *******************************************************************
   * Nextion component for page:Yesno
   *******************************************************************
   */
  NexObject Vyes        = NexObject(12, 2,  "va0");
  NexObject Riga0       = NexObject(12, 4,  "t0");
  NexObject Riga1       = NexObject(12, 5,  "t1");
  NexObject Riga2       = NexObject(12, 6,  "t2");
  NexObject Riga3       = NexObject(12, 7,  "t3");
  NexObject Yes         = NexObject(12, 8,  "p1");
  NexObject No          = NexObject(12, 9,  "p2");

  /**
   *******************************************************************
   * Nextion component for page:Filament
   *******************************************************************
   */
  NexObject FilLoad     = NexObject(13, 3,  "p2");
  NexObject FilUnload   = NexObject(13, 4,  "p3");
  NexObject Filgcode    = NexObject(13, 12, "vacmd");

  /**
   *******************************************************************
   * Nextion component for page:Select
   *******************************************************************
   */
  NexObject LcdRiga1    = NexObject(14, 1,  "t0");
  NexObject LcdRiga2    = NexObject(14, 2,  "t1");
  NexObject LcdRiga3    = NexObject(14, 3,  "t2");
  NexObject LcdRiga4    = NexObject(14, 4,  "t3");
  NexObject LcdValor    = NexObject(14, 5,  "t4");
  NexObject LcdUp       = NexObject(14, 6,  "p0");
  NexObject LcdSend     = NexObject(14, 7,  "p1");
  NexObject LcdDown     = NexObject(14, 8,  "p2");
  NexObject LcdMin      = NexObject(14, 9,  "max");
  NexObject LcdMax      = NexObject(14, 10, "max");
  NexObject LcdPos      = NexObject(14, 11, "pos");

  /**
   *******************************************************************
   * Nextion component for page:Bedlevel
   *******************************************************************
   */
  NexObject BedUp       = NexObject(15, 1,  "p0");
  NexObject BedSend     = NexObject(15, 2,  "p1");
  NexObject BedDown     = NexObject(15, 3,  "p2");
  NexObject BedMsg      = NexObject(15, 4,  "t0");
  NexObject BedZ        = NexObject(15, 5,  "t1");

  NexObject *nex_listen_list[] =
  {
    // Page 2 touch listen
    &Hotend0, &Hotend1, &Hotend2, &Fanpic, &NPlay,

    // Page 3 touch listen
    &sdlist, &ScrollUp, &ScrollDown, &sdrow0, &sdrow1, &sdrow2,
    &sdrow3, &sdrow4, &sdrow5, &Folderup, &sd_mount, &sd_dismount,

    // Page 4 touch listen

    // Page 5 touch listen
    &MotorOff, &XYHome, &XYUp, &XYRight, &XYDown, &XYLeft,
    &ZHome, &ZUp, &ZDown,
    &Extrude, &Retract,

    // Page 6 touch listen
    &Speed,

    // Page 7 touch listen
    &Send,

    // Page 8 touch listen
    &Rfid0, &Rfid1, &Rfid2, &Rfid3, &Rfid4, &Rfid5,

    // Page 10 touch listen
    &tenter, &tup, &tdown,

    // Page 12 touch listen
    &Yes,

    // Page 13 touch listen
    &FilLoad, &FilUnload,

    // Page 14 touch listen
    &LcdSend,
    
    // Page 15 touch listen
    &BedUp, &BedDown, &BedSend,

    NULL
  };

  NexObject *lcd_row_list[] =
  {
    &LcdRiga1,
    &LcdRiga2,
    &LcdRiga3,
    &LcdRiga4,
    NULL
  };

  NexObject *heater_list0[] =
  {
    &Hotend00,
    &Hotend10,
    &Bed0,
    &Chamber0,
    NULL
  };

  NexObject *heater_list1[] =
  {
    &Hotend01,
    &Hotend11,
    &Bed1,
    &Chamber1,
    NULL
  };

  NexObject *row_list[] =
  {
    &sdrow0,
    &sdrow1,
    &sdrow2,
    &sdrow3,
    &sdrow4,
    &sdrow5,
    NULL
  };

  NexObject *folder_list[] =
  {
    &Folder0,
    &Folder1,
    &Folder2,
    &Folder3,
    &Folder4,
    &Folder5,
    NULL
  };

  NexObject *speed_list[] =
  {
    &SpeedX,
    &SpeedY,
    &SpeedZ,
    &SpeedE,
    NULL
  };

  // Function pointer to menu functions.
  typedef void (*screenFunc_t)();

  /**
   *
   * Menu actions
   *
   */
  void menu_action_back() { Pprinter.show(); }
  void menu_action_function(screenFunc_t func) { (*func)(); }

  void setpagePrinter() {
    char temp[10] = { 0 };

    #if HOTENDS > 0
      Hotend00.setValue(1, "printer");
      #if HOTENDS > 1
        Hotend10.setValue(1, "printer");
      #endif
    #endif

    #if HAS_TEMP_BED
      Bed0.setValue(1, "printer");
    #endif

    Extruders.setValue(EXTRUDERS, "printer");

    LOOP_XYZE(i) {
      ZERO(temp);
      itoa(manual_feedrate_mm_m[i], temp, 10);
      speed_list[i]->setText(temp, "move");
    }

    #if HAS_SDSUPPORT
      card.mount();
      if (card.cardOK)
        SDstatus = 2;
      else
        SDstatus = 1;
      SD.setValue(SDstatus, "printer");
    #endif

    VSpeed.setValue(100, "printer");

    #if FAN_COUNT > 0
      Fan.setValue(1, "printer");
    #endif

    #if ENABLED(RFID_MODULE)
      RFID.setValue(1, "printer");
    #endif

    #define LANGUAGE_STRING(M) STRINGIFY(M)
    #define NEXTION_LANGUAGE LANGUAGE_STRING(LCD_LANGUAGE)
    Language.setText(NEXTION_LANGUAGE, "printer");
  }

  void start_menu(const bool encoder=false, const bool push=false) {
    Pselect.show();
    LcdUp.SetVisibility(encoder);
    LcdDown.SetVisibility(encoder);
    LcdSend.SetVisibility(push);
    lcdDrawUpdate = true;
    lcd_clicked = !push;
  }

  /**
   * START_SCREEN  Opening code for a screen having only static items.
   *               Do simplified scrolling of the entire screen.
   *
   * START_MENU    Opening code for a screen with menu items.
   *               Scroll as-needed to keep the selected line in view.
   */
  #define START_SCREEN() \
    start_menu(false, true); \
    do { \
      uint8_t _lcdLineNr = 0; \

  #define START_MENU() \
    start_menu(true, true); \
    uint32_t encoderLine; \
    uint8_t _lcdLineNr = 0; \
    do { \
      _lcdLineNr = 0; \
      LcdPos.getValue(&encoderLine); \
      HAL::delayMilliseconds(200)

  #define MENU_ITEM(TYPE, LABEL, ...) \
      if (lcdDrawUpdate) { \
        lcd_row_list[_lcdLineNr]->setText(PSTR(LABEL)); \
        LcdMax.setValue(_lcdLineNr); \
      } \
      if (lcd_clicked && encoderLine == _lcdLineNr) { \
        menu_action_ ## TYPE(__VA_ARGS__); \
        return; \
      } \
      ++_lcdLineNr

  #define MENU_BACK(LABEL) MENU_ITEM(back, LABEL)

  #define STATIC_ITEM(LABEL) \
      if (lcdDrawUpdate) { \
        lcd_row_list[_lcdLineNr]->setText(PSTR(LABEL)); \
        LcdMin.setValue(_lcdLineNr + 1); \
      } \
      ++_lcdLineNr \

  #define END_MENU() \
      printer.idle(); \
      lcdDrawUpdate = false; \
    } while(1)

  #define END_SCREEN() \
      lcdDrawUpdate = false; \
    } while(0)

  #if HAS_SDSUPPORT

    void UploadNewFirmware() {
      if(IS_SD_INSERTED || card.cardOK) {
        Firmware.startUpload();
        nexSerial.end();
        lcd_init();
      }
    }

    void printrowsd(uint8_t row, const bool folder, const char* filename) {
      if (folder) {
        folder_list[row]->SetVisibility(true);
        row_list[row]->attachPop(sdfolderPopCallback, row_list[row]);
      } else if (filename == "") {
        folder_list[row]->SetVisibility(false);
        row_list[row]->detachPop();
      } else {
        folder_list[row]->SetVisibility(false);
        row_list[row]->attachPop(sdfilePopCallback, row_list[row]);
      }
      row_list[row]->setText(filename);
    }

    static void setrowsdcard(uint32_t number = 0) {
      uint16_t fileCnt = card.getnrfilenames();
      uint32_t i = 0;
      card.getWorkDirName();

      if (card.fileName[0] != '/') {
        Folderup.SetVisibility(true);
        Folderup.attachPop(sdfolderUpPopCallback);
        sdfolder.setText(card.fileName);
      } else {
        Folderup.detachPop();
        Folderup.SetVisibility(false);
        sdfolder.setText("");
      }

      for (uint8_t row = 0; row < 6; row++) {
        i = row + number;
        if (i < fileCnt) {
          card.getfilename(i);
          printrowsd(row, card.filenameIsDir, card.fileName);
        } else {
          printrowsd(row, false, "");
        }
      }
      sendCommand("ref 0");
    }

    static void menu_action_sdfile(const char* filename) {
      card.openAndPrintFile(filename);
      Pprinter.show();
    }

    static void menu_action_sddirectory(const char* filename) {
      card.chdir(filename);
      setpageSD();
    }

    void setpageSD() {
      uint16_t fileCnt = card.getnrfilenames();

      if (fileCnt <= 6)
        slidermaxval = 0;
      else
        slidermaxval  = fileCnt - 6;

      uint16_t hig = 210 - slidermaxval * 10;
      if (hig < 10) hig = 10;

      sdlist.Set_cursor_height_hig(hig);
      sdlist.setMaxval(slidermaxval);
      sdlist.setValue(slidermaxval);
      sendCommand("ref 0");

      setrowsdcard();
    }

    void sdmountdismountPopCallback(void *ptr) {
      if (ptr == &sd_mount) {
        card.mount();
        if (card.cardOK)
          SDstatus = 2;
        else
          SDstatus = 1;
        SD.setValue(SDstatus, "printer");
      }
      else {
        card.unmount();
        SDstatus = 1;
        SD.setValue(SDstatus, "printer");
      }
      setpageSD();
    }

    void sdlistPopCallback(void *ptr) {
      UNUSED(ptr);
      uint32_t number = 0;
      sdlist.getValue(&number);
      number = slidermaxval - number;
      setrowsdcard(number);
    }

    void sdfilePopCallback(void *ptr) {
      ZERO(buffer);

      if (ptr == &sdrow0)
        sdrow0.getText(buffer, sizeof(buffer));
      else if (ptr == &sdrow1)
        sdrow1.getText(buffer, sizeof(buffer));
      else if (ptr == &sdrow2)
        sdrow2.getText(buffer, sizeof(buffer));
      else if (ptr == &sdrow3)
        sdrow3.getText(buffer, sizeof(buffer));
      else if (ptr == &sdrow4)
        sdrow4.getText(buffer, sizeof(buffer));
      else if (ptr == &sdrow5)
        sdrow5.getText(buffer, sizeof(buffer));

      menu_action_sdfile(buffer);
    }

    void sdfolderPopCallback(void *ptr) {
      ZERO(buffer);

      if (ptr == &sdrow0)
        sdrow0.getText(buffer, sizeof(buffer));
      else if (ptr == &sdrow1)
        sdrow1.getText(buffer, sizeof(buffer));
      else if (ptr == &sdrow2)
        sdrow2.getText(buffer, sizeof(buffer));
      else if (ptr == &sdrow3)
        sdrow3.getText(buffer, sizeof(buffer));
      else if (ptr == &sdrow4)
        sdrow4.getText(buffer, sizeof(buffer));
      else if (ptr == &sdrow5)
        sdrow5.getText(buffer, sizeof(buffer));

      menu_action_sddirectory(buffer);
    }

    void sdfolderUpPopCallback(void *ptr) {
      UNUSED(ptr);
      card.updir();
      setpageSD();
    }

    void PlayPausePopCallback(void *ptr) {
      UNUSED(ptr);

      if (card.cardOK && card.isFileOpen()) {
        if (IS_SD_PRINTING) {
          card.pauseSDPrint();
          print_job_counter.pause();
        }
        else {
          card.startFileprint();
          print_job_counter.start();
        }
      }
    }

    void StopPrint(const bool store_location = false) {
      printer.stopSDPrint(store_location);
    }

  #endif

  #if ENABLED(ADVANCED_PAUSE_FEATURE)

    void lcd_advanced_pause_toocold_menu() {
      START_MENU();
      STATIC_ITEM(MSG_HEATING_FAILED_LCD);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_MINTEMP STRINGIFY(EXTRUDE_MINTEMP) ".");
      MENU_BACK(MSG_BACK);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_NOZZLE);
      END_MENU();
    }

    static void lcd_filament_change_resume_print() {
      advanced_pause_menu_response = ADVANCED_PAUSE_RESPONSE_RESUME_PRINT;
      Pprinter.show();
    }

    static void lcd_filament_change_extrude_more() {
      advanced_pause_menu_response = ADVANCED_PAUSE_RESPONSE_EXTRUDE_MORE;
    }

    static void lcd_advanced_pause_option_menu() {
      START_MENU();
      STATIC_ITEM(MSG_FILAMENT_CHANGE_OPTION_HEADER);
      MENU_ITEM(function, MSG_FILAMENT_CHANGE_OPTION_RESUME, lcd_filament_change_resume_print);
      MENU_ITEM(function, MSG_FILAMENT_CHANGE_OPTION_EXTRUDE, lcd_filament_change_extrude_more);
      END_MENU();
    }

    static void lcd_advanced_pause_init_message() {
      START_SCREEN();
      STATIC_ITEM(MSG_FILAMENT_CHANGE_HEADER);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_INIT_1);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_INIT_2);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_INIT_3);
      END_SCREEN();
    }

    static void lcd_advanced_pause_cool_message() {
      START_SCREEN();
      STATIC_ITEM(MSG_FILAMENT_CHANGE_HEADER);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_COOL_1);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_COOL_2);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_COOL_3);
      END_SCREEN();
    }

    static void lcd_advanced_pause_unload_message() {
      START_SCREEN();
      STATIC_ITEM(MSG_FILAMENT_CHANGE_HEADER);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_UNLOAD_1);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_UNLOAD_2);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_UNLOAD_3);
      END_SCREEN();
    }

    static void lcd_advanced_pause_wait_for_nozzles_to_heat() {
      START_SCREEN();
      STATIC_ITEM(MSG_FILAMENT_CHANGE_HEADER);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_HEATING_1);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_HEATING_2);
      END_SCREEN();
    }

    static void lcd_advanced_pause_heat_nozzle() {
      START_SCREEN();
      STATIC_ITEM(MSG_FILAMENT_CHANGE_HEADER);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_HEAT_1);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_HEAT_2);
      END_SCREEN();
    }

    static void lcd_advanced_pause_printer_off() {
      START_SCREEN();
      STATIC_ITEM(MSG_FILAMENT_CHANGE_HEADER);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_ZZZ_1);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_ZZZ_2);
      END_SCREEN();
    }

    static void lcd_advanced_pause_insert_message() {
      START_SCREEN();
      STATIC_ITEM(MSG_FILAMENT_CHANGE_HEADER);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_INSERT_1);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_INSERT_2);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_INSERT_3);
      END_SCREEN();
    }

    static void lcd_advanced_pause_load_message() {
      START_SCREEN();
      STATIC_ITEM(MSG_FILAMENT_CHANGE_HEADER);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_LOAD_1);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_LOAD_2);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_LOAD_3);
      END_SCREEN();
    }

    static void lcd_advanced_pause_extrude_message() {
      START_SCREEN();
      STATIC_ITEM(MSG_FILAMENT_CHANGE_HEADER);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_EXTRUDE_1);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_EXTRUDE_2);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_EXTRUDE_3);
      END_SCREEN();
    }

    static void lcd_advanced_pause_resume_message() {
      START_SCREEN();
      STATIC_ITEM(MSG_FILAMENT_CHANGE_HEADER);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_RESUME_1);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_RESUME_2);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_RESUME_3);
      END_SCREEN();
    }

    void lcd_advanced_pause_show_message(AdvancedPauseMessage message) {

      static AdvancedPauseMessage old_message;

      if (old_message != message) {
        switch (message) {
          case ADVANCED_PAUSE_MESSAGE_INIT:
            lcd_advanced_pause_init_message();
            break;
          case ADVANCED_PAUSE_MESSAGE_COOLDOWN:
            lcd_advanced_pause_cool_message();
            break;
          case ADVANCED_PAUSE_MESSAGE_UNLOAD:
            lcd_advanced_pause_unload_message();
            break;
          case ADVANCED_PAUSE_MESSAGE_INSERT:
            lcd_advanced_pause_insert_message();
            break;
          case ADVANCED_PAUSE_MESSAGE_LOAD:
            lcd_advanced_pause_load_message();
            break;
          case ADVANCED_PAUSE_MESSAGE_EXTRUDE:
            lcd_advanced_pause_extrude_message();
            break;
          case ADVANCED_PAUSE_MESSAGE_CLICK_TO_HEAT_NOZZLE:
            lcd_advanced_pause_heat_nozzle();
            break;
          case ADVANCED_PAUSE_MESSAGE_PRINTER_OFF:
            lcd_advanced_pause_printer_off();
            break;
          case ADVANCED_PAUSE_MESSAGE_WAIT_FOR_NOZZLES_TO_HEAT:
            lcd_advanced_pause_wait_for_nozzles_to_heat();
            break;
          case ADVANCED_PAUSE_MESSAGE_OPTION:
            advanced_pause_menu_response = ADVANCED_PAUSE_RESPONSE_WAIT_FOR;
            lcd_advanced_pause_option_menu();
            break;
          case ADVANCED_PAUSE_MESSAGE_RESUME:
            lcd_advanced_pause_resume_message();
            break;
          case ADVANCED_PAUSE_MESSAGE_STATUS:
            Pprinter.show();
            break;
        }
        
        old_message = message;
      }
    }

  #endif // ADVANCED_PAUSE_FEATURE

  #if ENABLED(RFID_MODULE)
    void rfidPopCallback(void *ptr) {
      ZERO(buffer);

      String temp = "M522 ";
      uint32_t Rfid_read = 0;
      RfidR.getValue(&Rfid_read);

      if (ptr == &Rfid0)
        temp += "T0 ";
      else if (ptr == &Rfid1)
        temp += "T1 ";
      else if (ptr == &Rfid2)
        temp += "T2 ";
      else if (ptr == &Rfid3)
        temp += "T3 ";
      else if (ptr == &Rfid4)
        temp += "T4 ";
      else if (ptr == &Rfid5)
        temp += "T5 ";
      
      if(Rfid_read)
        temp += "R";
      else
        temp += "W";

      temp.toCharArray(buffer, sizeof(buffer));
      commands.enqueue_and_echo_command(buffer);
    }

    void rfid_setText(const char* message, uint32_t color /* = 65535 */) {
      char Rfid_status_message[25];
      strncpy(Rfid_status_message, message, 30);
      RfidText.Set_font_color_pco(color);
      RfidText.setText(Rfid_status_message);
    }
  #endif

  #if ENABLED(LCD_BED_LEVELING)

    #if ENABLED(PROBE_MANUALLY)
      bool lcd_wait_for_move;
    #endif

    void line_to_current(AxisEnum axis) {
      planner.buffer_line_kinematic(mechanics.current_position, MMM_TO_MMS(manual_feedrate_mm_m[axis]), tools.active_extruder);
    }

    void bedlevelPopCallBack(void *ptr) {

      if (ptr == &BedUp) {
        mechanics.current_position[Z_AXIS] += (LCD_Z_STEP);
        NOLESS(mechanics.current_position[Z_AXIS], -(LCD_PROBE_Z_RANGE) * 0.5);
        NOMORE(mechanics.current_position[Z_AXIS], (LCD_PROBE_Z_RANGE) * 0.5);
        line_to_current(Z_AXIS);
      }
      else if (ptr == &BedDown) {
        mechanics.current_position[Z_AXIS] -= (LCD_Z_STEP);
        NOLESS(mechanics.current_position[Z_AXIS], -(LCD_PROBE_Z_RANGE) * 0.5);
        NOMORE(mechanics.current_position[Z_AXIS], (LCD_PROBE_Z_RANGE) * 0.5);
        line_to_current(Z_AXIS);
      }
      else if (ptr == &BedSend) {
        #if ENABLED(PROBE_MANUALLY)
          if (bedlevel.g29_in_progress) commands.enqueue_and_echo_commands_P(PSTR("G29"));
          printer.wait_for_user = false;
        #endif
      }
    }

    void LcdBedLevelOn() {
      PBedLevel.show();
      BedMsg.setText(PSTR(MSG_MOVE_Z));
    }

    void LcdBedLevelOff() { Pprinter.show(); }

  #endif

  void hotPopCallback(void *ptr) {
    Ptemp.show();
    ZERO(buffer);
    if (ptr == &Hotend0) {
      if (heaters[0].target_temperature != 0) {
        itoa(heaters[0].target_temperature, buffer, 10);
      }
      theater.setValue(0);
    }
    if (ptr == &Hotend1) {
      if (heaters[1].target_temperature != 0) {
        itoa(heaters[1].target_temperature, buffer, 10);
      }
      theater.setValue(1);
    }

    #if HAS_TEMP_2
      if (ptr == &Hotend2) {
        if (heaters[2].target_temperature != 0) {
          itoa(heaters[2].target_temperature, buffer, 10);
        }
        theater.setValue(2);
      }
    #elif HAS_TEMP_BED
      if (ptr == &Hotend2) {
        if (heaters[BED_INDEX].target_temperature != 0) {
          itoa(heaters[BED_INDEX].target_temperature, buffer, 10);
        }
        theater.setValue(4);
      }
    #endif

    tset.setText(buffer);
  }

  void settempPopCallback(void *ptr) {
    ZERO(buffer);
    tset.getText(buffer, sizeof(buffer));

    uint16_t number = atoi(buffer);

    if (ptr == &tup) number += 1;
    if (ptr == &tdown) number -= 1;

    ZERO(buffer);
    itoa(number, buffer, 10);

    tset.setText(buffer);
  }

  void sethotPopCallback(void *ptr) {
    UNUSED(ptr);

    uint32_t Heater;
    char temp[5] = { 0 };

    tset.getText(temp, sizeof(temp));
    uint16_t temperature = atoi(temp);

    theater.getValue(&Heater);

    #if HAS_TEMP_BED
      if (Heater == 4)
        heaters[BED_INDEX].setTarget(temperature);
      else
    #endif
    #if HAS_TEMP_HOTEND
      heaters[(uint8_t)Heater].setTarget(temperature);
    #endif

    Pprinter.show();
  }

  void setgcodePopCallback(void *ptr) {
    UNUSED(ptr);
    ZERO(buffer);
    Tgcode.getText(buffer, sizeof(buffer));
    Tgcode.setText("");
    commands.enqueue_and_echo_command(buffer);
  }

  #if FAN_COUNT > 0
    void setfanPopCallback(void *ptr) {
      UNUSED(ptr);
      fans[0].Speed = (fans[0].Speed ? 0 : 255);
      Fantimer.enable(fans[0].Speed ? false : true);
    }
  #endif

  void setmovePopCallback(void *ptr) {
    UNUSED(ptr);

    #if EXTRUDERS > 1
      const uint8_t temp_extruder = tools.active_extruder;
      uint32_t new_extruder;
      char temp[5] = {0};

      ZERO(buffer);
      ext.getValue(&new_extruder);
      itoa(new_extruder, temp, 2);
      strcat(buffer, "T");
      strcat(buffer, temp);
      commands.enqueue_and_echo_command(buffer);
    #endif

    ZERO(buffer);
    movecmd.getText(buffer, sizeof(buffer));
    commands.enqueue_and_echo_commands_P(PSTR("G91"));
    commands.enqueue_and_echo_command(buffer);
    commands.enqueue_and_echo_commands_P(PSTR("G90"));

    #if EXTRUDERS > 1
      ZERO(buffer);
      itoa(temp_extruder, temp, 2);
      strcat(buffer, "T");
      strcat(buffer, temp);
      commands.enqueue_and_echo_command(buffer);
    #endif
  }

  void motoroffPopCallback(void *ptr) {
    UNUSED(ptr);
    commands.enqueue_and_echo_commands_P(PSTR("M84"));
  }

  void sendPopCallback(void *ptr) {
    UNUSED(ptr);
    lcd_clicked = true;
    printer.wait_for_user = false;
  }

  void filamentPopCallback(void *ptr) {
    UNUSED(ptr);
    ZERO(buffer);
    Filgcode.getText(buffer, sizeof(buffer));
    commands.enqueue_and_echo_commands_P(PSTR("G91"));
    commands.enqueue_and_echo_command(buffer);
    commands.enqueue_and_echo_commands_P(PSTR("G90"));
  }

  void YesPopCallback(void *ptr) {
    UNUSED(ptr);

    static uint32_t icon = 0;
    Vyes.getValue(&icon);
    switch(icon) {
      #if HAS_SDSUPPORT
        case 1:
        case 2: // StopPrint
          StopPrint(icon == 2); Pprinter.show(); break;
        case 3: // Stop & Save
          UploadNewFirmware(); break;
      #endif
    }
  }

  void lcd_init() {
    HAL::delayMilliseconds(2000);

    for (uint8_t i = 0; i < 10; i++) {
      NextionON = nexInit();
      if (NextionON) break;
      HAL::delayMilliseconds(1000);
    }

    if (!NextionON) {
      SERIAL_EM("Nextion LCD not connected!");
      return;
    }
    else {
      SERIAL_MSG("Nextion LCD");
      // Get Model
      ZERO(buffer);
      getConnect(buffer, sizeof(buffer));

      if (strstr(buffer, "NX4827T043") || strstr(buffer, "NX4827K043")) { // Model 4.3" Normal or Enhanced
        SERIAL_MSG(" 4.3");
        #if ENABLED(NEXTION_GFX)
          gfx.set_position(1, 24, 250, 155);
        #endif
      }
      else if (strstr(buffer, "NX8048T070") || strstr(buffer, "NX8048K070")) { // Model 7" Normal or Enhanced
        SERIAL_MSG(" 7");
        #if ENABLED(NEXTION_GFX)
          gfx.set_position(274, 213, 250, 155);
        #endif
      }
      SERIAL_CHR('"'); SERIAL_EM(" connected!");

      #if ENABLED(NEXTION_GFX)
        gfx.color_set(NX_AXIS + X_AXIS, 63488);
        gfx.color_set(NX_AXIS + Y_AXIS, 2016);
        gfx.color_set(NX_AXIS + Z_AXIS, 31);
        gfx.color_set(NX_MOVE, 2047);
        gfx.color_set(NX_TOOL, 65535);
        gfx.color_set(NX_LOW, 2047);
        gfx.color_set(NX_HIGH, 63488);
      #endif

      #if HAS_SDSUPPORT
        sd_mount.attachPop(sdmountdismountPopCallback, &sd_mount);
        sd_dismount.attachPop(sdmountdismountPopCallback, &sd_dismount);
        sdlist.attachPop(sdlistPopCallback);
        ScrollUp.attachPop(sdlistPopCallback);
        ScrollDown.attachPop(sdlistPopCallback);
        NPlay.attachPop(PlayPausePopCallback);
      #endif

      #if ENABLED(RFID_MODULE)
        Rfid0.attachPop(rfidPopCallback,  &Rfid0);
        Rfid1.attachPop(rfidPopCallback,  &Rfid1);
        Rfid2.attachPop(rfidPopCallback,  &Rfid2);
        Rfid3.attachPop(rfidPopCallback,  &Rfid3);
        Rfid4.attachPop(rfidPopCallback,  &Rfid4);
        Rfid5.attachPop(rfidPopCallback,  &Rfid5);
      #endif

      #if HAS_TEMP_0
        Hotend0.attachPop(hotPopCallback, &Hotend0);
      #endif
      #if HAS_TEMP_1
        Hotend1.attachPop(hotPopCallback, &Hotend1);
      #endif
      #if HAS_TEMP_2 || HAS_TEMP_BED
        Hotend2.attachPop(hotPopCallback, &Hotend2);
      #endif

      #if FAN_COUNT > 0
        Fanpic.attachPop(setfanPopCallback,   &Fanpic);
      #endif

      #if ENABLED(LCD_BED_LEVELING)
        BedUp.attachPop(bedlevelPopCallBack, &BedUp);
        BedSend.attachPop(bedlevelPopCallBack, &BedSend);
        BedDown.attachPop(bedlevelPopCallBack, &BedDown);
      #endif

      tenter.attachPop(sethotPopCallback,   &tenter);
      tup.attachPop(settempPopCallback,     &tup);
      tdown.attachPop(settempPopCallback,   &tdown);
      XYHome.attachPop(setmovePopCallback);
      XYUp.attachPop(setmovePopCallback);
      XYRight.attachPop(setmovePopCallback);
      XYDown.attachPop(setmovePopCallback);
      XYLeft.attachPop(setmovePopCallback);
      ZHome.attachPop(setmovePopCallback);
      ZUp.attachPop(setmovePopCallback);
      ZDown.attachPop(setmovePopCallback);
      Extrude.attachPop(setmovePopCallback);
      Retract.attachPop(setmovePopCallback);
      MotorOff.attachPop(motoroffPopCallback);
      Send.attachPop(setgcodePopCallback);
      Yes.attachPop(YesPopCallback);
      LcdSend.attachPop(sendPopCallback);
      FilLoad.attachPop(filamentPopCallback);
      FilUnload.attachPop(filamentPopCallback);

      setpagePrinter();
      startimer.enable();
    }
  }

  static void degtoLCD(const uint8_t h, const float temp) {

    heater_list0[h]->setValue(temp);

    #if ENABLED(NEXTION_GFX)
      if (!(print_job_counter.isRunning() || IS_SD_PRINTING) && !Wavetemp.getObjVis() && show_Wave) {
        Wavetemp.SetVisibility(true);
      }
    #endif

  }

  static void targetdegtoLCD(const uint8_t h, const float temp) {
    heater_list1[h]->setValue(temp);
  }

  static void coordtoLCD() {
    char* valuetemp;
    ZERO(buffer);

    if (PageID == 2) {
      LcdX.setText(ftostr4sign(mechanics.current_position[X_AXIS]));
      LcdY.setText(ftostr4sign(mechanics.current_position[Y_AXIS]));
      LcdZ.setText(ftostr52sp(FIXFLOAT(mechanics.current_position[Z_AXIS])));
    }
    else if (PageID == 5) {
      strcat(buffer, (mechanics.axis_homed[X_AXIS] ? "X" : "?"));
      if (mechanics.axis_homed[X_AXIS]) {
        valuetemp = ftostr4sign(mechanics.current_position[X_AXIS]);
        strcat(buffer, valuetemp);
      }

      strcat(buffer, (mechanics.axis_homed[Y_AXIS] ? " Y" : " ?"));
      if (mechanics.axis_homed[Y_AXIS]) {
        valuetemp = ftostr4sign(mechanics.current_position[Y_AXIS]);
        strcat(buffer, valuetemp);
      }

      strcat(buffer, (mechanics.axis_homed[Z_AXIS] ? " Z " : " ? "));
      if (mechanics.axis_homed[Z_AXIS]) {
        valuetemp = ftostr52sp(FIXFLOAT(mechanics.current_position[Z_AXIS]));
        strcat(buffer, valuetemp);
      }

      LedCoord5.setText(buffer);
    }
    else if (PageID == 15) {
      BedZ.setText(ftostr43sign(FIXFLOAT(mechanics.current_position[Z_AXIS])));
    }
  }

  void lcd_key_touch_update() {
    if (!NextionON) return;
    nexLoop(nex_listen_list);
  }

  void nextion_draw_update() {

    static uint8_t  PreviousPage = 0,
                    Previousfeedrate = 0,
                    PreviousfanSpeed = 0,
                    PreviouspercentDone = 0;
    static float    PreviousdegHeater[3] = { 0.0 },
                    PrevioustargetdegHeater[3] = { 0.0 };
    char* temp;

    if (!NextionON) return;

    PageID = Nextion_PageID();

    switch (PageID) {
      case 2:
        if (PreviousPage != 2) {
          lcd_setstatus(lcd_status_message);
          #if ENABLED(NEXTION_GFX)
            #if MECH(DELTA)
              gfx_clear((X_MAX_POS) * 2, (Y_MAX_POS) * 2, Z_MAX_POS);
            #else
              gfx_clear(X_MAX_POS, Y_MAX_POS, Z_MAX_POS);
            #endif
          #endif
        }

        #if FAN_COUNT > 0
          if (PreviousfanSpeed != fans[0].Speed) {
            if (fans[0].Speed > 0) {
              Fantimer.enable();
              ZERO(buffer);
              temp = itostr3(((float)fans[0].Speed / 255) * 100);
              strcat(buffer, temp);
              strcat(buffer, "%");
              Fanspeed.setText(buffer);
            }
            else {
              Fantimer.enable(false);
              Fanspeed.setText("");
            }
            PreviousfanSpeed = fans[0].Speed;
          }
        #endif

        if (Previousfeedrate != mechanics.feedrate_percentage) {
          VSpeed.setValue(mechanics.feedrate_percentage);
          Previousfeedrate = mechanics.feedrate_percentage;
        }

        #if HAS_TEMP_0
          if (PreviousdegHeater[0] != heaters[0].current_temperature) {
            PreviousdegHeater[0] = heaters[0].current_temperature;
            degtoLCD(0, PreviousdegHeater[0]);
          }
          if (PrevioustargetdegHeater[0] != heaters[0].target_temperature) {
            PrevioustargetdegHeater[0] = heaters[0].target_temperature;
            targetdegtoLCD(0, PrevioustargetdegHeater[0]);
          }
        #endif
        #if HAS_TEMP_1
          if (PreviousdegHeater[1] != heaters[1].current_temperature) {
            PreviousdegHeater[1] = heaters[1].current_temperature;
            degtoLCD(1, PreviousdegHeater[1]);
          }
          if (PrevioustargetdegHeater[1] != heaters[1].target_temperature) {
            PrevioustargetdegHeater[1] = heaters[1].target_temperature;
            targetdegtoLCD(1, PrevioustargetdegHeater[1]);
          }
        #endif
        #if HAS_TEMP_BED
          if (PreviousdegHeater[2] != heaters[BED_INDEX].current_temperature) {
            PreviousdegHeater[2] = heaters[BED_INDEX].current_temperature;
            degtoLCD(2, PreviousdegHeater[2]);
          }
          if (PrevioustargetdegHeater[2] != heaters[BED_INDEX].target_temperature) {
            PrevioustargetdegHeater[2] = heaters[BED_INDEX].target_temperature;
            targetdegtoLCD(2, PrevioustargetdegHeater[2]);
          }
        #endif

        coordtoLCD();

        #if HAS_SDSUPPORT

          if (card.isFileOpen()) {
            if (SDstatus != 3) {
              SDstatus = 3;
              SD.setValue(SDstatus);
              NPlay.setPic(28);
              NStop.setPic(29);
              NSStop.setPic(177);
            }
            if (IS_SD_PRINTING) {
              if (PreviouspercentDone != card.percentDone()) {
                // Progress bar solid part
                sdbar.setValue(card.percentDone());
                // Estimate End Time
                uint16_t time = print_job_counter.duration() / 60;
                uint16_t end_time = (time * (100 - card.percentDone())) / card.percentDone();
                if (end_time > (60 * 23) || end_time == 0) {
                  LcdTime.setText("S--:-- E--:--");
                }
                else {
                  char temp1[10], temp2[10];
                  sprintf_P(temp1, PSTR("S%i:%i"), time / 60, time%60);
                  sprintf_P(temp2, PSTR("E%i:%i"), end_time / 60, end_time%60);
                  ZERO(buffer);
                  strcat(buffer, temp1);
                  strcat(buffer, " ");
                  strcat(buffer, temp2);
                  LcdTime.setText(buffer);
                }
                PreviouspercentDone = card.percentDone();
              }
            }
            else {
              NPlay.setPic(26);
              NStop.setPic(29);
              NSStop.setPic(177);
            }
          }
          else if (card.cardOK && SDstatus != 2) {
            SDstatus = 2;
            SD.setValue(SDstatus);
            NPlay.setPic(27);
            NStop.setPic(30);
            NSStop.setPic(178);
          }
          else if (!card.cardOK && SDstatus != 1) {
            SDstatus = 1;
            SD.setValue(SDstatus);
            NPlay.setPic(27);
            NStop.setPic(30);
            NSStop.setPic(178);
          }

        #endif // SDSUPPORT

        break;
      #if HAS_SDSUPPORT
        case 3:
          if (PreviousPage != 3) setpageSD();
          break;
      #endif
      case 5:
        coordtoLCD();
        break;
      case 6:
        static uint32_t temp_feedrate = 0;
        VSpeed.getValue(&temp_feedrate, "printer");
        Previousfeedrate = mechanics.feedrate_percentage = (int)temp_feedrate;
        break;
      case 15:
        coordtoLCD();
        break;
    }

    PreviousPage = PageID;
  }

  void lcd_setstatus(const char* message, bool persist) {
    UNUSED(persist);
    if (lcd_status_message_level > 0 || !NextionON) return;
    strncpy(lcd_status_message, message, 30);
    if (PageID == 2) LcdStatus.setText(lcd_status_message);
  }

  void lcd_setstatusPGM(const char* message, int8_t level) {
    if (level < 0) level = lcd_status_message_level = 0;
    if (level < lcd_status_message_level || !NextionON) return;
    strncpy_P(lcd_status_message, message, 30);
    lcd_status_message_level = level;
    if (PageID == 2) LcdStatus.setText(lcd_status_message);
  }

  void lcd_status_printf_P(const uint8_t level, const char * const fmt, ...) {
    if (level < lcd_status_message_level || !NextionON) return;
    lcd_status_message_level = level;
    va_list args;
    va_start(args, fmt);
    vsnprintf(lcd_status_message, 30, fmt, args);
    va_end(args);
    if (PageID == 2) LcdStatus.setText(lcd_status_message);
  }

  void lcd_setalertstatusPGM(const char * const message) {
    lcd_setstatusPGM(message, 1);
  }

  void lcd_reset_alert_level() { lcd_status_message_level = 0; }

  void lcd_scrollinfo(const char* titolo, const char* message) {
    Pinfo.show();
    InfoText.setText(titolo);
    ScrollText.setText(message);
  }

  #if ENABLED(NEXTION_GFX)
    void gfx_origin(const float x, const float y, const float z) {
      gfx.origin(x, y, z);
    }

    void gfx_scale(const float scale) {
      gfx.set_scale(scale);
    }

    void gfx_clear(const float x, const float y, const float z, bool force_clear) {
      if (PageID == 2 && (print_job_counter.isRunning() || IS_SD_PRINTING || force_clear)) {
        Wavetemp.SetVisibility(false);
        show_Wave = !force_clear;
        gfx.clear(x, y, z);
      }
    }

    void gfx_cursor_to(const float x, const float y, const float z, bool force_cursor) {
      if (PageID == 2 && (print_job_counter.isRunning() || IS_SD_PRINTING || force_cursor))
        gfx.cursor_to(x, y, z);
    }

    void gfx_line_to(const float x, const float y, const float z) {
      if (PageID == 2 && (print_job_counter.isRunning() || IS_SD_PRINTING))
        gfx.line_to(NX_TOOL, x, y, z);
    }

    void gfx_plane_to(const float x, const float y, const float z) {
      uint8_t color;
      if (PageID == 2) {
        if (z < 10) color = NX_LOW;
        else color = NX_HIGH;
        gfx.line_to(color, x, y, z, true);
      }
    }
  #endif

#endif
