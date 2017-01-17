/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2016 Alberto Cotronei @MagoKimbra
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

#include "../../base.h"

#if ENABLED(NEXTION)

  #include "Nextion_lcd.h"
  #include "Nextion_gfx.h"
  #include "nextion_lib/Nextion.h"

  bool  NextionON                   = false,
        show_Wave                   = true;
  uint8_t PageID                    = 0,
          lcd_status_message_level  = 0;
  uint16_t slidermaxval             = 20;
  char buffer[100]                  = { 0 };
  char lcd_status_message[30]       = WELCOME_MSG;
  static millis_t next_lcd_update_ms;

  #if ENABLED(SDSUPPORT)
    uint8_t SDstatus    = 0; // 0 card not present, 1 SD not insert, 2 SD insert, 3 SD printing
    NexUpload Firmware(NEXTION_FIRMWARE_FILE, 57600);
  #endif

  #if ENABLED(NEXTION_GFX)
    GFX gfx = GFX(1, 24, 250, 155);
  #endif

  /**
   *******************************************************************
   * Nextion component all page
   *******************************************************************
   */
  NexPage Pstart        = NexPage       (0,   0,  "start");
  NexPage Pmenu         = NexPage       (1,   0,  "menu");
  NexPage Pprinter      = NexPage       (2,   0,  "printer");
  NexPage Psdcard       = NexPage       (3,   0,  "sdcard");
  NexPage Psetup        = NexPage       (4,   0,  "setup");
  NexPage Pmove         = NexPage       (5,   0,  "move");
  NexPage Pspeed        = NexPage       (6,   0,  "speed");
  NexPage Pgcode        = NexPage       (7,   0,  "gcode");
  NexPage Prfid         = NexPage       (8,   0,  "rfid");
  NexPage Pbrightness   = NexPage       (9,   0,  "brightness");
  NexPage Ptemp         = NexPage       (10,  0,  "temp");
  NexPage Pinfo         = NexPage       (11,  0,  "info");
  NexPage Pyesno        = NexPage       (12,  0,  "yesno");
  NexPage Pfilament     = NexPage       (13,  0,  "filament");

  /**
   *******************************************************************
   * Nextion component for page:start
   *******************************************************************
   */
  NexTimer startimer    = NexTimer      (0,   1,  "tm0");

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
  NexVariable Hotend00  = NexVariable   (2,   2,  "he00");
  NexVariable Hotend01  = NexVariable   (2,   3,  "he01");
  NexVariable Hotend10  = NexVariable   (2,   4,  "he10");
  NexVariable Hotend11  = NexVariable   (2,   5,  "he11");
  NexVariable Bed0      = NexVariable   (2,   6,  "bed0");
  NexVariable Bed1      = NexVariable   (2,   7,  "bed1");
  NexVariable Chamber0  = NexVariable   (2,   8,  "cha0");
  NexVariable Chamber1  = NexVariable   (2,   9,  "cha1");
  NexVariable Extruder  = NexVariable   (2,   10, "extruder");
  NexVariable Fan       = NexVariable   (2,   11, "fan");
  NexVariable SD        = NexVariable   (2,   12, "sd");
  NexVariable RFID      = NexVariable   (2,   13, "rfid");
  NexVariable Language  = NexVariable   (2,   14, "lang");
  NexVariable VSpeed    = NexVariable   (2,   15, "vspeed");
  NexTimer Fantimer     = NexTimer      (2,   16, "tm0");
  NexPicture Fanpic     = NexPicture    (2,   19, "p1");
  NexPicture NStop      = NexPicture    (2,   20, "p2");
  NexPicture NPlay      = NexPicture    (2,   21, "p3");
  NexText LedStatus     = NexText       (2,   24, "t0");
  NexText LedCoord1     = NexText       (2,   25, "t1");
  NexText Hotend0       = NexText       (2,   26, "t2");
  NexText Hotend1       = NexText       (2,   27, "t3");
  NexText Hotend2       = NexText       (2,   28, "t4");
  NexText Fanspeed      = NexText       (2,   29, "t5");
  NexWaveform Wavetemp  = NexWaveform   (2,   30, "s0");
  NexProgressBar sdbar  = NexProgressBar(2,   31, "j0");

  /**
   *******************************************************************
   * Nextion component for page:SDCard
   *******************************************************************
   */
  NexText sdrow0        = NexText       (3,   2,  "t0");
  NexText sdrow1        = NexText       (3,   3,  "t1");
  NexText sdrow2        = NexText       (3,   4,  "t2");
  NexText sdrow3        = NexText       (3,   5,  "t3");
  NexText sdrow4        = NexText       (3,   6,  "t4");
  NexText sdrow5        = NexText       (3,   7,  "t5");
  NexText sdfolder      = NexText       (3,   16, "t6");
  NexPicture Folder0    = NexPicture    (3,   8,  "p0");
  NexPicture Folder1    = NexPicture    (3,   9,  "p1");
  NexPicture Folder2    = NexPicture    (3,   10, "p2");
  NexPicture Folder3    = NexPicture    (3,   11, "p3");
  NexPicture Folder4    = NexPicture    (3,   12, "p4");
  NexPicture Folder5    = NexPicture    (3,   13, "p5");
  NexPicture Folderup   = NexPicture    (3,   14, "p6");
  NexPicture ScrollUp   = NexPicture    (3,   18, "p7");
  NexPicture ScrollDown = NexPicture    (3,   19, "p8");
  NexPicture sd_mount   = NexPicture    (3,   22, "p12");
  NexPicture sd_dismount= NexPicture    (3,   23, "p13");
  NexSlider sdlist      = NexSlider     (3,   1,  "h0");

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
  NexPicture MotorOff   = NexPicture    (5,  17,  "p0");
  NexPicture XYHome     = NexPicture    (5,   2,  "p4");
  NexPicture XYUp       = NexPicture    (5,   3,  "p5");
  NexPicture XYRight    = NexPicture    (5,   4,  "p6");
  NexPicture XYDown     = NexPicture    (5,   5,  "p7");
  NexPicture XYLeft     = NexPicture    (5,   6,  "p8");
  NexPicture ZHome      = NexPicture    (5,   7,  "p9");
  NexPicture ZUp        = NexPicture    (5,   8,  "p10");
  NexPicture ZDown      = NexPicture    (5,   9,  "p11");
  NexPicture Extrude    = NexPicture    (5,  20,  "p12");
  NexPicture Retract    = NexPicture    (5,  22,  "p14");
  NexVariable movecmd   = NexVariable   (5,  11,  "vacmd");
  NexVariable ext       = NexVariable   (5,  19,  "va0");
  NexText LedCoord5     = NexText       (5,  12,  "t0");

  /**
   *******************************************************************
   * Nextion component for page:Speed
   *******************************************************************
   */
  NexSlider Speed       = NexSlider     (6,   7,  "h0");

  /**
   *******************************************************************
   * Nextion component for page:GCode
   *******************************************************************
   */
  NexText Tgcode        = NexText       (7,   1,  "tgcode");
  NexButton Send        = NexButton     (7,   44, "b39");

  /**
   *******************************************************************
   * Nextion component for page:Rfid
   *******************************************************************
   */
  NexText RfidText      = NexText       (8,   8,  "t0");
  NexButton Rfid0       = NexButton     (8,   2,  "b0");
  NexButton Rfid1       = NexButton     (8,   3,  "b1");
  NexButton Rfid2       = NexButton     (8,   4,  "b2");
  NexButton Rfid3       = NexButton     (8,   5,  "b3");
  NexButton Rfid4       = NexButton     (8,   6,  "b4");
  NexButton Rfid5       = NexButton     (8,   7,  "b5");
  NexDSButton RfidR     = NexDSButton   (8,   9,  "bt0");

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
  NexText tset          = NexText       (10,  1,  "t0");
  NexVariable theater   = NexVariable   (10,  2,  "va0");
  NexPicture tenter     = NexPicture    (10,  3,  "p5");
  NexPicture tup        = NexPicture    (10,  6,  "p8");
  NexPicture tdown      = NexPicture    (10,  7,  "p9");

  /**
   *******************************************************************
   * Nextion component for page:Info
   *******************************************************************
   */
  NexText InfoText          = NexText       (11,   2,  "t0");
  NexScrolltext ScrollText  = NexScrolltext (11,   3,  "g0");

  /**
   *******************************************************************
   * Nextion component for page:Yesno
   *******************************************************************
   */
  NexVariable Vyes          = NexVariable   (12,  2,  "va0");
  NexText Riga0             = NexText       (12,  4,  "t0");
  NexText Riga1             = NexText       (12,  5,  "t1");
  NexText Riga2             = NexText       (12,  6,  "t2");
  NexText Riga3             = NexText       (12,  7,  "t3");
  NexPicture Yes            = NexPicture    (12,  8,  "p1");
  NexPicture No             = NexPicture    (12,  9,  "p2");

  NexTouch *nex_listen_list[] =
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

    NULL
  };

  NexVariable *heater_list0[] =
  {
    &Hotend00,
    &Hotend10,
    &Bed0,
    &Chamber0,
    NULL
  };

  NexVariable *heater_list1[] =
  {
    &Hotend01,
    &Hotend11,
    &Bed1,
    &Chamber1,
    NULL
  };

  NexText *row_list[] =
  {
    &sdrow0,
    &sdrow1,
    &sdrow2,
    &sdrow3,
    &sdrow4,
    &sdrow5,
    NULL
  };

  NexPicture *folder_list[] =
  {
    &Folder0,
    &Folder1,
    &Folder2,
    &Folder3,
    &Folder4,
    &Folder5,
    NULL
  };

  void setpagePrinter() {

    #if HOTENDS > 0
      Hotend00.setValue(1, "printer");
      #if HOTENDS > 1
        Hotend10.setValue(1, "printer");
      #endif
    #endif

    #if HAS(TEMP_BED)
      Bed0.setValue(1, "printer");
    #endif

    Extruder.setValue(EXTRUDERS, "printer");

    #if ENABLED(SDSUPPORT)
      card.mount();
      if (card.cardOK)
        SDstatus = 2;
      else
        SDstatus = 1;
      SD.setValue(SDstatus, "printer");
    #endif

    VSpeed.setValue(100, "printer");

    #if HAS(FAN)
      Fan.setValue(1, "printer");
    #endif

    #if ENABLED(RFID_MODULE)
      RFID.setValue(1, "printer");
    #endif

    #define LANGUAGE_STRING_(M) STRINGIFY_(M)
    #define LANGUAGE_STRING(M) LANGUAGE_STRING_(M)
    #define NEXTION_LANGUAGE LANGUAGE_STRING(LCD_LANGUAGE)
    Language.setText(NEXTION_LANGUAGE, "printer");
  }

  #if ENABLED(SDSUPPORT)

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
      card.updir();
      setpageSD();
    }

    void PlayPausePopCallback(void *ptr) {
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

    void StopPrint(const bool ssr = false) {
      if (card.cardOK && card.isFileOpen() && IS_SD_PRINTING) {
        card.stopSDPrint(ssr);
        clear_command_queue();
        quickstop_stepper();
        print_job_counter.stop();
        #if ENABLED(AUTOTEMP)
          thermalManager.autotempShutdown();
        #endif
        wait_for_heatup = false;
        lcd_setstatus(MSG_PRINT_ABORTED, true);
      }
    }

  #endif

  #if ENABLED(FILAMENT_CHANGE_FEATURE)

    static void lcd_filament_change_resume_print() {
      filament_change_menu_response = FILAMENT_CHANGE_RESPONSE_RESUME_PRINT;
      Pprinter.show();
    }

    static void lcd_filament_change_extrude_more() {
      filament_change_menu_response = FILAMENT_CHANGE_RESPONSE_EXTRUDE_MORE;
    }

    static void lcd_filament_change_option_menu() {
      Vyes.setValue(5, "yesno");
      Pyesno.show();
      Riga0.setText(MSG_FILAMENT_CHANGE_OPTION_HEADER);
      Riga1.setText(MSG_FILAMENT_CHANGE_OPTION_RESUME);
      Riga2.setText("");
      Riga3.setText("");
    }

    static void lcd_filament_change_init_message() {
      Vyes.setValue(0, "yesno");
      Pyesno.show();
      Riga0.setText(MSG_FILAMENT_CHANGE_HEADER);
      Riga1.setText(MSG_FILAMENT_CHANGE_INIT_1);
      #ifdef MSG_FILAMENT_CHANGE_INIT_2
        Riga2.setText(MSG_FILAMENT_CHANGE_INIT_2);
      #else
        Riga2.setText("");
      #endif
      #ifdef MSG_FILAMENT_CHANGE_INIT_3
        Riga3.setText(MSG_FILAMENT_CHANGE_INIT_3);
      #else
        Riga3.setText("");
      #endif
    }

    static void lcd_filament_change_unload_message() {
      Vyes.setValue(0, "yesno");
      Pyesno.show();
      Riga0.setText(MSG_FILAMENT_CHANGE_HEADER);
      Riga1.setText(MSG_FILAMENT_CHANGE_UNLOAD_1);
      #ifdef MSG_FILAMENT_CHANGE_UNLOAD_2
        Riga2.setText(MSG_FILAMENT_CHANGE_UNLOAD_2);
      #else
        Riga2.setText("");
      #endif
      #ifdef MSG_FILAMENT_CHANGE_UNLOAD_3
        Riga3.setText(MSG_FILAMENT_CHANGE_UNLOAD_3);
      #else
        Riga3.setText("");
      #endif
    }

    static void lcd_filament_change_insert_message() {
      Vyes.setValue(4, "yesno");
      Pyesno.show();
      Riga0.setText(MSG_FILAMENT_CHANGE_HEADER);
      Riga1.setText(MSG_FILAMENT_CHANGE_INSERT_1);
      #ifdef MSG_FILAMENT_CHANGE_INSERT_2
        Riga2.setText(MSG_FILAMENT_CHANGE_INSERT_2);
      #else
        Riga2.setText("");
      #endif
      #ifdef MSG_FILAMENT_CHANGE_INSERT_3
        Riga3.setText(MSG_FILAMENT_CHANGE_INSERT_3);
      #else
        Riga3.setText("");
      #endif
    }

    static void lcd_filament_change_load_message() {
      Vyes.setValue(0, "yesno");
      Pyesno.show();
      Riga0.setText(MSG_FILAMENT_CHANGE_HEADER);
      Riga1.setText(MSG_FILAMENT_CHANGE_LOAD_1);
      #ifdef MSG_FILAMENT_CHANGE_LOAD_2
        Riga2.setText(MSG_FILAMENT_CHANGE_LOAD_2);
      #else
        Riga2.setText("");
      #endif
      #ifdef MSG_FILAMENT_CHANGE_LOAD_3
        Riga3.setText(MSG_FILAMENT_CHANGE_LOAD_3);
      #else
        Riga3.setText("");
      #endif
    }

    static void lcd_filament_change_extrude_message() {
      Vyes.setValue(0, "yesno");
      Pyesno.show();
      Riga0.setText(MSG_FILAMENT_CHANGE_HEADER);
      Riga1.setText(MSG_FILAMENT_CHANGE_EXTRUDE_1);
      #ifdef MSG_FILAMENT_CHANGE_EXTRUDE_2
        Riga2.setText(MSG_FILAMENT_CHANGE_EXTRUDE_2);
      #else
        Riga2.setText("");
      #endif
      #ifdef MSG_FILAMENT_CHANGE_EXTRUDE_3
        Riga3.setText(MSG_FILAMENT_CHANGE_EXTRUDE_3);
      #else
        Riga3.setText("");
      #endif
    }

    static void lcd_filament_change_resume_message() {
      Vyes.setValue(0, "yesno");
      Pyesno.show();
      Riga0.setText(MSG_FILAMENT_CHANGE_HEADER);
      Riga1.setText(MSG_FILAMENT_CHANGE_RESUME_1);
      #ifdef MSG_FILAMENT_CHANGE_RESUME_2
        Riga2.setText(MSG_FILAMENT_CHANGE_RESUME_2);
      #else
        Riga2.setText("");
      #endif
      #ifdef MSG_FILAMENT_CHANGE_RESUME_3
        Riga3.setText(MSG_FILAMENT_CHANGE_RESUME_3);
      #else
        Riga2.setText("");
      #endif
    }

    void lcd_filament_change_show_message(FilamentChangeMessage message) {
      switch (message) {
        case FILAMENT_CHANGE_MESSAGE_INIT:
          lcd_filament_change_init_message();
          break;
        case FILAMENT_CHANGE_MESSAGE_UNLOAD:
          lcd_filament_change_unload_message();
          break;
        case FILAMENT_CHANGE_MESSAGE_INSERT:
          lcd_filament_change_insert_message();
          break;
        case FILAMENT_CHANGE_MESSAGE_LOAD:
          lcd_filament_change_load_message();
          break;
        case FILAMENT_CHANGE_MESSAGE_EXTRUDE:
          lcd_filament_change_extrude_message();
          break;
        case FILAMENT_CHANGE_MESSAGE_OPTION:
          filament_change_menu_response = FILAMENT_CHANGE_RESPONSE_WAIT_FOR;
          lcd_filament_change_option_menu();
          break;
        case FILAMENT_CHANGE_MESSAGE_RESUME:
          lcd_filament_change_resume_message();
          break;
        case FILAMENT_CHANGE_MESSAGE_STATUS:
          Pprinter.show();
          break;
      }
    }

  #endif // FILAMENT_CHANGE_FEATURE

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
      enqueue_and_echo_command(buffer);
    }

    void rfid_setText(const char* message, uint32_t color /* = 65535 */) {
      char Rfid_status_message[25];
      strncpy(Rfid_status_message, message, 30);
      RfidText.Set_font_color_pco(color);
      RfidText.setText(Rfid_status_message);
    }
  #endif

  void hotPopCallback(void *ptr) {
    Ptemp.show();
    ZERO(buffer);
    if (ptr == &Hotend0) {
      if (thermalManager.degTargetHotend(0) != 0) {
        itoa(thermalManager.degTargetHotend(0), buffer, 10);
      }
      theater.setValue(0);
    }
    if (ptr == &Hotend1) {
      if (thermalManager.degTargetHotend(1) != 0) {
        itoa(thermalManager.degTargetHotend(1), buffer, 10);
      }
      theater.setValue(1);
    }

    #if HAS_TEMP_2
      if (ptr == &Hotend2) {
        if (thermalManager.degTargetHotend(2) != 0) {
          itoa(thermalManager.degTargetHotend(2), buffer, 10);
        }
        theater.setValue(2);
      }
    #elif HAS_TEMP_BED
      if (ptr == &Hotend2) {
        if (thermalManager.degTargetBed() != 0) {
          itoa(thermalManager.degTargetBed(), buffer, 10);
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
    uint32_t Heater;
    char temp[5] = { 0 };

    tset.getText(temp, sizeof(temp));
    uint16_t temperature = atoi(temp);

    theater.getValue(&Heater);

    if (Heater == 4)
      thermalManager.setTargetBed(temperature);
    else
      thermalManager.setTargetHotend(temperature, (uint8_t)Heater);

    Pprinter.show();
  }

  void setgcodePopCallback(void *ptr) {
    ZERO(buffer);
    Tgcode.getText(buffer, sizeof(buffer));
    enqueue_and_echo_command(buffer);
  }

  void setfanPopCallback(void *ptr) {
    if (fanSpeed) {
      fanSpeed = 0;
      Fantimer.disable();
    }
    else {
      fanSpeed = 255;
      Fantimer.enable();
    }
  }

  void setmovePopCallback(void *ptr) {

    #if EXTRUDERS > 1
      const uint8_t temp_extruder = active_extruder;
      uint32_t new_extruder;
      char temp[5] = {0};

      ZERO(buffer);
      ext.getValue(&new_extruder);
      itoa(new_extruder, temp, 2);
      strcat(buffer, "T");
      strcat(buffer, temp);
      enqueue_and_echo_command(buffer);
    #endif

    ZERO(buffer);
    movecmd.getText(buffer, sizeof(buffer));
    enqueue_and_echo_commands_P(PSTR("G91"));
    enqueue_and_echo_command(buffer);
    enqueue_and_echo_commands_P(PSTR("G90"));

    #if EXTRUDERS > 1
      ZERO(buffer);
      itoa(temp_extruder, temp, 2);
      strcat(buffer, "T");
      strcat(buffer, temp);
      enqueue_and_echo_command(buffer);
    #endif
  }

  void motoroffPopCallback(void *ptr) {
    enqueue_and_echo_commands_P(PSTR("M84"));
  }

  void YesPopCallback(void *ptr) {
    static uint32_t icon = 0;
    Vyes.getValue(&icon);
    switch(icon) {
      #if ENABLED(SDSUPPORT)
        case 1:
        case 2: // StopPrint
          StopPrint(icon == 2); Pprinter.show(); break;
        case 3: // Stop & Save
          UploadNewFirmware(); break;
      #endif
      #if ENABLED(FILAMENT_CHANGE_FEATURE)
        case 4: // Filament click
          wait_for_user = false; break;
        case 5: // Filament resume print
          lcd_filament_change_resume_print(); break;
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
      SERIAL_EM("Nextion LCD connected!");

      #if ENABLED(NEXTION_GFX)
        gfx.color_set(NX_AXIS + X_AXIS, 63488);
        gfx.color_set(NX_AXIS + Y_AXIS, 2016);
        gfx.color_set(NX_AXIS + Z_AXIS, 31);
        gfx.color_set(NX_MOVE, 2047);
        gfx.color_set(NX_TOOL, 65535);
        gfx.color_set(NX_LOW, 2047);
        gfx.color_set(NX_HIGH, 63488);
      #endif

      #if ENABLED(SDSUPPORT)
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

      #if HAS(TEMP_0)
        Hotend0.attachPop(hotPopCallback, &Hotend0);
      #endif
      #if HAS(TEMP_1)
        Hotend1.attachPop(hotPopCallback, &Hotend1);
      #endif
      #if HAS(TEMP_2) || HAS(TEMP_BED)
        Hotend2.attachPop(hotPopCallback, &Hotend2);
      #endif

      Fanpic.attachPop(setfanPopCallback,   &Fanpic);
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

      setpagePrinter();
      startimer.enable();
    }
  }

  static void degtoLCD(const uint8_t h, const float temp) {
    static const uint16_t maxTemp =
      #if HOTENDS == 1
        HEATER_0_MAXTEMP;
      #else
        200;
      #endif

    heater_list0[h]->setValue(temp);

    #if ENABLED(NEXTION_GFX) && ENABLED(NEXTION_WAVETEMP)
      if (!(print_job_counter.isRunning() || IS_SD_PRINTING) && !Wavetemp.GetSatus() && show_Wave) {
        Wavetemp.SetVisibility(true);
      }
    #endif

    #if ENABLED(NEXTION_WAVETEMP)
      if (h == 0) {
        uint8_t wavetemp = (temp / maxTemp) * 150;
        Wavetemp.addValue(0, wavetemp);
      }
    #endif
  }

  static void targetdegtoLCD(const uint8_t h, const float temp) {
    heater_list1[h]->setValue(temp);
  }

  static void coordtoLCD() {
    char* valuetemp;
    ZERO(buffer);

    #if NOMECH(DELTA)
      strcat(buffer, (axis_homed[X_AXIS] ? "X" : "?"));
      if (axis_homed[X_AXIS]) {
        valuetemp = ftostr4sign(current_position[X_AXIS]);
        strcat(buffer, valuetemp);
      }

      strcat(buffer, (axis_homed[Y_AXIS] ? " Y" : " ?"));
      if (axis_homed[Y_AXIS]) {
        valuetemp = ftostr4sign(current_position[Y_AXIS]);
        strcat(buffer, valuetemp);
      }
    #endif

    strcat(buffer, (axis_homed[Z_AXIS] ? " Z " : " ? "));
    if (axis_homed[Z_AXIS]) {
      valuetemp = ftostr52sp(current_position[Z_AXIS] + 0.00001);
      strcat(buffer, valuetemp);
    }

    if (PageID == 2) LedCoord1.setText(buffer);
    else LedCoord5.setText(buffer);
  }

  void lcd_update() {
    static uint8_t  PreviousPage = 0,
                    Previousfeedrate = 0,
                    PreviousfanSpeed = 0,
                    PreviouspercentDone = 0;
    static float    PreviousdegHeater[3] = { 0.0 },
                    PrevioustargetdegHeater[3] = { 0.0 };
    char* temp;

    if (!NextionON) return;

    nexLoop(nex_listen_list);

    millis_t ms = millis();

    if (ELAPSED(ms, next_lcd_update_ms)) {

      next_lcd_update_ms = ms + NEXTION_UPDATE_INTERVAL;

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

          if (PreviousfanSpeed != fanSpeed) {
            if (fanSpeed > 0) {
              Fantimer.enable();
              ZERO(buffer);
              temp = itostr3(((float)fanSpeed / 255) * 100);
              strcat(buffer, temp);
              strcat(buffer, "%");
              Fanspeed.setText(buffer);
            }
            else {
              Fantimer.disable();
              Fanspeed.setText("");
            }
            PreviousfanSpeed = fanSpeed;
          }

          if (Previousfeedrate != feedrate_percentage) {
            VSpeed.setValue(feedrate_percentage);
            Previousfeedrate = feedrate_percentage;
          }

          #if HAS(TEMP_0)
            if (PreviousdegHeater[0] != thermalManager.degHotend(0)) {
              PreviousdegHeater[0] = thermalManager.degHotend(0);
              degtoLCD(0, PreviousdegHeater[0]);
            }
            if (PrevioustargetdegHeater[0] != thermalManager.degTargetHotend(0)) {
              PrevioustargetdegHeater[0] = thermalManager.degTargetHotend(0);
              targetdegtoLCD(0, PrevioustargetdegHeater[0]);
            }
          #endif
          #if HAS(TEMP_1)
            if (PreviousdegHeater[1] != thermalManager.degHotend(1)) {
              PreviousdegHeater[1] = thermalManager.degHotend(1);
              degtoLCD(1, PreviousdegHeater[1]);
            }
            if (PrevioustargetdegHeater[1] != thermalManager.degTargetHotend(1)) {
              PrevioustargetdegHeater[1] = thermalManager.degTargetHotend(1);
              targetdegtoLCD(1, PrevioustargetdegHeater[1]);
            }
          #endif
          #if HAS(TEMP_BED)
            if (PreviousdegHeater[2] != thermalManager.degBed()) {
              PreviousdegHeater[2] = thermalManager.degBed();
              degtoLCD(2, PreviousdegHeater[2]);
            }
            if (PrevioustargetdegHeater[2] != thermalManager.degTargetBed()) {
              PrevioustargetdegHeater[2] = thermalManager.degTargetBed();
              targetdegtoLCD(2, PrevioustargetdegHeater[2]);
            }
          #endif

          coordtoLCD();

          #if ENABLED(SDSUPPORT)

            if (card.isFileOpen()) {
              if (SDstatus != 3) {
                SDstatus = 3;
                SD.setValue(SDstatus);
                NPlay.setPic(28);
                NStop.setPic(29);
              }
              if (IS_SD_PRINTING) {
                if (PreviouspercentDone != card.percentDone()) {
                  // Progress bar solid part
                  sdbar.setValue(card.percentDone());
                  // Estimate End Time
                  uint16_t time = print_job_counter.duration() / 60;
                  uint16_t end_time = (time * (100 - card.percentDone())) / card.percentDone();
                  if (end_time > (60 * 23) || end_time == 0) {
                    lcd_setstatus("End --:--");
                  }
                  else {
                    char temp[30];
                    sprintf_P(temp, PSTR("End %i:%i"), end_time / 60, end_time%60);
                    lcd_setstatus(temp);
                  }
                  PreviouspercentDone = card.percentDone();
                }
              }
              else {
                NPlay.setPic(26);
                NStop.setPic(29);
              }
            }
            else if (card.cardOK && SDstatus != 2) {
              SDstatus = 2;
              SD.setValue(SDstatus);
              NPlay.setPic(27);
              NStop.setPic(30);
            }
            else if (!card.cardOK && SDstatus != 1) {
              SDstatus = 1;
              SD.setValue(SDstatus);
              NPlay.setPic(27);
              NStop.setPic(30);
            }

          #endif // SDSUPPORT

          break;
        #if ENABLED(SDSUPPORT)
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
          Previousfeedrate = feedrate_percentage = (int)temp_feedrate;
          break;
      }

      PreviousPage = PageID;
    }
  }

  void lcd_setstatus(const char* message, bool persist) {
    if (lcd_status_message_level > 0 || !NextionON) return;
    strncpy(lcd_status_message, message, 30);
    if (PageID == 2) LedStatus.setText(lcd_status_message);
  }

  void lcd_setstatuspgm(const char* message, uint8_t level) {
    if (level >= lcd_status_message_level && NextionON) {
      strncpy_P(lcd_status_message, message, 30);
      lcd_status_message_level = level;
      if (PageID == 2) LedStatus.setText(lcd_status_message);
    }
  }

  void lcd_setalertstatuspgm(const char* message) {
    lcd_setstatuspgm(message, 1);
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
