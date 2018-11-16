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
 * nextionlcd.cpp
 *
 * Copyright (c) 2014 Alberto Cotronei @MagoKimbra
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

#include "../../../MK4duo.h"

#if HAS_NEXTION_LCD

  #include "library/nextion.h"
  #include "nextion_gfx.h"

  LcdUI       lcdui;

  char        LcdUI::status_message[30] = WELCOME_MSG;
  uint8_t     LcdUI::status_message_level; // = 0

  #if HAS_LCD_MENU

    LCDViewActionEnum LcdUI::lcdDrawUpdate = LCDVIEW_CLEAR_CALL_REDRAW;

    uint32_t    LcdUI::encoderPosition;

    screenFunc_t  LcdUI::currentScreen;

    #if ENABLED(REVERSE_MENU_DIRECTION)
      int8_t LcdUI::encoderDirection = 1;
    #endif

    bool          LcdUI::lcd_clicked;
    float         move_menu_scale;

    #if LCD_TIMEOUT_TO_STATUS
      bool LcdUI::defer_return_to_status;
    #endif

    extern bool no_reentry; // Flag to prevent recursion into menu handlers

    int8_t manual_move_axis = (int8_t)NO_AXIS;
    millis_t manual_move_start_time = 0;

    #if IS_KINEMATIC
      bool LcdUI::processing_manual_move = false;
      float manual_move_offset = 0;
    #endif

    #if E_MANUAL > 1
      int8_t LcdUI::manual_move_e_index = 0;
    #endif

    bool menu_redraw = true;

  #endif

  bool        NextionON                 = false,
              show_Wave                 = true;
  uint8_t     PageID                    = 0;
  uint16_t    slidermaxval              = 20;
  char        buffer[70]                = { 0 };

  #if HAS_SD_SUPPORT
    // 0 card not present, 1 SD not insert, 2 SD insert, 3 SD printing
    enum SDstatus_enum {NO_SD = 0, SD_NO_INSERT = 1, SD_INSERT = 2, SD_PRINTING = 3, SD_PAUSE = 4 };
    SDstatus_enum SDstatus    = NO_SD;
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
  NexObject Ptxtmenu      = NexObject(14, 0,  "txtmenu");
  NexObject Poptions      = NexObject(15, 0,  "options");
  NexObject Ptime         = NexObject(16, 0,  "time");
  NexObject Pusertemp     = NexObject(17, 0,  "usertemp");

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
  NexObject Version     = NexObject(1, 10,  "t0");

  /**
   *******************************************************************
   * Nextion component for page:printer
   *******************************************************************
   */
  NexObject LcdX        = NexObject(2,  4,  "vx");
  NexObject LcdY        = NexObject(2,  5,  "vy");
  NexObject LcdZ        = NexObject(2,  6,  "vz");
  NexObject Extruders   = NexObject(2,  7,  "extruder");
  NexObject Hotend00    = NexObject(2,  8,  "he00");
  NexObject Hotend01    = NexObject(2,  9,  "he01");
  NexObject Hotend10    = NexObject(2, 10,  "he10");
  NexObject Hotend11    = NexObject(2, 11,  "he11");
  NexObject Bed0        = NexObject(2, 12,  "bed0");
  NexObject Bed1        = NexObject(2, 13,  "bed1");
  NexObject Chamber0    = NexObject(2, 14,  "cha0");
  NexObject Chamber1    = NexObject(2, 15,  "cha1");
  NexObject DHT0        = NexObject(2, 16,  "dht0");
  NexObject SD          = NexObject(2, 17,  "sd");
  NexObject RFID        = NexObject(2, 18,  "rfid");
  NexObject Fan         = NexObject(2, 19,  "fan");
  NexObject Fanspeed    = NexObject(2, 20,  "fs");
  NexObject VSpeed      = NexObject(2, 21,  "vs");
  NexObject Language    = NexObject(2, 22,  "lang");
  NexObject LightStatus = NexObject(2, 23,  "light");
  NexObject NStop       = NexObject(2, 34,  "p1");
  NexObject NPlay       = NexObject(2, 35,  "p2");
  NexObject Light       = NexObject(2, 36,  "p3");
  NexObject LcdStatus   = NexObject(2, 91,  "t0");
  NexObject LcdCommand  = NexObject(2, 92,  "t1");
  NexObject LcdTime     = NexObject(2, 93,  "t2");
  NexObject progressbar = NexObject(2, 94,  "j0");
  NexObject Wavetemp    = NexObject(2, 95,  "s0");
  NexObject Hot0Touch   = NexObject(2, 96,  "m0");
  NexObject Hot1Touch   = NexObject(2, 97,  "m1");
  NexObject Hot2Touch   = NexObject(2, 98,  "m2");
  NexObject FanTouch    = NexObject(2, 99,  "m3");

  /**
   *******************************************************************
   * Nextion component for page:SDCard
   *******************************************************************
   */
  NexObject sdlist      = NexObject(3,   1, "h0");
  NexObject sdrow0      = NexObject(3,   2, "t0");
  NexObject sdrow1      = NexObject(3,   3, "t1");
  NexObject sdrow2      = NexObject(3,   4, "t2");
  NexObject sdrow3      = NexObject(3,   5, "t3");
  NexObject sdrow4      = NexObject(3,   6, "t4");
  NexObject sdrow5      = NexObject(3,   7, "t5");
  NexObject Folder0     = NexObject(3,   8, "p0");
  NexObject Folder1     = NexObject(3,   9, "p1");
  NexObject Folder2     = NexObject(3,  10, "p2");
  NexObject Folder3     = NexObject(3,  11, "p3");
  NexObject Folder4     = NexObject(3,  12, "p4");
  NexObject Folder5     = NexObject(3,  13, "p5");
  NexObject Folderup    = NexObject(3,  14, "p6");
  NexObject sdfolder    = NexObject(3,  16, "t6");
  NexObject ScrollUp    = NexObject(3,  18, "p7");
  NexObject ScrollDown  = NexObject(3,  19, "p8");
  NexObject sd_mount    = NexObject(3,  21, "p12");
  NexObject sd_dismount = NexObject(3,  22, "p13");

  /**
   *******************************************************************
   * Nextion component for page:Setup
   *******************************************************************
   */
  NexObject TxtMenu     = NexObject(4,   3, "m2");

  /**
   *******************************************************************
   * Nextion component for page:Move
   *******************************************************************
   */
  NexObject XYHome      = NexObject(5,   2, "p4");
  NexObject XYUp        = NexObject(5,   3, "p5");
  NexObject XYRight     = NexObject(5,   4, "p6");
  NexObject XYDown      = NexObject(5,   5, "p7");
  NexObject XYLeft      = NexObject(5,   6, "p8");
  NexObject ZHome       = NexObject(5,   7, "p9");
  NexObject ZUp         = NexObject(5,   8, "p10");
  NexObject ZDown       = NexObject(5,   9, "p11");
  NexObject movecmd     = NexObject(5,  11, "vacmd");
  NexObject LedCoord5   = NexObject(5,  12, "t0");
  NexObject MotorOff    = NexObject(5,  17, "p0");
  NexObject ext         = NexObject(5,  18, "va0");
  NexObject Extrude     = NexObject(5,  19, "p12");
  NexObject Retract     = NexObject(5,  21, "p14");
  NexObject SpeedX      = NexObject(5,  22, "vafrx");
  NexObject SpeedY      = NexObject(5,  23, "vafry");
  NexObject SpeedZ      = NexObject(5,  24, "vafrz");
  NexObject SpeedE      = NexObject(5,  25, "vafre");

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
  NexObject Tgcode      = NexObject(7,   1, "tgcode");
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
  NexObject theater     = NexObject(10,  1, "va0");
  NexObject tenter      = NexObject(10,  2, "p5");
  NexObject tset        = NexObject(10, 15, "tmp");

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
  NexObject FilLoad     = NexObject(13,  3, "p2");
  NexObject FilUnload   = NexObject(13,  4, "p3");
  NexObject FilExtr     = NexObject(13,  5, "p4");
  NexObject Filgcode    = NexObject(13, 10, "vacmd");

  /**
   *******************************************************************
   * Nextion component for page:Select
   *******************************************************************
   */
  NexObject MenuRow1    = NexObject(14,  1, "t0");
  NexObject MenuRow2    = NexObject(14,  2, "t1");
  NexObject MenuRow3    = NexObject(14,  3, "t2");
  NexObject MenuRow4    = NexObject(14,  4, "t3");
  NexObject MenuRow5    = NexObject(14,  5, "t4");
  NexObject MenuRow6    = NexObject(14,  6, "t5");
  NexObject EncUp       = NexObject(14,  7, "p0");
  NexObject EncSend     = NexObject(14,  8, "p1");
  NexObject EncDown     = NexObject(14,  9, "p2");
  NexObject EncMenu     = NexObject(14, 10, "p3");

  NexObject *nex_listen_list[] =
  {
    // Page 2 touch listen
    &FanTouch, &NPlay, &Light,

    // Page 3 touch listen
    &sdlist, &ScrollUp, &ScrollDown, &sdrow0, &sdrow1, &sdrow2,
    &sdrow3, &sdrow4, &sdrow5, &Folderup, &sd_mount, &sd_dismount,

    // Page 4 touch listen
    &TxtMenu,

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
    &tenter,

    // Page 12 touch listen
    &Yes, &No,

    // Page 13 touch listen
    &FilLoad, &FilUnload, &FilExtr,

    // Page 14 touch listen
    &EncUp, &EncDown, &EncSend, &EncMenu,

    NULL
  };

  NexObject *heater_list0[] =
  {
    &Hotend00,
    &Hotend10,
    &Bed0,
    &Chamber0,
    &DHT0,
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

  /**
   *
   * Menu actions
   *
   */
  static void setpagePrinter() {
    char temp[10] = { 0 };

    Version.setText(SHORT_BUILD_VERSION, "menu");

    #if HOTENDS > 0
      Hotend00.setValue(1, "printer");
      #if HOTENDS > 1
        Hotend10.setValue(1, "printer");
      #elif HAS_TEMP_CHAMBER
        Chamber0.setValue(1, "printer");
      #elif ENABLED(DHT_SENSOR)
        DHT0.setValue(1, "printer");
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

    #if HAS_SD_SUPPORT
      if (!card.isOK()) card.mount();
      HAL::delayMilliseconds(500);
      if (card.isOK()) {
        SDstatus = SD_INSERT;
        card.beginautostart();  // Initial boot
      }
      else
        SDstatus = SD_NO_INSERT;
      SD.setValue(SDstatus, "printer");
    #endif

    VSpeed.setValue(100, "printer");

    #if FAN_COUNT > 0
      Fan.setValue(1, "printer");
    #endif

    #if HAS_CASE_LIGHT
      LightStatus.setValue(caselight.status ? 2 : 1, "printer");
    #endif

    #if ENABLED(RFID_MODULE)
      RFID.setValue(1, "printer");
    #endif

    #define LANGUAGE_STRING(M) STRINGIFY(M)
    #define NEXTION_LANGUAGE LANGUAGE_STRING(LCD_LANGUAGE)
    Language.setText(NEXTION_LANGUAGE, "printer");
  }

  #if HAS_SD_SUPPORT

    void UploadNewFirmware() {
      if (IS_SD_INSERTED() || card.isOK()) {
        Firmware.startUpload();
        nexSerial.end();
        lcdui.init();
      }
    }

    static void printrowsd(uint8_t row, const bool folder, PGM_P filename) {
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
      uint16_t fileCnt = card.get_num_Files();
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

      if (fileCnt) {
        for (uint8_t row = 0; row < 6; row++) {
          i = row + number;
          if (i < fileCnt) {
            card.getfilename_sorted(i);
            printrowsd(row, card.isFilenameIsDir(), card.fileName);
          } else {
            printrowsd(row, false, "");
          }
        }
      }
      sendCommand("ref 0");
    }

    static void menu_action_sdfile(PGM_P filename) {
      card.openAndPrintFile(filename);
      Pprinter.show();
    }

    static void menu_action_sddirectory(PGM_P filename) {
      card.chdir(filename);
      setpageSD();
    }

    void setpageSD() {
      uint16_t fileCnt = card.get_num_Files();

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
        if (card.isOK())
          SDstatus = SD_INSERT;
        else
          SDstatus = SD_NO_INSERT;
        SD.setValue(SDstatus, "printer");
      }
      else {
        card.unmount();
        SDstatus = SD_NO_INSERT;
        SD.setValue(SDstatus, "printer");
      }
      setpageSD();
    }

    void sdlistPopCallback(void *ptr) {
      UNUSED(ptr);
      uint16_t number = slidermaxval - sdlist.getValue();
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

      if (card.isOK() && card.isFileOpen()) {
        if (IS_SD_PRINTING()) {
          card.pauseSDPrint();
          print_job_counter.pause();
          #if ENABLED(PARK_HEAD_ON_PAUSE)
            commands.enqueue_and_echo_P(PSTR("M125"));
          #endif
        }
        else {
          card.startFileprint();
          print_job_counter.start();
        }
      }
    }

  #endif

  #if ENABLED(RFID_MODULE)

    void rfidPopCallback(void *ptr) {
      ZERO(buffer);

      String temp = "M522 ";
      uint16_t Rfid_read = RfidR.getValue();

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
      commands.enqueue_and_echo(buffer);
    }

    void rfid_setText(PGM_P message, uint32_t color /* = 65535 */) {
      char Rfid_status_message[25];
      strncpy(Rfid_status_message, message, 30);
      RfidText.Set_font_color_pco(color);
      RfidText.setText(Rfid_status_message);
    }

  #endif

  void sethotPopCallback(void *ptr) {
    UNUSED(ptr);

    uint16_t  Heater      = theater.getValue(),
              temperature = tset.getValue();

    #if HAS_TEMP_BED
      if (Heater == 2)
        heaters[BED_INDEX].setTarget(temperature);
      else
    #endif
    #if HAS_TEMP_CHAMBER
      if (Heater == 3)
        heaters[CHAMBER_INDEX].setTarget(temperature);
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
    Tgcode.getText(buffer, sizeof(buffer), "gcode");
    Tgcode.setText("", "gcode");
    commands.enqueue_and_echo(buffer);
  }

  #if FAN_COUNT > 0
    void setfanPopCallback(void *ptr) {
      UNUSED(ptr);
      fans[0].Speed = (fans[0].Speed ? 0 : 255);
      Fanspeed.setValue(((float)(fans[0].Speed) / 255) * 100);
    }
  #endif

  #if HAS_CASE_LIGHT
    void setlightPopCallback(void *ptr) {
      UNUSED(ptr);
      caselight.status = !caselight.status;
      caselight.update();
    }
  #endif

  void setmovePopCallback(void *ptr) {
    UNUSED(ptr);

    #if EXTRUDERS > 1
      const uint8_t temp_extruder = tools.active_extruder;
      char temp[5] = { 0 };

      ZERO(buffer);
      itoa(ext.getValue(), temp, 2);
      strcat(buffer, "T");
      strcat(buffer, temp);
      commands.enqueue_and_echo(buffer);
    #endif

    ZERO(buffer);
    movecmd.getText(buffer, sizeof(buffer));
    commands.enqueue_and_echo_P(PSTR("G91"));
    commands.enqueue_and_echo(buffer);
    commands.enqueue_and_echo_P(PSTR("G90"));

    #if EXTRUDERS > 1
      ZERO(buffer);
      itoa(temp_extruder, temp, 2);
      strcat(buffer, "T");
      strcat(buffer, temp);
      commands.enqueue_and_echo(buffer);
    #endif
  }

  void motoroffPopCallback(void *ptr) {
    UNUSED(ptr);
    commands.enqueue_and_echo_P(PSTR("M84"));
  }

  void filamentPopCallback(void *ptr) {
    ZERO(buffer);
    Filgcode.getText(buffer, sizeof(buffer));
    if (ptr == &FilExtr)
      commands.enqueue_and_echo(buffer);
    else {
      commands.enqueue_and_echo_P(PSTR("G91"));
      commands.enqueue_and_echo(buffer);
      commands.enqueue_and_echo_P(PSTR("G90"));
    }
  }

  void YesNoPopCallback(void *ptr) {

    if (ptr == &Yes) {
      switch(Vyes.getValue()) {
        #if HAS_SD_SUPPORT
          case 1: // Stop Print
            card.setAbortSDprinting(true);
            lcdui.setstatusPGM(PSTR(MSG_PRINT_ABORTED), -1);
            Pprinter.show();
            break;
          case 2: // Upload Firmware
            UploadNewFirmware(); break;
        #endif
        case 3: // Unconditional stop
          printer.setWaitForUser(false);
          Pprinter.show();
          break;
        default: break;
      }
    }
    else {
      switch(Vyes.getValue()) {
        #if HAS_SD_SUPPORT
          case 2:
            Psetup.show(); break;
        #endif
        default:
          Pprinter.show(); break;
      }
    }
  }

  static void degtoLCD(const uint8_t h, float temp) {

    NOMORE(temp, 999);

    heater_list0[h]->setValue(temp);

    #if ENABLED(NEXTION_GFX)
      if (!printer.isPrinting() && !Wavetemp.getObjVis() && show_Wave) {
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
      LcdX.setText(ftostr41sign(LOGICAL_X_POSITION(mechanics.current_position[X_AXIS])));
      LcdY.setText(ftostr41sign(LOGICAL_Y_POSITION(mechanics.current_position[Y_AXIS])));
      LcdZ.setText(ftostr41sign(FIXFLOAT(LOGICAL_Z_POSITION(mechanics.current_position[Z_AXIS]))));
    }
    else if (PageID == 5) {
      if (printer.isXHomed()) {
        valuetemp = ftostr4sign(LOGICAL_X_POSITION(mechanics.current_position[X_AXIS]));
        strcat(buffer, "X");
        strcat(buffer, valuetemp);
      }
      else
        strcat(buffer, "?");

      if (printer.isYHomed()) {
        valuetemp = ftostr4sign(LOGICAL_Y_POSITION(mechanics.current_position[Y_AXIS]));
        strcat(buffer, " Y");
        strcat(buffer, valuetemp);
      }
      else
        strcat(buffer, " ?");

      if (printer.isZHomed()) {
        valuetemp = ftostr52sp(FIXFLOAT(LOGICAL_Z_POSITION(mechanics.current_position[Z_AXIS])));
        strcat(buffer, " Z");
        strcat(buffer, valuetemp);
      }
      else
        strcat(buffer, " ?");

      LedCoord5.setText(buffer);
    }
  }

  void nextion_draw_update() {

    static uint8_t  PreviousPage = 0,
                    Previousfeedrate = 0,
                    PreviousfanSpeed = 0,
                    PreviouspercentDone = 0;
    static float    PreviousdegHeater[3] = { 0.0 },
                    PrevioustargetdegHeater[3] = { 0.0 };

    if (!NextionON || PageID == 14) return;

    PageID = Nextion_PageID();

    switch (PageID) {
      case 2:
        if (PreviousPage != 2) {
          lcdui.setstatus(lcdui.status_message);
          #if ENABLED(NEXTION_GFX)
            #if MECH(DELTA)
              gfx_clear(mechanics.data.print_radius * 2, mechanics.data.print_radius * 2, mechanics.data.height);
            #else
              gfx_clear(X_MAX_POS, Y_MAX_POS, Z_MAX_POS);
            #endif
          #endif
        }

        #if FAN_COUNT > 0
          if (PreviousfanSpeed != fans[0].Speed) {
            Fanspeed.setValue(((float)(fans[0].Speed) / 255) * 100);
            PreviousfanSpeed = fans[0].Speed;
          }
        #endif

        #if HAS_CASE_LIGHT
          LightStatus.setValue(caselight.status ? 2 : 1);
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
        #elif HAS_TEMP_CHAMBER
          if (PreviousdegHeater[1] != heaters[CHAMBER_INDEX].current_temperature) {
            PreviousdegHeater[1] = heaters[CHAMBER_INDEX].current_temperature;
            degtoLCD(3, PreviousdegHeater[1]);
          }
          if (PrevioustargetdegHeater[1] != heaters[CHAMBER_INDEX].target_temperature) {
            PrevioustargetdegHeater[1] = heaters[CHAMBER_INDEX].target_temperature;
            targetdegtoLCD(3, PrevioustargetdegHeater[1]);
          }
        #elif ENABLED(DHT_SENSOR)
          if (PreviousdegHeater[1] != dhtsensor.Humidity) {
            PreviousdegHeater[1] = dhtsensor.Humidity;
            degtoLCD(4, PreviousdegHeater[1]);
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

        if (PreviouspercentDone != printer.progress) {
          // Progress bar solid part
          progressbar.setValue(printer.progress);
          // Estimate End Time
          ZERO(buffer);
          char buffer1[10];
          uint8_t digit;
          duration_t Time = print_job_counter.duration();
          digit = Time.toDigital(buffer1, true);
          strcat(buffer, "S");
          strcat(buffer, buffer1);
          Time = (print_job_counter.duration() * (100 - printer.progress)) / (printer.progress + 0.1);
          digit += Time.toDigital(buffer1, true);
          if (digit > 14)
            strcat(buffer, "E");
          else
            strcat(buffer, " E");
          strcat(buffer, buffer1);
          LcdTime.setText(buffer);
          PreviouspercentDone = printer.progress;
        }

        #if HAS_SD_SUPPORT

          if (card.isFileOpen()) {
            if (IS_SD_PRINTING() && SDstatus != SD_PRINTING) {
              SDstatus = SD_PRINTING;
              SD.setValue(SDstatus);
            }
            else if (!IS_SD_PRINTING() && SDstatus != SD_PAUSE) {
              SDstatus = SD_PAUSE;
              SD.setValue(SDstatus);
            }
          }
          else if (card.isOK() && SDstatus != SD_INSERT) {
            SDstatus = SD_INSERT;
            SD.setValue(SDstatus);
          }
          else if (!card.isOK() && SDstatus != SD_NO_INSERT) {
            SDstatus = SD_NO_INSERT;
            SD.setValue(SDstatus);
          }

        #endif // HAS_SD_SUPPORT

        break;

      #if HAS_SD_SUPPORT
        case 3:
          if (PreviousPage != 3) setpageSD();
          break;
      #endif

      case 5:
        coordtoLCD();
        break;

      case 6:
        Previousfeedrate = mechanics.feedrate_percentage = (int)VSpeed.getValue("printer");
        break;

      case 15:
        coordtoLCD();
        break;

      default: break;

    }

    PreviousPage = PageID;

  }

  void nextion_printer_view() {
    if (PageID == 14) {
      PageID = 2;
      Pprinter.show();
    }
  }

  void nextion_menu_view() { 
    if (PageID != 14 || menu_redraw) {
      PageID = 14;
      Ptxtmenu.show();
      menu_redraw = false;
    }
  }

  void lcd_scrollinfo(PGM_P titolo, PGM_P message) {
    Pinfo.show();
    InfoText.setText(titolo);
    ScrollText.setText(message);
  }

  void lcd_yesno(const uint8_t val, PGM_P msg1, PGM_P msg2, PGM_P msg3) {
    Vyes.setValue(val, "yesno");
    Pyesno.show();
    Riga0.setText(msg1);
    Riga1.setText(msg2);
    Riga3.setText(msg3);
  }

  #if ENABLED(NEXTION_GFX)
    void gfx_origin(const float x, const float y, const float z) {
      gfx.origin(x, y, z);
    }

    void gfx_scale(const float scale) {
      gfx.set_scale(scale);
    }

    void gfx_clear(const float x, const float y, const float z, bool force_clear) {
      if (PageID == 2 && (printer.isPrinting() || force_clear)) {
        Wavetemp.SetVisibility(false);
        show_Wave = !force_clear;
        gfx.clear(x, y, z);
      }
    }

    void gfx_cursor_to(const float x, const float y, const float z, bool force_cursor) {
      if (PageID == 2 && (printer.isPrinting() || force_cursor))
        gfx.cursor_to(x, y, z);
    }

    void gfx_line_to(const float x, const float y, const float z) {
      if (PageID == 2 && printer.isPrinting()) {
        #if ENABLED(ARDUINO_ARCH_SAM)
          gfx.line_to(NX_TOOL, x, y, z, true);
        #else
          gfx.line_to(NX_TOOL, x, y, z);
        #endif
      }
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

  #if HAS_LCD_MENU

    constexpr uint8_t   lcd_width = 25;
    constexpr uint16_t  sel_color = 2016,
                        txt_color = 65535;

    NexObject *menu_row_list[] =
    {
      &MenuRow1,
      &MenuRow2,
      &MenuRow3,
      &MenuRow4,
      &MenuRow5,
      &MenuRow6,
      NULL
    };

    bool LcdUI::use_click() {
      const bool click = lcd_clicked;
      lcd_clicked = false;
      return click;
    }

    void encoderPopCallback(void *ptr) {
      if (ptr == &TxtMenu)
        lcdui.goto_screen(menu_main);
      else if (ptr == &EncMenu)
        lcdui.return_to_status();
      else if (ptr == &EncUp)
        lcdui.encoderPosition += lcdui.encoderDirection;
      else if (ptr == &EncDown)
        lcdui.encoderPosition -= lcdui.encoderDirection;
      else if (ptr == &EncSend) {
        lcdui.lcd_clicked = true;
        printer.setWaitForUser(false);
      }
    }

    inline static void nextion_put_space(const uint8_t row, const uint8_t len) {
      for (uint8_t i = 0; i < len; i++)
        menu_row_list[row]->setChar(' ');
    }

    inline static void nextion_put_char(const uint8_t row, PGM_P c, const uint8_t len) {
      for (uint8_t i = 0; i < len; i++)
        menu_row_list[row]->setChar(*c++);
    }

    void draw_edit_screen(PGM_P const pstr, PGM_P const value/*=NULL*/) {

      const uint8_t labellen  = utf8_strlen_P(pstr),
                    vallen    = utf8_strlen(value);

      bool extra_row = labellen > lcd_width - vallen - 2;

      constexpr uint8_t row = 2;

      menu_row_list[row]->Set_font_color_pco(sel_color);

      if (extra_row) {
        menu_row_list[row - 1]->Set_font_color_pco(sel_color);
        menu_row_list[row - 1]->setText(pstr);
        menu_row_list[row]->startChar();
        nextion_put_space(row, lcd_width - vallen - 1);
        nextion_put_char(row, value, labellen);
      }
      else {
        menu_row_list[row]->startChar();
        nextion_put_char(row, pstr, labellen);
        nextion_put_char(row, ":", 1);
        nextion_put_space(row, lcd_width - labellen - vallen - 2);
        nextion_put_char(row, value, labellen);
      }
      menu_row_list[row]->endChar();
    }

    void draw_menu_item(const bool sel, const uint8_t row, PGM_P const pstr, const char pre_char, const char post_char) {
      UNUSED(pre_char); UNUSED(post_char);
      if (sel) menu_row_list[row]->Set_font_color_pco(sel_color);
      else menu_row_list[row]->Set_font_color_pco(txt_color);
      menu_row_list[row]->setText(pstr);
    }

    void draw_menu_item_static(const uint8_t row, PGM_P const pstr, const bool center/*=true*/, const bool invert/*=false*/, PGM_P value/*=NULL*/) {
      UNUSED(center); UNUSED(invert);

      if (value != NULL) {
        const uint8_t labellen  = utf8_strlen_P(pstr),
                      vallen    = utf8_strlen(value);
        menu_row_list[row]->startChar();
        nextion_put_char(row, pstr, labellen);
        nextion_put_space(row, lcd_width - labellen - vallen - 1);
        nextion_put_char(row, value, labellen);
        menu_row_list[row]->endChar();
      }
      else
        menu_row_list[row]->setText(pstr);
    }

    void _draw_menu_item_edit(const bool sel, const uint8_t row, PGM_P const pstr, PGM_P const data, const bool pgm) {

      const uint8_t labellen  = utf8_strlen_P(pstr),
                    vallen    = utf8_strlen(data);

      if (sel) menu_row_list[row]->Set_font_color_pco(sel_color);
      else menu_row_list[row]->Set_font_color_pco(txt_color);
      menu_row_list[row]->startChar();
      nextion_put_char(row, pstr, labellen);
      nextion_put_char(row, ":", 1);
      nextion_put_space(row, lcd_width - labellen - vallen - 1);
      if (pgm)
        nextion_put_char(row, data, labellen);
      else
        nextion_put_char(row, (char*)data, labellen);
      menu_row_list[row]->endChar();
    }

    /**
     * If the most recent manual move hasn't been fed to the planner yet,
     * and the planner can accept one, send a move immediately.
     */
    void LcdUI::manage_manual_move() {

      if (processing_manual_move) return;

      if (manual_move_axis != (int8_t)NO_AXIS && ELAPSED(millis(), manual_move_start_time) && !planner.is_full()) {

        #if IS_KINEMATIC

          const float old_feedrate = mechanics.feedrate_mm_s;
          mechanics.feedrate_mm_s = MMM_TO_MMS(manual_feedrate_mm_m[manual_move_axis]);

          #if EXTRUDERS > 1
            const int8_t old_extruder = tools.active_extruder;
            if (manual_move_axis == E_AXIS) tools.active_extruder = manual_move_e_index;
          #endif

          // Set movement on a single axis
          mechanics.set_destination_to_current();
          mechanics.destination[manual_move_axis] += manual_move_offset;

          // Reset for the next move
          manual_move_offset = 0;
          manual_move_axis = (int8_t)NO_AXIS;

          // DELTA and SCARA machines use segmented moves, which could fill the planner during the call to
          // move_to_destination. This will cause idle() to be called, which can then call this function while the
          // previous invocation is being blocked. Modifications to manual_move_offset shouldn't be made while
          // processing_manual_move is true or the planner will get out of sync.
          processing_manual_move = true;
          mechanics.prepare_move_to_destination(); // will call set_current_to_destination
          processing_manual_move = false;

          mechanics.feedrate_mm_s = old_feedrate;
          #if EXTRUDERS > 1
            tools.active_extruder = old_extruder;
          #endif

        #else

          planner.buffer_line(mechanics.current_position, MMM_TO_MMS(manual_feedrate_mm_m[manual_move_axis]), tools.active_extruder);
          manual_move_axis = (int8_t)NO_AXIS;

        #endif
      }
    }

  #endif

  /**
   * LcdUI Function
   */
  void LcdUI::clear_lcd() {
    menu_redraw = true;
    nextion_menu_view();
  }

  void LcdUI::init() {

    for (uint8_t i = 0; i < 10; i++) {
      ZERO(buffer);
      NextionON = nexInit(buffer);
      if (NextionON) break;
      HAL::delayMilliseconds(1000);
    }

    if (!NextionON) {
      SERIAL_LM(ER, "Nextion not connected!");
      return;
    }
    else {
      SERIAL_MSG("Nextion");
      // Get Model

      if (strstr(buffer, "3224")) {       // Model 2.4" or 2.8" Normal or Enhanced
        SERIAL_MSG(" 2.4");
        #if ENABLED(NEXTION_GFX)
          gfx.set_position(1, 24, 250, 155);
        #endif
      }
      else if (strstr(buffer, "4024")) {  // Model 3.2" Normal or Enhanced
        SERIAL_MSG(" 3.2");
        #if ENABLED(NEXTION_GFX)
          gfx.set_position(1, 24, 250, 155);
        #endif
      }
      else if (strstr(buffer, "4832")) {  // Model 3.5" Normal or Enhanced
        SERIAL_MSG(" 3.5");
        #if ENABLED(NEXTION_GFX)
          gfx.set_position(1, 24, 250, 155);
        #endif
      }
      else if (strstr(buffer, "4827")) {  // Model 4.3" Normal or Enhanced
        SERIAL_MSG(" 4.3");
        #if ENABLED(NEXTION_GFX)
          gfx.set_position(1, 24, 250, 155);
        #endif
      }
      else if (strstr(buffer, "8048")) {  // Model 7" Normal or Enhanced
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

      #if HAS_SD_SUPPORT
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

      #if FAN_COUNT > 0
        FanTouch.attachPop(setfanPopCallback, &FanTouch);
      #endif

      #if HAS_CASE_LIGHT
        Light.attachPop(setlightPopCallback, &Light);
      #endif

      tenter.attachPop(sethotPopCallback,   &tenter);
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
      Yes.attachPop(YesNoPopCallback, &Yes);
      No.attachPop(YesNoPopCallback, &No);
      FilLoad.attachPop(filamentPopCallback);
      FilUnload.attachPop(filamentPopCallback);
      FilExtr.attachPop(filamentPopCallback);

      #if HAS_LCD_MENU
        TxtMenu.attachPop(encoderPopCallback, &TxtMenu);
        EncUp.attachPop(encoderPopCallback, &EncUp);
        EncDown.attachPop(encoderPopCallback, &EncDown);
        EncSend.attachPop(encoderPopCallback, &EncSend);
        EncMenu.attachPop(encoderPopCallback, &EncMenu);
      #endif

      setpagePrinter();
      startimer.enable();
    }
  }

  bool LcdUI::get_blink() {
    static uint8_t blink = 0;
    static millis_t next_blink_ms = 0;
    millis_t ms = millis();
    if (ELAPSED(ms, next_blink_ms)) {
      blink ^= 0xFF;
      next_blink_ms = ms + 250;
    }
    return blink != 0;
  }

  void LcdUI::update() {
    if (!NextionON) return;
    nexLoop(nex_listen_list);

    #if HAS_LCD_MENU
      static millis_t next_menu_update_ms;
      const millis_t ms = millis();
      if (ELAPSED(ms, next_menu_update_ms)) {
        lcdui.run_current_screen();
        next_menu_update_ms = ms + 250;
      }
    #endif
  }

  bool LcdUI::detected() { return NextionON; }

  void LcdUI::setalertstatusPGM(PGM_P const message) {
    lcdui.setstatusPGM(message, 1);
  }

  bool LcdUI::has_status() { return (status_message[0] != '\0'); }

  void LcdUI::setstatus(PGM_P message, bool persist) {
    UNUSED(persist);
    if (status_message_level > 0 || !NextionON) return;
    strncpy(status_message, message, 30);
    if (PageID == 2) LcdStatus.setText(status_message);
  }

  void LcdUI::setstatusPGM(PGM_P message, int8_t level) {
    if (level < 0) level = status_message_level = 0;
    if (level < status_message_level || !NextionON) return;
    strncpy_P(status_message, message, 30);
    status_message_level = level;
    if (PageID == 2) LcdStatus.setText(status_message);
  }
  
  void LcdUI::status_printf_P(const uint8_t level, PGM_P const fmt, ...) {
    if (level < status_message_level || !NextionON) return;
    status_message_level = level;
    va_list args;
    va_start(args, fmt);
    vsnprintf(status_message, 30, fmt, args);
    va_end(args);
    if (PageID == 2) LcdStatus.setText(status_message);
  }

  void LcdUI::reset_status() {
    static const char paused[] PROGMEM = MSG_PRINT_PAUSED;
    static const char printing[] PROGMEM = MSG_PRINTING;
    static const char welcome[] PROGMEM = WELCOME_MSG;
    PGM_P msg;
    if (print_job_counter.isPaused())
      msg = paused;
    #if HAS_SD_SUPPORT
      else if (IS_SD_PRINTING())
        return lcdui.setstatus(card.fileName, true);
    #endif
    else if (print_job_counter.isRunning())
      msg = printing;
    else
      msg = welcome;

    lcdui.setstatusPGM(msg, -1);
  }

  void LcdUI::status_screen() { nextion_printer_view(); }

  #if ENABLED(ADVANCED_PAUSE_FEATURE)

    void LcdUI::draw_hotend_status(const uint8_t row, const uint8_t hotend) {

      ZERO(buffer);
      strcat(buffer, "H");
      //strcat(buffer, (char)('0' + hotend)); // Da vedere!!!
      strcat(buffer, " ");
      strcat(buffer, itostr3(heaters[hotend].current_temperature));
      strcat(buffer, "/");

      if (get_blink() || !heaters[hotend].isIdle())
        strcat(buffer, itostr3(heaters[hotend].target_temperature));

      menu_row_list[row]->setText(buffer);

    }

  #endif // ADVANCED_PAUSE_FEATURE

#endif
