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

  bool NextionON                    = false;
  uint8_t NextionPage               = 0,
          lcd_status_message_level  = 0;
  uint16_t slidermaxval             = 20;
  char buffer[100]                  = { 0 };
  char lcd_status_message[30]       = WELCOME_MSG;
  static millis_t next_lcd_update_ms;

  #if ENABLED(SDSUPPORT)
    uint8_t SDstatus    = 0; // 0 SD not insert, 1 SD insert, 2 SD printing
    NexUpload Firmware(NEXTION_FIRMWARE_FILE, 57600);
  #endif

  #if ENABLED(NEXTION_GFX)
    GFX gfx = GFX(200, 154, 20, 27);
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
  NexText LedStatus     = NexText       (2,   1,  "t0");
  NexText LedCoord1     = NexText       (2,   2,  "t1");
  NexText Hotend0       = NexText       (2,   14, "t2");
  NexText Hotend1       = NexText       (2,   15, "t3");
  NexText Hotend2       = NexText       (2,   16, "t4");
  NexText Fanspeed      = NexText       (2,   18, "t5");
  NexPicture Fanpic     = NexPicture    (2,   4,  "p1");
  NexPicture NStop      = NexPicture    (2,   19, "p2");
  NexPicture NPlay      = NexPicture    (2,   20, "p3");
  NexPicture Logo       = NexPicture    (2,   17, "p4");
  NexVariable Hotend    = NexVariable   (2,   5,  "he");
  NexVariable Bed       = NexVariable   (2,   6,  "bed");
  NexVariable Fan       = NexVariable   (2,   7,  "fan");
  NexVariable SD        = NexVariable   (2,   10, "sd");
  NexVariable RFID      = NexVariable   (2,   11, "rfid");
  NexVariable VSpeed    = NexVariable   (2,   12, "vspeed");
  NexVariable Extruder  = NexVariable   (2,   13, "extruder");
  NexTimer Fantimer     = NexTimer      (2,   8,  "tm0");
  NexProgressBar sdbar  = NexProgressBar(2,   9,  "j0");

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
  NexSlider sdlist      = NexSlider     (3,   1,  "h0");

  /**
   *******************************************************************
   * Nextion component for page:Setup
   *******************************************************************
   */
  NexPicture DFirmware  = NexPicture    (4,   4,  "p0");

  /**
   *******************************************************************
   * Nextion component for page:Move
   *******************************************************************
   */
  NexPicture XYHome     = NexPicture    (5,   2,  "p4");
  NexPicture XYUp       = NexPicture    (5,   3,  "p5");
  NexPicture XYRight    = NexPicture    (5,   4,  "p6");
  NexPicture XYDown     = NexPicture    (5,   5,  "p7");
  NexPicture XYLeft     = NexPicture    (5,   6,  "p8");
  NexPicture ZHome      = NexPicture    (5,   7,  "p9");
  NexPicture ZUp        = NexPicture    (5,   8,  "p10");
  NexPicture ZDown      = NexPicture    (5,   9,  "p11");
  NexVariable movecmd   = NexVariable   (5,   11, "vacmd");
  NexText LedCoord5     = NexText       (5,   12, "t0");

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
  NexButton Benter      = NexButton     (7,   41, "benter");

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
  NexText tset0         = NexText       (10,  1,  "set0");
  NexVariable tset1     = NexVariable   (10,  2,  "set1");
  NexPicture tset       = NexPicture    (10,  3,  "p5");
  NexPicture tup        = NexPicture    (10,  6,  "p8");
  NexPicture tdown      = NexPicture    (10,  7,  "p9");

  /**
   *******************************************************************
   * Nextion component for page:Info
   *******************************************************************
   */
  NexText InfoText          = NexText       (11,   2,  "t0");
  NexScrolltext ScrollText  = NexScrolltext (11,   3,  "g0");

  NexTouch *nex_listen_list[] =
  {
    // Page 2 touch listen
    &Hotend0, &Hotend1, &Hotend2, &Fanpic, &NPlay, &NStop, &Logo,

    // Page 3 touch listen
    &sdlist, &ScrollUp, &ScrollDown, &sdrow0, &sdrow1, &sdrow2,
    &sdrow3, &sdrow4, &sdrow5, &Folderup,

    // Page 4 touch listen
    &DFirmware,

    // Page 5 touch listen
    &XYHome, &XYUp, &XYRight, &XYDown, &XYLeft, &ZHome, &ZUp, &ZDown,

    // Page 6 touch listen
    &Speed,

    // Page 7 touch listen
    &Benter,

    // Page 8 touch listen
    &Rfid0, &Rfid1, &Rfid2, &Rfid3, &Rfid4, &Rfid5,

    // Page 10 touch listen
    &tset, &tup, &tdown,
    NULL
  };

  NexText *hotend_list[] =
  {
    &Hotend0,
    &Hotend1,
    &Hotend2,
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

    Hotend.setValue(HOTENDS, "printer");
    Extruder.setValue(EXTRUDERS, "printer");

    #if HAS(TEMP_BED)
      Bed.setValue(1, "printer");
    #endif

    #if ENABLED(SDSUPPORT)
      card.mount();
      if (card.cardOK) {
        SDstatus = 1;
        SD.setValue(1, "printer");
      }
    #endif

    VSpeed.setValue(100, "printer");

    #if HAS(FAN)
      Fan.setValue(1, "printer");
    #endif

    #if ENABLED(RFID_MODULE)
      RFID.setValue(1, "printer");
    #endif

  }

  #if ENABLED(SDSUPPORT)
    void printrowsd(uint8_t row, const bool folder, const char* filename) {
      if (folder) {
        folder_list[row]->setShow();
        row_list[row]->attachPop(sdfolderPopCallback, row_list[row]);
      } else if (filename == "") {
        folder_list[row]->setHide();
        row_list[row]->detachPop();
      } else {
        folder_list[row]->setHide();
        row_list[row]->attachPop(sdfilePopCallback, row_list[row]);
      }
      row_list[row]->setText(filename);
    }

    static void setrowsdcard(uint32_t number = 0) {
      uint16_t fileCnt = card.getnrfilenames();
      uint32_t i = 0;
      card.getWorkDirName();

      if (fullName[0] != '/') {
        Folderup.setShow();
        Folderup.attachPop(sdfolderUpPopCallback);
        sdfolder.setText(fullName);
      } else {
        Folderup.detachPop();
        Folderup.setHide();
        sdfolder.setText("");
      }

      for (uint8_t row = 0; row < 6; row++) {
        i = row + number;
        if (i < fileCnt) {
          card.getfilename(i);
          printrowsd(row, card.filenameIsDir, fullName);
        } else {
          printrowsd(row, false, "");
        }
      }
      sendCommand("ref 0");
    }

    static void menu_action_sdfile(const char* filename) {
      char cmd[30];
      char* c;
      sprintf_P(cmd, PSTR("M23 %s"), filename);
      for(c = &cmd[4]; *c; c++) *c = tolower(*c);
      enqueue_and_echo_command(cmd);
      enqueue_and_echo_commands_P(PSTR("M24"));
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
    
    void UploadNewFirmware() {
      if(IS_SD_INSERTED || card.cardOK) {
        Firmware.startUpload();
        nexSerial.end();
        lcd_init();
      }
    }
  #endif

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
      enqueue_and_echo_commands_P(buffer);
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
      tset1.setText("M104 T0 S");
    }
    if (ptr == &Hotend1) {
      if (thermalManager.degTargetHotend(1) != 0) {
        itoa(thermalManager.degTargetHotend(1), buffer, 10);
      }
      tset1.setText("M104 T1 S");
    }

    #if HAS_TEMP_2
      if (ptr == &Hotend2) {
        if (thermalManager.degTargetHotend(2) != 0) {
          itoa(thermalManager.degTargetHotend(2), buffer, 10);
        }
        tset1.setText("M104 T2 S");
      }
    #elif HAS_TEMP_BED
      if (ptr == &Hotend2) {
        if (thermalManager.degTargetBed() != 0) {
          itoa(thermalManager.degTargetBed(), buffer, 10);
        }
        tset1.setText("M140 S");
      }
    #endif

    tset0.setText(buffer);
  }

  void settempPopCallback(void *ptr) {
    uint16_t number;

    ZERO(buffer);
    tset0.getText(buffer, sizeof(buffer));

    number = atoi(buffer);

    if (ptr == &tup) number += 1;
    if (ptr == &tdown) number -= 1;

    ZERO(buffer);
    itoa(number, buffer, 10);

    tset0.setText(buffer);
  }

  void sethotPopCallback(void *ptr) {
    char temp[5] = { 0 };
    ZERO(buffer);
    tset0.getText(temp, sizeof(temp));
    tset1.getText(buffer, sizeof(buffer));
    strcat(buffer, temp);
    enqueue_and_echo_commands_P(buffer);
    Pprinter.show();
  }

  void setgcodePopCallback(void *ptr) {
    ZERO(buffer);
    Tgcode.getText(buffer, sizeof(buffer));
    enqueue_and_echo_commands_P(buffer);
    Pmenu.show();
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
    ZERO(buffer);
    movecmd.getText(buffer, sizeof(buffer));
    enqueue_and_echo_commands_P(PSTR("G91"));
    enqueue_and_echo_commands_P(buffer);
    enqueue_and_echo_commands_P(PSTR("G90"));
  }

  #if ENABLED(SDSUPPORT)
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

    void StopPopCallback(void *ptr) {
      if (card.cardOK && card.isFileOpen() && IS_SD_PRINTING) {
        card.stopSDPrint(ptr == &Logo);
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
    
    void DFirmwareCallback(void *ptr) {
      UploadNewFirmware();
    }
  #endif

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
        gfx.color_set(VC_AXIS + X_AXIS, 63488);
        gfx.color_set(VC_AXIS + Y_AXIS, 2016);
        gfx.color_set(VC_AXIS + Z_AXIS, 31);
        gfx.color_set(VC_MOVE, 2047);
        gfx.color_set(VC_TOOL, 65535);
      #endif

      #if ENABLED(SDSUPPORT)
        sdlist.attachPop(sdlistPopCallback);
        ScrollUp.attachPop(sdlistPopCallback);
        ScrollDown.attachPop(sdlistPopCallback);
        NPlay.attachPop(PlayPausePopCallback);
        NStop.attachPop(StopPopCallback, &NStop);
        Logo.attachPop(StopPopCallback, &Logo);
        DFirmware.attachPop(DFirmwareCallback);
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
      tset.attachPop(sethotPopCallback,     &tset);
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
      Benter.attachPop(setgcodePopCallback);

      setpagePrinter();
      startimer.enable();
    }
  }

  static void temptoLCD(int h, float T1, float T2) {
    char valuetemp[25] = {0};
    uint32_t color;
    ZERO(buffer);
    itoa(T1, valuetemp, 10);
    strcat(buffer, valuetemp);
    strcat(buffer, "/");
    itoa(T2, valuetemp, 10);
    strcat(buffer, valuetemp);
    strcat(buffer, "  ");

    if (T2 > 0) {
      uint32_t prc = T1 / T2 * 100;

      if (prc <= 50)
        color = 1023;
      else if (prc <= 75)
        color = 65504;
      else if (prc <= 95)
        color = 64512;
      else
        color = 63488;
    }
    else
      color = 65535;

    hotend_list[h]->setText(buffer);
    hotend_list[h]->Set_font_color_pco(color);
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

    if (NextionPage == 2) LedCoord1.setText(buffer);
    else LedCoord5.setText(buffer);
  }

  void lcd_update() {
    static uint8_t  PreviousPage = 0,
                    PreviousfanSpeed = 0,
                    PreviouspercentDone = 0,
                    PreviousdegHotend[3] = { 0 };
    char* temp;

    if (!NextionON) return;

    nexLoop(nex_listen_list);

    millis_t ms = millis();

    if (ms > next_lcd_update_ms) {

      sendCurrentPageId(&NextionPage);

      switch (NextionPage) {
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

          static uint32_t temp_feedrate = 0;
          VSpeed.getValue(&temp_feedrate);
          feedrate_percentage = (int)temp_feedrate;

          #if HAS(TEMP_0)
            if (PreviousdegHotend[0] != thermalManager.degHotend(0)) {
              temptoLCD(0, thermalManager.degHotend(0), thermalManager.degTargetHotend(0));
              PreviousdegHotend[0] = thermalManager.degHotend(0);
            }
          #endif
          #if HAS(TEMP_1)
            if (PreviousdegHotend[1] != thermalManager.degHotend(1)) {
              temptoLCD(1, thermalManager.degHotend(1), thermalManager.degTargetHotend(1));
              PreviousdegHotend[1] = thermalManager.degHotend(1);
            }
          #endif
          #if HAS(TEMP_2)
            if (PreviousdegHotend[2] != thermalManager.degHotend(2)) {
              temptoLCD(2, thermalManager.degHotend(2), thermalManager.degTargetHotend(2));
              PreviousdegHotend[2] = thermalManager.degHotend(2);
            }
          #elif HAS(TEMP_BED)
            if (PreviousdegHotend[2] != thermalManager.degBed()) {
              temptoLCD(2, thermalManager.degBed(), thermalManager.degTargetBed());
              PreviousdegHotend[2] = thermalManager.degBed();
            }
          #endif

          coordtoLCD();

          #if ENABLED(SDSUPPORT)
            if (card.isFileOpen()) {
              if (SDstatus != 2) {
                SDstatus = 2;
                SD.setValue(2);
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
            else if (card.cardOK && SDstatus != 1) {
              SDstatus = 1;
              SD.setValue(1);
              NPlay.setPic(27);
              NStop.setPic(30);
            }
            else if (!card.cardOK && SDstatus != 0) {
              SDstatus = 0;
              SD.setValue(0);
              NPlay.setPic(27);
              NStop.setPic(30);
            }
          #endif
          break;
        #if ENABLED(SDSUPPORT)
          case 3:
            if (PreviousPage != 3) setpageSD();
            break;
        #endif
        case 5:
          coordtoLCD();
          break;
      }

      next_lcd_update_ms = ms + LCD_UPDATE_INTERVAL;
      PreviousPage = NextionPage;
    }
  }

  void lcd_setstatus(const char* message, bool persist) {
    if (lcd_status_message_level > 0 || !NextionON) return;
    strncpy(lcd_status_message, message, 30);
    LedStatus.setText(lcd_status_message);
  }

  void lcd_setstatuspgm(const char* message, uint8_t level) {
    if (level >= lcd_status_message_level && NextionON) {
      strncpy_P(lcd_status_message, message, 30);
      lcd_status_message_level = level;
      LedStatus.setText(lcd_status_message);
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
    void gfx_clear(float x, float y, float z) {
      if ((NextionPage == 2) && (print_job_counter.isRunning() || IS_SD_PRINTING))
        gfx.clear(x, y, z);
    }

    void gfx_cursor_to(float x, float y, float z) {
      if ((NextionPage == 2) && (print_job_counter.isRunning() || IS_SD_PRINTING))
        gfx.cursor_to(x, y, z);
    }

    void gfx_line_to(float x, float y, float z){
      if ((NextionPage == 2) && (print_job_counter.isRunning() || IS_SD_PRINTING))
        gfx.line_to(VC_TOOL, x, y, z);
    }
  #endif

#endif
