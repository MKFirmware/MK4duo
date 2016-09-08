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
  #include "Nextion.h"

  bool NextionON        = false;
  uint8_t NextionPage   = 0;
  char buffer[100]      = {0};
  uint32_t slidermaxval = 20;
  char lcd_status_message[30] = WELCOME_MSG;
  uint8_t lcd_status_message_level = 0;
  static millis_t next_lcd_update_ms;

  #if ENABLED(SDSUPPORT)
    uint8_t SDstatus    = 0; // 0 SD not insert, 1 SD insert, 2 SD printing
    NexUpload Firmware(NEXTION_FIRMWARE_FILE, 57600);
  #endif

  #if ENABLED(NEXTION_GFX)
    GFX gfx = GFX(200, 190);
  #endif

  // Page
  NexPage Pstart        = NexPage(0, 0, "start");
  NexPage Pinfo         = NexPage(1, 0, "info");
  NexPage Ptemp         = NexPage(2, 0, "temp");
  NexPage Pmenu         = NexPage(3, 0, "menu");
  NexPage Psdcard       = NexPage(4, 0, "sdcard");
  NexPage Psetup        = NexPage(5, 0, "setup");
  NexPage Pmove         = NexPage(6, 0, "move");
  NexPage Pspeed        = NexPage(7, 0, "speed");
  NexPage Pgcode        = NexPage(8, 0, "gcode");

  // Page 0 Start
  NexTimer startimer    = NexTimer(0,  1, "tm0");

  // Page 1 Info
  NexText Hotend0       = NexText(1,  22,  "t0");
  NexText Hotend1       = NexText(1,  23,  "t1");
  NexText Hotend21      = NexText(1,  50,  "h2");
  NexText Hotend2       = NexText(1,  24,  "t2");
  NexText LedStatus     = NexText(1,  1,  "status");
  NexText LedCoord1     = NexText(1,  2,  "icoord");
  NexPicture MSD1       = NexPicture(1, 4, "p1");
  NexPicture Hend0      = NexPicture(1, 6, "p3");
  NexPicture Hend1      = NexPicture(1, 7, "p4");
  NexPicture Hend2      = NexPicture(1, 8, "p5");
  NexPicture Fanpic     = NexPicture(1, 9, "p6");
  NexPicture NPlay      = NexPicture(1, 15, "p7");
  NexPicture NStop      = NexPicture(1, 16, "p8");
  NexPicture Speedpic   = NexPicture(1, 19, "p9");
  NexHotspot hot0       = NexHotspot(1, 27, "hot0");
  NexHotspot hot1       = NexHotspot(1, 28, "hot1");
  NexHotspot hot2       = NexHotspot(1, 29, "hot2");
  NexVar Hotend         = NexVar(1, 10, "he");
  NexVar Bed            = NexVar(1, 11, "bed");
  NexVar Fan            = NexVar(1, 12, "fn");
  NexVar SD             = NexVar(1, 17, "sd");
  NexVar RFID           = NexVar(1, 18, "rfid");
  NexVar VSpeed         = NexVar(1, 20, "vspeed");
  NexVar Extruder       = NexVar(1, 21, "extruder");
  NexTimer fantimer     = NexTimer(1, 13, "tm0");
  NexProgressBar sdbar  = NexProgressBar(1, 14, "j0");

  // Page 2 Temp
  NexText set0          = NexText(2, 2,  "set0");
  NexVar set1           = NexVar(2, 3,   "set1");
  NexHotspot m11        = NexHotspot(2, 8,  "p5");
  NexHotspot tup        = NexHotspot(2, 11, "p8");
  NexHotspot tdown      = NexHotspot(2, 12, "p9");
  
  // Page 3 Menu
  NexPicture MSD3       = NexPicture(3,  2, "p1");
  NexPicture Exit1      = NexPicture(3,  4, "p3");

  // Page 4 SDCard
  NexText sdrow0        = NexText(4,  2,  "t0");
  NexText sdrow1        = NexText(4,  3,  "t1");
  NexText sdrow2        = NexText(4,  4,  "t2");
  NexText sdrow3        = NexText(4,  5,  "t3");
  NexText sdrow4        = NexText(4,  6,  "t4");
  NexText sdrow5        = NexText(4,  7,  "t5");
  NexText sdfolder      = NexText(4,  16,  "sdfolder");
  NexPicture Folder0    = NexPicture(4,  8, "p0");
  NexPicture Folder1    = NexPicture(4, 9, "p1");
  NexPicture Folder2    = NexPicture(4, 10, "p2");
  NexPicture Folder3    = NexPicture(4, 11, "p3");
  NexPicture Folder4    = NexPicture(4, 12, "p4");
  NexPicture Folder5    = NexPicture(4, 13, "p5");
  NexPicture Folderup   = NexPicture(4, 14, "p6");
  NexPicture Exit2      = NexPicture(4, 15, "p7");
  NexHotspot ScrollUp   = NexHotspot(4, 26, "m0");
  NexHotspot ScrollDown = NexHotspot(4, 27, "m1");
  NexSlider sdlist      = NexSlider(4, 1, "h0");

  // Page 5 Setup
  NexPicture MSD5       = NexPicture(5,  2, "p1");
  NexPicture Exit3      = NexPicture(5,  4, "p3");

  // Page 6 Move
  NexPicture MSD6       = NexPicture(6,  2, "p1");
  NexPicture XYHome     = NexPicture(6,  5, "p4");
  NexPicture XYUp       = NexPicture(6,  6, "p5");
  NexPicture XYRight    = NexPicture(6,  7, "p6");
  NexPicture XYDown     = NexPicture(6,  8, "p7");
  NexPicture XYLeft     = NexPicture(6,  9, "p8");
  NexPicture ZHome      = NexPicture(6, 10, "p9");
  NexPicture ZUp        = NexPicture(6, 11, "p10");
  NexPicture ZDown      = NexPicture(6, 12, "p11");
  NexVar movecmd        = NexVar(6, 18, "vacmd");
  NexText LedCoord6     = NexText(6, 19, "mcoord");

  // Page 7 Speed
  NexPicture SpeedOk    = NexPicture(7, 2,  "p0");

  // Page 8 Gcode
  NexText Tgcode        = NexText(8, 1, "tgcode");
  NexButton Benter      = NexButton(8, 41, "benter");

  // Page 9 Rfid
  NexText RfidText      = NexText(9,    11, "t0");
  NexPicture MSD9       = NexPicture(9, 2,  "p1");
  NexButton Rfid0       = NexButton(9,  5,  "b0");
  NexButton Rfid1       = NexButton(9,  6,  "b1");
  NexButton Rfid2       = NexButton(9,  7,  "b2");
  NexButton Rfid3       = NexButton(9,  8,  "b3");
  NexButton Rfid4       = NexButton(9,  9,  "b4");
  NexButton Rfid5       = NexButton(9,  10, "b5");
  NexDSButton RfidR     = NexDSButton(9,12, "bt0");
  
  NexTouch *nex_listen_list[] =
  {
    &Pstart,
    &MSD1,
    &MSD3,
    &MSD5,
    &MSD6,
    &MSD9,
    &Fanpic,
    &Speedpic,
    &NPlay,
    &NStop,
    &hot0,
    &hot1,
    &hot2,
    &m11,
    &tup,
    &tdown,
    &sdlist,
    &ScrollUp,
    &ScrollDown,
    &sdrow0,
    &sdrow1,
    &sdrow2,
    &sdrow3,
    &sdrow4,
    &sdrow5,
    &Folderup,
    &Exit1,
    &Exit2,
    &Exit3,
    &XYHome,
    &XYUp,
    &XYRight,
    &XYDown,
    &XYLeft,
    &ZHome,
    &ZUp,
    &ZDown,
    &SpeedOk,
    &Benter,
    &Rfid0,
    &Rfid1,
    &Rfid2,
    &Rfid3,
    &Rfid4,
    &Rfid5,
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

  void setpageInfo() {
    if (NextionPage == 0) {
      Pinfo.show();

      #if HAS(TEMP_0)
        Hotend.setValue(1);
      #endif
      #if HAS(TEMP_1)
        Hotend.setValue(2);
      #endif
      #if HAS(TEMP_2)
        Hotend.setValue(3);
      #elif HAS(TEMP_BED)
        Bed.setValue(1);
        Hotend21.setText("BED");
      #endif

      Extruder.setValue(EXTRUDERS);

      VSpeed.setValue(100);

      #if HAS(FAN)
        Fan.setValue(1);
      #endif

      #if ENABLED(RFID_MODULE)
        RFID.setValue(1);
      #endif

      lcd_setstatus(lcd_status_message);
    }

    Pinfo.show();

    #if ENABLED(NEXTION_GFX)
      #if MECH(DELTA)
        gfx_clear((X_MAX_POS) * 2, (Y_MAX_POS) * 2, Z_MAX_POS);
      #else
        gfx_clear(X_MAX_POS, Y_MAX_POS, Z_MAX_POS);
      #endif
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
      setpageInfo();
    }

    static void menu_action_sddirectory(const char* filename) {
      card.chdir(filename);
      setpageSDPopCallback(&MSD1);
    }

    void setpageSDPopCallback(void *ptr) {
      Psdcard.show();
      uint16_t fileCnt = card.getnrfilenames();

      if (fileCnt <= 6)
        slidermaxval = 0;
      else
        slidermaxval  = fileCnt - 6;

      uint16_t hig = 210 - slidermaxval * 10;
      if (hig < 10) hig = 10;

      sdlist.setHigVal(hig);
      sdlist.setMaxVal(slidermaxval);
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
      memset(buffer, 0, sizeof(buffer));

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
      memset(buffer, 0, sizeof(buffer));

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
      setpageSDPopCallback(&MSD1);
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

      memset(buffer, 0, sizeof(buffer));
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
      RfidText.setColor(color);
      RfidText.setText(Rfid_status_message);
    }
  #endif

  void ExitPopCallback(void *ptr) {
    setpageInfo();
  }

  void PstartPopCallback(void *ptr) {
    setpageInfo();
  }

  void hotPopCallback(void *ptr) {
    Ptemp.show();
    memset(buffer, 0, sizeof(buffer));
    if (ptr == &hot0) {
      if (degTargetHotend(0) != 0) {
        itoa(degTargetHotend(0), buffer, 10);
      }
      set1.setText("M104 T0 S");
    }
    if (ptr == &hot1) {
      if (degTargetHotend(1) != 0) {
        itoa(degTargetHotend(1), buffer, 10);
      }
      set1.setText("M104 T1 S");
    }

    #if HAS_TEMP_2
      if (ptr == &hot2) {
        if (degTargetHotend(2) != 0) {
          itoa(degTargetHotend(2), buffer, 10);
        }
        set1.setText("M104 T2 S");
      }
    #elif HAS_TEMP_BED
      if (ptr == &hot2) {
        if (degTargetBed() != 0) {
          itoa(degTargetBed(), buffer, 10);
        }
        set1.setText("M140 S");
      }
    #endif

    set0.setText(buffer);
  }

  void settempPopCallback(void *ptr) {
    uint16_t number;

    memset(buffer, 0, sizeof(buffer));
    set0.getText(buffer, sizeof(buffer));

    number = atoi(buffer);

    if (ptr == &tup) number += 1;
    if (ptr == &tdown) number -= 1;

    memset(buffer, 0, sizeof(buffer));
    itoa(number, buffer, 10);

    set0.setText(buffer);
  }

  void sethotPopCallback(void *ptr) {
    memset(buffer, 0, sizeof(buffer));
    set1.getText(buffer, sizeof(buffer));
    enqueue_and_echo_commands_P(buffer);
    setpageInfo();
  }

  void setgcodePopCallback(void *ptr) {
    memset(buffer, 0, sizeof(buffer));
    Tgcode.getText(buffer, sizeof(buffer));
    enqueue_and_echo_commands_P(buffer);
    Pmenu.show();
  }

  void setfanPopCallback(void *ptr) {
    if (fanSpeed) {
      fanSpeed = 0;
      fantimer.disable();
    }
    else {
      fanSpeed = 255;
      fantimer.enable();
    }
  }

  void setmovePopCallback(void *ptr) {
    memset(buffer, 0, sizeof(buffer));
    movecmd.getText(buffer, sizeof(buffer));
    enqueue_and_echo_commands_P(PSTR("G91"));
    enqueue_and_echo_commands_P(buffer);
    enqueue_and_echo_commands_P(PSTR("G90"));
  }

  #if ENABLED(SDSUPPORT)
    void PlayPausePopCallback(void *ptr) {
      if (card.cardOK && card.isFileOpen()) {
        if (IS_SD_PRINTING)
          card.pausePrint();
        else
          card.startPrint();
      }
    }

    void StopPopCallback(void *ptr) {
      quickstop_stepper();
      card.stopPrint();
      autotempShutdown();
      lcd_setstatus(MSG_PRINT_ABORTED, true);
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
    }
    else {
      SERIAL_EM("Nextion LCD connected!");

      Pstart.attachPop(ExitPopCallback);
      Exit1.attachPop(ExitPopCallback);
      Exit3.attachPop(ExitPopCallback);

      #if ENABLED(NEXTION_GFX)
        gfx.color_set(VC_AXIS + X_AXIS, 63488);
        gfx.color_set(VC_AXIS + Y_AXIS, 2016);
        gfx.color_set(VC_AXIS + Z_AXIS, 31);
        gfx.color_set(VC_MOVE, 2047);
        gfx.color_set(VC_TOOL, 65535);
      #endif

      #if ENABLED(SDSUPPORT)
        MSD1.attachPop(setpageSDPopCallback);
        MSD3.attachPop(setpageSDPopCallback);
        MSD5.attachPop(setpageSDPopCallback);
        MSD6.attachPop(setpageSDPopCallback);
        MSD9.attachPop(setpageSDPopCallback);
        sdlist.attachPop(sdlistPopCallback);
        ScrollUp.attachPop(sdlistPopCallback);
        ScrollDown.attachPop(sdlistPopCallback);
        Exit2.attachPop(ExitPopCallback);
        NPlay.attachPop(PlayPausePopCallback);
        NStop.attachPop(StopPopCallback);
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
        hot0.attachPop(hotPopCallback,      &hot0);
      #endif
      #if HAS_TEMP_1
        hot1.attachPop(hotPopCallback,      &hot1);
      #endif
      #if HAS_TEMP_2 || HAS_TEMP_BED
        hot2.attachPop(hotPopCallback,      &hot2);
      #endif

      Fanpic.attachPop(setfanPopCallback,   &Fanpic);
      m11.attachPop(sethotPopCallback,      &m11);
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
      SpeedOk.attachPop(ExitPopCallback);
      Benter.attachPop(setgcodePopCallback);

      startimer.enable();
    }
  }

  static void temptoLCD(int h, int T1, int T2) {
    char valuetemp[25] = {0};
    memset(buffer, 0, sizeof(buffer));
    itoa(T1, valuetemp, 10);
    strcat(buffer, valuetemp);
    strcat(buffer, "/");
    itoa(T2, valuetemp, 10);
    strcat(buffer, valuetemp);
    uint32_t color = 1023;
    uint32_t prc = (T1/(T2 + 0.01)) * 100;

    if (prc >= 50 && prc < 75)
      color = 65519;
    else if (prc >= 75 && prc < 95)
      color = 64487;
    else if (prc >= 95)
      color = 63488;

    hotend_list[h]->setText(buffer);
    hotend_list[h]->setColor(color);
  }

  static void coordtoLCD() {
    char* valuetemp;

    memset(buffer, 0, sizeof(buffer));
    strcat(buffer, (axis_known_position[X_AXIS] ? "X" : "?"));
    if (axis_homed[X_AXIS]) {
      valuetemp = ftostr4sign(current_position[X_AXIS]);
      strcat(buffer, valuetemp);
    }

    strcat(buffer, (axis_known_position[Y_AXIS] ? " Y" : " ?"));
    if (axis_homed[Y_AXIS]) {
      valuetemp = ftostr4sign(current_position[Y_AXIS]);
      strcat(buffer, valuetemp);
    }

    strcat(buffer, (axis_known_position[Z_AXIS] ? " Z " : " ? "));
    if (axis_homed[Z_AXIS]) {
      valuetemp = ftostr52sp(current_position[Z_AXIS] + 0.00001);
      strcat(buffer, valuetemp);
    }

    LedCoord1.setText(buffer);
    LedCoord6.setText(buffer);
  }

  void lcd_update() {

    if (!NextionON) return;

    nexLoop(nex_listen_list);

    millis_t ms = millis();

    if (ms > next_lcd_update_ms) {

      sendCurrentPageId(&NextionPage);

      if (NextionPage == 1) {
        if (fanSpeed > 0) fantimer.enable();
        else fantimer.disable();

        uint32_t temp_feedrate = 0;
        VSpeed.getValue(&temp_feedrate);
        feedrate_percentage = (int)temp_feedrate;

        #if HAS(TEMP_0)
          temptoLCD(0, degHotend(0), degTargetHotend(0));
        #endif
        #if HAS(TEMP_1)
          temptoLCD(1, degHotend(1), degTargetHotend(1));
        #endif
        #if HAS(TEMP_2)
          temptoLCD(2, degHotend(2), degTargetHotend(2));
        #elif HAS(TEMP_BED)
          temptoLCD(2, degBed(), degTargetBed());
        #endif

        coordtoLCD();

        #if ENABLED(SDSUPPORT)
          if (card.isFileOpen()) {
            if (SDstatus != 2) {
              SDstatus = 2;
              SD.setValue(2);
              NPlay.setShow();
              NStop.setShow();
            }
            if(IS_SD_PRINTING) {
              // Progress bar solid part
              sdbar.setValue(card.percentDone());
              NPlay.setPic(17);

              // Estimate End Time
              uint16_t time = print_job_counter.duration() / 60;
              uint16_t end_time = (time * (100 - card.percentDone())) / card.percentDone();
              if (end_time > (60 * 23)) {
                lcd_setstatus("End --:--");
              }
              else if (end_time >= 0) {
                char temp[30];
                sprintf_P(temp, PSTR("End %i:%i"), end_time / 60, end_time%60);
                lcd_setstatus(temp);
              }
            }
            else {
              NPlay.setPic(16);
            }
          }
          else if (card.cardOK && SDstatus != 1) {
            SDstatus = 1;
            SD.setValue(1);
            MSD1.setShow();
            NPlay.setHide();
            NStop.setHide();
          }
          else if (!card.cardOK && SDstatus != 0) {
            SDstatus = 0;
            SD.setValue(0);
            MSD1.setHide();
            NPlay.setHide();
            NStop.setHide();
          }
        #endif
      }
      else if (NextionPage == 6) {
        coordtoLCD();
      }
      next_lcd_update_ms = ms + LCD_UPDATE_INTERVAL;
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

  #if ENABLED(NEXTION_GFX)
    void gfx_clear(float x, float y, float z) {
      if ((NextionPage == 1) && (print_job_counter.isRunning() || IS_SD_PRINTING))
        gfx.clear(x, y, z);
    }

    void gfx_cursor_to(float x, float y, float z) {
      if ((NextionPage == 1) && (print_job_counter.isRunning() || IS_SD_PRINTING))
        gfx.cursor_to(x, y, z);
    }

    void gfx_line_to(float x, float y, float z){
      if ((NextionPage == 1) && (print_job_counter.isRunning() || IS_SD_PRINTING))
        gfx.line_to(VC_TOOL, x, y, z);
    }
  #endif

#endif
