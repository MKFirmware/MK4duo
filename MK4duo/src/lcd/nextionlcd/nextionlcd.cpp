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

/**
 * nextionlcd.cpp
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
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

#include "nextion_gfx.h"

#define NEXTION_LCD_FIRMWARE_VERSION 131

NextionLCD nexlcd;

/** LcdUI Parameters */
char    LcdUI::status_message[NEXTION_MAX_MESSAGE_LENGTH + 1];
uint8_t LcdUI::alert_level = 0,
        LcdUI::lang = 0;

#if HAS_LCD_MENU

  extern bool no_reentry; // Flag to prevent recursion into menu handlers

  int8_t manual_move_axis = (int8_t)NO_AXIS;
  short_timer_t manual_move_timer;

  #if IS_KINEMATIC
    float manual_move_offset = 0;
  #endif

  LCDViewActionEnum LcdUI::lcdDrawUpdate = LCDVIEW_CLEAR_CALL_REDRAW;

  uint32_t      LcdUI::encoderPosition;

  screenFunc_t  LcdUI::currentScreen;

  bool          LcdUI::screen_changed;

  int8_t        LcdUI::manual_move_e_index = 0;

  #if LCD_HAS_WAIT_FOR_MOVE
    bool LcdUI::wait_for_move = false;
  #endif

  #if HAS_SD_SUPPORT && ENABLED(SCROLL_LONG_FILENAMES)
    uint8_t LcdUI::filename_scroll_pos, LcdUI::filename_scroll_max;
  #endif

  #if ENABLED(REVERSE_MENU_DIRECTION) || ENABLED(REVERSE_ENCODER_DIRECTION)
    int8_t LcdUI::encoderDirection = 1;
  #endif

  bool LcdUI::lcd_clicked = false;

  #if LCD_TIMEOUT_TO_STATUS
    bool LcdUI::defer_return_to_status;
  #endif

  #if IS_KINEMATIC
    bool LcdUI::processing_manual_move = false;
  #endif

  #if ENABLED(AUTO_BED_LEVELING_UBL) || ENABLED(G26_MESH_VALIDATION)
    bool LcdUI::external_control; // = false
  #endif

#endif

/**
 * Class Nextion LCD
 */
/** Public Parameters */
bool    NextionLCD::NextionON           = false;        

uint8_t NextionLCD::PageID              = 0;

#if HAS_SD_SUPPORT
  uint8_t NextionLCD::lcd_sd_status     = 2; // UNKNOWN
#endif

#if HAS_LCD_MENU
  bool  NextionLCD::line_encoder_touch  = false;
  #if LCD_TIMEOUT_TO_STATUS
    short_timer_t NextionLCD::return_to_status_timer(millis());
  #endif
#endif

/** Private Parameters */
#if HAS_SD_SUPPORT
  NexUpload NextionLCD::Firmware(NEXTION_FIRMWARE_FILE, 57600);
#endif

/**
 *******************************************************************
 * Nextion component for page:menu
 *******************************************************************
 */
NexObject SDMenu        = NexObject(1,  2);

/**
 *******************************************************************
 * Nextion component for page:printer
 *******************************************************************
 */
NexObject LcdStatus       = NexObject(2,  6);
NexObject LcdX            = NexObject(2,  7);
NexObject LcdY            = NexObject(2,  8);
NexObject LcdZ            = NexObject(2,  9);
#if HAS_HOTENDS
  NexObject Hotend_deg[4] = { NexObject(2, 11), NexObject(2, 13), NexObject(2, 15), NexObject(2, 17) };
  NexObject Hotend_trg[4] = { NexObject(2, 12), NexObject(2, 14), NexObject(2, 16), NexObject(2, 18) };
#endif
#if HAS_BEDS
  NexObject Bed_deg       = NexObject(2, 19);
  NexObject Bed_trg       = NexObject(2, 20);
#endif
#if HAS_CHAMBERS
  NexObject Chamber_deg   = NexObject(2, 21);
  NexObject Chamber_trg   = NexObject(2, 22);
#endif
#if HAS_DHT
  NexObject DHT0          = NexObject(2, 23);
#endif
NexObject SD              = NexObject(2, 24);
NexObject Fanspeed        = NexObject(2, 26);
NexObject VSpeed          = NexObject(2, 27);
NexObject LightStatus     = NexObject(2, 28);
NexObject LcdTime         = NexObject(2, 30);
NexObject progressbar     = NexObject(2, 31);

/**
 *******************************************************************
 * Nextion component for page:Setup
 *******************************************************************
 */
#if HAS_SD_SUPPORT
  NexObject NextionFW = NexObject(3,  1);
#endif
#if HAS_LCD_MENU
  NexObject TxtMenu   = NexObject(3,  3);
#endif

/**
 *******************************************************************
 * Nextion component for page:Move
 *******************************************************************
 */
NexObject LcdCoord  = NexObject(4,  12);
NexObject SpeedX    = NexObject(4,  22);
NexObject SpeedY    = NexObject(4,  23);
NexObject SpeedZ    = NexObject(4,  24);

#if ENABLED(RFID_MODULE)

  /**
   *******************************************************************
   * Nextion component for page:Rfid
   *******************************************************************
   */
  NexObject RfidText    = NexObject(7,  8);

#endif

#if HAS_LCD_MENU

  /**
   *******************************************************************
   * Nextion component for page:Select
   *******************************************************************
   */
  NexObject EncRow1     = NexObject(11,  1);
  NexObject EncRow2     = NexObject(11,  2);
  NexObject EncRow3     = NexObject(11,  3);
  NexObject EncRow4     = NexObject(11,  4);
  NexObject EncRow5     = NexObject(11,  5);
  NexObject EncRow6     = NexObject(11,  6);
  NexObject EncUp       = NexObject(11,  7);
  NexObject EncSend     = NexObject(11,  8);
  NexObject EncDown     = NexObject(11,  9);
  NexObject EncExit     = NexObject(11, 10);

#endif

NexObject *nex_listen_list[] =
{
  // Page 1 touch listen
  &SDMenu,

  // Page 3 touch listen
  #if HAS_SD_SUPPORT
    &NextionFW,
  #endif
  #if HAS_LCD_MENU
    &TxtMenu,
  #endif

  #if HAS_LCD_MENU
    // Page 11 touch listen
    &EncRow1,
    &EncRow2,
    &EncRow3,
    &EncRow4,
    &EncRow5,
    &EncRow6,
    &EncUp,
    &EncDown,
    &EncSend,
    &EncExit,
  #endif

  NULL
};

#if HAS_LCD_MENU

  // Page txtmenu touch listen
  NexObject *txtmenu_list[] =
  {
    &EncRow1,
    &EncRow2,
    &EncRow3,
    &EncRow4,
    &EncRow5,
    &EncRow6,
    NULL
  };

#endif

/** Public Function */
void NextionLCD::init() {

  char cmd[NEXTION_BUFFER_SIZE] = { 0 };

  #if ENABLED(NEXTION_CONNECT_DEBUG)
    SERIAL_EM(" NEXTION connected at 9600 baud, ready");
  #endif

  for (uint8_t i = 0; i < 10; i++) {
    ZERO(cmd);
    nexSerial.begin(9600);  // Try at 9600
    if (getConnect(cmd)) {
      #if ENABLED(NEXTION_CONNECT_DEBUG)
        SERIAL_EM(" NEXTION connected at 9600 baud, changing baudrate");
      #endif
      sendCommandPGM(PSTR("baud=" STRINGIFY(NEXTION_BAUDRATE)));
      HAL::delayMilliseconds(100);
    }
    nexSerial.end();
    HAL::delayMilliseconds(100);
    nexSerial.begin(NEXTION_BAUDRATE);  // Try at NEXTION_BAUDRATE
    NextionON = getConnect(cmd);
    if (NextionON) break;
    nexSerial.end();
    HAL::delayMilliseconds(1000);
  }

  if (!NextionON) {
    SERIAL_LM(ER, "Nextion not connected!");
    return;
  }
  else {
    // Read Version Firmware Nextion
    sendCommandPGM(PSTR("get version.val"));
    const uint16_t nextion_version = recvRetNumber();

    // Write Version Firmware MK4duo
    sendCommandPGM(PSTR("p[1].b[10].txt=\"" SHORT_BUILD_VERSION "\""));

    // Start timer for logo anim
    sendCommandPGM(PSTR("tm0.en=1"));

    SERIAL_MSG("Nextion");

    // Get Model
    if (strstr_P(cmd, PSTR("3224"))) {       // Model 2.4" or 2.8" Normal or Enhanced
      SERIAL_MSG(" 2.4");
      #if ENABLED(NEXTION_GFX)
        gfx.set_position(1, 24, 250, 155);
      #endif
    }
    else if (strstr_P(cmd, PSTR("4024"))) {  // Model 3.2" Normal or Enhanced
      SERIAL_MSG(" 3.2");
      #if ENABLED(NEXTION_GFX)
        gfx.set_position(1, 24, 250, 155);
      #endif
    }
    else if (strstr_P(cmd, PSTR("4832"))) {  // Model 3.5" Normal or Enhanced
      SERIAL_MSG(" 3.5");
      #if ENABLED(NEXTION_GFX)
        gfx.set_position(1, 24, 250, 155);
      #endif
    }
    else if (strstr_P(cmd, PSTR("4827"))) {  // Model 4.3" Normal or Enhanced
      SERIAL_MSG(" 4.3");
      #if ENABLED(NEXTION_GFX)
        gfx.set_position(8, 28, 235, 140);
      #endif
    }
    else if (strstr_P(cmd, PSTR("8048"))) {  // Model 7" Normal, Enhanced or Intelligent
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

    set_status_page();

    #if LCD_TIMEOUT_TO_STATUS
      return_to_status_timer.start();
    #endif

    #if HAS_LCD_MENU
      // Check the Nextion Firmware
      if (nextion_version < NEXTION_LCD_FIRMWARE_VERSION) lcdui.goto_screen(lcd_nextion_allert);
    #endif

  }
}

void NextionLCD::read_serial() {

  static char     serial_nextion_buffer[MAX_CMD_SIZE];
  static uint8_t  cnt_0xFF  = 0,
                  index     = 0;

  while (nexSerial.available()) {

    int c;
    if ((c = nexSerial.read()) < 0) continue;

    if (c == 0xFF) cnt_0xFF++;
    else if (index < MAX_CMD_SIZE - 1) {
      cnt_0xFF = 0;
      serial_nextion_buffer[index++] = (char)c;
    }

    if (cnt_0xFF >= 3) {
      serial_nextion_buffer[index] = 0;
      char *command = serial_nextion_buffer;
      cnt_0xFF = index = 0;
      while (*command == ' ') command++;
      if (*command == NEX_RET_GCODE_OPERATION) {
        command++;
        commands.process_now(command);
      }
      else if (*command == NEX_RET_CURRENT_PAGE_ID_HEAD) {
        command++;
        set_page(uint8_t(*command));
      }
      else if (*command == NEX_RET_EVENT_TOUCH_HEAD) {
        command++;
        parse_key_touch(command);
      }
    }

  }
}

void NextionLCD::sendCommand(const char* cmd) {
  uint16_t crc_init = 0xFFFF;
  while (char c = *cmd++) {
    nexSerial.write(c);
    crc_modbus(&crc_init, c);
  }
  const uint8_t crc[3] = { crc_init & 0xFF, crc_init >> 8, 0x01 };
  nexSerial.write(crc, 3);
  sendCRC_end();
}

void NextionLCD::sendCommandPGM(PGM_P cmd) {
  uint16_t crc_init = 0xFFFF;
  while (char c = pgm_read_byte(cmd++)) {
    nexSerial.write(c);
    crc_modbus(&crc_init, c);
  }
  const uint8_t crc[3] = { crc_init & 0xFF, crc_init >> 8, 0x01 };
  nexSerial.write(crc, 3);
  sendCRC_end();
}

void NextionLCD::status_screen_update() {

  static uint8_t    PreviousPage          = 0xFF,
                    PreviousFeedrate      = 0xFF,
                    PreviousFanSpeed      = 0xFF,
                    PreviousPercent       = 0xFF,
                    PreviousSD            = 0xFF;

  static int16_t    PreviousDegHotend[MAX_HOTEND] = { 0xFF };

  #if HAS_BEDS
    static int16_t  PreviousDegBed        = 0xFF;
  #endif
  #if HAS_CHAMBERS
    static int16_t  PreviousDegChamber    = 0xFF;
  #endif
  #if HAS_DHT
    static int16_t  PreviousDegDht        = 0xFF;
  #endif

  char cmd[NEXTION_BUFFER_SIZE] = { 0 };

  const uint8_t max_hotends = MIN(4, tempManager.heater.hotends);

  #if ENABLED(NEXTION_GFX)           
    static bool GfxVis = false;
  #endif

  if (!NextionON) return;

  #if ENABLED(NEXTION_GFX)
    if (printer.isPrinting()) {
      if (!GfxVis) {
        GfxVis = true;
        nexlcd.sendCommandPGM(PSTR("prt.val=1"));
      }
    }
    else {
      if (GfxVis) {
        GfxVis = false;
        nexlcd.sendCommandPGM(PSTR("prt.val=0"));
      }
    }
  #endif

  if (PageID == 2) {

    if (PreviousPage != 2) {
      #if ENABLED(NEXTION_GFX)
        mechanics.nextion_gfx_clear();
      #endif
    }

    #if HAS_FAN
      if (PreviousFanSpeed != fans[0]->actual_speed()) {
        setValue(Fanspeed, fans[0]->percent());
        PreviousFanSpeed = fans[0]->actual_speed();
      }
    #endif

    #if HAS_CASE_LIGHT
      setValue(LightStatus, caselight.status ? 2 : 1);
    #endif

    if (PreviousFeedrate != mechanics.feedrate_percentage) {
      setValue(VSpeed, mechanics.feedrate_percentage);
      PreviousFeedrate = mechanics.feedrate_percentage;
    }

    #if HAS_HOTENDS
      for (uint8_t h = 0; h < max_hotends; h++) {
        setValue(Hotend_deg[h], hotends[h]->deg_current());
        if (PreviousDegHotend[h] != hotends[h]->deg_target()) {
          setValue(Hotend_trg[h], hotends[h]->deg_target());
          PreviousDegHotend[h] = hotends[h]->deg_target();
        }
      }
    #endif
    #if HAS_BEDS
      if (tempManager.heater.beds) {
        setValue(Bed_deg, beds[0]->deg_current());
        if (PreviousDegBed != beds[0]->deg_target()) {
          setValue(Bed_trg, beds[0]->deg_target());
          PreviousDegBed = beds[0]->deg_target();
        }
      }
    #endif
    #if HAS_CHAMBERS
      if (tempManager.heater.chambers) {
        setValue(Chamber_deg, chambers[0]->deg_current());
        if (PreviousDegChamber != chambers[0]->deg_target()) {
          setValue(Chamber_trg, chambers[0]->deg_target());
          PreviousDegChamber = chambers[0]->deg_target();
        }
      }
    #endif
    #if HAS_DHT
      if (lcdui.get_blink(3)) {
        if (PreviousDegDht != dhtsensor.humidity + 500) {
          setValue(DHT0, dhtsensor.humidity + 500);
          PreviousDegDht = dhtsensor.humidity + 500;
        }
      }
      else {
        if (PreviousDegDht != dhtsensor.temperature) {
          setValue(DHT0, dhtsensor.temperature);
          PreviousDegDht = dhtsensor.temperature;
        }
      }
    #endif

    if (PreviousPercent != printer.progress) {
      // Progress bar solid part
      setValue(progressbar, printer.progress);
      // Estimate End Time
      ZERO(cmd);
      char cmd1[10];
      uint8_t digit;
      duration_t Time = print_job_counter.duration();
      digit = Time.toDigital(cmd1, true);
      strcat(cmd, "S");
      strcat(cmd, cmd1);
      Time = (print_job_counter.duration() * (100 - printer.progress)) / (printer.progress + 0.1);
      digit += Time.toDigital(cmd1, true);
      if (digit > 14)
        strcat(cmd, "E");
      else
        strcat(cmd, " E");
      strcat(cmd, cmd1);
      setText(LcdTime, cmd);
      PreviousPercent = printer.progress;
    }

    if (printer.isPrinting()) {
      if (PreviousSD != SD_HOST_PRINTING) {
        setValue(SD, SD_HOST_PRINTING);
        PreviousSD = SD_HOST_PRINTING;
      }
    }
    else if (printer.isPaused()) {
      if (PreviousSD != SD_HOST_PAUSE) {
        setValue(SD, SD_HOST_PAUSE);
        PreviousSD = SD_HOST_PAUSE;
      }
    }
    #if HAS_SD_SUPPORT
      else if (IS_SD_MOUNTED()) {
        if (PreviousSD != SD_INSERT) {
          setValue(SD, SD_INSERT);
          PreviousSD = SD_INSERT;
        }
      }
      else if (!IS_SD_MOUNTED()) {
        if (PreviousSD != SD_NO_INSERT) {
          setValue(SD, SD_NO_INSERT);
          PreviousSD = SD_NO_INSERT;
        }
      }
    #else
      else {
        if (PreviousSD != NO_SD) {
          setValue(SD, NO_SD);
          PreviousSD = NO_SD;
        }
      }
    #endif

  }

  if (lcdui.get_blink(2)) coordtoLCD();

  PreviousPage = PageID;

}

void NextionLCD::moveto(const uint8_t col, const uint8_t row) {
  nexlcd.startChar(*txtmenu_list[row]);
  nexlcd.put_space(col - 1);
}

void NextionLCD::setText(NexObject &nexobject, PGM_P buffer) {
  char cmd[NEXTION_MAX_MESSAGE_LENGTH + 5];
  sprintf_P(cmd, PSTR("p[%u].b[%u].txt="), nexobject.pid, nexobject.cid);
  nexSerial.print(cmd);
  sprintf_P(cmd, PSTR("\"%s\""), buffer);
  nexSerial.print(cmd);
  sendCommand_end();
}

void NextionLCD::startChar(NexObject &nexobject) {
  char cmd[NEXTION_BUFFER_SIZE] = { 0 };
  sprintf_P(cmd, PSTR("p[%u].b[%u].txt=\""), nexobject.pid, nexobject.cid);
  nexSerial.print(cmd);
}

void NextionLCD::setChar(const char pchar) {
  nexSerial.write(pchar);
}

void NextionLCD::endChar() {
  nexSerial.print("\"");
  sendCommand_end();
}

void NextionLCD::setValue(NexObject &nexobject, const uint16_t number) {
  char cmd[NEXTION_BUFFER_SIZE] = { 0 };
  sprintf_P(cmd, PSTR("p[%u].b[%u].val=%u"), nexobject.pid, nexobject.cid, number);
  sendCommand(cmd);
}

void NextionLCD::Set_font_color_pco(NexObject &nexobject, const uint16_t number) {
  char cmd[NEXTION_BUFFER_SIZE] = { 0 };
  sprintf_P(cmd, PSTR("p[%u].b[%u].pco=%u"), nexobject.pid, nexobject.cid, number);
  sendCommand(cmd);
  Refresh(nexobject);
}

#if HAS_SD_SUPPORT

  void NextionLCD::UploadNewFirmware() {
    if (IS_SD_INSERTED() || card.isMounted()) {
      Firmware.startUpload();
      nexSerial.end();
      init();
    }
  }

#endif

#if HAS_LCD_MENU

  constexpr uint16_t  hot_color = 63488,
                      sel_color = 2016,
                      txt_color = 65535;

  void NextionLCD::put_space(const uint8_t max_length) {
    for (uint8_t i = 0; i < max_length; i++)
      setChar(' ');
  }

  void NextionLCD::put_str_P(PGM_P str, const uint8_t idx/*=0xFF*/) {
    const uint8_t len = strlen_P(str);
    for (uint8_t i = 0; i < len; i++) {
      char ch = pgm_read_byte(str++);
      setChar(ch);
    }
    if (idx != 0xFF) {
      setChar(' ');
      setChar(DIGIT(idx));
    }
  }

  void NextionLCD::put_str(const char * str) {
    const uint8_t len = strlen(str);
    for (uint8_t i = 0; i < len; i++)
      setChar(*str++);
  }

  void NextionLCD::mark_as_selected(const uint8_t row, const bool sel) {
    const uint16_t color = sel ? sel_color : txt_color;
    Set_font_color_pco(*txtmenu_list[row], color);
  }

  void NextionLCD::wrap_string(uint8_t &y, const char * const string, read_byte_cb_t cb_read_byte, bool wordwrap/*=false*/) {
    if (!string) return;
    uint8_t x = 0;
    startChar(*txtmenu_list[y]);
    uint8_t *p = (uint8_t*)string;
    wchar_t ch;
    if (wordwrap) {
      uint8_t *wrd = p, c = 0;
      for (;;) {
        p = get_utf8_value_cb(p, cb_read_byte, &ch);
        const bool eol = !ch;
        if (eol || ch == ' ' || ch == '-' || ch == '+' || ch == '.') {
          if (!c && ch == ' ') continue;  // collapse extra spaces
          if (x + c > LCD_WIDTH && x >= (LCD_WIDTH) / 4) { // should it wrap?
            x = 0; y++;                   // move x to string len (plus space)
            endChar();                    // simulate carriage return
            startChar(*txtmenu_list[y]);
          }
          c += !eol;                      // +1 so the space will be printed
          x += c;                         // advance x to new position
          while (c) {                     // character countdown
            --c;                          // count down to zero
            wrd = get_utf8_value_cb(wrd, cb_read_byte, &ch); // get characters again
            setChar(ch);                  // word (plus space) to the LCD
          }
          if (eol) {                      // all done
            endChar();
            break;
          }
          wrd = nullptr;                  // set up for next word
        }
        else {
          if (!wrd) wrd = p;              // starting a new word?
          c++;                            // count word characters
        }
      }
    }
    else {
      for (;;) {
        p = get_utf8_value_cb(p, cb_read_byte, &ch);
        if (!ch) {
          endChar();
          break;
        }
        setChar(ch);
        x++;
        if (x >= LCD_WIDTH) {
          x = 0; y++;
          endChar();
          startChar(*txtmenu_list[y]);
        }
      }
    }
  }

#endif

#if ENABLED(NEXTION_GFX)

  void NextionLCD::gfx_origin(const float x, const float y, const float z) {
    const xyz_pos_t pos = { x, y, z };
    gfx.origin(pos);
  }

  void NextionLCD::gfx_scale(const float scale) {
    gfx.set_scale(scale);
  }

  void NextionLCD::gfx_clear(const float x, const float y, const float z) {
    if (PageID == 2 && printer.isPrinting()) {
      const xyz_pos_t pos = { x, y, z };
      gfx.clear(pos);
    }
  }

  void NextionLCD::gfx_cursor_to(xyz_pos_t pos, bool force_cursor) {
    if (PageID == 2 && (printer.isPrinting() || force_cursor)) {
      #if MECH(DELTA)
        pos.x += mechanics.data.print_radius;
        pos.y += mechanics.data.print_radius;
      #endif
      gfx.cursor_to(pos);
    }
  }

  void NextionLCD::gfx_line_to(xyz_pos_t pos) {
    if (PageID == 2 && printer.isPrinting()) {
      #if MECH(DELTA)
        pos.x += mechanics.data.print_radius;
        pos.y += mechanics.data.print_radius;
      #endif
      #if ENABLED(ARDUINO_ARCH_SAM)
        gfx.line_to(NX_TOOL, pos, true);
      #else
        gfx.line_to(NX_TOOL, pos);
      #endif
    }
  }

  void NextionLCD::gfx_plane_to(xyz_pos_t pos) {
    uint8_t color;
    if (PageID == 2 && printer.isPrinting()) {
      #if MECH(DELTA)
        pos.x += mechanics.data.print_radius;
        pos.y += mechanics.data.print_radius;
      #endif
      if (pos.z < 10) color = NX_LOW;
      else color = NX_HIGH;
      gfx.line_to(color, pos, true);
    }
  }

#endif

#if ENABLED(RFID_MODULE)

  void NextionLCD::rfid_setText(PGM_P message, uint32_t color/*=65535*/) {
    char Rfid_status_message[25];
    strncpy(Rfid_status_message, message, 25);
    Set_font_color_pco(RfidText, color);
    setText(RfidText, Rfid_status_message);
  }

#endif

/** Private Function */
void NextionLCD::set_status_page() {
  char temp[10] = { 0 };
  const uint8_t max_hotends = MIN(4, tempManager.heater.hotends);

  #if HAS_HOTENDS
    for (uint8_t h = 0; h < max_hotends; h++) setValue(Hotend_deg[h], 25);
  #endif
  #if HAS_BEDS
    if (tempManager.heater.beds) setValue(Bed_deg, 25);
  #endif
  #if HAS_CHAMBERS
    if (tempManager.heater.chambers) setValue(Chamber_deg, 25);
  #endif
  #if HAS_DHT
    setValue(DHT0, 25);
  #endif

  #define EXTRUDERS_STRING(M) STRINGIFY(M)
  #define NEXTION_EXTRUDERS EXTRUDERS_STRING(EXTRUDERS)
  sendCommandPGM(PSTR("p[2].b[10].val=" NEXTION_EXTRUDERS));

  ZERO(temp);
  itoa(manual_feedrate_mm_s.x, temp, 10);
  setText(SpeedX, temp);
  ZERO(temp);
  itoa(manual_feedrate_mm_s.y, temp, 10);
  setText(SpeedY, temp);
  ZERO(temp);
  itoa(manual_feedrate_mm_s.z, temp, 10);
  setText(SpeedZ, temp);

  #if HAS_SD_SUPPORT
    if (!card.isMounted()) {
      card.mount();
      HAL::delayMilliseconds(500);
    }
    if (card.isMounted()) {
      setValue(SD, SD_INSERT);
      card.beginautostart();  // Initial boot
    }
    else
      setValue(SD, SD_NO_INSERT);
  #endif

  setValue(VSpeed, 100);

  #if HAS_FAN
    sendCommandPGM(PSTR("p[2].b[25].val=1"));
  #endif

  #if HAS_CASE_LIGHT
    setValue(LightStatus, caselight.status ? 2 : 1);
  #endif

  #if ENABLED(RFID_MODULE)
    sendCommandPGM(PSTR("p[2].b[29].val=1"));
  #endif

}

void NextionLCD::coordtoLCD() {
  char* valuetemp;
  char cmd[NEXTION_BUFFER_SIZE] = { 0 };
  const xyz_pos_t lpos = mechanics.position.asLogical();

  if (PageID == 2) {
    setText(LcdX, ftostr41sign(lpos.x));
    setText(LcdY, ftostr41sign(lpos.y));
    setText(LcdZ, ftostr41sign(FIXFLOAT(lpos.z)));
  }
  else if (PageID == 4) {
    if (mechanics.home_flag.XHomed) {
      valuetemp = ftostr4sign(lpos.x);
      strcat(cmd, "X");
      strcat(cmd, valuetemp);
    }
    else
      strcat(cmd, "?");

    if (mechanics.home_flag.YHomed) {
      valuetemp = ftostr4sign(lpos.y);
      strcat(cmd, " Y");
      strcat(cmd, valuetemp);
    }
    else
      strcat(cmd, " ?");

    if (mechanics.home_flag.ZHomed) {
      valuetemp = ftostr52sp(FIXFLOAT(lpos.z));
      strcat(cmd, " Z");
      strcat(cmd, valuetemp);
    }
    else
      strcat(cmd, " ?");

    setText(LcdCoord, cmd);
  }
}

void NextionLCD::set_page(const uint8_t page) {
  PageID = page;
}

void NextionLCD::parse_key_touch(const char* cmd) {
  for (uint8_t i = 0; nex_listen_list[i] != NULL; i++) {
    if (nex_listen_list[i]->pid == cmd[0] && nex_listen_list[i]->cid == cmd[1]) {
      if (cmd[2] == NEX_EVENT_POP) PopCallback(nex_listen_list[i]);
      break;
    }
  }
}

void NextionLCD::Refresh(NexObject &nexobject) {
  char cmd[NEXTION_BUFFER_SIZE] = { 0 };
  sprintf_P(cmd, PSTR("ref p[%u].b[%u]"), nexobject.pid, nexobject.cid);
  sendCommand(cmd);
}

void NextionLCD::PopCallback(NexObject *nexobject) {

  if (false)  {}

  #if HAS_SD_SUPPORT
    else if (nexobject == &NextionFW)   lcdui.goto_screen(menu_firmware);
    else if (nexobject == &SDMenu)      SDMenuPopCallback();
  #endif

  #if HAS_LCD_MENU
    else if ( nexobject == &TxtMenu)    lcdui.goto_screen(menu_main);
    else if ( nexobject == &EncRow1 ||
              nexobject == &EncRow2 ||
              nexobject == &EncRow3 ||
              nexobject == &EncRow4 ||
              nexobject == &EncRow5 ||
              nexobject == &EncRow6 ||
              nexobject == &EncUp   ||
              nexobject == &EncDown ||
              nexobject == &EncSend ||
              nexobject == &EncExit)    encoderPopCallback(nexobject);
  #endif

}

#if HAS_LCD_MENU

  void NextionLCD::encoderPopCallback(NexObject *nexobject) {
    // Click on encoder
    if (nexobject == &EncExit)
      lcdui.return_to_status();
    else if (nexobject == &EncUp)
      lcdui.encoderPosition += lcdui.encoderDirection;
    else if (nexobject == &EncDown)
      lcdui.encoderPosition -= lcdui.encoderDirection;
    else if (nexobject == &EncSend) {
      lcdui.lcd_clicked = true;
      printer.setWaitForUser(false);
    }

    // Click on text row
    if (line_encoder_touch) {
      for (uint8_t row = 0; row < LCD_HEIGHT; row++) {
        if (nexobject == txtmenu_list[row]) {
          lcdui.encoderPosition = row + encoderTopLine;
          lcdui.lcd_clicked = true;
        }
      }
    }

    #if LCD_TIMEOUT_TO_STATUS
      return_to_status_timer.start();
    #endif

    lcdui.refresh(LCDVIEW_REDRAW_NOW);

  }

#endif

#if HAS_SD_SUPPORT

  void NextionLCD::SDMenuPopCallback() {
    if (card.isMounted()) lcdui.goto_screen(menu_sdcard);
  }

#endif

uint16_t NextionLCD::recvRetNumber() {
  uint8_t temp[8] = { 0 };

  if (sizeof(temp) != nexSerial.readBytes((char *)temp, sizeof(temp)))
    return 0;

  if (temp[0] == NEX_RET_NUMBER_HEAD && temp[5] == 0xFF && temp[6] == 0xFF && temp[7] == 0xFF)
    return (uint16_t)(((uint32_t)temp[4] << 24) | ((uint32_t)temp[3] << 16) | (temp[2] << 8) | (temp[1]));
  else
    return 0;

}

bool NextionLCD::getConnect(char* buffer) {
  HAL::delayMilliseconds(100);
  sendCommand("");
  HAL::delayMilliseconds(100);
  sendCommandPGM(PSTR("connect"));
  HAL::delayMilliseconds(100);

  String temp = String("");

  #if ENABLED(NEXTION_CONNECT_DEBUG)
    SERIAL_MSG(" NEXTION Debug Connect received:");
  #endif

  while (nexSerial.available()) {
    uint8_t c = nexSerial.read();
    #if ENABLED(NEXTION_CONNECT_DEBUG)
      SERIAL_CHR((char)c);
    #endif
    temp += (char)c;
  }

  #if ENABLED(NEXTION_CONNECT_DEBUG)
    SERIAL_EOL();
  #endif

  strncpy(buffer, temp.c_str(), NEXTION_BUFFER_SIZE);

  return strstr(buffer, "comok");

}

/**
 * Class NexUpload
 */
#if HAS_SD_SUPPORT

  SdFile nextion_file;

  uint32_t  NexUpload::_baudrate,
            NexUpload::_unuploadByte,
            NexUpload::_upload_baudrate;

  const char* NexUpload::_file_name;

  NexUpload::NexUpload(const char* file_name, uint32_t upload_baudrate) {
    _file_name = file_name;
    _upload_baudrate = upload_baudrate;
  }

  void NexUpload::startUpload() {
    if (!_checkFile()) {
      SERIAL_LM(ER, "The file is error");
      return;
    }
    if (_getBaudrate() == 0) {
      SERIAL_LM(ER, "baudrate error");
      return;
    }
    if (!_setUploadBaudrate(_upload_baudrate)) {
      SERIAL_LM(ER, "modify baudrate error");
      return;
    }
    if (!_uploadTftFile()) {
      SERIAL_LM(ER, "upload file error");
      return;
    }
    nextion_file.sync();
    nextion_file.close();
    SERIAL_EM("upload ok");
  }

  uint16_t NexUpload::_getBaudrate() {
    const uint32_t baudrate_array[7] = { 115200, 57600, 38400, 19200, 9600, 4800, 2400 };
    for (uint8_t i = 0; i < 7; i++) {
      if (_searchBaudrate(baudrate_array[i])) {
        _baudrate = baudrate_array[i];
        break;
      }
    }
    return _baudrate;
  }

  bool NexUpload::_checkFile() {
    SERIAL_EMT("Start checkFile ", _file_name);
    if (!nextion_file.open(&card.root, _file_name, FILE_READ)) {
      SERIAL_LM(ER, "file is not exit");
      return false;
    }
    _unuploadByte = nextion_file.fileSize();
    return true;
  }

  bool NexUpload::_searchBaudrate(uint32_t baudrate) {
    String string = String("");
    nexSerial.end();
    HAL::delayMilliseconds(100);
    nexSerial.begin(baudrate);
    nexlcd.sendCommandPGM(NULL_STR);
    nexlcd.sendCommandPGM(PSTR("connect"));
    recvRetString(string);

    if(string.indexOf("comok") != -1)
      return true;

    return false;
  }

  uint16_t NexUpload::recvRetString(String &string, uint32_t timeout, bool recv_flag) {
    bool exit_flag = false;
    millis_l start = millis();

    while (millis() - start <= timeout) {
      while (nexSerial.available()) {
        uint8_t c = nexSerial.read();

        if (c == 0) continue;

        string += (char)c;
        if (recv_flag) {
          if (string.indexOf(0x05) != -1)
            exit_flag = true;
        }
      }
      if (exit_flag) break;
    }

    uint16_t ret = string.length();
    return ret;
  }

  bool NexUpload::_setUploadBaudrate(uint32_t baudrate) {
    String string = String("");
    String cmd = String("");

    String filesize_str = String(_unuploadByte, 10);
    String baudrate_str = String(baudrate, 10);
    cmd = "whmi-wri " + filesize_str + "," + baudrate_str + ",0";

    nexlcd.sendCommandPGM(NULL_STR);
    nexlcd.sendCommand(cmd.c_str());
    HAL::delayMilliseconds(50);
    nexSerial.begin(baudrate);
    recvRetString(string, 500);
    if (string.indexOf(0x05) != -1)
      return true;

    return false;
  }

  bool NexUpload::_uploadTftFile() {
    uint8_t c;
    uint16_t  send_timer = 0,
              last_send_num = 0;

    String string = String("");
    send_timer = _unuploadByte / 4096 + 1;
    last_send_num = _unuploadByte % 4096;

    while(send_timer) {
      if (send_timer == 1) {
        for (uint16_t j = 1; j <= 4096; j++) {
          if(j <= last_send_num) {
            c = (uint8_t)nextion_file.read();
            nexSerial.write(c);
          }
          else
            break;
        }
      }
      else {
        for (uint16_t i = 1; i <= 4096; i++) {
          c = (uint8_t)nextion_file.read();
          nexSerial.write(c);
        }
      }

      NexUpload::recvRetString(string, 500, true);
      if (string.indexOf(0x05) != -1)
        string = "";
      else
        return false;

      --send_timer;
    }

    return true;
  }

#endif  // SDSUPPORT

/**
 * Class LcdUI
 */
#if HAS_LCD_MENU

  /**
   * If the most recent manual move hasn't been fed to the planner yet,
   * and the planner can accept one, send a move immediately.
   */
  void LcdUI::manage_manual_move() {

    if (processing_manual_move) return;

    if (manual_move_axis != (int8_t)NO_AXIS && manual_move_timer.expired(move_menu_scale < 0.99f ? 0UL : 250UL, false) && !planner.is_full()) {

      #if IS_KINEMATIC

        const float old_feedrate = mechanics.feedrate_mm_s;
        mechanics.feedrate_mm_s = manual_feedrate_mm_s[manual_move_axis];

        toolManager.extruder.previous = toolManager.extruder.active;
        if (manual_move_axis == E_AXIS) toolManager.extruder.active = manual_move_e_index;

        // Set movement on a single axis
        mechanics.destination = mechanics.position;
        mechanics.destination[manual_move_axis] += manual_move_offset;

        // Reset for the next move
        manual_move_offset = 0;
        manual_move_axis = (int8_t)NO_AXIS;

        // DELTA and SCARA machines use segmented moves, which could fill the planner during the call to
        // move_to_destination. This will cause idle() to be called, which can then call this function while the
        // previous invocation is being blocked. Modifications to manual_move_offset shouldn't be made while
        // processing_manual_move is true or the planner will get out of sync.
        processing_manual_move = true;
        mechanics.prepare_move_to_destination();
        processing_manual_move = false;

        mechanics.feedrate_mm_s = old_feedrate;
        toolManager.extruder.active = toolManager.extruder.previous;

      #else

        planner.buffer_line(mechanics.position, manual_feedrate_mm_s[manual_move_axis], toolManager.extruder.active);
        manual_move_axis = (int8_t)NO_AXIS;

      #endif
    }
  }

  bool LcdUI::use_click() {
    const bool click = lcd_clicked;
    lcd_clicked = false;
    return click;
  }

#endif

void LcdUI::clear_lcd() {
  nexlcd.PageID = 11;
  nexlcd.sendCommandPGM(PSTR("page pg11"));
}

void LcdUI::init() { nexlcd.init(); }

bool LcdUI::get_blink(uint8_t moltiplicator/*=1*/) {
  static uint8_t blink = 0;
  static short_timer_t next_blink_timer(millis());
  if (next_blink_timer.expired(moltiplicator * 1000)) blink ^= 0xFF;
  return blink != 0;
}

void LcdUI::kill_screen(PGM_P lcd_msg) {
  set_alert_status_P(lcd_msg);
}

void LcdUI::update() {

  static short_timer_t next_lcd_update_timer(millis());

  if (!nexlcd.NextionON) return;

  nexlcd.read_serial();

  #if ENABLED(AUTO_BED_LEVELING_UBL)
    if (lcdui.external_control)
      ubl.encoder_diff = lcdui.encoderPosition;
  #endif

  #if HAS_LCD_MENU
    // Handle any queued Move Axis motion
    manage_manual_move();
  #endif

  #if HAS_SD_SUPPORT

    const uint8_t sd_status = (uint8_t)IS_SD_INSERTED();
    if (sd_status != nexlcd.lcd_sd_status && detected()) {

      uint8_t old_sd_status = nexlcd.lcd_sd_status; // prevent re-entry to this block!
      nexlcd.lcd_sd_status = sd_status;

      if (sd_status) {
        HAL::delayMilliseconds(500);  // Some boards need a delay to get settled
        card.mount();
        if (old_sd_status == 2)
          card.beginautostart();  // Initial boot
        else
          set_status_P(GET_TEXT(MSG_SD_INSERTED));
      }
      #if PIN_EXISTS(SD_DETECT)
        else {
          card.unmount();
          if (nexlcd.lcd_sd_status != 2) set_status_P(GET_TEXT(MSG_SD_REMOVED));
        }
      #endif

      refresh();
      next_lcd_update_timer.start();
    }

  #endif // HAS_SD_SUPPORT

  if (next_lcd_update_timer.expired(LCD_UPDATE_INTERVAL))
    nexlcd.status_screen_update();

  #if HAS_LCD_MENU

    if (nexlcd.PageID == 11) {

      #if LCD_TIMEOUT_TO_STATUS
        if (defer_return_to_status)
          nexlcd.return_to_status_timer.start();
        else if (nexlcd.return_to_status_timer.expired(LCD_TIMEOUT_TO_STATUS))
          return_to_status();
      #endif

      switch (lcdDrawUpdate) {
        case LCDVIEW_CALL_NO_REDRAW:
          refresh(LCDVIEW_NONE);
          break;
        case LCDVIEW_CLEAR_CALL_REDRAW:
        case LCDVIEW_CALL_REDRAW_NEXT:
          refresh(LCDVIEW_REDRAW_NOW);
        case LCDVIEW_REDRAW_NOW:
        case LCDVIEW_NONE:
          break;
      } // switch

      run_current_screen();

      switch (lcdDrawUpdate) {
        case LCDVIEW_CLEAR_CALL_REDRAW:
          clear_lcd(); break;
        case LCDVIEW_REDRAW_NOW:
          refresh(LCDVIEW_NONE);
        case LCDVIEW_NONE:
        case LCDVIEW_CALL_REDRAW_NEXT:
        case LCDVIEW_CALL_NO_REDRAW:
        default: break;
      } // switch

    }
    else {
      #if LCD_TIMEOUT_TO_STATUS
        nexlcd.return_to_status_timer = millis();
      #endif
    }

  #endif

}

bool LcdUI::detected() { return nexlcd.NextionON; }

void LcdUI::quick_feedback(const bool clear_buttons/*=true*/) {
  UNUSED(clear_buttons);
  #if HAS_LCD_MENU
    refresh();
  #endif
  // Buzz and wait. The delay is needed for buttons to settle!
  sound.playtone(LCD_FEEDBACK_FREQUENCY_DURATION_MS, LCD_FEEDBACK_FREQUENCY_HZ);
}

void LcdUI::set_alert_status_P(PGM_P const message) {
  set_status_P(message, 1);
  #if HAS_LCD_MENU
    return_to_status();
  #endif
}

void LcdUI::set_status(const char* const message, const bool) {
  if (alert_level || !nexlcd.NextionON) return;
  strncpy(status_message, message, NEXTION_MAX_MESSAGE_LENGTH);
  nexlcd.setText(LcdStatus, status_message);
}

void LcdUI::set_status_P(PGM_P const message, int8_t level/*=0*/) {
  if (level < 0) level = alert_level = 0;
  if (level < alert_level || !nexlcd.NextionON) return;
  alert_level = level;

  // Get a pointer to the null terminator
  PGM_P pend = message + strlen_P(message);

  while ((pend - message) > NEXTION_MAX_MESSAGE_LENGTH) {
    --pend;
    while (!((pgm_read_byte(pend) & 0xC0u) != 0x80u)) --pend;
  };

  uint8_t maxLen = pend - message;
  strncpy_P(status_message, message, maxLen);
  status_message[maxLen] = '\0';

  nexlcd.setText(LcdStatus, status_message);
}

#include <stdarg.h>

void LcdUI::status_printf_P(const uint8_t level, PGM_P const message, ...) {
  if (level < alert_level || !nexlcd.NextionON) return;
  alert_level = level;
  va_list args;
  va_start(args, message);
  vsnprintf(status_message, NEXTION_MAX_MESSAGE_LENGTH, message, args);
  va_end(args);
  nexlcd.setText(LcdStatus, status_message);
}

PGM_P print_paused = GET_TEXT(MSG_PRINT_PAUSED);

void LcdUI::reset_status() {

  PGM_P printing  = GET_TEXT(MSG_PRINTING);
  PGM_P welcome   = GET_TEXT(MSG_WELCOME);

  #if ENABLED(SERVICE_TIME_1)
    static const char service1[] PROGMEM = { "> " SERVICE_NAME_1 "!" };
  #endif
  #if ENABLED(SERVICE_TIME_2)
    static const char service2[] PROGMEM = { "> " SERVICE_NAME_2 "!" };
  #endif
  #if ENABLED(SERVICE_TIME_3)
    static const char service3[] PROGMEM = { "> " SERVICE_NAME_3 "!" };
  #endif
  PGM_P msg;
  if (print_job_counter.isPaused())
    msg = print_paused;
  #if HAS_SD_SUPPORT
    else if (IS_SD_PRINTING())
      return lcdui.set_status(card.fileName, true);
  #endif
  else if (print_job_counter.isRunning())
    msg = printing;
  #if ENABLED(SERVICE_TIME_1)
    else if (print_job_counter.needService(1)) msg = service1;
  #endif
  #if ENABLED(SERVICE_TIME_2)
    else if (print_job_counter.needService(2)) msg = service2;
  #endif
  #if ENABLED(SERVICE_TIME_3)
    else if (print_job_counter.needService(3)) msg = service3;
  #endif
  else
    msg = welcome;

  lcdui.set_status_P(msg, -1);
}

void LcdUI::status_screen() {
  if (nexlcd.PageID == 11) {
    nexlcd.PageID = 2;
    nexlcd.sendCommandPGM(PSTR("page pg2"));
  }
}

bool LcdUI::button_pressed() { return lcd_clicked; }

bool LcdUI::has_status() { return (status_message[0] != '\0'); }

/**
 * Print pause, resume and stop
 */
void LcdUI::pause_print() {

  #if HAS_LCD_MENU
    synchronize(GET_TEXT(MSG_PAUSE_PRINT));
  #endif

  host_action.prompt_open(PROMPT_PAUSE_RESUME, PSTR("LCD Pause"), PSTR("Resume"));

  set_status_P(print_paused);

  #if ENABLED(PARK_HEAD_ON_PAUSE)
    lcd_pause_show_message(PAUSE_MESSAGE_PAUSING, PAUSE_MODE_PAUSE_PRINT);  // Show message immediately to let user know about pause in progress
    commands.inject_P(PSTR("M25\nM24"));
  #elif HAS_SD_SUPPORT
    commands.inject_P(PSTR("M25"));
  #else
    host_action.pause();
  #endif

}

void LcdUI::resume_print() {
  reset_status();
  #if ENABLED(PARK_HEAD_ON_PAUSE)
    printer.setWaitForHeatUp(false);
    printer.setWaitForUser(false);
  #endif
  #if HAS_SD_SUPPORT
    commands.inject_P(M24_CMD);
  #endif
  host_action.resume();
  print_job_counter.start();
}

void LcdUI::stop_print() {
  #if HAS_SD_SUPPORT
    printer.setWaitForHeatUp(false);
    printer.setWaitForUser(false);
    if (IS_SD_PRINTING()) card.setAbortSDprinting(true);
  #endif
  host_action.cancel();
  host_action.prompt_open(PROMPT_INFO, PSTR("LCD Aborted"), PSTR("Dismiss"));
  print_job_counter.stop();
  set_status_P(GET_TEXT(MSG_PRINT_ABORTED));
  #if HAS_LCD_MENU
    return_to_status();
  #endif
}

#if ENABLED(AUTO_BED_LEVELING_UBL)
  void LcdUI::ubl_plot(const uint8_t x_plot, const uint8_t y_plot) {}
#endif

#if ENABLED(AUTO_BED_LEVELING_UBL) || ENABLED(G26_MESH_VALIDATION)
  void LcdUI::wait_for_release() { HAL::delayMilliseconds(50); }
#endif

// Draw function
#if HAS_LCD_MENU

  inline void wrap_string_P(uint8_t &y, PGM_P const pstr, const bool wordwrap=false)        { nexlcd.wrap_string(y, pstr,    read_byte_rom,  wordwrap);  }
  inline void wrap_string(uint8_t &y, const char * const string, const bool wordwrap=false) { nexlcd.wrap_string(y, string,  read_byte_ram,  wordwrap);  }

  #if ENABLED(ADVANCED_PAUSE_FEATURE)

    void LcdUI::draw_hotend_status(const uint8_t row, const uint8_t hotend) {

      UNUSED(row);

      static short_timer_t nex_update_timer(millis());

      char cmd[NEXTION_BUFFER_SIZE] = { 0 };

      if (nex_update_timer.expired(1500)) {

        strcat(cmd, "H");
        strcat(cmd, ui8tostr1(hotend));
        strcat(cmd, " ");
        strcat(cmd, i16tostr3(hotends[hotend]->deg_current()));
        strcat(cmd, "/");

        if (get_blink() || !hotends[hotend]->isIdle())
          strcat(cmd, i16tostr3(hotends[hotend]->deg_target()));

        nexlcd.Set_font_color_pco(*txtmenu_list[LCD_HEIGHT - 1], hot_color);
        nexlcd.setText(*txtmenu_list[LCD_HEIGHT - 1], cmd);

      }
    }

  #endif // ADVANCED_PAUSE_FEATURE

  // Draw a static line of text in the same idiom as a menu item
  void MenuItem_static::draw(const uint8_t row, PGM_P const pstr, const uint8_t style/*=SS_DEFAULT*/, const char * const valstr/*=nullptr*/) {
    nexlcd.line_encoder_touch = true;
    nexlcd.mark_as_selected(row, (style & SS_INVERT));
    nexlcd.startChar(*txtmenu_list[row]);
    nexlcd.put_str_P(pstr, itemIndex);
    if (valstr) nexlcd.put_str(valstr);
    nexlcd.endChar();
  }

  // Draw a generic menu item
  void MenuItemBase::_draw(const bool sel, const uint8_t row, PGM_P const pstr, const char, const char) {
    nexlcd.line_encoder_touch = true;
    nexlcd.mark_as_selected(row, sel);
    nexlcd.startChar(*txtmenu_list[row]);
    nexlcd.put_str_P(pstr, itemIndex);
    nexlcd.endChar();
  }

  // Draw a menu item with an editable value
  void  MenuEditItemBase::draw(const bool sel, const uint8_t row, PGM_P const pstr, const char* const data, const bool pgm) {
    const uint8_t labellen  = strlen_P(pstr) + (itemIndex != NO_INDEX ? 2 : 0),
                  vallen    = data ? (pgm ? strlen_P(data) : strlen((char*)data)) : 0;
    nexlcd.line_encoder_touch = true;
    nexlcd.mark_as_selected(row, sel);
    nexlcd.startChar(*txtmenu_list[row]);
    nexlcd.put_str_P(pstr, itemIndex);
    nexlcd.setChar(':');
    nexlcd.put_space(LCD_WIDTH - labellen - vallen - 1);
    if (pgm)
      nexlcd.put_str_P(data);
    else
      nexlcd.put_str((char*)data);
    nexlcd.endChar();
  }

  // Low-level draw_edit_screen can be used to draw an edit screen from anyplace
  void MenuEditItemBase::draw_edit_screen(PGM_P const pstr, const char* const value/*=nullptr*/) {
    lcdui.encoder_direction_normal();

    const uint8_t labellen  = strlen_P(pstr)+ (itemIndex != 0xFF ? 2 : 0),
                  vallen    = strlen(value);

    bool extra_row = labellen > LCD_WIDTH - vallen - 1;

    constexpr uint8_t row = 2;

    nexlcd.line_encoder_touch = false;

    nexlcd.Set_font_color_pco(*txtmenu_list[row], sel_color);

    if (extra_row) {
      nexlcd.Set_font_color_pco(*txtmenu_list[row - 1], sel_color);
      nexlcd.startChar(*txtmenu_list[row - 1]);
      nexlcd.put_str_P(pstr, itemIndex);
      nexlcd.endChar();
      nexlcd.startChar(*txtmenu_list[row]);
      nexlcd.put_space(LCD_WIDTH - vallen);
      nexlcd.put_str(value);
    }
    else {
      nexlcd.startChar(*txtmenu_list[row]);
      nexlcd.put_str_P(pstr, itemIndex);
      nexlcd.setChar(':');
      nexlcd.put_space(LCD_WIDTH - labellen - vallen - 1);
      nexlcd.put_str(value);
    }
    nexlcd.endChar();
  }

  void LcdUI::draw_select_screen_prompt(PGM_P const pref, const char * const string/*=nullptr*/, PGM_P const suff/*=nullptr*/) {
    nexlcd.startChar(*txtmenu_list[1]);
    nexlcd.put_str_P(pref);
    if (string) {
      nexlcd.endChar();
      nexlcd.startChar(*txtmenu_list[2]);
      nexlcd.put_str(string);
    }
    if (suff) nexlcd.put_str_P(suff);
    nexlcd.endChar();
  }

  void MenuItem_confirm::draw_select_screen(PGM_P const yes, PGM_P const no, const bool yesno, PGM_P const pref, const char * const string, PGM_P const suff) {
    lcdui.draw_select_screen_prompt(pref, string, suff);
    nexlcd.startChar(*txtmenu_list[LCD_HEIGHT - 1]);
    nexlcd.setChar(yesno ? ' ' : '[');
    nexlcd.put_str_P(no);
    nexlcd.setChar(yesno ? ' ' : ']');
    nexlcd.put_space(2);
    nexlcd.setChar(yesno ? '[' : ' ');
    nexlcd.put_str_P(yes);
    nexlcd.setChar(yesno ? ']' : ' ');
    nexlcd.endChar();
  }

  #if HAS_SD_SUPPORT

    void MenuItem_sdbase::draw(const bool sel, const uint8_t row, PGM_P const, SDCard &theCard, const bool isDir) {
      nexlcd.mark_as_selected(row, sel);
      nexlcd.startChar(*txtmenu_list[row]);
      if (isDir) nexlcd.put_str_P(PSTR(LCD_STR_FOLDER));
      nexlcd.put_str(theCard.fileName);
      nexlcd.endChar();
    }

  #endif // SDSUPPORT

#endif // HAS_LCD_MENU

#endif // HAS_NEXTION_LCD
