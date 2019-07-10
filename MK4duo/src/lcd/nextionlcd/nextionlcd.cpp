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

/**
 * nextionlcd.cpp
 *
 * Copyright (c) 2019 Alberto Cotronei @MagoKimbra
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

#define NEXTION_LCD_FIRMWARE_VERSION  120

NextionLCD nexlcd;

LcdUI lcdui;

/** LcdUI Parameters */
char    LcdUI::status_message[NEXTION_MAX_MESSAGE_LENGTH + 1];
uint8_t LcdUI::status_message_level = 0;

#if HAS_LCD_MENU

  extern bool no_reentry; // Flag to prevent recursion into menu handlers

  int8_t manual_move_axis = (int8_t)NO_AXIS;
  millis_s manual_move_ms = 0;

  #if IS_KINEMATIC
    float manual_move_offset = 0;
  #endif

  LCDViewActionEnum LcdUI::lcdDrawUpdate = LCDVIEW_CLEAR_CALL_REDRAW;

  uint16_t      LcdUI::encoderPosition;

  screenFunc_t  LcdUI::currentScreen;

  #if HAS_SD_SUPPORT && ENABLED(SCROLL_LONG_FILENAMES)
    uint8_t LcdUI::filename_scroll_pos, LcdUI::filename_scroll_max;
  #endif

  #if ENABLED(REVERSE_MENU_DIRECTION)
    int8_t LcdUI::encoderDirection = 1;
  #endif

  bool LcdUI::lcd_clicked = false;

  #if LCD_TIMEOUT_TO_STATUS
    bool LcdUI::defer_return_to_status;
  #endif

  #if IS_KINEMATIC
    bool LcdUI::processing_manual_move = false;
  #endif

  #if E_MANUAL > 1
    int8_t LcdUI::manual_move_e_index = 0;
  #endif

  #if ENABLED(AUTO_BED_LEVELING_UBL) || ENABLED(G26_MESH_VALIDATION)
    bool LcdUI::external_control; // = false
  #endif

#endif

/**
 * Class Nextion LCD
 */
/** Public Parameters */
bool    NextionLCD::NextionON                   = false;        

uint8_t NextionLCD::PageID                      = 0;

char    NextionLCD::buffer[NEXTION_BUFFER_SIZE] = { 0 };

#if HAS_SD_SUPPORT
  uint8_t NextionLCD::lcd_sd_status             = 2; // UNKNOWN
#endif

#if HAS_LCD_MENU
  bool  NextionLCD::line_encoder_touch          = false;
  #if LCD_TIMEOUT_TO_STATUS
    millis_l NextionLCD::return_to_status_ms    = millis();
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
NexObject LcdStatus     = NexObject(2,  6);
NexObject LcdX          = NexObject(2,  7);
NexObject LcdY          = NexObject(2,  8);
NexObject LcdZ          = NexObject(2,  9);
#if HOTENDS > 0
  NexObject Hotend00    = NexObject(2, 11);
  NexObject Hotend01    = NexObject(2, 12);
#endif
#if HOTENDS > 1
  NexObject Hotend10    = NexObject(2, 13);
  NexObject Hotend11    = NexObject(2, 14);
#endif
#if HOTENDS > 2
  NexObject Hotend20    = NexObject(2, 15);
  NexObject Hotend21    = NexObject(2, 16);
#endif
#if HOTENDS > 3
  NexObject Hotend30    = NexObject(2, 17);
  NexObject Hotend31    = NexObject(2, 18);
#endif
#if BEDS > 0
  NexObject Bed0        = NexObject(2, 19);
  NexObject Bed1        = NexObject(2, 20);
#endif
#if CHAMBERS > 0
  NexObject Chamber0    = NexObject(2, 21);
  NexObject Chamber1    = NexObject(2, 22);
#endif
#if ENABLED(DHT_SENSOR)
  NexObject DHT0        = NexObject(2, 23);
#endif
NexObject SD            = NexObject(2, 24);
NexObject Fanspeed      = NexObject(2, 26);
NexObject VSpeed        = NexObject(2, 27);
NexObject LightStatus   = NexObject(2, 28);
NexObject LcdTime       = NexObject(2, 30);
NexObject progressbar   = NexObject(2, 31);

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

  #if ENABLED(NEXTION_CONNECT_DEBUG)
    SERIAL_EM(" NEXTION connected at 115200 baud, ready");
  #endif

  for (uint8_t i = 0; i < 10; i++) {
    ZERO(buffer);
    nexSerial.begin(115200);
    NextionON = getConnect(buffer);
    if (NextionON) break;
    HAL::delayMilliseconds(1000);
  }

  if (!NextionON) {
    SERIAL_LM(ER, "Nextion not connected!");
    return;
  }
  else {

    // Read Version Firmware Nextion
    sendCommandPGM(PSTR("get pg0.version.val"));
    const uint16_t nextion_version = recvRetNumber();

    // Write Version Firmware MK4duo
    sendCommandPGM(PSTR("p[1].b[10].txt=\"" SHORT_BUILD_VERSION "\""));

    // Start timer for logo anim
    sendCommandPGM(PSTR("tm0.en=1"));

    SERIAL_MSG("Nextion");

    // Get Model
    if (strstr_P(buffer, PSTR("3224"))) {       // Model 2.4" or 2.8" Normal or Enhanced
      SERIAL_MSG(" 2.4");
      #if ENABLED(NEXTION_GFX)
        gfx.set_position(1, 24, 250, 155);
      #endif
    }
    else if (strstr_P(buffer, PSTR("4024"))) {  // Model 3.2" Normal or Enhanced
      SERIAL_MSG(" 3.2");
      #if ENABLED(NEXTION_GFX)
        gfx.set_position(1, 24, 250, 155);
      #endif
    }
    else if (strstr_P(buffer, PSTR("4832"))) {  // Model 3.5" Normal or Enhanced
      SERIAL_MSG(" 3.5");
      #if ENABLED(NEXTION_GFX)
        gfx.set_position(1, 24, 250, 155);
      #endif
    }
    else if (strstr_P(buffer, PSTR("4827"))) {  // Model 4.3" Normal or Enhanced
      SERIAL_MSG(" 4.3");
      #if ENABLED(NEXTION_GFX)
        gfx.set_position(1, 24, 250, 155);
      #endif
    }
    else if (strstr_P(buffer, PSTR("8048"))) {  // Model 7" Normal or Enhanced
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
      return_to_status_ms = millis();
    #endif

    #if HAS_LCD_MENU
      // Check the Nextion Firmware
      if (nextion_version < NEXTION_LCD_FIRMWARE_VERSION) lcdui.goto_screen(menu_nextion);
    #endif

  }
}

void NextionLCD::read_serial() {

  bool    str_start_flag  = false;
  uint8_t inputChar       = 0,
          startChar       = 0,
          cnt_0xFF        = 0,
          index           = 0;

  ZERO(buffer);
  while (nexSerial.available()) {
    inputChar = nexSerial.read();
    HAL::delayMicroseconds(120);
    if (  !str_start_flag && (
          inputChar == NEX_RET_GCODE_OPERATION
      ||  inputChar == NEX_RET_CURRENT_PAGE_ID_HEAD
      ||  inputChar == NEX_RET_EVENT_TOUCH_HEAD
    )) {
      str_start_flag = true;
      startChar = inputChar;
    }
    else if (str_start_flag) {
      if (inputChar == 0xFF) cnt_0xFF++;
      else buffer[index++] = inputChar;
      if (cnt_0xFF >= 3 || index == sizeof(buffer)) {
        if (startChar == NEX_RET_GCODE_OPERATION)
          commands.process_now(buffer);
        else if (startChar == NEX_RET_CURRENT_PAGE_ID_HEAD)
          set_page(uint8_t(buffer[0]));
        else
          parse_key_touch();
        cnt_0xFF = startChar = index = 0;
        ZERO(buffer);
        str_start_flag = false;
      }
    }
  }
}

void NextionLCD::sendCommand(const char* cmd) {
  nexSerial.print(cmd);
  sendCommand_end();
}

void NextionLCD::sendCommandPGM(PGM_P cmd) {
  while (char c = pgm_read_byte(cmd++)) nexSerial.write(c);
  sendCommand_end();
}

void NextionLCD::status_screen_update() {

  static uint8_t  PreviousPage                = 0xFF,
                  Previousfeedrate            = 0xFF,
                  PreviousfanSpeed            = 0xFF,
                  PreviouspercentDone         = 0xFF;

  #if ENABLED(NEXTION_GFX)           
    static bool GfxVis = false;
  #endif

  if (!NextionON) return;

  #if ENABLED(NEXTION_GFX)
    if (printer.isPrinting()) {
      if (!GfxVis) {
        GfxVis = true;
        nexlcd.sendCommandPGM(PSTR("p[2].b[5].val=1"));
      }
    }
    else {
      if (GfxVis) {
        GfxVis = false;
        nexlcd.sendCommandPGM(PSTR("p[2].b[5].val=0"));
      }
    }
  #endif

  if (PageID == 2) {

    if (PreviousPage != 2) {
      #if ENABLED(NEXTION_GFX)
        mechanics.nextion_gfx_clear();
      #endif
    }

    #if FAN_COUNT > 0
      if (PreviousfanSpeed != fans[0].actual_speed()) {
        setValue(Fanspeed, fans[0].percent());
        PreviousfanSpeed = fans[0].actual_speed();
      }
    #endif

    #if HAS_CASE_LIGHT
      setValue(LightStatus, caselight.status ? 2 : 1);
    #endif

    if (Previousfeedrate != mechanics.feedrate_percentage) {
      setValue(VSpeed, mechanics.feedrate_percentage);
      Previousfeedrate = mechanics.feedrate_percentage;
    }

    #if HOTENDS > 0
      setValue(Hotend00, hotends[0].current_temperature);
      setValue(Hotend01, hotends[0].target_temperature);
    #endif
    #if HOTENDS > 1
      setValue(Hotend10, hotends[1].current_temperature);
      setValue(Hotend11, hotends[1].target_temperature);
    #endif
    #if HOTENDS > 2
      setValue(Hotend20, hotends[2].current_temperature);
      setValue(Hotend21, hotends[2].target_temperature);
    #endif
    #if HOTENDS > 3
      setValue(Hotend30, hotends[3].current_temperature);
      setValue(Hotend31, hotends[3].target_temperature);
    #endif
    #if BEDS > 0
      setValue(Bed0, beds[0].current_temperature);
      setValue(Bed1, beds[0].target_temperature);
    #endif
    #if CHAMBERS > 0
      setValue(Chamber0, chambers[0].current_temperature);
      setValue(Chamber1, chambers[0].target_temperature);
    #endif
    #if ENABLED(DHT_SENSOR)
      if (lcdui.get_blink(3))
        setValue(DHT0, dhtsensor.Humidity + 500);
      else
        setValue(DHT0, dhtsensor.Temperature);
    #endif

    if (PreviouspercentDone != printer.progress) {
      // Progress bar solid part
      setValue(progressbar, printer.progress);
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
      setText(LcdTime, buffer);
      PreviouspercentDone = printer.progress;
    }

    if (printer.isPrinting())
      setValue(SD, SD_HOST_PRINTING);
    else if (printer.isPaused())
      setValue(SD, SD_HOST_PAUSE);
    else if (IS_SD_OK())
      setValue(SD, SD_INSERT);
    #if HAS_SD_SUPPORT
      else if (!IS_SD_OK())
        setValue(SD, SD_NO_INSERT);
    #else
      else
        setValue(SD, NO_SD);
    #endif

  }

  coordtoLCD();

  PreviousPage = PageID;

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
  char cmd[40];
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
  char cmd[40];
  sprintf_P(cmd, PSTR("p[%u].b[%u].val=%u"), nexobject.pid, nexobject.cid, number);
  sendCommand(cmd);
}

void NextionLCD::Set_font_color_pco(NexObject &nexobject, const uint16_t number) {
  char cmd[40];
  sprintf_P(cmd, PSTR("p[%u].b[%u].pco=%u"), nexobject.pid, nexobject.cid, number);
  sendCommand(cmd);
  Refresh(nexobject);
}

#if HAS_SD_SUPPORT

  void NextionLCD::UploadNewFirmware() {
    if (IS_SD_INSERTED() || card.isDetected()) {
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

  void NextionLCD::put_str_P(PGM_P str) {
    const uint8_t len = strlen_P(str);
    for (uint8_t i = 0; i < len; i++) {
      char ch = pgm_read_byte(str++);
      setChar(ch);
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
    gfx.origin(x, y, z);
  }

  void NextionLCD::gfx_scale(const float scale) {
    gfx.set_scale(scale);
  }

  void NextionLCD::gfx_clear(const float x, const float y, const float z) {
    if (PageID == 2 && printer.isPrinting())
      gfx.clear(x, y, z);
  }

  void NextionLCD::gfx_cursor_to(const float x, const float y, const float z, bool force_cursor) {
    if (PageID == 2 && (printer.isPrinting() || force_cursor))
      gfx.cursor_to(x, y, z);
  }

  void NextionLCD::gfx_line_to(const float x, const float y, const float z) {
    if (PageID == 2 && printer.isPrinting()) {
      #if ENABLED(ARDUINO_ARCH_SAM)
        gfx.line_to(NX_TOOL, x, y, z, true);
      #else
        gfx.line_to(NX_TOOL, x, y, z);
      #endif
    }
  }

  void NextionLCD::gfx_plane_to(const float x, const float y, const float z) {
    uint8_t color;
    if (PageID == 2) {
      if (z < 10) color = NX_LOW;
      else color = NX_HIGH;
      gfx.line_to(color, x, y, z, true);
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

  #if HOTENDS > 0
    setValue(Hotend00, 25);
  #endif
  #if HOTENDS > 1
    setValue(Hotend10, 25);
  #endif
  #if HOTENDS > 2
    setValue(Hotend20, 25);
  #endif
  #if HOTENDS > 3
    setValue(Hotend30, 25);
  #endif
  #if BEDS > 0
    setValue(Bed0, 25);
  #endif
  #if HAS_TEMP_CHAMBER0
    setValue(Chamber0, 25);
  #endif
  #if ENABLED(DHT_SENSOR)
    setValue(DHT0, 25);
  #endif

  #define EXTRUDERS_STRING(M) STRINGIFY(M)
  #define NEXTION_EXTRUDERS EXTRUDERS_STRING(EXTRUDERS)
  sendCommandPGM(PSTR("p[2].b[10].val=" NEXTION_EXTRUDERS));

  ZERO(temp);
  itoa(manual_feedrate_mm_m[X_AXIS], temp, 10);
  setText(SpeedX, temp);
  ZERO(temp);
  itoa(manual_feedrate_mm_m[Y_AXIS], temp, 10);
  setText(SpeedY, temp);
  ZERO(temp);
  itoa(manual_feedrate_mm_m[Z_AXIS], temp, 10);
  setText(SpeedZ, temp);

  #if HAS_SD_SUPPORT
    if (!card.isDetected()) {
      card.mount();
      HAL::delayMilliseconds(500);
    }
    if (card.isDetected()) {
      setValue(SD, SD_INSERT);
      card.beginautostart();  // Initial boot
    }
    else
      setValue(SD, SD_NO_INSERT);
  #endif

  setValue(VSpeed, 100);

  #if FAN_COUNT > 0
    setValue(Fanspeed, 1);
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
  ZERO(buffer);

  if (PageID == 2) {
    setText(LcdX, ftostr41sign(LOGICAL_X_POSITION(mechanics.current_position[X_AXIS])));
    setText(LcdY, ftostr41sign(LOGICAL_Y_POSITION(mechanics.current_position[Y_AXIS])));
    setText(LcdZ, ftostr41sign(FIXFLOAT(LOGICAL_Z_POSITION(mechanics.current_position[Z_AXIS]))));
  }
  else if (PageID == 4) {
    if (mechanics.home_flag.XHomed) {
      valuetemp = ftostr4sign(LOGICAL_X_POSITION(mechanics.current_position[X_AXIS]));
      strcat(buffer, "X");
      strcat(buffer, valuetemp);
    }
    else
      strcat(buffer, "?");

    if (mechanics.home_flag.YHomed) {
      valuetemp = ftostr4sign(LOGICAL_Y_POSITION(mechanics.current_position[Y_AXIS]));
      strcat(buffer, " Y");
      strcat(buffer, valuetemp);
    }
    else
      strcat(buffer, " ?");

    if (mechanics.home_flag.ZHomed) {
      valuetemp = ftostr52sp(FIXFLOAT(LOGICAL_Z_POSITION(mechanics.current_position[Z_AXIS])));
      strcat(buffer, " Z");
      strcat(buffer, valuetemp);
    }
    else
      strcat(buffer, " ?");

    setText(LcdCoord, buffer);
  }
}

void NextionLCD::set_page(const uint8_t page) {
  PageID = page;
}

void NextionLCD::parse_key_touch() {
  for (uint8_t i = 0; nex_listen_list[i] != NULL; i++) {
    if (nex_listen_list[i]->pid == buffer[0] && nex_listen_list[i]->cid == buffer[1]) {
      if (buffer[2] == NEX_EVENT_POP) PopCallback(nex_listen_list[i]);
      break;
    }
  }
}

void NextionLCD::Refresh(NexObject &nexobject) {
  char cmd[20];
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
      return_to_status_ms = millis();
    #endif

    lcdui.refresh(LCDVIEW_REDRAW_NOW);

  }

#endif

#if HAS_SD_SUPPORT

  void NextionLCD::SDMenuPopCallback() {
    if (card.isDetected()) lcdui.goto_screen(menu_sdcard);
  }

#endif

uint16_t NextionLCD::recvRetNumber(void) {
  uint8_t temp[8] = { 0 };

  nexSerial.setTimeout(NEX_TIMEOUT);

  if (sizeof(temp) != nexSerial.readBytes((char *)temp, sizeof(temp)))
    return NULL;

  if (temp[0] == NEX_RET_NUMBER_HEAD && temp[5] == 0xFF && temp[6] == 0xFF && temp[7] == 0xFF)
    return (uint16_t)(((uint32_t)temp[4] << 24) | ((uint32_t)temp[3] << 16) | (temp[2] << 8) | (temp[1]));
  else
    return NULL;

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

  void NexUpload::startUpload(void) {
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

  uint16_t NexUpload::_getBaudrate(void) {
    const uint32_t baudrate_array[7] = { 115200, 57600, 38400, 19200, 9600, 4800, 2400 };
    for (uint8_t i = 0; i < 7; i++) {
      if (_searchBaudrate(baudrate_array[i])) {
        _baudrate = baudrate_array[i];
        break;
      }
    }
    return _baudrate;
  }

  bool NexUpload::_checkFile(void) {
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
    nexlcd.sendCommandPGM(PSTR(""));
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

    nexlcd.sendCommandPGM(PSTR(""));
    nexlcd.sendCommand(cmd.c_str());
    HAL::delayMilliseconds(50);
    nexSerial.begin(baudrate);
    recvRetString(string, 500);
    if (string.indexOf(0x05) != -1)
      return true;

    return false;
  }

  bool NexUpload::_uploadTftFile(void) {
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

    if (manual_move_axis != (int8_t)NO_AXIS && expired(&manual_move_ms, (move_menu_scale < 0.99f ? 1U : 250U)) && !planner.is_full()) {

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
  static millis_s next_blink_ms = 0;
  if (expired(&next_blink_ms, millis_s(1000U * moltiplicator))) blink ^= 0xFF;
  return blink != 0;
}

void LcdUI::kill_screen(PGM_P lcd_msg) {
  set_alert_status_P(lcd_msg);
}

void LcdUI::update() {

  static millis_s next_lcd_update_ms = 0;

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
          set_status_P(PSTR(MSG_SD_INSERTED));
      }
      #if PIN_EXISTS(SD_DETECT)
        else {
          card.unmount();
          if (nexlcd.lcd_sd_status != 2) set_status_P(PSTR(MSG_SD_REMOVED));
        }
      #endif

      refresh();
      next_lcd_update_ms = millis();
    }

  #endif // HAS_SD_SUPPORT

  if (expired(&next_lcd_update_ms, LCD_UPDATE_INTERVAL))
    nexlcd.status_screen_update();

  #if HAS_LCD_MENU

    if (nexlcd.PageID == 11) {

      #if LCD_TIMEOUT_TO_STATUS
        if (defer_return_to_status)
          nexlcd.return_to_status_ms = millis();
        else if (expired(&nexlcd.return_to_status_ms, millis_l(LCD_TIMEOUT_TO_STATUS)))
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
        nexlcd.return_to_status_ms = millis();
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

void LcdUI::set_status(const char* const message, bool persist) {
  UNUSED(persist);
  if (status_message_level || !nexlcd.NextionON) return;
  strncpy(status_message, message, NEXTION_MAX_MESSAGE_LENGTH);
  nexlcd.setText(LcdStatus, status_message);
}

void LcdUI::set_status_P(PGM_P const message, int8_t level/*=0*/) {
  if (level < 0) level = status_message_level = 0;
  if (level < status_message_level || !nexlcd.NextionON) return;
  status_message_level = level;

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

void LcdUI::status_printf_P(const uint8_t level, PGM_P const fmt, ...) {
  if (level < status_message_level || !nexlcd.NextionON) return;
  status_message_level = level;
  va_list args;
  va_start(args, fmt);
  vsnprintf(status_message, NEXTION_MAX_MESSAGE_LENGTH, fmt, args);
  va_end(args);
  nexlcd.setText(LcdStatus, status_message);
}

void LcdUI::reset_status() {
  static const char paused[] PROGMEM = MSG_PRINT_PAUSED;
  static const char printing[] PROGMEM = MSG_PRINTING;
  static const char welcome[] PROGMEM = WELCOME_MSG;
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
    msg = paused;
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

  synchronize(PSTR(MSG_PAUSE_PRINT));

  #if HAS_SD_RESTART
    if (restart.enabled && IS_SD_PRINTING()) restart.save_job(true, false);
  #endif

  host_action.prompt_open(PROMPT_PAUSE_RESUME, PSTR("LCD Pause"), PSTR("Resume"));

  set_status_P(PSTR(MSG_PRINT_PAUSED));

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
    commands.inject_P(PSTR("M24"));
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
  host_action.prompt_open(PROMPT_INFO, PSTR("Lcd Abort"));
  print_job_counter.stop();
  set_status_P(PSTR(MSG_PRINT_ABORTED));
  return_to_status();
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

      static millis_s nex_update_ms = 0;

      if (expired(&nex_update_ms, 1500U)) {

        ZERO(nexlcd.buffer);
        strcat(nexlcd.buffer, MSG_FILAMENT_CHANGE_NOZZLE "H");
        strcat(nexlcd.buffer, ui8tostr1(hotend));
        strcat(nexlcd.buffer, " ");
        strcat(nexlcd.buffer, i16tostr3(hotends[hotend].current_temperature));
        strcat(nexlcd.buffer, "/");

        if (get_blink() || !hotends[hotend].isIdle())
          strcat(nexlcd.buffer, i16tostr3(hotends[hotend].target_temperature));

        nexlcd.Set_font_color_pco(*txtmenu_list[LCD_HEIGHT - 1], hot_color);
        nexlcd.setText(*txtmenu_list[LCD_HEIGHT - 1], nexlcd.buffer);

      }
    }

  #endif // ADVANCED_PAUSE_FEATURE

  // Draw a static line of text in the same idiom as a menu item
  void draw_menu_item_static(const uint8_t row, PGM_P const pstr, const bool center/*=true*/, const bool invert/*=false*/, const char* valstr/*=NULL*/) {
    UNUSED(center);
    nexlcd.line_encoder_touch = true;
    nexlcd.mark_as_selected(row, invert);
    nexlcd.startChar(*txtmenu_list[row]);
    nexlcd.put_str_P(pstr);
    if (valstr != NULL) nexlcd.put_str(valstr);
    nexlcd.endChar();
  }

  // Draw a generic menu item
  void draw_menu_item(const bool sel, const uint8_t row, PGM_P const pstr, const char pre_char, const char post_char) {
    UNUSED(pre_char); UNUSED(post_char);
    nexlcd.line_encoder_touch = true;
    nexlcd.mark_as_selected(row, sel);
    nexlcd.startChar(*txtmenu_list[row]);
    nexlcd.put_str_P(pstr);
    nexlcd.endChar();
  }

  // Draw a menu item with an editable value
  void _draw_menu_item_edit(const bool sel, const uint8_t row, PGM_P const pstr, const char* const data, const bool pgm) {
    const uint8_t labellen  = strlen_P(pstr),
                  vallen = (pgm ? strlen_P(data) : strlen((char*)data));
    nexlcd.line_encoder_touch = true;
    nexlcd.mark_as_selected(row, sel);
    nexlcd.startChar(*txtmenu_list[row]);
    nexlcd.put_str_P(pstr);
    nexlcd.put_str_P(PSTR(":"));
    nexlcd.put_space(LCD_WIDTH - labellen - vallen - 1);
    if (pgm)
      nexlcd.put_str_P(data);
    else
      nexlcd.put_str((char*)data);
    nexlcd.endChar();
  }

  void draw_edit_screen(PGM_P const pstr, const char* const value/*=nullptr*/) {

    const uint8_t labellen  = strlen_P(pstr),
                  vallen    = strlen(value);

    bool extra_row = labellen > LCD_WIDTH - vallen - 1;

    constexpr uint8_t row = 2;

    nexlcd.line_encoder_touch = false;

    nexlcd.Set_font_color_pco(*txtmenu_list[row], sel_color);

    if (extra_row) {
      nexlcd.Set_font_color_pco(*txtmenu_list[row - 1], sel_color);
      nexlcd.startChar(*txtmenu_list[row - 1]);
      nexlcd.put_str_P(pstr);
      nexlcd.endChar();
      nexlcd.startChar(*txtmenu_list[row]);
      nexlcd.put_space(LCD_WIDTH - vallen);
      nexlcd.put_str(value);
    }
    else {
      nexlcd.startChar(*txtmenu_list[row]);
      nexlcd.put_str_P(pstr);
      nexlcd.put_str_P(PSTR(":"));
      nexlcd.put_space(LCD_WIDTH - labellen - vallen - 1);
      nexlcd.put_str(value);
    }
    nexlcd.endChar();
  }

  inline void draw_select_screen_prompt(PGM_P const pref, const char * const string/*=nullptr*/, PGM_P const suff/*=nullptr*/) {
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

  void draw_select_screen(PGM_P const yes, PGM_P const no, const bool yesno, PGM_P const pref, const char * const string, PGM_P const suff) {
    draw_select_screen_prompt(pref, string, suff);
    nexlcd.startChar(*txtmenu_list[LCD_HEIGHT - 1]);
    nexlcd.put_str_P(yesno ? PSTR(" ") : PSTR("["));
    nexlcd.put_str_P(no);
    nexlcd.put_str_P(yesno ? PSTR(" ") : PSTR("]"));
    nexlcd.put_space(2);
    nexlcd.put_str_P(yesno ? PSTR("[") : PSTR(" "));
    nexlcd.put_str_P(yes);
    nexlcd.put_str_P(yesno ? PSTR("]") : PSTR(" "));
    nexlcd.endChar();
  }

  #if HAS_SD_SUPPORT

    void draw_sd_menu_item(const bool sel, const uint8_t row, PGM_P const pstr, SDCard &theCard, const bool isDir) {
      UNUSED(pstr);
      nexlcd.mark_as_selected(row, sel);
      nexlcd.startChar(*txtmenu_list[row]);
      if (isDir) nexlcd.put_str_P(PSTR(LCD_STR_FOLDER));
      nexlcd.put_str(theCard.fileName);
      nexlcd.endChar();
    }

  #endif // SDSUPPORT

#endif // HAS_LCD_MENU

#endif // HAS_NEXTION_LCD
