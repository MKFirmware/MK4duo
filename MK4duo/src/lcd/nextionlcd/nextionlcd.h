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
#pragma once

/**
 * nextion_lcd.h
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

// For debug Connect
//#define NEXTION_CONNECT_DEBUG

#if HAS_NEXTION_LCD

#include "nextion_gfx.h"

#if NEXTION_SERIAL > 0
  #if NEXTION_SERIAL == 1
    #define nexSerial Serial1
  #elif NEXTION_SERIAL == 2
    #define nexSerial Serial2
  #elif NEXTION_SERIAL == 3
    #define nexSerial Serial3
  #endif
#else
  #define nexSerial Serial1
#endif

#define NEX_RET_CMD_FINISHED                (0x01)
#define NEX_RET_EVENT_LAUNCHED              (0x88)
#define NEX_RET_EVENT_UPGRADED              (0x89)
#define NEX_RET_EVENT_TOUCH_HEAD            (0x65)
#define NEX_RET_EVENT_POSITION_HEAD         (0x67)
#define NEX_RET_EVENT_SLEEP_POSITION_HEAD   (0x68)
#define NEX_RET_CURRENT_PAGE_ID_HEAD        (0x66)
#define NEX_RET_STRING_HEAD                 (0x70)
#define NEX_RET_NUMBER_HEAD                 (0x71)
#define NEX_RET_VALUE_HEAD                  (0x72)
#define NEX_RET_INVALID_CMD                 (0x00)
#define NEX_RET_INVALID_COMPONENT_ID        (0x02)
#define NEX_RET_INVALID_PAGE_ID             (0x03)
#define NEX_RET_INVALID_PICTURE_ID          (0x04)
#define NEX_RET_INVALID_FONT_ID             (0x05)
#define NEX_RET_INVALID_BAUD                (0x11)
#define NEX_RET_INVALID_VARIABLE            (0x1A)
#define NEX_RET_INVALID_OPERATION           (0x1B)
#define NEX_RET_GCODE_OPERATION             (0x2A)

#define NEX_EVENT_POP                       (0x00)
#define NEX_EVENT_PUSH                      (0x01)

#define NEXTION_BUFFER_SIZE                  50
#define LCD_UPDATE_INTERVAL                 300U
#define NEX_TIMEOUT                         100U

// 0 card not present, 1 SD not insert, 2 SD insert, 3 SD_HOST printing, 4 SD_HOST paused
enum SDstatus_enum : uint8_t { NO_SD = 0, SD_NO_INSERT = 1, SD_INSERT = 2, SD_HOST_PRINTING = 3, SD_HOST_PAUSE = 4 };

constexpr uint8_t end[3] = { 0xFF, 0xFF, 0xFF };

class NexObject {

  public: /** Constructor */

    NexObject(uint8_t OBJ_PID, uint8_t OBJ_CID) :
      pid(OBJ_PID),
      cid(OBJ_CID)
      {}

  public: /** Public Parameters */

    const uint8_t pid,
                  cid;

};

#if HAS_SD_SUPPORT

  class NexUpload {

    public: /** Constructor */

      NexUpload(const char* file_name, uint32_t upload_baudrate);

      /**
       * destructor.
       */
      ~NexUpload(){}

    private: /** Private Parameters */

      static uint32_t     _baudrate,        /* nextion serial baudrate */
                          _unuploadByte,    /* unupload byte of tft file */
                          _upload_baudrate; /* upload baudrate */

      static const char*  _file_name;       /* nextion tft file name */

    public: /** Public Function */

      static void startUpload(void);

    private: /** Private Function */

      static uint16_t _getBaudrate(void);

      static bool _checkFile(void);

      static bool _searchBaudrate(uint32_t baudrate);

      static bool _setUploadBaudrate(uint32_t baudrate);

      static bool _uploadTftFile(void);

      static uint16_t recvRetString(String &string, uint32_t timeout = 100, bool recv_flag=false);

    private:

      
    };

#endif // SDSUPPORT

class NextionLCD {

  public: /** Constructor */

    NextionLCD() {}

  public: /** Public Parameters */

    static bool     NextionON;

    static uint8_t  PageID;

    static char     buffer[NEXTION_BUFFER_SIZE];

    #if HAS_SD_SUPPORT
      static uint8_t lcd_sd_status;
    #endif

    #if HAS_LCD_MENU
      static bool line_encoder_touch;
      #if LCD_TIMEOUT_TO_STATUS
        static millis_l return_to_status_ms;
      #endif
    #endif

  private: /** Private Parameters */

    #if HAS_SD_SUPPORT
      static NexUpload Firmware;
    #endif

  public: /** Public Function */

    static void init();

    static void read_serial();

    static void sendCommand(const char* cmd);
    static void sendCommandPGM(PGM_P cmd);

    static void status_screen_update();

    static void setText(NexObject &nexobject, PGM_P buffer);
    static void startChar(NexObject &nexobject);
    static void setChar(const char pchar);
    static void endChar();
    static void setValue(NexObject &nexobject, const uint16_t number);
    static void Set_font_color_pco(NexObject &nexobject, const uint16_t number);

    #if HAS_SD_SUPPORT
      static void UploadNewFirmware();
    #endif

    #if HAS_LCD_MENU
      static void put_space(const uint8_t max_length);
      static void put_str_P(PGM_P str);
      static void put_str(const char * str);
      static void mark_as_selected(const uint8_t row, const bool sel);
      static void wrap_string(uint8_t &y, const char * const string, read_byte_cb_t cb_read_byte, const bool wordwrap=false);
    #endif

    #if ENABLED(NEXTION_GFX)
      static void gfx_origin(const float x, const float y, const float z);
      static void gfx_scale(const float scale);
      static void gfx_clear(const float x, const float y, const float z);
      static void gfx_cursor_to(const float x, const float y, const float z, bool force_cursor=false);
      static void gfx_line_to(const float x, const float y, const float z);
      static void gfx_plane_to(const float x, const float y, const float z);
    #endif

    #if ENABLED(RFID_MODULE)
      static void rfid_setText(PGM_P message, uint32_t color=65535);
    #endif

  private: /** Private Function */

    static void set_status_page();
    static void coordtoLCD();

    static void set_page(const uint8_t page);
    static void parse_key_touch();
    static void Refresh(NexObject &nexobject);

    static void PopCallback(NexObject *nexobject);

    #if HAS_LCD_MENU
      static void encoderPopCallback(NexObject *nexobject);
    #endif
    #if HAS_SD_SUPPORT
      static void SDMenuPopCallback();
    #endif

    static uint16_t recvRetNumber();

    static bool getConnect(char* buffer);

    FORCE_INLINE static void sendCommand_end() { nexSerial.write(end, 3); }

};

extern NextionLCD nexlcd;

#endif // HAS_NEXTION_LCD
