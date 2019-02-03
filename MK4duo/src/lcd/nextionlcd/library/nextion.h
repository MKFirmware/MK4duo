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
#pragma once

/**
 * nextion.h
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

#define NEX_TIMEOUT                         100

/**
 * Push touch event occuring when your finger or pen coming to Nextion touch pannel. 
 */
#define NEX_EVENT_PUSH  (0x01)

/**
 * Pop touch event occuring when your finger or pen leaving from Nextion touch pannel. 
 */
#define NEX_EVENT_POP   (0x00)  

/**
 * Type of callback funciton when an touch event occurs. 
 * 
 * @param ptr - user pointer for any purpose. Commonly, it is a pointer to a object. 
 * @return none. 
 */

class NexObject {

  public: /** Constructor */

    NexObject(uint8_t OBJ_PID, uint8_t OBJ_CID, const char* OBJ_NAME=NULL) :
      __pid(OBJ_PID),
      __cid(OBJ_CID),
      __name(OBJ_NAME)
      {}

  public: /** Public Parameters */

    const uint8_t     __pid,
                      __cid;
    const char* const __name;

};

/**
 * Class Nextion
 */
class NextionLCD {

  public: /** Constructor */

    NextionLCD() {}

  public: /** Public Function */

    /**
     * Init Nextion.  
     * 
     * @return true if success, false for failure. 
     */
    static bool init(char *buffer);

    static void sendCommand(const char* cmd);
    static void sendCommandPGM(PGM_P cmd);
    static void recvRetCommandFinished(void);

    static void show(NexObject &nexobject);
    static void enable(NexObject &nexobject, const bool en=true);
    static void getText(NexObject &nexobject, char *buffer, PGM_P const page=NULL);
    static void setText(NexObject &nexobject, PGM_P buffer, PGM_P const page=NULL);
    static void startChar(NexObject &nexobject, const char * page=NULL);
    static void setChar(const char pchar);
    static void endChar();
    static void setValue(NexObject &nexobject, const uint16_t number, PGM_P const page=NULL);
    static void addValue(NexObject &nexobject, const uint8_t ch, const uint8_t number);
    static void Set_cursor_height_hig(NexObject &nexobject, const uint16_t number);
    static void setMaxval(NexObject &nexobject, const uint16_t number);
    static void setMinval(NexObject &nexobject, const uint16_t number);
    static void Set_background_color_bco(NexObject &nexobject, const uint16_t number);
    static void Set_font_color_pco(NexObject &nexobject, const uint16_t number);
    static void Set_place_xcen(NexObject &nexobject, const uint16_t number);
    static void Set_place_ycen(NexObject &nexobject, const uint16_t number);
    static void setFont(NexObject &nexobject, const uint16_t number);
    static void setCropPic(NexObject &nexobject, const uint16_t number);
    static void setPic(NexObject &nexobject, const uint16_t number);
    static void SetVisibility(NexObject &nexobject, const bool visible);
    static void Refresh(NexObject &nexobject);

    static uint16_t getValue(NexObject &nexobject, PGM_P const page=NULL);
    static uint16_t Get_cursor_height_hig(NexObject &nexobject);
    static uint16_t getMaxval(NexObject &nexobject);
    static uint16_t getMinval(NexObject &nexobject);
    static uint16_t Get_background_color_bco(NexObject &nexobject);
    static uint16_t Get_font_color_pco(NexObject &nexobject);
    static uint16_t Get_place_xcen(NexObject &nexobject);
    static uint16_t Get_place_ycen(NexObject &nexobject);
    static uint16_t getFont(NexObject &nexobject);
    static uint16_t getCropPic(NexObject &nexobject);
    static uint16_t getPic(NexObject &nexobject);
    static uint16_t recvRetNumber(void);

    static uint8_t pageID();

  private: /** Private Function */

    static bool getConnect(char* buffer);

    static void recvRetString(char *buffer);
    static void setCurrentBrightness(uint8_t dimValue);
    static void sendRefreshAll(void);

};

extern NextionLCD nexlcd;

/**
 * Class NexUpload
 */
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
