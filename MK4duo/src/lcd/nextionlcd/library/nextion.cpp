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
 * nextion.cpp
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

#include "../../../../MK4duo.h"

#if HAS_NEXTION_LCD

  #include "nextion.h"

  NextionLCD nexlcd;

  void NextionLCD::show(NexObject &nexobject) {
    char cmd[40];
    sprintf_P(cmd, PSTR("page %s"), nexobject.__name);
    sendCommand(cmd);
    recvRetCommandFinished();
  }

  bool NextionLCD::getObjVis(NexObject &nexobject) { return nexobject.__vis; }
  
  void NextionLCD::enable(NexObject &nexobject, const bool en /* true */) {
    char cmd[40];
    sprintf_P(cmd, PSTR("%s.en=%s"), nexobject.__name, en ? "1" : "0");
    sendCommand(cmd);
    recvRetCommandFinished();
  }

  void NextionLCD::getText(NexObject &nexobject, char *buffer, const char * page) {
    char cmd[40];
    if (page)
      sprintf_P(cmd, PSTR("get %s.%s.txt"), page, nexobject.__name);
    else
      sprintf_P(cmd, PSTR("get %s.txt"), nexobject.__name);
    sendCommand(cmd);
    recvRetString(buffer);
  }

  void NextionLCD::setText(NexObject &nexobject, PGM_P buffer, const char * page) {
    char cmd[40];
    if (page)
      sprintf_P(cmd, PSTR("%s.%s.txt=\"%s\""), page, nexobject.__name, buffer);
    else
      sprintf_P(cmd, PSTR("%s.txt=\"%s\""), nexobject.__name, buffer);
    sendCommand(cmd);
    recvRetCommandFinished();
  }

  void NextionLCD::startChar(NexObject &nexobject, const char * page) {
    recvRetCommandFinished();
    char cmd[40];
    if (page)
      sprintf_P(cmd, PSTR("%s.%s.txt=\""), page, nexobject.__name);
    else
      sprintf_P(cmd, PSTR("%s.txt=\""), nexobject.__name);
    nexSerial.print(cmd);
  }

  void NextionLCD::setChar(const char pchar) {
    nexSerial.write(pchar);
  }

  void NextionLCD::endChar() {
    nexSerial.print("\"");
    nexSerial.write(0xFF);
    nexSerial.write(0xFF);
    nexSerial.write(0xFF);
  }

  uint16_t NextionLCD::getValue(NexObject &nexobject, const char * page) {
    char cmd[40];
    if (page)
      sprintf_P(cmd, PSTR("get %s.%s.val"), page, nexobject.__name);
    else
      sprintf_P(cmd, PSTR("get %s.val"), nexobject.__name);
    sendCommand(cmd);
    return recvRetNumber();
  }

  void NextionLCD::setValue(NexObject &nexobject, const uint16_t number, const char * page) {
    char buf[10];
    char cmd[40];
    utoa(number, buf, 10);
    if (page)
      sprintf_P(cmd, PSTR("%s.%s.val=%s"), page, nexobject.__name, buf);
    else
      sprintf_P(cmd, PSTR("%s.val=%s"), nexobject.__name, buf);
    sendCommand(cmd);
    recvRetCommandFinished();
  }

  void NextionLCD::addValue(NexObject &nexobject, const uint8_t ch, const uint8_t number) {
    char buf[15];
    if (ch > 3) return;
    sprintf_P(buf, PSTR("add %u,%u,%u"), nexobject.__cid, ch, number);
    sendCommand(buf);
  }

  uint16_t NextionLCD::Get_cursor_height_hig(NexObject &nexobject) {
    char cmd[40];
    sprintf_P(cmd, PSTR("get %s.hig"), nexobject.__name);
    sendCommand(cmd);
    return recvRetNumber();
  }

  void NextionLCD::Set_cursor_height_hig(NexObject &nexobject, const uint16_t number) {
    char buf[10];
    char cmd[40];
    utoa(number, buf, 10);
    sprintf_P(cmd, PSTR("%s.hig=%s"), nexobject.__name, buf);
    sendCommand(cmd);
    Refresh(nexobject);
  }

  uint16_t NextionLCD::getMaxval(NexObject &nexobject) {
    char cmd[40];
    sprintf_P(cmd, PSTR("get %s.maxval"), nexobject.__name);
    sendCommand(cmd);
    return recvRetNumber();
  }

  void NextionLCD::setMaxval(NexObject &nexobject, const uint16_t number) {
    char buf[10];
    char cmd[40];
    utoa(number, buf, 10);
    sprintf_P(cmd, PSTR("%s.maxval=%s"), nexobject.__name, buf);
    sendCommand(cmd);
    Refresh(nexobject);
  }

  uint16_t NextionLCD::getMinval(NexObject &nexobject) {
    char cmd[40];
    sprintf_P(cmd, PSTR("get %s.minval"), nexobject.__name);
    sendCommand(cmd);
    return recvRetNumber();
  }

  void NextionLCD::setMinval(NexObject &nexobject, const uint16_t number) {
    char buf[10];
    char cmd[40];
    utoa(number, buf, 10);
    sprintf_P(cmd, PSTR("%s.minval=%s"), nexobject.__name, buf);
    sendCommand(cmd);
    Refresh(nexobject);
  }

  uint16_t NextionLCD::Get_background_color_bco(NexObject &nexobject) {
    char cmd[40];
    sprintf_P(cmd, PSTR("get %s.bco"), nexobject.__name);
    sendCommand(cmd);
    return recvRetNumber();
  }

  void NextionLCD::Set_background_color_bco(NexObject &nexobject, const uint16_t number) {
    char buf[10];
    char cmd[40];
    utoa(number, buf, 10);
    sprintf_P(cmd, PSTR("%s.bco=%s"), nexobject.__name, buf);
    sendCommand(cmd);
    Refresh(nexobject);
  }

  uint16_t NextionLCD::Get_font_color_pco(NexObject &nexobject) {
    char cmd[40];
    sprintf_P(cmd, PSTR("get %s.pco"), nexobject.__name);
    sendCommand(cmd);
    return recvRetNumber();
  }

  void NextionLCD::Set_font_color_pco(NexObject &nexobject, const uint16_t number) {
    char buf[10];
    char cmd[40];
    utoa(number, buf, 10);
    sprintf_P(cmd, PSTR("%s.pco=%s"), nexobject.__name, buf);
    sendCommand(cmd);
    Refresh(nexobject);
  }

  uint16_t NextionLCD::Get_place_xcen(NexObject &nexobject) {
    char cmd[40];
    sprintf_P(cmd, PSTR("get %s.xcen"), nexobject.__name);
    sendCommand(cmd);
    return recvRetNumber();
  }

  void NextionLCD::Set_place_xcen(NexObject &nexobject, const uint16_t number) {
    char buf[10];
    char cmd[40];
    utoa(number, buf, 10);
    sprintf_P(cmd, PSTR("%s.xcen=%s"), nexobject.__name, buf);
    sendCommand(cmd);
    Refresh(nexobject);
  }

  uint16_t NextionLCD::Get_place_ycen(NexObject &nexobject) {
    char cmd[40];
    sprintf_P(cmd, PSTR("get %s.ycen"), nexobject.__name);
    sendCommand(cmd);
    return recvRetNumber();
  }

  void NextionLCD::Set_place_ycen(NexObject &nexobject, const uint16_t number) {
    char buf[10];
    char cmd[40];
    utoa(number, buf, 10);
    sprintf_P(cmd, PSTR("%s.ycen=%s"), nexobject.__name, buf);
    sendCommand(cmd);
    Refresh(nexobject);
  }

  uint16_t NextionLCD::getFont(NexObject &nexobject) {
    char cmd[40];
    sprintf_P(cmd, PSTR("get %s.font"), nexobject.__name);
    sendCommand(cmd);
    return recvRetNumber();
  }

  void NextionLCD::setFont(NexObject &nexobject, const uint16_t number) {
    char buf[10];
    char cmd[40];
    utoa(number, buf, 10);
    sprintf_P(cmd, PSTR("%s.font=%s"), nexobject.__name, buf);
    sendCommand(cmd);
    Refresh(nexobject);
  }

  uint16_t NextionLCD::getCropPic(NexObject &nexobject) {
    char cmd[40];
    sprintf_P(cmd, PSTR("get %s.picc"), nexobject.__name);
    sendCommand(cmd);
    return recvRetNumber();
  }

  void NextionLCD::setCropPic(NexObject &nexobject, const uint16_t number) {
    char buf[10];
    char cmd[40];
    utoa(number, buf, 10);
    sprintf_P(cmd, PSTR("%s.picc=%s"), nexobject.__name, buf);
    sendCommand(cmd);
    Refresh(nexobject);
  }

  uint16_t NextionLCD::getPic(NexObject &nexobject) {
    char cmd[40];
    sprintf_P(cmd, PSTR("get %s.pic"), nexobject.__name);
    sendCommand(cmd);
    return recvRetNumber();
  }

  void NextionLCD::setPic(NexObject &nexobject, const uint16_t number) {
    char buf[10];
    char cmd[40];
    utoa(number, buf, 10);
    sprintf_P(cmd, PSTR("%s.pic=%s"), nexobject.__name, buf);
    sendCommand(cmd);
    Refresh(nexobject);
  }

  void NextionLCD::SetVisibility(NexObject &nexobject, const bool visible) {
    char cmd[40];
    nexobject.__vis = visible;
    sprintf_P(cmd, PSTR("vis %s,%s"), nexobject.__name, visible ? PSTR("1") : PSTR("0"));
    sendCommand(cmd);
    recvRetCommandFinished();
  }

  void NextionLCD::Refresh(NexObject &nexobject) {
    char cmd[20];
    sprintf_P(cmd, PSTR("ref %s"), nexobject.__name);
    sendCommand(cmd);
    recvRetCommandFinished();
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
      sendCommandPGM(PSTR(""));
      sendCommandPGM(PSTR("connect"));
      NexUpload::recvRetString(string);

      if(string.indexOf("comok") != -1)
        return true;

      return false;
    }

    uint16_t NexUpload::recvRetString(String &string, uint32_t timeout, bool recv_flag) {
      bool exit_flag = false;
      millis_t start = millis();

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

      sendCommandPGM(PSTR(""));
      sendCommand(cmd.c_str());
      HAL::delayMilliseconds(50);
      nexSerial.begin(baudrate);
      NexUpload::recvRetString(string, 500);
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

#endif // NEXTION
