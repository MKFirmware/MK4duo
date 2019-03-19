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

/** Public Function */
bool NextionLCD::init(char *buffer) {
  // Try default baudrate
  nexSerial.begin(9600);

  ZERO(buffer);

  // If baudrate is 9600 set to 115200 and reconnect
  if (getConnect(buffer)) {
    #if ENABLED(NEXTION_CONNECT_DEBUG)
      SERIAL_EM(" NEXTION connected at 9600 baud, changing baudrate");
    #endif
    sendCommandPGM(PSTR("baud=115200"));
    HAL::delayMilliseconds(100);
    nexSerial.end();
    HAL::delayMilliseconds(100);
    nexSerial.begin(115200);
  }
  else { // Else try to 115200 baudrate
    #if ENABLED(NEXTION_CONNECT_DEBUG)
      SERIAL_EM(" NEXTION connected at 115200 baud, ready");
    #endif
    nexSerial.end();
    HAL::delayMilliseconds(100);
    nexSerial.begin(115200);
  }

  if (getConnect(buffer)) return true;
  else return false;
}

void NextionLCD::sendCommand(const char* cmd) {
  recvRetCommandFinished();
  nexSerial.print(cmd);
  nexSerial.write(0xFF);
  nexSerial.write(0xFF);
  nexSerial.write(0xFF);
}

void NextionLCD::sendCommandPGM(PGM_P cmd) {
  recvRetCommandFinished();
  while (char c = pgm_read_byte(cmd++)) nexSerial.write(c);
  nexSerial.write(0xFF);
  nexSerial.write(0xFF);
  nexSerial.write(0xFF);
}

void NextionLCD::recvRetCommandFinished(void) {    
  while (nexSerial.available()) nexSerial.read();
}

void NextionLCD::show(NexObject &nexobject) {
  char cmd[40];
  sprintf_P(cmd, PSTR("page %s"), nexobject.__name);
  sendCommand(cmd);
  recvRetCommandFinished();
}

void NextionLCD::enable(NexObject &nexobject, const bool en /* true */) {
  char cmd[40];
  sprintf_P(cmd, PSTR("%s.en=%s"), nexobject.__name, en ? "1" : "0");
  sendCommand(cmd);
  recvRetCommandFinished();
}

void NextionLCD::getText(NexObject &nexobject, char *buffer, PGM_P const page) {
  char cmd[40];
  if (page)
    sprintf_P(cmd, PSTR("get %s.%s.txt"), page, nexobject.__name);
  else
    sprintf_P(cmd, PSTR("get %s.txt"), nexobject.__name);
  sendCommand(cmd);
  recvRetString(buffer);
}

void NextionLCD::setText(NexObject &nexobject, PGM_P buffer, PGM_P const page) {
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

uint16_t NextionLCD::getValue(NexObject &nexobject, PGM_P const page) {
  char cmd[40];
  if (page)
    sprintf_P(cmd, PSTR("get %s.%s.val"), page, nexobject.__name);
  else
    sprintf_P(cmd, PSTR("get %s.val"), nexobject.__name);
  sendCommand(cmd);
  return recvRetNumber();
}

void NextionLCD::setValue(NexObject &nexobject, const uint16_t number, PGM_P const page) {
  char cmd[40];
  if (page)
    sprintf_P(cmd, PSTR("%s.%s.val=%u"), page, nexobject.__name, number);
  else
    sprintf_P(cmd, PSTR("%s.val=%u"), nexobject.__name, number);
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
  char cmd[40];
  sprintf_P(cmd, PSTR("%s.hig=%u"), nexobject.__name, number);
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
  char cmd[40];
  sprintf_P(cmd, PSTR("%s.maxval=%u"), nexobject.__name, number);
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
  char cmd[40];
  sprintf_P(cmd, PSTR("%s.minval=%u"), nexobject.__name, number);
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
  char cmd[40];
  sprintf_P(cmd, PSTR("%s.bco=%u"), nexobject.__name, number);
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
  char cmd[40];
  sprintf_P(cmd, PSTR("%s.pco=%u"), nexobject.__name, number);
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
  char cmd[40];
  sprintf_P(cmd, PSTR("%s.xcen=%u"), nexobject.__name, number);
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
  char cmd[40];
  sprintf_P(cmd, PSTR("%s.ycen=%u"), nexobject.__name, number);
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
  char cmd[40];
  sprintf_P(cmd, PSTR("%s.font=%u"), nexobject.__name, number);
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
  char cmd[40];
  sprintf_P(cmd, PSTR("%s.picc=%u"), nexobject.__name, number);
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
  char cmd[40];
  sprintf_P(cmd, PSTR("%s.pic=%u"), nexobject.__name, number);
  sendCommand(cmd);
  Refresh(nexobject);
}

void NextionLCD::SetVisibility(NexObject &nexobject, const bool visible) {
  char cmd[40];
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

uint8_t NextionLCD::pageID() {
  uint8_t temp[5] = { 0 };

  sendCommandPGM(PSTR("sendme"));

  nexSerial.setTimeout(NEX_TIMEOUT);

  if (sizeof(temp) != nexSerial.readBytes((char *)temp, sizeof(temp)))
    return NULL;

  if (temp[0] == NEX_RET_CURRENT_PAGE_ID_HEAD && temp[2] == 0xFF && temp[3] == 0xFF && temp[4] == 0xFF)
    return temp[1];
  else
    return NULL;
}

/** Private Function */
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

  if (strstr(buffer, "comok")) return true;

  return false;
}

void NextionLCD::recvRetString(char *buffer) {
  bool str_start_flag = false;
  uint8_t cnt_0xFF  = 0,
          index     = 0;

  millis_t start = millis();
  while (millis() - start <= NEX_TIMEOUT) {
    while (nexSerial.available()) {
      uint8_t c = nexSerial.read();
      if (c == NEX_RET_STRING_HEAD) {
        str_start_flag = true;
      }
      else if (str_start_flag) {
        if (c == 0xFF) {
          cnt_0xFF++;                    
          if (cnt_0xFF >= 3) break;
        }
        else {
          buffer[index++] = (char)c;
          if (index == sizeof(buffer)) break;
        }
      }
    }

    if (cnt_0xFF >= 3) break;
  }
}
  
void NextionLCD::setCurrentBrightness(uint8_t dimValue) {
  char cmd[10];
  sprintf_P(cmd, PSTR("dim=%i"), int(dimValue));
  sendCommandPGM(cmd);
  HAL::delayMilliseconds(10);
  recvRetCommandFinished();
}

void NextionLCD::sendRefreshAll(void) {
  sendCommandPGM(PSTR("ref 0"));
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

#endif // NEXTION
