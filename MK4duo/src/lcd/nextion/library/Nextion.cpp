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

#include "../../../../MK4duo.h"

#if ENABLED(NEXTION)

  #include "Nextion.h"

  NexObject::NexObject(uint8_t pid, uint8_t cid, const char *name) {
    this->__pid = pid;
    this->__cid = cid;
    this->__name = name;
    this->__vis = true;
    this->__cb_push = NULL;
    this->__cb_pop = NULL;
    this->__cbpop_ptr = NULL;
    this->__cbpush_ptr = NULL;
  }

  bool NexObject::getObjVis(void) { return __vis; }

  void NexObject::attachPush(NexTouchEventCb push, void *ptr) {
    this->__cb_push = push;
    this->__cbpush_ptr = ptr;
  }

  void NexObject::detachPush(void) {
    this->__cb_push = NULL;
    this->__cbpush_ptr = NULL;
  }

  void NexObject::attachPop(NexTouchEventCb pop, void *ptr) {
    this->__cb_pop = pop;
    this->__cbpop_ptr = ptr;
  }

  void NexObject::detachPop(void) {
    this->__cb_pop = NULL;    
    this->__cbpop_ptr = NULL;
  }

  void NexObject::push(void) { if (__cb_push) __cb_push(__cbpush_ptr); }

  void NexObject::pop(void) { if (__cb_pop) __cb_pop(__cbpop_ptr); }

  void NexObject::iterate(NexObject **list, const uint8_t pid, const uint8_t cid, const int32_t event) {
    NexObject *e = NULL;
    uint16_t i = 0;

    if (NULL == list) return;

    for (i = 0; (e = list[i]) != NULL; i++) {
      if (e->__pid == pid && e->__cid == cid) {
        if (NEX_EVENT_PUSH == event)
          e->push();
        else if (NEX_EVENT_POP == event)
          e->pop();
        break;
      }
    }
  }

  /**
   * FUNCTION FOR ALL OBJECT
   */

  void NexObject::show() {
    String cmd;
    cmd += "page ";
    cmd += this->__name;
    sendCommand(cmd.c_str());
    recvRetCommandFinished();
  }

  void NexObject::enable(const bool en /* true */) {
    String cmd;
    cmd += this->__name;
    cmd += ".en=";
    cmd += en ? "1" : "0";
    sendCommand(cmd.c_str());
    recvRetCommandFinished();
  }

  void NexObject::getText(char *buffer, uint16_t len, const char *pname) {
    String cmd;
    cmd += "get ";
    if (pname) {
      cmd += pname;
      cmd += ".";
    }
    cmd += this->__name;
    cmd += ".txt";
    sendCommand(cmd.c_str());
    recvRetString(buffer, len);
  }

  void NexObject::setText(const char *buffer, const char *pname) {
    String cmd;
    if (pname) {
      cmd += pname;
      cmd += ".";
    }
    cmd += this->__name;
    cmd += ".txt=\"";
    cmd += buffer;
    cmd += "\"";
    sendCommand(cmd.c_str());
    recvRetCommandFinished();
  }

  uint16_t NexObject::getValue(const char *pname) {
    String cmd;
    cmd += "get ";
    if (pname) {
      cmd += pname;
      cmd += ".";
    }
    cmd += this->__name;
    cmd += ".val";
    sendCommand(cmd.c_str());
    return recvRetNumber();
  }

  void NexObject::setValue(const uint16_t number, const char *pname) {
    char buf[10] = {0};
    String cmd;
    utoa(number, buf, 10);

    if (pname) {
      cmd += pname;
      cmd += ".";
    }
    cmd += this->__name;
    cmd += ".val=";
    cmd += buf;

    sendCommand(cmd.c_str());
    recvRetCommandFinished();
  }

  void NexObject::addValue(const uint8_t ch, const uint8_t number) {
    char buf[15] = {0};
    if (ch > 3) return;
    sprintf(buf, "add %u,%u,%u", this->__cid, ch, number);
    sendCommand(buf);
  }

  uint16_t NexObject::Get_cursor_height_hig() {
    String cmd;
    cmd += "get ";
    cmd += this->__name;
    cmd += ".hig";
    sendCommand(cmd.c_str());
    return recvRetNumber();
  }

  void NexObject::Set_cursor_height_hig(const uint16_t number) {
    char buf[10] = {0};
    String cmd;

    utoa(number, buf, 10);
    cmd += this->__name;
    cmd += ".hig=";
    cmd += buf;
    sendCommand(cmd.c_str());

    cmd = "";
    cmd += "ref ";
    cmd += this->__name;
    sendCommand(cmd.c_str());
    recvRetCommandFinished();
  }

  uint16_t NexObject::getMaxval() {
    String cmd;
    cmd += "get ";
    cmd += this->__name;
    cmd += ".maxval";
    sendCommand(cmd.c_str());
    return recvRetNumber();
  }

  void NexObject::setMaxval(const uint16_t number) {
    char buf[10] = {0};
    String cmd;

    utoa(number, buf, 10);
    cmd += this->__name;
    cmd += ".maxval=";
    cmd += buf;
    sendCommand(cmd.c_str());

    cmd = "";
    cmd += "ref ";
    cmd += this->__name;
    sendCommand(cmd.c_str());
    recvRetCommandFinished();
  }

  uint16_t NexObject::getMinval() {
    String cmd;
    cmd += "get ";
    cmd += this->__name;
    cmd += ".minval";
    sendCommand(cmd.c_str());
    return recvRetNumber();
  }

  void NexObject::setMinval(const uint16_t number) {
    char buf[10] = {0};
    String cmd;

    utoa(number, buf, 10);
    cmd += this->__name;
    cmd += ".minval=";
    cmd += buf;
    sendCommand(cmd.c_str());

    cmd = "";
    cmd += "ref ";
    cmd += this->__name;
    sendCommand(cmd.c_str());
    recvRetCommandFinished();
  }

  uint16_t NexObject::Get_background_color_bco() {
    String cmd;
    cmd += "get ";
    cmd += this->__name;
    cmd += ".bco";
    sendCommand(cmd.c_str());
    return recvRetNumber();
  }

  void NexObject::Set_background_color_bco(const uint16_t number) {
    char buf[10] = {0};
    String cmd;

    utoa(number, buf, 10);
    cmd += this->__name;
    cmd += ".bco=";
    cmd += buf;
    sendCommand(cmd.c_str());

    cmd="";
    cmd += "ref ";
    cmd += this->__name;
    sendCommand(cmd.c_str());
    recvRetCommandFinished();
  }

  uint16_t NexObject::Get_font_color_pco() {
    String cmd;
    cmd += "get ";
    cmd += this->__name;
    cmd += ".pco";
    sendCommand(cmd.c_str());
    return recvRetNumber();
  }

  void NexObject::Set_font_color_pco(const uint16_t number) {
    char buf[10] = {0};
    String cmd;

    utoa(number, buf, 10);
    cmd += this->__name;
    cmd += ".pco=";
    cmd += buf;
    sendCommand(cmd.c_str());

    cmd = "";
    cmd += "ref ";
    cmd += this->__name;
    sendCommand(cmd.c_str());
    recvRetCommandFinished();
  }

  uint16_t NexObject::Get_place_xcen() {
    String cmd;
    cmd += "get ";
    cmd += this->__name;
    cmd += ".xcen";
    sendCommand(cmd.c_str());
    return recvRetNumber();
  }

  void NexObject::Set_place_xcen(const uint16_t number) {
    char buf[10] = {0};
    String cmd;

    utoa(number, buf, 10);
    cmd += this->__name;
    cmd += ".xcen=";
    cmd += buf;
    sendCommand(cmd.c_str());

    cmd = "";
    cmd += "ref ";
    cmd += this->__name;
    sendCommand(cmd.c_str());
    recvRetCommandFinished();
  }

  uint16_t NexObject::Get_place_ycen() {
    String cmd;
    cmd += "get ";
    cmd += this->__name;
    cmd += ".ycen";
    sendCommand(cmd.c_str());
    return recvRetNumber();
  }

  void NexObject::Set_place_ycen(const uint16_t number) {
    char buf[10] = {0};
    String cmd;

    utoa(number, buf, 10);
    cmd += this->__name;
    cmd += ".ycen=";
    cmd += buf;
    sendCommand(cmd.c_str());

    cmd = "";
    cmd += "ref ";
    cmd += this->__name;
    sendCommand(cmd.c_str());
    recvRetCommandFinished();
  }

  uint16_t NexObject::getFont() {
    String cmd;
    cmd += "get ";
    cmd += this->__name;
    cmd += ".font";
    sendCommand(cmd.c_str());
    return recvRetNumber();
  }

  void NexObject::setFont(const uint16_t number) {
    char buf[10] = {0};
    String cmd;

    utoa(number, buf, 10);
    cmd += this->__name;
    cmd += ".font=";
    cmd += buf;
    sendCommand(cmd.c_str());

    cmd = "";
    cmd += "ref ";
    cmd += this->__name;
    sendCommand(cmd.c_str());
    recvRetCommandFinished();
  }

  uint16_t NexObject::getCropPic() {
    String cmd;
    cmd += "get ";
    cmd += this->__name;
    cmd += ".picc";
    sendCommand(cmd.c_str());
    return recvRetNumber();
  }

  void NexObject::setCropPic(const uint16_t number) {
    char buf[10] = {0};
    String cmd;

    utoa(number, buf, 10);
    cmd += this->__name;
    cmd += ".picc=";
    cmd += buf;
    sendCommand(cmd.c_str());

    cmd = "";
    cmd += "ref ";
    cmd += this->__name;
    sendCommand(cmd.c_str());
    recvRetCommandFinished();
  }

  uint16_t NexObject::getPic() {
    String cmd = String("get ");
    cmd += this->__name;
    cmd += ".pic";
    sendCommand(cmd.c_str());
    return recvRetNumber();
  }

  void NexObject::setPic(const uint16_t number) {
    char buf[10] = {0};
    String cmd;

    utoa(number, buf, 10);
    cmd += this->__name;
    cmd += ".pic=";
    cmd += buf;

    sendCommand(cmd.c_str());
    recvRetCommandFinished();
  }

  void NexObject::SetVisibility(const bool visible) {
    String cmd;
    cmd += "vis ";
    cmd += this->__name;
    cmd += ',';
    cmd += (visible ? '1' : '0');
    __vis = visible;
    sendCommand(cmd.c_str());
    recvRetCommandFinished();
  }

  /**
   * Class NexUpload
   */
  #if HAS_SD_SUPPORT

    SdFile nextion_file;

    NexUpload::NexUpload(const char *file_name, uint32_t upload_baudrate) {
      _file_name = file_name;
      _upload_baudrate = upload_baudrate;
    }

    NexUpload::NexUpload(const String file_name, uint32_t upload_baudrate) {
      NexUpload(file_name.c_str(), upload_baudrate);
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
      if (!nextion_file.open(&card.root, _file_name, O_READ)) {
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
      sendCommand("");
      sendCommand("connect");
      this->recvRetString(string);

      if(string.indexOf("comok") != -1)
        return true;

      return false;
    }

    uint16_t NexUpload::recvRetString(String &string, uint32_t timeout,bool recv_flag) {
      uint16_t ret = 0;
      uint8_t c = 0;
      millis_t start;
      bool exit_flag = false;
      start = millis();
      while (millis() - start <= timeout) {
        while (nexSerial.available()) {
          c = nexSerial.read();

          if (c == 0) continue;

          string += (char)c;
          if (recv_flag) {
            if (string.indexOf(0x05) != -1)
              exit_flag = true;
          }
        }
        if (exit_flag) break;
      }
      ret = string.length();
      return ret;
    }

    bool NexUpload::_setUploadBaudrate(uint32_t baudrate) {
      String string = String("");
      String cmd = String("");

      String filesize_str = String(_unuploadByte, 10);
      String baudrate_str = String(baudrate, 10);
      cmd = "whmi-wri " + filesize_str + "," + baudrate_str + ",0";

      sendCommand("");
      sendCommand(cmd.c_str());
      HAL::delayMilliseconds(50);
      nexSerial.begin(baudrate);
      this->recvRetString(string, 500);
      if (string.indexOf(0x05) != -1)
        return true;

      return false;
    }

    bool NexUpload::_uploadTftFile(void) {
      uint8_t c;
      uint16_t send_timer = 0;
      uint16_t last_send_num = 0;
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

        this->recvRetString(string, 500, true);
        if (string.indexOf(0x05) != -1)
          string = "";
        else
          return false;

        --send_timer;
      }

      return true;
    }

  #endif  // SDSUPPORT

  bool getConnect(char* buffer) {
    HAL::delayMilliseconds(100);
    sendCommand("");
    HAL::delayMilliseconds(100);
    sendCommand("connect");
    HAL::delayMilliseconds(100);

    uint8_t   c = 0;
    String temp = String("");

    #if ENABLED(NEXTION_CONNECT_DEBUG)
      SERIAL_MSG(" NEXTION Debug Connect receveid:");
    #endif

    while (nexSerial.available()) {
      c = nexSerial.read();
      #if ENABLED(NEXTION_CONNECT_DEBUG)
        SERIAL_CHR((char)c);
      #endif
      temp += (char)c;
    }

    #if ENABLED(NEXTION_CONNECT_DEBUG)
      SERIAL_EOL();
    #endif

    strncpy(buffer, temp.c_str(), 70);

    if (strstr(buffer, "comok")) return true;

    return false;
  }

  //
  // PUBBLIC FUNCTION
  //

  bool nexInit(char *buffer) {

    // Try default baudrate
    nexSerial.begin(9600);

    ZERO(buffer);
    bool connect = getConnect(buffer);

    // If baudrate is 9600 set to 115200 and reconnect
    if (connect) {
      sendCommand("baud=115200");
      nexSerial.end();
      HAL::delayMilliseconds(1000);
      nexSerial.begin(115200);
      return true;
    }
    else { // Else try to 115200 baudrate
      nexSerial.end();
      HAL::delayMilliseconds(1000);
      nexSerial.begin(115200);
      connect = getConnect(buffer);
      if (connect) return true;
    }
    return false;
  }
  
  void nexLoop(NexObject *nex_listen_list[]) {
    static uint8_t __buffer[10];
    uint16_t i;
    uint8_t c;  

    while (nexSerial.available() > 0) {   
      HAL::delayMilliseconds(5);
      c = nexSerial.read();

      if (c == NEX_RET_EVENT_TOUCH_HEAD) {
        if (nexSerial.available() >= 6) {
          __buffer[0] = c;
          for (i = 1; i < 7; i++) __buffer[i] = nexSerial.read();
          __buffer[i] = 0x00;

          if (0xFF == __buffer[4] && 0xFF == __buffer[5] && 0xFF == __buffer[6])
            NexObject::iterate(nex_listen_list, __buffer[1], __buffer[2], (int32_t)__buffer[3]);
        }
      }
    }
  }

  uint16_t recvRetNumber() {
    uint8_t temp[8] = {0};

    nexSerial.setTimeout(NEX_TIMEOUT);
    if (sizeof(temp) != nexSerial.readBytes((char *)temp, sizeof(temp)))
      return NULL;

    if (temp[0] == NEX_RET_NUMBER_HEAD
        && temp[5] == 0xFF
        && temp[6] == 0xFF
        && temp[7] == 0xFF
    )

    return (uint16_t)(((uint32_t)temp[4] << 24) | ((uint32_t)temp[3] << 16) | (temp[2] << 8) | (temp[1]));
  }

  void recvRetString(char *buffer, uint16_t len) {
    uint16_t ret = 0;
    bool str_start_flag = false;
    uint8_t cnt_0xFF = 0;
    String temp = String("");
    uint8_t c = 0;
    millis_t start;

    if (!buffer || len == 0) return;

    start = millis();
    while (millis() - start <= NEX_TIMEOUT) {
      while (nexSerial.available()) {
        c = nexSerial.read();
        if (str_start_flag) {
          if (c == 0xFF) {
            cnt_0xFF++;                    
            if (cnt_0xFF >= 3) break;
          }
          else
            temp += (char)c;
        }
        else if (c == NEX_RET_STRING_HEAD)
          str_start_flag = true;
      }
    
      if (cnt_0xFF >= 3) break;
    }

    ret = temp.length();
    ret = ret > len ? len : ret;
    strncpy(buffer, temp.c_str(), ret);
  }

  void sendCommand(const char* cmd) {
    while (nexSerial.available()) nexSerial.read();
    nexSerial.print(cmd);
    nexSerial.write(0xFF);
    nexSerial.write(0xFF);
    nexSerial.write(0xFF);
  }

  void recvRetCommandFinished() {    
    while (nexSerial.available()) nexSerial.read();
  }

  uint8_t Nextion_PageID() {
    uint8_t temp[5] = {0};

    sendCommand("sendme");

    nexSerial.setTimeout(NEX_TIMEOUT);

    if (sizeof(temp) != nexSerial.readBytes((char *)temp, sizeof(temp)))
      return 2;

    if (temp[0] == NEX_RET_CURRENT_PAGE_ID_HEAD && temp[2] == 0xFF && temp[3] == 0xFF && temp[4] == 0xFF)
      return temp[1];
    else
      return 2;
  }

  void setCurrentBrightness(uint8_t dimValue) {
    char buf[10] = {0};
    String cmd;
    utoa(dimValue, buf, 10);
    cmd += "dim=";
    cmd += buf;
    sendCommand(cmd.c_str());
    HAL::delayMilliseconds(10);
    recvRetCommandFinished();
  }

  void sendRefreshAll(void) {
    sendCommand("ref 0");
  }

#endif // NEXTION
