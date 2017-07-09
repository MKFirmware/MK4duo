/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2017 Alberto Cotronei @MagoKimbra
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
 * MFRC522 Serial
 * Designed for module MFRC522 width UART
 *
 * Copyright (C) 2016 Alberto Cotronei MagoKimbra
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
 */

#ifndef _MFRC522_H_
#define _MFRC522_H_

#if ENABLED(RFID_MODULE)

  //------------------MFRC522 register ---------------
  #define         MFRC522_HEADER      0xAB
  #define         COMMAND_READ_ID     0x02
  #define         COMMAND_READ_BLOCK  0x03
  #define         COMMAND_WRITE_BLOCK 0x04

  #define         STATUS_ERROR        0
  #define         STATUS_OK           1

  #define         MIFARE_KEYA         0x00
  #define         MIFARE_KEYB         0x01

  #define         BLOCK_START         5
  #define         BLOCK_TOTAL         26 // block total = block used + 1

  class MFRC522 {

    public: /** Constructor */

      MFRC522();

    public: /** Public Parameters */

      typedef struct {
        char brand[16];           // 16 byte Block 5
        char serialnumber[16];    // 16 byte Block 9
        char type[8];             //  8 byte Block 13
        char color[8];            //  8 byte Block 13
        char free1[12];           // 12 byte Block 17
        float size;               //  4 byte Block 17
        int temphotendmin;        //  4 byte Block 21
        int temphotendmax;        //  4 byte Block 21
        int tempbedmin;           //  4 byte Block 21
        int tempbedmax;           //  4 byte Block 21
        int temphotend;           //  4 byte Block 25
        int tempbed;              //  4 byte Block 25
        int density;              //  4 byte Block 25
        unsigned long lenght;     //  4 byte Block 25
      } data_t;

      typedef union {
        data_t data;
        byte RfidPacket[sizeof(data_t)];
      } RfidPacket_t;

      typedef union {
        unsigned long Spool_ID;
        byte RfidPacketID[4];
      } RfidPacketID_t;

      RfidPacket_t RfidData[EXTRUDERS];
      RfidPacketID_t RfidDataID[EXTRUDERS];

    public: /** Public Function */

      bool init();
      bool getID(uint8_t e);
      bool readBlock(uint8_t e);
      bool writeBlock(uint8_t e);
      void printInfo(uint8_t e);

    private: /** Private Parameters */

    private: /** Private Function */

      void write(uint8_t value);
      byte read();
      bool communicate(uint8_t sendDataLength, uint8_t command, uint8_t *sendData = NULL);
      void clear();
      bool available();

  };

  extern MFRC522 RFID522;

#endif // ENABLED(RFID_MODULE)

#endif /* _MFRC522_H_ */
