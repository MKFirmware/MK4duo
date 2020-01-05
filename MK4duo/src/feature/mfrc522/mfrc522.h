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
 * MFRC522 Serial
 * Designed for module MFRC522 with UART
 *
 * Copyright (c) 2020 Alberto Cotronei MagoKimbra
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
#pragma once

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

    uint32_t Spool_ID[EXTRUDERS] =  ARRAY_BY_EXTRUDERS(0);

    bool  Spool_must_read[EXTRUDERS]  = ARRAY_BY_EXTRUDERS(false),
          Spool_must_write[EXTRUDERS] = ARRAY_BY_EXTRUDERS(false);

    struct rfid_data_t {
      char  brand[16],          // 16 byte Block 5
            serialnumber[16],   // 16 byte Block 9
            type[8],            //  8 byte Block 13
            color[8],           //  8 byte Block 13
            free[12];           // 12 byte Block 17
      float size;               //  4 byte Block 17
      int   temphotendmin,      //  4 byte Block 21
            temphotendmax,      //  4 byte Block 21
            tempbedmin,         //  4 byte Block 21
            tempbedmax,         //  4 byte Block 21
            temphotend,         //  4 byte Block 25
            tempbed,            //  4 byte Block 25
            density;            //  4 byte Block 25
      uint32_t lenght;          //  4 byte Block 25
    };

    union rfid_packet_t {
      rfid_data_t data;
      byte RfidPacket[sizeof(rfid_data_t)];
    };

    union rfid_packetID_t {
      unsigned long Spool_ID;
      byte packet[4];
    };

    rfid_packet_t   data[EXTRUDERS];
    rfid_packetID_t data_id[EXTRUDERS];

  private: /** Private Parameters */

  public: /** Public Function */

    bool init();
    void print_info(const uint8_t e);
    void spin();

  private: /** Private Function */

    bool getID(const uint8_t e);
    bool readBlock(const uint8_t e);
    bool writeBlock(const uint8_t e);

    // Function for Serial rfid module
    bool communicate(const uint8_t sendDataLength, const uint8_t command, const uint8_t* sendData=NULL);
    void write(const uint8_t value);
    byte read();
    void clear();
    bool available();

};

extern MFRC522 rfid522;

#endif // ENABLED(RFID_MODULE)
