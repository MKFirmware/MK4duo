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

#include "../../../MK4duo.h"

#if ENABLED(ARDUINO_ARCH_SAM)

#if ENABLED(EEPROM_SETTINGS)

  extern void eeprom_flush(void);

  bool EEPROM::access_start(const uint8_t oflag) {

    #if HAS_EEPROM_SD
      return card.open_eeprom_sd(oflag);
    #else
      UNUSED(oflag);
    #endif

    return false;
  }

  bool EEPROM::access_finish(){
    #if HAS_EEPROM_FLASH
      eeprom_flush();
    #elif HAS_EEPROM_SD
      card.close_eeprom_sd();
    #endif
    return false;
  }

  bool EEPROM::write_data(int &pos, const uint8_t *value, uint16_t size, uint16_t *crc) {

    while(size--) {

      #if HAS_EEPROM_SD

        uint8_t v = *value;
        if (!card.write_eeprom_data(v)) {
          SERIAL_LM(ECHO, MSG_ERR_EEPROM_WRITE);
          return true;
        }

      #else

        uint8_t * const p = (uint8_t * const)pos;
        uint8_t v = *value;
        // EEPROM has only ~100,000 write cycles,
        // so only write bytes that have changed!
        if (v != eeprom_read_byte(p)) {
          eeprom_write_byte(p, v);
          if (eeprom_read_byte(p) != v) {
            SERIAL_LM(ECHO, MSG_ERR_EEPROM_WRITE);
            return true;
          }
        }
      #endif

      crc16(crc, &v, 1);
      pos++;
      value++;
    };
    return false;
  }

  bool EEPROM::read_data(int &pos, uint8_t *value, uint16_t size, uint16_t *crc) {
    do {
      #if HAS_EEPROM_SD
        uint8_t c = card.read_eeprom_data();
      #else
        uint8_t c = eeprom_read_byte((unsigned char*)pos);
      #endif
      *value = c;
      crc16(crc, &c, 1);
      pos++;
      value++;
    } while (--size);
    return false;
  }

  void EEPROM::crc16(uint16_t *crc, const void * const data, uint16_t cnt) {
    uint8_t *ptr = (uint8_t *)data;
    while (cnt--) {
      *crc = (uint16_t)(*crc ^ (uint16_t)(((uint16_t)*ptr++) << 8));
      for (uint8_t x = 0; x < 8; x++)
        *crc = (uint16_t)((*crc & 0x8000) ? ((uint16_t)(*crc << 1) ^ 0x1021) : (*crc << 1));
    }
  }

#endif // EEPROM_SETTINGS
#endif // ARDUINO_ARCH_SAM
