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

#if ENABLED(__AVR__) && ENABLED(EEPROM_SETTINGS)

  namespace MemoryStore {

    bool access_start(const bool read)  { UNUSED(read); return false; }
    bool access_finish(const bool read) { UNUSED(read); return false; }

    bool write_data(int &pos, const uint8_t *value, uint16_t size, uint16_t *crc) {

      while(size--) {

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

        crc16(crc, &v, 1);
        pos++;
        value++;
      };
      return false;
    }

    bool read_data(int &pos, uint8_t *value, uint16_t size, uint16_t *crc) {
      do {
        uint8_t c = eeprom_read_byte((unsigned char*)pos);
        *value = c;
        crc16(crc, &c, 1);
        pos++;
        value++;
      } while (--size);
      return false;
    }

  }

#endif // ENABLED(__AVR__) && EEPROM_SETTINGS
