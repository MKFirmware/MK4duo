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

#if ENABLED(ARDUINO_ARCH_SAM) && ENABLED(EEPROM_SETTINGS)

  #if ENABLED(E2END)
    #define EEPROM_SIZE E2END
  #else
    #define EEPROM_SIZE 4096
  #endif

  extern void eeprom_flush(void);

  namespace MemoryStore {

    #if HAS_EEPROM_SD
      char eeprom_data[EEPROM_SIZE];
    #endif

    bool access_start(const bool read) {
      #if HAS_EEPROM_SD
        if (read) {
          int16_t bytes_read = 0;
          card.open_eeprom_sd(true);
          bytes_read = card.read_eeprom_data(eeprom_data, EEPROM_SIZE);
          if (bytes_read < 0) {
            card.close_eeprom_sd();
            return true;
          }
          card.close_eeprom_sd();
        }
      #else
        UNUSED(read);
      #endif

      return false;
    }

    bool access_finish(const bool read) {
      #if HAS_EEPROM_FLASH
        UNUSED(read);
        eeprom_flush();
        return false;
      #elif HAS_EEPROM_SD
        if (!read) {
          card.open_eeprom_sd(false);
          int16_t bytes_written = card.write_eeprom_data(eeprom_data, EEPROM_SIZE);
          card.close_eeprom_sd();
          return (bytes_written != EEPROM_SIZE);
        }
      #else
        UNUSED(read);
        return false;
      #endif
    }

    bool write_data(int &pos, const uint8_t *value, uint16_t size, uint16_t *crc) {

      #if HAS_EEPROM_SD
        for (int i = 0; i < size; i++)
          eeprom_data[pos + i] = value[i];
        crc16(crc, value, size);
        pos += size;
      #else
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
      #endif

      return false;
    }

    bool read_data(int &pos, uint8_t *value, uint16_t size, uint16_t *crc) {

      #if HAS_EEPROM_SD
        for (int i = 0; i < size; i++) {
          uint8_t c = eeprom_data[pos + i];
          value[i] = c;
          crc16(crc, &c, 1);
        }
        pos += size;
      #else
        do {
          uint8_t c = eeprom_read_byte((unsigned char*)pos);
          *value = c;
          crc16(crc, &c, 1);
          pos++;
          value++;
        } while (--size);
      #endif

      return false;
    }
  }

#endif // ARDUINO_ARCH_SAM && EEPROM_SETTINGS
