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

MemoryStore memorystore;

extern void eeprom_flush(void);

#if HAS_EEPROM_SD
  char MemoryStore::eeprom_data[EEPROM_SIZE];
#endif

bool MemoryStore::access_start(const bool read) {
  #if HAS_EEPROM_SD
    ZERO(eeprom_data);
    if (read) {
      int16_t bytes_read = 0;
      card.open_eeprom_sd(true);
      bytes_read = card.read_eeprom_data(eeprom_data, EEPROM_SIZE);
      SERIAL_LMV(ECHO, "SD EEPROM bytes read: ", (int)bytes_read);
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

bool MemoryStore::access_finish(const bool read) {
  #if HAS_EEPROM_FLASH
    UNUSED(read);
    eeprom_flush();
  #elif HAS_EEPROM_SD
    if (!read) {
      card.open_eeprom_sd(false);
      int16_t bytes_written = card.write_eeprom_data(eeprom_data, EEPROM_SIZE);
      SERIAL_LMV(ECHO, "SD EEPROM bytes written: ", (int)bytes_written);
      card.close_eeprom_sd();
      return (bytes_written != EEPROM_SIZE);
    }
  #else
    UNUSED(read);
  #endif

  return false;
}

bool MemoryStore::write_data(int &pos, const uint8_t *value, size_t size, uint16_t *crc) {

  while(size--) {
    uint8_t v = *value;
    #if HAS_EEPROM_SD
      eeprom_data[pos] = v;
    #else
      uint8_t * const p = (uint8_t * const)pos;
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

bool MemoryStore::read_data(int &pos, uint8_t *value, size_t size, uint16_t *crc) {

  while(size--) {
    #if HAS_EEPROM_SD
      uint8_t c = eeprom_data[pos];
    #else
      uint8_t c = eeprom_read_byte((unsigned char*)pos);
    #endif
    *value = c;
    crc16(crc, &c, 1);
    pos++;
    value++;
  };

  return false;
}

size_t MemoryStore::capacity() { return EEPROM_SIZE + 1; }

#endif // ARDUINO_ARCH_SAM && EEPROM_SETTINGS
