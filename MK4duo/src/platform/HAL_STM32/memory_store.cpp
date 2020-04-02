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

#ifdef ARDUINO_ARCH_STM32

#include "../../../MK4duo.h"

#if HAS_EEPROM_FLASH
  #include <EEPROM.h>
#endif

#if HAS_EEPROM

MemoryStore memorystore;

/** Public Parameters */
#if HAS_EEPROM_SD
  char MemoryStore::eeprom_data[EEPROM_SIZE];
#endif

/** Public Function */
bool MemoryStore::access_start() {
  #if HAS_EEPROM_FLASH
    eeprom_buffer_fill();
  #endif
  return false;
}

bool MemoryStore::access_write() {
  #if HAS_EEPROM_FLASH
    eeprom_buffer_flush();
  #elif HAS_EEPROM_SD
    card.write_eeprom();
  #endif
  return false;
}

bool MemoryStore::write_data(int &pos, const uint8_t *value, size_t size, uint16_t *crc) {

  while (size--) {
    uint8_t v = *value;
    #if HAS_EEPROM_FLASH
      eeprom_buffered_write_byte(pos, v);
    #elif HAS_EEPROM_SD
      eeprom_data[pos] = v;
    #else
      uint8_t * const p = (uint8_t * const)pos;
      // EEPROM has only ~100,000 write cycles,
      // so only write bytes that have changed!
      if (v != eeprom_read_byte(p)) {
        eeprom_write_byte(p, v);
        delay(2);
        if (eeprom_read_byte(p) != v) {
          SERIAL_LM(ECHO, STR_ERR_EEPROM_WRITE);
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

bool MemoryStore::read_data(int &pos, uint8_t *value, size_t size, uint16_t *crc, const bool writing/*=true*/) {

  while (size--) {
    #if HAS_EEPROM_FLASH
      const uint8_t c = eeprom_buffered_read_byte(pos);
    #elif HAS_EEPROM_SD
      const uint8_t c = eeprom_data[pos];
    #else
      const uint8_t c = eeprom_read_byte((uint8_t*)pos);
    #endif
    if (writing) *value = c;
    crc16(crc, &c, 1);
    pos++;
    value++;
  };

  return false;
}

size_t MemoryStore::capacity() { return E2END + 1; }

#endif // HAS_EEPROM

#endif // ARDUINO_ARCH_STM32
