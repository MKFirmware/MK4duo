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

/** Public Function */
size_t  MemoryStore::capacity()     { return E2END + 1; }
bool    MemoryStore::access_start() { eeprom_buffer_fill();   return false; }
bool    MemoryStore::access_write() { eeprom_buffer_flush();  return false; }

bool MemoryStore::write_data(int &pos, const uint8_t *value, size_t size, uint16_t *crc) {

  while (size--) {
    uint8_t v = *value;
    eeprom_buffered_write_byte(pos, v);
    crc16(crc, &v, 1);
    pos++;
    value++;
  };

  return false;
}

bool MemoryStore::read_data(int &pos, uint8_t *value, size_t size, uint16_t *crc, const bool writing/*=true*/) {

  while (size--) {
    const uint8_t c = eeprom_buffered_read_byte(pos);
    if (writing) *value = c;
    crc16(crc, &c, 1);
    pos++;
    value++;
  };

  return false;
}

#endif // HAS_EEPROM_FLASH
#endif // ARDUINO_ARCH_STM32
