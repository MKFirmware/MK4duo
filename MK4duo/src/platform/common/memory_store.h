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

#ifndef _MEMORY_STORE_H_
#define _MEMORY_STORE_H_

#if ENABLED(E2END)
  #define EEPROM_SIZE E2END
#else
  #define EEPROM_SIZE 4096
#endif

class MemoryStore {

  public: /** Constructor */

    MemoryStore() {}

  private: /** Private Parameters */

    #if HAS_EEPROM_SD
      static char eeprom_data[EEPROM_SIZE];
    #endif

  public: /** Public Function */

    static bool access_start(const bool read);
    static bool access_finish(const bool read);
    static bool write_data(int &pos, const uint8_t *value, size_t size, uint16_t *crc);
    static bool read_data(int &pos, uint8_t* value, size_t size, uint16_t *crc);

    static size_t capacity();

    static inline bool write_data(const int pos, const uint8_t* value, const size_t size=sizeof(uint8_t)) {
      int data_pos = pos;
      uint16_t crc = 0;
      return write_data(data_pos, value, size, &crc);
    }

    static inline bool write_data(const int pos, const uint8_t value) { return write_data(pos, &value); }

    static inline bool read_data(const int pos, uint8_t* value, const size_t size=1) {
      int data_pos = pos;
      uint16_t crc = 0;
      return read_data(data_pos, value, size, &crc);
    }

};

extern MemoryStore memorystore;

#endif /* _MEMORY_STORE_H_ */
