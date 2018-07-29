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

#ifndef _HAL_MEMORY_STORE_H_
#define _HAL_MEMORY_STORE_H_

namespace MemoryStore {

  bool access_start(const bool read);
  bool access_finish(const bool read);
  bool write_data(int &pos, const uint8_t *value, uint16_t size, uint16_t *crc);
  bool read_data(int &pos, uint8_t* value, uint16_t size, uint16_t *crc);

} // MemoryStore

#endif /* _HAL_MEMORY_STORE_H_ */
