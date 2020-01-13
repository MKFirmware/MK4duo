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
 * gcode.h
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#define CODE_G60

/**
 * G60:  save current position
 *        S<slot> specifies memory slot # (0-based) to save into (default 0)
 */
inline void gcode_G60() {

  const uint8_t slot = parser.byteval('S');

  if (slot >= NUM_POSITON_SLOTS) {
    SERIAL_LMV(ER, MSG_HOST_INVALID_POS_SLOT, NUM_POSITON_SLOTS);
    return;
  }

  planner.synchronize();
  mechanics.stored_position[slot] = mechanics.position;
  printer.setPosSaved(true);

  SERIAL_MSG(MSG_HOST_SAVED_POS);
  SERIAL_MV(" S", slot);
  SERIAL_MV("<-X:", mechanics.stored_position[slot].x);
  SERIAL_MV(" Y:", mechanics.stored_position[slot].y);
  SERIAL_MV(" Z:", mechanics.stored_position[slot].z);
  SERIAL_EMV(" E:", mechanics.stored_position[slot].e);

}
