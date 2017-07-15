/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2017 Alberto Cotronei @MagoKimbra
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
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#define CODE_G60

/**
 * G60:  save current position
 *        S<slot> specifies memory slot # (0-based) to save into (default 0)
 */
inline void gcode_G60(void) {

  const uint8_t slot = parser.byteval('S');

  if (slot >= NUM_POSITON_SLOTS) {
    SERIAL_LMV(ER, MSG_INVALID_POS_SLOT, (int)NUM_POSITON_SLOTS);
    return;
  } 
  COPY_ARRAY(mechanics.stored_position[slot], mechanics.current_position);
  printer.pos_saved = true;

  SERIAL_MSG(MSG_SAVED_POS);
  SERIAL_MV(" S", slot);
  SERIAL_MV("<-X:", mechanics.stored_position[slot][X_AXIS]);
  SERIAL_MV(" Y:", mechanics.stored_position[slot][Y_AXIS]);
  SERIAL_MV(" Z:", mechanics.stored_position[slot][Z_AXIS]);
  SERIAL_EMV(" E:", mechanics.stored_position[slot][E_AXIS]);
}
