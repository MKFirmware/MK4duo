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

#define CODE_G61

/**
 * G61:  Apply/restore saved coordinates.
 *        X Y Z E - Value to add at stored coordinates.
 *        F<speed> - Set Feedrate.
 *        S<slot> specifies memory slot # (0-based) to save into (default 0).
 */
inline void gcode_G61(void) {

  if (!printer.pos_saved) return;

  const uint8_t slot = parser.byteval('S');

  if (slot >= NUM_POSITON_SLOTS) {
    SERIAL_LMV(ER, MSG_INVALID_POS_SLOT, (int)NUM_POSITON_SLOTS);
    return;
  }

  SERIAL_MSG(MSG_RESTORING_POS);
  SERIAL_MV(" S", slot);
  SERIAL_MSG("->");

  if (parser.seen('F') && parser.value_linear_units() > 0.0)
    mechanics.feedrate_mm_s = MMM_TO_MMS(parser.value_linear_units());

  LOOP_XYZE(i) {
    if (parser.seen(axis_codes[i])) {
      mechanics.destination[i] = parser.value_axis_units((AxisEnum)i) + mechanics.stored_position[slot][i];
    }
    else {
      mechanics.destination[i] = mechanics.current_position[i];
    }
    SERIAL_MV(" ", axis_codes[i]);
    SERIAL_MV(":", mechanics.destination[i]);
  }
  SERIAL_EOL();

  // finish moves
  mechanics.prepare_move_to_destination();
  stepper.synchronize();
}
