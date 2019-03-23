/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
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
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
 */

#if MECH(DELTA)

  #define CODE_G34

  /**
   * G34: Set Delta Height calculated from toolhead position (only DELTA)
   */
  inline void gcode_G34(void) {

    if (mechanics.axis_unhomed_error()) return;

    mechanics.data.height -= mechanics.current_position[Z_AXIS];
    mechanics.recalc_delta_settings();
    SERIAL_EMV("  New delta height:", mechanics.data.height, 3);
    sound.feedback();

  }

#endif
