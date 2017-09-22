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

/**
 * mcode
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#if ENABLED(DUAL_X_CARRIAGE)

  #define CODE_M605

  /**
   * M605: Set dual x-carriage movement mode
   *
   *    M605 S0: Full control mode. The slicer has full control over x-carriage movement
   *    M605 S1: Auto-park mode. The inactive head will auto park/unpark without slicer involvement
   *    M605 S2 [Xnnn] [Rmmm]: Duplication mode. The second extruder will duplicate the first with nnn
   *                         units x-offset and an optional differential hotend temperature of
   *                         mmm degrees. E.g., with "M605 S2 X100 R2" the second extruder will duplicate
   *                         the first with a spacing of 100mm in the x direction and 2 degrees hotter.
   *
   *    Note: the X axis should be homed after changing dual x-carriage mode.
   */
  inline void gcode_M605(void) {
    stepper.synchronize();
    if (parser.seen('S')) mechanics.dual_x_carriage_mode = (DualXMode)parser.value_byte();
    switch(mechanics.dual_x_carriage_mode) {
      case DXC_FULL_CONTROL_MODE:
      case DXC_AUTO_PARK_MODE:
        break;
      case DXC_DUPLICATION_MODE:
        if (parser.seen('X')) mechanics.duplicate_hotend_x_offset = max(parser.value_linear_units(), X2_MIN_POS - mechanics.x_home_pos(0));
        if (parser.seen('R')) mechanics.duplicate_hotend_temp_offset = parser.value_celsius_diff();
        SERIAL_SM(ECHO, MSG_HOTEND_OFFSET);
        SERIAL_CHR(' ');
        SERIAL_VAL(tools.hotend_offset[X_AXIS][0]);
        SERIAL_CHR(',');
        SERIAL_VAL(tools.hotend_offset[Y_AXIS][0]);
        SERIAL_CHR(' ');
        SERIAL_VAL(mechanics.duplicate_hotend_x_offset);
        SERIAL_CHR(',');
        SERIAL_EV(tools.hotend_offset[Y_AXIS][1]);
        break;
      default:
        mechanics.dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;
        break;
    }
    mechanics.active_hotend_parked = false;
    mechanics.hotend_duplication_enabled = false;
    mechanics.delayed_move_time = 0;
  }

#endif // ENABLED(DUAL_X_CARRIAGE)
