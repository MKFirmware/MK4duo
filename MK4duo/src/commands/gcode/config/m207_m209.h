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
 * mcode
 *
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
 */

#if ENABLED(FWRETRACT)

  #define CODE_M207
  #define CODE_M208
  #define CODE_M209

  /**
   * M207: Set firmware retraction values
   *
   *   S[+units]    data.retract_length
   *   W[+units]    data.swap_retract_length (multi-extruder)
   *   F[units/min] data.retract_feedrate_mm_s
   *   Z[units]     data.retract_zlift
   */
  inline void gcode_M207(void) {
    if (parser.seenval('S')) fwretract.data.retract_length = parser.value_axis_units(E_AXIS);
    if (parser.seenval('F')) fwretract.data.retract_feedrate_mm_s = MMM_TO_MMS(parser.value_axis_units(E_AXIS));
    if (parser.seenval('Z')) fwretract.data.retract_zlift = parser.value_linear_units();
    if (parser.seenval('W')) fwretract.data.swap_retract_length = parser.value_axis_units(E_AXIS);
  }

  /**
   * M208: Set firmware un-retraction values
   *
   *   S[+units]    data.retract_recover_length (in addition to M207 S*)
   *   W[+units]    data.swap_retract_recover_length (multi-extruder)
   *   F[units/min] data.retract_recover_feedrate_mm_s
   *   R[units/min] data.swap_retract_recover_feedrate_mm_s
   */
  inline void gcode_M208(void) {
    if (parser.seen('S')) fwretract.data.retract_recover_length = parser.value_axis_units(E_AXIS);
    if (parser.seen('F')) fwretract.data.retract_recover_feedrate_mm_s = MMM_TO_MMS(parser.value_axis_units(E_AXIS));
    if (parser.seen('R')) fwretract.data.swap_retract_recover_feedrate_mm_s = MMM_TO_MMS(parser.value_axis_units(E_AXIS));
    if (parser.seen('W')) fwretract.data.swap_retract_recover_length = parser.value_axis_units(E_AXIS);
  }

  /**
   * M209: Enable automatic retract (M209 S1)
   *   For slicers that don't support G10/11, reversed extrude-only
   *   moves will be classified as retraction.
   */
  inline void gcode_M209(void) {
    if (MIN_AUTORETRACT <= MAX_AUTORETRACT) {
      if (parser.seen('S')) {
        fwretract.autoretract_enabled = parser.value_bool();
      }
    }
  }

#endif // FWRETRACT
