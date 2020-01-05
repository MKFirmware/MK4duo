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
 * mcode
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#if ENABLED(PREVENT_COLD_EXTRUSION)

#define CODE_M302

/**
 * M302: Allow cold extrudes, or set the minimum extrude temperature
 *
 *       S<temperature> sets the minimum extrude temperature
 *       P<bool> enables (1) or disables (0) cold extrusion
 *
 *  Examples:
 *
 *       M302         ; report current cold extrusion state
 *       M302 P0      ; enable cold extrusion checking
 *       M302 P1      ; disables cold extrusion checking
 *       M302 S0      ; always allow extrusion (disables checking)
 *       M302 S170    ; only allow extrusion above 170
 *       M302 S170 P1 ; set min extrude temp to 170 but leave disabled
 */
inline void gcode_M302() {
  bool seen_S = parser.seen('S');
  if (seen_S) {
    tempManager.extrude_min_temp = parser.value_celsius();
    printer.setAllowColdExtrude(tempManager.extrude_min_temp == 0);
  }

  if (parser.seen('P'))
    printer.setAllowColdExtrude((tempManager.extrude_min_temp == 0) || parser.value_bool());
  else if (!seen_S) {
    // Report current state
    SERIAL_MSG("Cold extrudes are ");
    SERIAL_STR(printer.isAllowColdExtrude() ? PSTR("en") : PSTR("dis"));
    SERIAL_MV("abled (min temp ", tempManager.extrude_min_temp);
    SERIAL_EM("C)");
  }
}

#endif // ENABLED(PREVENT_COLD_EXTRUSION)
