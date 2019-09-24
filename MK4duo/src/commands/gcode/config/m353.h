/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2019 Alberto Cotronei @MagoKimbra
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
 * Copyright (c) 2019 Alberto Cotronei @MagoKimbra
 */

#if HAS_LINEAR_EXTRUDER

#define CODE_M353

/**
 * M353: Set total number Extruder, Hotend, Bed, Chamber, Fan
 *
 *    E[int] Set number extruder
 *    H[int] Set number hotend
 *    B[int] Set number bed
 *    C[int] Set number chamber
 *    F[int] Set number fan
 *
 */
inline void gcode_M353() {

  #if DISABLED(DISABLE_M503)
    // No arguments? Show M353 report.
    if (!parser.seen("EHBCF")) {
      printer.print_M353();
      return;
    }
  #endif

  if (parser.seen('E')) {
    uint8_t drv = parser.value_int();
    LIMIT(drv, 0, MAX_EXTRUDER);
    tools.change_number_extruder(drv);
  }

  if (parser.seen('H')) {
    uint8_t h = parser.value_int();
    LIMIT(h, 0, MAX_HOTEND);
    thermalManager.change_number_heater(IS_HOTEND, h);
  }

  if (parser.seen('B')) {
    uint8_t h = parser.value_int();
    LIMIT(h, 0, MAX_BED);
    thermalManager.change_number_heater(IS_BED, h);
  }

  if (parser.seen('C')) {
    uint8_t h = parser.value_int();
    LIMIT(h, 0, MAX_CHAMBER);
    thermalManager.change_number_heater(IS_CHAMBER, h);
  }

  if (parser.seen('F')) {
    uint8_t f = parser.value_int();
    LIMIT(f, 0, MAX_FAN);
    //thermalmanager.change_number_fan(f);
  }

}

#endif // HAS_LINEAR_EXTRUDER
