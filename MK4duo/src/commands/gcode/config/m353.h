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
    const uint8_t drv = parser.value_byte();
    if (drv > MAX_EXTRUDER)
      SERIAL_LM(ECHO, "Too many extruders");
    else
      tools.change_number_extruder(drv);
  }

  if (parser.seen('H')) {
    const uint8_t h = parser.value_byte();
    if (h > tools.data.extruder.total || h > MAX_HOTEND)
      SERIAL_LM(ECHO, "Too many hotends");
    else
      thermalManager.change_number_heater(IS_HOTEND, h);
  }

  if (parser.seen('B')) {
    const uint8_t h = parser.value_byte();
    if (h > MAX_BED)
      SERIAL_LM(ECHO, "Too many beds");
    else
      thermalManager.change_number_heater(IS_BED, h);
  }

  if (parser.seen('C')) {
    const uint8_t h = parser.value_byte();
    if (h > MAX_CHAMBER)
      SERIAL_LM(ECHO, "Too many chambers");
    else
      thermalManager.change_number_heater(IS_CHAMBER, h);
  }

  if (parser.seen('F')) {
    const uint8_t f = parser.value_byte();
    if (f > MAX_FAN)
      SERIAL_LM(ECHO, "Too many fans");
    //else
      //thermalmanager.change_number_fan(f);
  }

}

#endif // HAS_LINEAR_EXTRUDER
