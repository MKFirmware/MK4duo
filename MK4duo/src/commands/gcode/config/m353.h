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

#if HAS_LINEAR_EXTRUDER

#define CODE_M353

/**
 * M353: Set total number Extruder, Hotend, Bed, Chamber, Fan
 *
 *    D[int] Set number driver extruder
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
    if (!parser.seen("DEHBCF")) {
      printer.print_M353();
      return;
    }
  #endif

  if (parser.seen('D')) {
    const uint8_t d = parser.value_byte();
    if (d > MAX_DRIVER_E)
      SERIAL_LM(ECHO, "Too many drivers");
    else
      stepper.change_number_driver(d);
  }

  if (parser.seen('E')) {
    const uint8_t e = parser.value_byte();
    if (e > MAX_EXTRUDER)
      SERIAL_LM(ECHO, "Too many extruders");
    else
      toolManager.change_number_extruder(e);
  }

  if (parser.seen('H')) {
    const uint8_t h = parser.value_byte();
    if (h > toolManager.extruder.total || h > MAX_HOTEND)
      SERIAL_LM(ECHO, "Too many hotends");
    else
      tempManager.change_number_heater(IS_HOTEND, h);
  }

  if (parser.seen('B')) {
    const uint8_t h = parser.value_byte();
    if (h > MAX_BED)
      SERIAL_LM(ECHO, "Too many beds");
    else
      tempManager.change_number_heater(IS_BED, h);
  }

  if (parser.seen('C')) {
    const uint8_t h = parser.value_byte();
    if (h > MAX_CHAMBER)
      SERIAL_LM(ECHO, "Too many chambers");
    else
      tempManager.change_number_heater(IS_CHAMBER, h);
  }

  if (parser.seen('F')) {
    const uint8_t f = parser.value_byte();
    if (f > MAX_FAN)
      SERIAL_LM(ECHO, "Too many fans");
    else
      fanManager.change_number_fan(f);
  }

}

#endif // HAS_LINEAR_EXTRUDER
