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

#if EXTRUDERS > 0 && HAS_FILAMENT_SENSOR

  #define CODE_M412

  /**
   * M412: Enable or disable Filament Runout detection
   *  S[bool]   Enable / Disable Sensor control
   *  H[bool]   Enable / Disable Host control
   *  R[bool]   Reset control
   *  D[float]  Distance mm
   *
   */
  inline void gcode_M412(void) {

    #if DISABLED(DISABLE_M503)
      // No arguments? Show M412 report.
      if (!parser.seen("DHRS")) {
        SERIAL_STR(ECHO);
        SERIAL_EONOFF("Filament runout", filamentrunout.isEnabled());
        return;
      }
    #endif

    const bool  seenR = parser.seen('R'),
                seenS = parser.seen('S');

    if (seenR || seenS) filamentrunout.reset();
    if (seenS) filamentrunout.setEnabled(parser.value_bool());
    if (parser.seen('H')) filamentrunout.setHostHandling(parser.value_bool());

    #if FILAMENT_RUNOUT_DISTANCE_MM > 0
      if (parser.seen('D')) filamentrunout.response.runout_distance_mm = parser.value_float();
    #endif

  }

#endif // EXTRUDERS > 0 && HAS_EXT_ENCODER
