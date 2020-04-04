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

#if HAS_MULTI_MODE

  #define CODE_M3
  #define CODE_M4

  /**
   * M3: Setting laser beam or fire laser - CNC clockwise speed
   * M4: Turn on laser beam or CNC counter clockwise speed
   *      S - Laser intensity or CNC speed
   *      L - Laser Duration
   *      P - PPM
   *      D - Diagnostic
   *      B - Set mode
   */
  void gcode_M3_M4(bool clockwise) {
    planner.synchronize();

    switch (printer.mode) {

      #if HAS_LASER_SPINDLE
        case PRINTER_MODE_LASER:
          if (printer.isRunning()) laser.set_power();
          break;
      #endif

      #if ENABLED(CNCROUTER)
        case PRINTER_MODE_CNC:
          if (parser.seenval('S')) cnc.setRouterSpeed(parser.value_ulong(), clockwise);
          break;
      #endif

      default: break; // other tools

    } // printer.mode

  }

  inline void gcode_M3() { gcode_M3_M4(true); }
  inline void gcode_M4() { gcode_M3_M4(false); }

#endif // HAS_MULTI_MODE
