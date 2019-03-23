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

#if HAS_MULTI_MODE

  #define CODE_M5

  /**
   * M5: Turn off laser beam - CNC off
   */
  inline void gcode_M5(void) {
    planner.synchronize();

    switch (printer.mode) {

      #if ENABLED(LASER)
        case PRINTER_MODE_LASER: {
          if (laser.status != LASER_OFF) {
            laser.status = LASER_OFF;
            laser.mode = CONTINUOUS;
            laser.duration = 0;

            if (laser.diagnostics)
              SERIAL_EM("Laser M5 called and laser OFF");
          }
        }
        break;
      #endif

      #if ENABLED(CNCROUTER)
        case PRINTER_MODE_CNC:
          cnc.disable_router();
        break;
      #endif

      default: break; // other tools

    } // printer.mode

  }

#endif // HAS_MULTI_MODE
