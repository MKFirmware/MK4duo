/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2017 Alberto Cotronei @MagoKimbra
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
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#if HAS_MULTI_MODE

  #define M3
  #define M4
  #define M5
  #define M6

  /**
   * M3: Setting laser beam or fire laser - CNC clockwise speed
   * M4: Turn on laser beam or CNC counter clockwise speed
   *      S - Laser intensity or CNC speed
   *      L - Laser Duration
   *      P - PPM
   *      D - Diagnostic
   *      B - Set mode
   */
  inline void gcode_M3_M4(bool clockwise) {
    stepper.synchronize();

    switch (printer.mode) {

      #if ENABLED(LASER) && ENABLED(LASER_FIRE_SPINDLE)
        case PRINTER_MODE_LASER: {
          if (IsRunning()) {
            #if ENABLED(INTENSITY_IN_BYTE)
              if (parser.seenval('S')) laser.intensity = (float)(parser.value_byte() / 255) * 100.0;
            #else
              if (parser.seenval('S')) laser.intensity = parser.value_float();
            #endif
            if (parser.seenval('L')) laser.duration = parser.value_ulong();
            if (parser.seenval('P')) laser.ppm = parser.value_float();
            if (parser.seenval('D')) laser.diagnostics = parser.value_bool();
            if (parser.seenval('B')) laser.set_mode(parser.value_int());
          }
          laser.status = LASER_ON;
        }
        break;
      #endif

      #if ENABLED(CNCROUTER)
        case PRINTER_MODE_CNC:
          if (parser.seenval('S')) setCNCRouterSpeed(parser.value_ulong(), clockwise);
        break;
      #endif

      default: break; // other tools

    } // printer.mode

    mechanics.prepare_move_to_destination();
  }

  /**
   * M5: Turn off laser beam - CNC off
   */
  inline void gcode_M5() {
    stepper.synchronize();

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
          disable_cncrouter();
        break;
      #endif

      default: break; // other tools

    } // printer.mode

    mechanics.prepare_move_to_destination();

  }

  #if ENABLED(CNCROUTER)
    /*
     * M6: CNC tool change
     */
    inline void gcode_M6() { tool_change_cnc(CNC_M6_TOOL_ID); }
  #endif

#endif // HAS_MULTI_MODE
