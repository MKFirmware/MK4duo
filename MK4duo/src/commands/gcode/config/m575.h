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

#define CODE_M575

/**
 * M575 - Change serial baud rate
 *
 *   P<index>    - Serial port index. Omit for all.
 *   B<baudrate> - Baud rate (bits per second)
 */
inline void gcode_M575() {

  const int8_t port   = parser.intval('P', -99);
  const int32_t baud  = parser.ulongval('B');

  switch (baud) {
    case 2400: case 9600: case 19200: case 38400: case 57600:
    case 115200: case 250000: case 500000: case 1000000: {
      const bool set1 = (port == -99 || port == 0);
      if (set1) {
        SERIAL_SM(ECHO, " Serial 1");
        SERIAL_MV(" baud rate set to ", int(baud));
      }
      #if NUM_SERIAL > 1
        const bool set2 = (port == -99 || port == 1);
        if (set2) {
          SERIAL_SM(ECHO, " Serial 2");
          SERIAL_MV(" baud rate set to ", int(baud));
        }
      #endif

      SERIAL_FLUSH();

      if (set1) { MKSERIAL1.end(); MKSERIAL1.begin(baud); }

      #if NUM_SERIAL > 1
        if (set2) { MKSERIAL2.end(); MKSERIAL2.begin(baud); }
      #endif

    } break;
    default: SERIAL_LM(ECHO, "?(B)aud rate is implausible.");
  }
}
