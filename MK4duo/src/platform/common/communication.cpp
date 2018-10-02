/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 Alberto Cotronei @MagoKimbra
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

#include "../../../MK4duo.h"

FSTRINGVALUE(START, "start");
FSTRINGVALUE(OK, "ok");
FSTRINGVALUE(OKSPACE, "ok ");
FSTRINGVALUE(ER, "Error:");
FSTRINGVALUE(WT, "wait");
FSTRINGVALUE(ECHO, "echo:");
FSTRINGVALUE(CFG, "Config:");
FSTRINGVALUE(CAP, "Cap:");
FSTRINGVALUE(INFO, "Info:");
FSTRINGVALUE(BUSY, "busy:");
FSTRINGVALUE(RESEND, "Resend:");
FSTRINGVALUE(WARNING, "Warning:");
FSTRINGVALUE(TNAN, "NAN");
FSTRINGVALUE(TINF, "INF");
FSTRINGVALUE(PAUSE, "// action:pause");
FSTRINGVALUE(RESUME, "// action:resume");
FSTRINGVALUE(DISCONNECT, "// action:disconnect");
FSTRINGVALUE(REQUESTPAUSE, "RequestPause:");

void serialprintPGM(PGM_P str) {
  while (char c = pgm_read_byte(str++)) MKSERIAL.write(c);
}

void print_spaces(uint8_t count) { count *= (PROPORTIONAL_FONT_RATIO); while (count--) MKSERIAL.write(' '); }

#if ENABLED(DEBUG_FEATURE)

  void print_xyz(PGM_P prefix, PGM_P suffix, const float x, const float y, const float z) {
    SERIAL_PS(prefix);
    SERIAL_CHR('(');
    SERIAL_VAL(x);
    SERIAL_MV(", ", y);
    SERIAL_MV(", ", z);
    SERIAL_CHR(")");

    if (suffix) SERIAL_PS(suffix);
    else SERIAL_EOL();
  }

  void print_xyz(PGM_P prefix, PGM_P suffix, const float xyz[]) {
    print_xyz(prefix, suffix, xyz[X_AXIS], xyz[Y_AXIS], xyz[Z_AXIS]);
  }

  #if HAS_PLANAR
    void print_xyz(PGM_P prefix, PGM_P suffix, const vector_3 &xyz) {
      print_xyz(prefix, suffix, xyz.x, xyz.y, xyz.z);
    }
  #endif

#endif // ENABLED(DEBUG_FEATURE)
