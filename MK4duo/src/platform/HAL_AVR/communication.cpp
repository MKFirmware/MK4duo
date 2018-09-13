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

#if ENABLED(__AVR__)

// Things to write to serial from Program memory. Saves 400 to 2k of RAM.
void serialprintPGM(const char* str) {
  while (char ch = pgm_read_byte(str++)) MKSERIAL.write(ch);
}

void serial_print_pair(const char* msg, const char *v)   { serialprintPGM(msg); MKSERIAL.print(v); }
void serial_print_pair(const char* msg, char v)          { serialprintPGM(msg); MKSERIAL.print(v); }
void serial_print_pair(const char* msg, int v)           { serialprintPGM(msg); MKSERIAL.print(v); }
void serial_print_pair(const char* msg, long v)          { serialprintPGM(msg); MKSERIAL.print(v); }
void serial_print_pair(const char* msg, float v, int n)  { serialprintPGM(msg); MKSERIAL.print(v, n); }
void serial_print_pair(const char* msg, double v)        { serialprintPGM(msg); MKSERIAL.print(v); }
void serial_print_pair(const char* msg, uint8_t v)       { serial_print_pair(msg, (int)v); }
void serial_print_pair(const char* msg, uint16_t v)      { serial_print_pair(msg, (int)v); }
void serial_print_pair(const char* msg, uint32_t v)      { serial_print_pair(msg, (long)v); }
void serial_print_pair(const char* msg, bool v)          { serial_print_pair(msg, (int)v); }
void serial_print_pair(const char* msg, void *v)         { serial_print_pair(msg, (uint32_t)v); }

void serial_spaces(uint8_t count) { count *= (PROPORTIONAL_FONT_RATIO); while (count--) MKSERIAL.write(' '); }

#if ENABLED(DEBUG_FEATURE)

  void print_xyz(const char* prefix, const char* suffix, const float x, const float y, const float z) {
    SERIAL_PS(prefix);
    SERIAL_CHR('(');
    SERIAL_VAL(x);
    SERIAL_MV(", ", y);
    SERIAL_MV(", ", z);
    SERIAL_CHR(")");

    if (suffix) SERIAL_PS(suffix);
    else SERIAL_EOL();
  }

  void print_xyz(const char* prefix, const char* suffix, const float xyz[]) {
    print_xyz(prefix, suffix, xyz[X_AXIS], xyz[Y_AXIS], xyz[Z_AXIS]);
  }

  #if HAS_PLANAR
    void print_xyz(const char* prefix, const char* suffix, const vector_3 &xyz) {
      print_xyz(prefix, suffix, xyz.x, xyz.y, xyz.z);
    }
  #endif

#endif // ENABLED(DEBUG_FEATURE)

#endif // __AVR__