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
#pragma once

//
// Utility functions for timer expired and pending
//
bool expired(millis_l *start, const millis_l period);
bool expired(millis_s *start, const millis_s period);
FORCE_INLINE bool pending(millis_l *start, const millis_l period) { return !expired(start, period); }
FORCE_INLINE bool pending(millis_s *start, const millis_s period) { return !expired(start, period); }

//
// Utility functions to create and print hex strings as nybble, byte, and word.
//
FORCE_INLINE char hex_nybble(const uint8_t n) {
  return (n & 0xF) + ((n & 0xF) < 10 ? '0' : 'A' - 10);
}

char* hex_byte(const uint8_t b);
char* hex_word(const uint16_t w);
char* hex_address(const void * const w);

void print_hex_nybble(const uint8_t n);
void print_hex_byte(const uint8_t b);
void print_hex_word(const uint16_t w);
void print_hex_address(const void * const w);
void print_hex_long(const uint32_t w, const char delimiter);

#if ENABLED(AUTO_BED_LEVELING_UBL) || ENABLED(G26_MESH_VALIDATION)

  /**
   * These support functions allow the use of large bit arrays of flags that take very
   * little RAM. Currently they are limited to being 16x16 in size. Changing the declaration
   * to unsigned long will allow us to go to 32x32 if higher resolution Mesh's are needed
   * in the future.
   */
  FORCE_INLINE void bitmap_clear(uint16_t bits[16], const uint8_t x, const uint8_t y)  { CBI(bits[y], x); }
  FORCE_INLINE void bitmap_set(uint16_t bits[16], const uint8_t x, const uint8_t y)    { SBI(bits[y], x); }
  FORCE_INLINE bool is_bitmap_set(uint16_t bits[16], const uint8_t x, const uint8_t y) { return TEST(bits[y], x); }

#endif

//
// Utility functions to convert number into string
//

// Crc 16 bit for eeprom check
void crc16(uint16_t *crc, const void * const data, uint16_t cnt);

// Convert uint8_t to string percentage
char* ui8tostr4pct(const uint8_t i);

// Convert uint8_t to string with 1 format
char* ui8tostr1(const uint8_t i);

// Convert uint8_t to string with 123 format
char* ui8tostr3(const uint8_t i);

// Convert int8_t to rj string with 123 or -12 format
char* i8tostr3(const int8_t i);

// Convert uint16_t to string with 123 format
char* ui16tostr3(const uint16_t i);

// Convert uint16_t to string with 1234 format
char* ui16tostr4(const uint16_t i);

// Convert uint32_t to string with 1234 format
char* ui32tostr4(const uint32_t i);

// Convert int16_t to string with 123 format
char* i16tostr3(const int16_t i);

// Convert int16_t to lj string with 123 format
char* i16tostr3left(const int16_t i);

// Convert int16_t to rj string with _123, -123, _-12, or __-1 format
char* i16tostr4sign(const int16_t i);

// Convert float to string with 1.23 format
char* ftostr12ns(const float &f);

// Convert float to fixed-length string with 023.45 / -23.45 format
char* ftostr52(const float &f);

// Convert float to fixed-length string with +123.4 / -123.4 format
char* ftostr41sign(const float &f);

// Convert signed float to string (6 digit) with -1.234 / _0.000 / +1.234 format
char* ftostr43sign(const float &f, char plus=' ');

// Convert signed float to string (5 digit) with -1.2345 / _0.0000 / +1.2345 format
char* ftostr54sign(const float &x, char plus=' ');

// Convert float to rj string with 12345 format
char* ftostr5rj(const float &f);

// Convert signed float to string with +1234.5 format
char* ftostr51sign(const float &f);

// Convert float to space-padded string with -_23.4_ format
char* ftostr52sp(const float &f);

// Convert signed float to string with +123.45 format
char* ftostr52sign(const float &f);

// Convert unsigned float to string with 1234.5 format omitting trailing zeros
char* ftostr51rj(const float &x);

// Convert float to rj string with 123 or -12 format
FORCE_INLINE char* ftostr3(const float &f) { return i16tostr3(int16_t(f + (f < 0 ? -0.5f : 0.5f))); }

#if ENABLED(LCD_DECIMAL_SMALL_XY)
  // Convert float to rj string with 1234, _123, 12.3, _1.2, -123, _-12, or -1.2 format
  char* ftostr4sign(const float &f);
#else
  // Convert float to rj string with 1234, _123, -123, __12, _-12, ___1, or __-1 format
  FORCE_INLINE char* ftostr4sign(const float &f) { return i16tostr4sign(int16_t(f + (f < 0 ? -0.5f : 0.5f))); }
#endif

// Convert float length to string
void ftostrlength(char *buffer, const float f);

// Convert uint8_t to uint8_t percentage
FORCE_INLINE uint8_t ui8topercent(const uint8_t i) { return (int(i) * 100 + 127) / 255; }
