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

#if ENABLED(ARDUINO_ARCH_SAM)

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

FSTRINGVALUE(Com::tStart,"start")
FSTRINGVALUE(Com::tOk,"ok")
FSTRINGVALUE(Com::tOkSpace,"ok ")
FSTRINGVALUE(Com::tError,"Error:")
FSTRINGVALUE(Com::tWait,"wait")
FSTRINGVALUE(Com::tEcho,"Echo:")
FSTRINGVALUE(Com::tConfig,"Config:")
FSTRINGVALUE(Com::tCap,"Cap:")
FSTRINGVALUE(Com::tInfo,"Info:")
FSTRINGVALUE(Com::tBusy,"busy:")
FSTRINGVALUE(Com::tResend,"Resend:")
FSTRINGVALUE(Com::tWarning,"Warning:")
FSTRINGVALUE(Com::tNAN,"NAN")
FSTRINGVALUE(Com::tINF,"INF")
FSTRINGVALUE(Com::tPauseCommunication,"// action:pause")
FSTRINGVALUE(Com::tContinueCommunication,"// action:resume")
FSTRINGVALUE(Com::tDisconnectCommunication,"// action:disconnect")
FSTRINGVALUE(Com::tRequestPauseCommunication,"RequestPause:")

void Com::serialprintPGM(FSTRINGPARAM(str)) {
  while (char c = HAL::readFlashByte(str++)) HAL::serialWriteByte(c);
}

void Com::print(char c, int base) {
  print((long)c, base);
}

void Com::print(unsigned char b, int base) {
  print((unsigned long)b, base);
}

void Com::print(int n, int base) {
  print((long)n, base);
}

void Com::print(unsigned int n, int base) {
  print((unsigned long)n, base);
}

void Com::print(long n, int base) {
  if (base == 0) write(n);
  else if (base == 10) {
    if (n < 0) { print('-'); n = -n; }
    printNumber(n, 10);
  }
  else
    printNumber(n, base);
}

void Com::print(unsigned long n, int base) {
  if (base == 0) write(n);
  else printNumber(n, base);
}

void Com::print(double n, int digits) {
  printFloat(n, digits);
}

void Com::println(void) {
  print('\r');
  print('\n');
}

// Private Function
void Com::printNumber(unsigned long n, uint8_t base) {
  if (n) {
    unsigned char buf[8 * sizeof(long)]; // Enough space for base 2
    int8_t i = 0;
    while (n) {
      buf[i++] = n % base;
      n /= base;
    }
    while (i--)
      print((char)(buf[i] + (buf[i] < 10 ? '0' : 'A' - 10)));
  }
  else
    print('0');
}

void Com::printFloat(double number, uint8_t digits) {
  if (isnan(number)) {
    serialprintPGM(TNAN);
    return;
  }
  if (isinf(number)) {
    serialprintPGM(TINF);
    return;
  }

  // Handle negative numbers
  if (number < 0.0) {
    print('-');
    number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i = 0; i < digits; ++i) rounding *= 0.1;
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits) {
    print('.');
    // Extract digits from the remainder one at a time
    while (digits--) {
      remainder *= 10.0;
      int toPrint = int(remainder);
      print(toPrint);
      remainder -= toPrint;
    }
  }
}

#endif // ARDUINO_ARCH_SAM