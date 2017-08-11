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

#include "../../../base.h"

#if ENABLED(ARDUINO_ARCH_SAM)

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

void Com::printInfoLN(FSTRINGPARAM(text)) {
  PS_PGM(tInfo);
  PS_PGM(text);
  println();
}

void Com::PS_PGM(FSTRINGPARAM(ptr)) {
  char c;
  while ((c = HAL::readFlashByte(ptr++)) != 0)
    HAL::serialWriteByte(c);
}

void Com::printNumber(uint32_t n) {
  char buf[11]; // Assumes 8-bit chars plus zero byte.
  char *str = &buf[10];
  *str = '\0';
  do {
    unsigned long m = n;
    n /= 10;
    *--str = '0' + (m - 10 * n);
  } while(n);

  print(str);
}

void Com::printFloat(float number, uint8_t digits) {
  if (isnan(number)) {
    PS_PGM(TNAN);
    return;
  }
  if (isinf(number)) {
    PS_PGM(TINF);
    return;
  }
  // Handle negative numbers
  if (number < 0.0) {
    print('-');
    number = -number;
  }
  // Round correctly so that print(1.999, 2) prints as "2.00"
  float rounding = 0.5;
  for (uint8_t i = 0; i < digits; ++i)
    rounding /= 10.0;

  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  float remainder = number - (float)int_part;
  printNumber(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    print('.');

  // Extract digits from the remainder one at a time
  while (digits-- > 0) {
    remainder *= 10.0;
    int toPrint = int(remainder);
    print(toPrint);
    remainder -= toPrint;
  }
}

void Com::print(const char* text) {
  while(*text) {
    HAL::serialWriteByte(*text++);
  }
}

void Com::print(long value) {
  if (value < 0) {
    HAL::serialWriteByte('-');
    value = -value;
  }
  printNumber(value);
}

#endif // ARDUINO_ARCH_SAM