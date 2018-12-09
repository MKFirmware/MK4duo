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
FSTRINGVALUE(PAUSE, "//action:pause");
FSTRINGVALUE(RESUME, "//action:resume");
FSTRINGVALUE(DISCONNECT, "//action:disconnect");
FSTRINGVALUE(POWEROFF, "//action:poweroff");
FSTRINGVALUE(REQUESTPAUSE, "RequestPause:");

/** Public Parameters */
int8_t Com::serial_port = -1;

/** Public Function */
void Com::setBaudrate() {
  MKSERIAL1.begin(BAUDRATE_1);
  HAL::delayMilliseconds(1000);
  #if NUM_SERIAL > 1
    MKSERIAL2.begin(BAUDRATE_2);
    HAL::delayMilliseconds(1000);
  #endif
  SERIAL_STR(START);
  SERIAL_EOL();
}

void Com::serialFlush() {
  if (serial_port == -1 || serial_port == 0) MKSERIAL1.flush();
  #if NUM_SERIAL > 1
    if (serial_port == -1 || serial_port == 1) MKSERIAL2.flush();
  #endif
}

int Com::serialRead(const uint8_t index) {
  switch (index) {
    case 0: return MKSERIAL1.read();
    #if NUM_SERIAL > 1
      case 1: return MKSERIAL2.read();
    #endif
    default: return -1;
  }
}

bool Com::serialDataAvailable() {
  return (MKSERIAL1.available() ? true :
    #if NUM_SERIAL > 1
      MKSERIAL2.available() ? true :
    #endif
    false);
}

bool Com::serialDataAvailable(const uint8_t index) {
  switch (index) {
    case 0: return MKSERIAL1.available();
    #if NUM_SERIAL > 1
      case 1: MKSERIAL2.available();
    #endif
    default: return false;
  }
}

// Functions for serial printing from PROGMEM. (Saves loads of SRAM.)
void Com::printPGM(PGM_P str) {
  while (char c = pgm_read_byte(str++)) {
    if (serial_port == -1 || serial_port == 0) MKSERIAL1.write(c);
    #if NUM_SERIAL > 1
      if (serial_port == -1 || serial_port == 1) MKSERIAL2.write(c);
    #endif
  }
}

void Com::write(const uint8_t c) {
  if (serial_port == -1 || serial_port == 0) MKSERIAL1.write(c);
  #if NUM_SERIAL > 1
    if (serial_port == -1 || serial_port == 1) MKSERIAL2.write(c);
  #endif
}

void Com::write(const char* str) {
  while (*str) {
    if (serial_port == -1 || serial_port == 0) MKSERIAL1.write(*str);
    #if NUM_SERIAL > 1
    if (serial_port == -1 || serial_port == 1) MKSERIAL2.write(*str);
    #endif
    str++;
  }
}

void Com::write(const uint8_t* buffer, size_t size) {
  while (size--) {
    if (serial_port == -1 || serial_port == 0) MKSERIAL1.write(*buffer);
    #if NUM_SERIAL > 1
      if (serial_port == -1 || serial_port == 1) MKSERIAL2.write(*buffer);
    #endif
    buffer++;
  }
}

void Com::print(const String& s) {
  for (int i = 0; i < (int)s.length(); i++) {
    if (serial_port == -1 || serial_port == 0) MKSERIAL1.write(s[i]);
    #if NUM_SERIAL > 1
      if (serial_port == -1 || serial_port == 1) MKSERIAL2.write(s[i]);
    #endif
  }
}

void Com::print(const char* str) {
  write(str);
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

void Com::print_spaces(uint8_t count) {
  count *= (PROPORTIONAL_FONT_RATIO);
  while (count--) {
    if (serial_port == -1 || serial_port == 0) MKSERIAL1.write(' ');
    #if NUM_SERIAL > 1
      if (serial_port == -1 || serial_port == 1) MKSERIAL2.write(' ');
    #endif
  }
}

void Com::print_logic(PGM_P const label, const bool logic) {
  SERIAL_PS(label);
  SERIAL_MSG(": ");
  SERIAL_PS(logic ? PSTR("true") : PSTR("false"));
}

void Com::print_onoff(PGM_P const label, const bool onoff) {
  SERIAL_PS(label);
  SERIAL_MSG(": ");
  SERIAL_PS(onoff ? PSTR(MSG_ON) : PSTR(MSG_OFF));
}

#if ENABLED(DEBUG_FEATURE)

  void Com::print_xyz(PGM_P prefix, PGM_P suffix, const float x, const float y, const float z) {
    SERIAL_PS(prefix);
    SERIAL_CHR('(');
    SERIAL_VAL(x);
    SERIAL_MV(", ", y);
    SERIAL_MV(", ", z);
    SERIAL_CHR(")");

    if (suffix) SERIAL_PS(suffix);
    else SERIAL_EOL();
  }

  void Com::print_xyz(PGM_P prefix, PGM_P suffix, const float xyz[]) {
    print_xyz(prefix, suffix, xyz[X_AXIS], xyz[Y_AXIS], xyz[Z_AXIS]);
  }

  #if HAS_PLANAR
    void Com::print_xyz(PGM_P prefix, PGM_P suffix, const vector_3 &xyz) {
      print_xyz(prefix, suffix, xyz.x, xyz.y, xyz.z);
    }
  #endif

#endif // ENABLED(DEBUG_FEATURE)

// Private function
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
