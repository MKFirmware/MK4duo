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

#include "../../../../MK4duo.h"

#if ENABLED(LASER) && ENABLED(LASER_RASTER)

  const char b64_alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
      "abcdefghijklmnopqrstuvwxyz"
      "0123456789+/";

  /* 'Private' declarations */
  inline void a3_to_a4(unsigned char *a4, unsigned char *a3);
  inline void a4_to_a3(unsigned char *a3, unsigned char *a4);
  inline unsigned char b64_lookup(char c);

  int base64_encode(char *output, char *input, int inputLen) {
    int i = 0, j = 0;
    int encLen = 0;
    unsigned char a3[3];
    unsigned char a4[4];

    while (inputLen--) {
      a3[i++] = *(input++);
      if (i == 3) {
        a3_to_a4(a4, a3);

        for (i = 0; i < 4; i++) {
          output[encLen++] = b64_alphabet[a4[i]];
        }

        i = 0;
      }
    }

    if (i) {
      for (j = i; j < 3; j++) {
        a3[j] = '\0';
      }

      a3_to_a4(a4, a3);

      for (j = 0; j < i + 1; j++) {
        output[encLen++] = b64_alphabet[a4[j]];
      }

      while (i++ < 3) {
        output[encLen++] = '=';
      }
    }

    output[encLen] = '\0';
    return encLen;
  }

  int base64_decode(unsigned char *output, char *input, int inputLen) {
    int i = 0, j = 0;
    int decLen = 0;
    unsigned char a3[3];
    unsigned char a4[4];

    while (inputLen--) {
      if (*input == '=') {
        break;
      }

      a4[i++] = *(input++);
      if (i == 4) {
        for (i = 0; i <4; i++) {
          a4[i] = b64_lookup(a4[i]);
        }

        a4_to_a3(a3,a4);

        for (i = 0; i < 3; i++) {
          output[decLen++] = a3[i];
        }
        i = 0;
      }
    }

    if (i) {
      for (j = i; j < 4; j++) {
        a4[j] = '\0';
      }

      for (j = 0; j <4; j++) {
        a4[j] = b64_lookup(a4[j]);
      }

      a4_to_a3(a3, a4);

      for (j = 0; j < i - 1; j++) {
        output[decLen++] = a3[j];
      }
    }
    output[decLen] = '\0';
    return decLen;
  }

  int base64_enc_len(int plainLen) {
    int n = plainLen;
    return (n + 2 - ((n + 2) % 3)) / 3 * 4;
  }

  int base64_dec_len(char * input, int inputLen) {
    int i = 0;
    int numEq = 0;
    for (i = inputLen - 1; input[i] == '='; i--) {
      numEq++;
    }

    return ((6 * inputLen) / 8) - numEq;
  }

  inline void a3_to_a4(unsigned char *a4, unsigned char *a3) {
    a4[0] = (a3[0] & 0xFC) >> 2;
    a4[1] = ((a3[0] & 0x03) << 4) + ((a3[1] & 0xF0) >> 4);
    a4[2] = ((a3[1] & 0x0F) << 2) + ((a3[2] & 0xC0) >> 6);
    a4[3] = (a3[2] & 0x3F);
  }

  inline void a4_to_a3(unsigned char *a3, unsigned char *a4) {
    a3[0] = (a4[0] << 2) + ((a4[1] & 0x30) >> 4);
    a3[1] = ((a4[1] & 0xF) << 4) + ((a4[2] & 0x3C) >> 2);
    a3[2] = ((a4[2] & 0x3) << 6) + a4[3];
  }

  inline unsigned char b64_lookup(char c) {
    if(c >='A' && c <='Z') return c - 'A';
    if(c >='a' && c <='z') return c - 71;
    if(c >='0' && c <='9') return c + 4;
    if(c == '+') return 62;
    if(c == '/') return 63;
    return -1;
  }

#endif
