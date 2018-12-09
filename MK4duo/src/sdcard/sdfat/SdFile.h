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
#pragma once

/**
 * \file
 * \brief SdFile class
 */

/**
 * Arduino SdFat Library
 * Copyright (C) 2009 by William Greiman
 *
 * This file is part of the Arduino Sd2Card Library
 */

#include "SdBaseFile.h"

#include <stdint.h>
#include <string.h>

/**
 * \class SdFile
 * \brief SdBaseFile with Print.
 */
class SdFile : public SdBaseFile {
  public:
    SdFile() {}
    SdFile(const char * path, uint8_t oflag);

    #if ENABLED(CPU_32_BIT)

      #ifdef COMPAT_PRE1
        void write(uint8_t b);
      #else
        size_t write(uint8_t b);
      #endif

      int write(const char * str);
      int write(const void* buf, size_t nbyte);

    #else

      #if ARDUINO >= 100
        size_t write(uint8_t b);
      #else
        void write(uint8_t b);
      #endif

      int16_t write(const void* buf, uint16_t nbyte);
      void write(const char * str);

    #endif

    void write_P(PGM_P str);
    void writeln_P(PGM_P str);

};
