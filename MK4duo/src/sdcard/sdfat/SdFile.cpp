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

/**
 * Arduino SdFat Library
 * Copyright (C) 2009 by William Greiman
 *
 * This file is part of the Arduino Sd2Card Library
 */

#include "../../../MK4duo.h"

#if HAS_SD_SUPPORT

#include "SdFile.h"

/**
 *  Create a file object and open it in the current working directory.
 *
 * \param[in] path A path with a valid 8.3 DOS name for a file to be opened.
 *
 * \param[in] oflag Values for \a oflag are constructed by a bitwise-inclusive
 * OR of open flags. see SdBaseFile::open(SdBaseFile*, const char*, uint8_t).
 */
SdFile::SdFile(const char* path, uint8_t oflag) : SdBaseFile(path, oflag) { }

/**
 * Write data to an open file.
 *
 * \note Data is moved to the cache but may not be written to the
 * storage device until sync() is called.
 *
 * \param[in] buf Pointer to the location of the data to be written.
 *
 * \param[in] nbyte Number of bytes to write.
 *
 * \return For success write() returns the number of bytes written, always
 * \a nbyte.  If an error occurs, write() returns -1.  Possible errors
 * include write() is called before a file has been opened, write is called
 * for a read-only file, device is full, a corrupt file system or an I/O error.
 *
 */
int SdFile::write(const void* buf, size_t nbyte) {
  return SdBaseFile::write(buf, nbyte);
}

/**
 * Write a byte to a file. Required by the Arduino Print class.
 * \param[in] b the byte to be written.
 * Use writeError to check for errors.
 */
#if ENABLED(CPU_32_BIT)
  #ifdef COMPAT_PRE1
    void SdFile::write(uint8_t b) {
      dBaseFile::write(&b, 1);
    }
  #else
    size_t SdFile::write(uint8_t b) {
      return SdBaseFile::write(&b, 1) == 1 ? 1 : 0;
    }
  #endif

  
  /**
   * Write a string to a file. Used by the Arduino Print class.
   * \param[in] str Pointer to the string.
   * Use getWriteError to check for errors.
   * \return count of characters written for success or -1 for failure.
   */
  int SdFile::write(const char * str) {
    return SdBaseFile::write(str, strlen(str));
  }
#else
  #if ARDUINO >= 100
    size_t SdFile::write(uint8_t b) {
      return SdBaseFile::write(&b, 1);
    }
  #else
    void SdFile::write(uint8_t b) {
      SdBaseFile::write(&b, 1);
    }
  #endif
  
  /**
   * Write a string to a file. Used by the Arduino Print class.
   * \param[in] str Pointer to the string.
   * Use writeError to check for errors.
   */
  void SdFile::write(const char * str) {
    SdBaseFile::write(str, strlen(str));
  }
#endif


/**
 * Write a PROGMEM string to a file.
 * \param[in] str Pointer to the PROGMEM string.
 * Use writeError to check for errors.
 */
void SdFile::write_P(PGM_P str) {
  for (uint8_t c; (c = pgm_read_byte(str)); str++) write(c);
}

/**
 * Write a PROGMEM string followed by CR/LF to a file.
 * \param[in] str Pointer to the PROGMEM string.
 * Use writeError to check for errors.
 */
void SdFile::writeln_P(PGM_P str) {
  write_P(str);
  write_P(PSTR("\r\n"));
}

#endif // HAS_SD_SUPPORT
