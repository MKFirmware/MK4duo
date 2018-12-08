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
 * Copyright (C) 2012 by William Greiman
 *
 * This file is part of the Arduino SdFat Library
 *
 * This Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Arduino SdFat Library. If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "../../../MK4duo.h"

#if HAS_SD_SUPPORT

#include "sdfat.h"

/**
 * Initialize an SdFat object.
 *
 * Initializes the SD card, SD volume, and root directory.
 *
 * \param[in] chipSelectPin SD chip select pin. See Sd2Card::init().
 * \param[in] sckRateID value for SPI SCK rate. See Sd2Card::init().
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
bool SdFat::begin(uint8_t chipSelectPin, uint8_t sckRateID) {
  return card_.init(sckRateID, chipSelectPin) && vol_.init(&card_) && chdir(1);
}

/** Change a volume's working directory to root
 *
 * Changes the volume's working directory to the SD's root directory.
 * Optionally set the current working directory to the volume's
 * working directory.
 *
 * \param[in] set_cwd Set the current working directory to this volume's
 *  working directory if true.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
bool SdFat::chdir(bool set_cwd) {
  if (set_cwd) chvol();
  if (vwd_.isOpen()) vwd_.close();
  return vwd_.openRoot(&vol_);
}

/** Change a volume's working directory
 *
 * Changes the volume working directory to the \a path subdirectory.
 * Optionally set the current working directory to the volume's
 * working directory.
 *
 * Example: If the volume's working directory is "/DIR", chdir("SUB")
 * will change the volume's working directory from "/DIR" to "/DIR/SUB".
 *
 * If path is "/", the volume's working directory will be changed to the
 * root directory
 *
 * \param[in] path The name of the subdirectory.
 *
 * \param[in] set_cwd Set the current working directory to this volume's
 *  working directory if true.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
bool SdFat::chdir(char const * path, bool set_cwd) {
  SdBaseFile dir;
  if (path[0] == '/' && path[1] == '\0') return chdir(set_cwd);

  if (!dir.open(&vwd_, path, FILE_READ)) return false;
  if (!dir.isDir()) return false;
  vwd_ = dir;
  if (set_cwd) chvol();
  return true;
}

/**
 * Test for the existence of a file.
 *
 * \param[in] name Name of the file to be tested for.
 *
 * \return true if the file exists else false.
 */
bool SdFat::exists(char const * name) { return vwd_.exists(name); }

/** Make a subdirectory in the volume working directory.
 *
 * \param[in] path A path with a valid 8.3 DOS name for the subdirectory.
 *
 * \param[in] pFlag Create missing parent directories if true.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
bool SdFat::mkdir(char const * path, bool pFlag) {
  SdBaseFile sub;
  return sub.mkdir(&vwd_, path, pFlag);
}

/** Remove a file from the volume working directory.
*
* \param[in] path A path with a valid 8.3 DOS name for the file.
*
* \return The value one, true, is returned for success and
* the value zero, false, is returned for failure.
*/
bool SdFat::remove(char const * path) {
  return SdBaseFile::remove(&vwd_, path);
}

/** Remove a subdirectory from the volume's working directory.
 *
 * \param[in] path A path with a valid 8.3 DOS name for the subdirectory.
 *
 * The subdirectory file will be removed only if it is empty.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
bool SdFat::rmdir(char const * path) {
  SdBaseFile sub;
  if (!sub.open(path, FILE_READ)) return false;
  return sub.rmdir();
}

/** Truncate a file to a specified length.  The current file position
 * will be maintained if it is less than or equal to \a length otherwise
 * it will be set to end of file.
 *
 * \param[in] path A path with a valid 8.3 DOS name for the file.
 * \param[in] length The desired length for the file.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 * Reasons for failure include file is read only, file is a directory,
 * \a length is greater than the current file size or an I/O error occurs.
 */
bool SdFat::truncate(char const * path, uint32_t length) {
  SdBaseFile file;
  if (!file.open(path, O_WRITE)) return false;
  return file.truncate(length);
}

#endif // HAS_SD_SUPPORT
