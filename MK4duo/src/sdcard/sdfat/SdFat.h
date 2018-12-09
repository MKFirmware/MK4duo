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
#pragma once

#include "SdFatConfig.h"
#include "SdFile.h"

#define FILE_READ   O_READ
#define FILE_WRITE (O_CREAT | O_APPEND | O_WRITE | O_TRUNC)

/**
 * \class SdFat
 * \brief Integration class for the %SdFat library.
 */
class SdFat {

  public: /** Constructor */

    SdFat() {}
    Sd2Card* card() { return &card_; }
    
  public:

    bool chdir(bool set_cwd = false);
    bool chdir(const char * path, bool set_cwd = false);
    void chvol();
    void errorHalt();
    void errorHalt_P(PGM_P msg);
    void errorHalt(char const *msg);
    void errorPrint();
    void errorPrint_P(PGM_P msg);
    void errorPrint(char const *msg);
    bool exists(const char * name);
    bool begin(uint8_t chipSelectPin = SD_CHIP_SELECT_PIN, uint8_t sckRateID = SPI_FULL_SPEED);
    void initErrorHalt();
    void initErrorHalt(char const *msg);
    void initErrorHalt_P(PGM_P msg);
    void initErrorPrint();
    void initErrorPrint(char const *msg);
    void initErrorPrint_P(PGM_P msg);
    void ls();
    bool mkdir(const char * path, bool pFlag = true);
    bool remove(const char * path);
    bool rename(const char * oldPath, const char * newPath);
    bool rmdir(const char * path);
    bool truncate(const char * path, uint32_t length);

    SdVolume* vol() { return &vol_; }

    SdBaseFile* vwd() { return &vwd_; }

  private:

    Sd2Card card_;
    SdVolume vol_;
    SdBaseFile vwd_;

};
