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

  private: /** Private Parameters */

    Sd2Card     card_;
    SdVolume    vol_;
    SdBaseFile  vwd_;

  public: /** Public Function */

    /**
     * return a pointer to the Sd2Card object.
     */
    Sd2Card* card() { return &card_; }

    /**
     * return a pointer to the SdVolume object.
     */
    SdVolume* vol() { return &vol_; }

    /**
     * return a pointer to the volume working directory.
     */
    SdBaseFile* vwd() { return &vwd_; }

    bool begin(uint8_t chipSelectPin=SD_CHIP_SELECT_PIN, uint8_t sckRateID=SPI_FULL_SPEED);

    bool chdir(bool set_cwd = false);
    bool chdir(char const * path, bool set_cwd=false);

    bool exists(char const * name);
    
    bool mkdir(char const * path, bool pFlag=true);
    bool remove(char const * path);
    bool rmdir(char const * path);
    bool truncate(char const * path, uint32_t length);

    void chvol() { SdBaseFile::cwd_ = &vwd_; }

};
