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

struct watch_t {

  millis_t  startwatch = 0,
            stopwatch  = 0;

  watch_t(const millis_t duration=0) {
    this->stopwatch = duration;
    if (duration) this->start();
  }

  FORCE_INLINE void start() { this->startwatch = millis(); }
  FORCE_INLINE void stop()  { this->startwatch = 0; }
  FORCE_INLINE bool isRunning() { return this->startwatch; }

  FORCE_INLINE bool elapsed(const millis_t period=0) {
    const millis_t  now = millis(),
                    end = period ? period : this->stopwatch;

    if (this->startwatch == 0 || now >= this->startwatch + end || now < this->startwatch) {
      this->startwatch = 0;
      return true;
    }
    return false;
  }

};
