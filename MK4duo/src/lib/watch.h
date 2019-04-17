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

template <class T>
class Watch {

  public: /** Constructor */

    Watch(const T duration=0) {
      stopwatch = duration;
      if (duration) start();
    }

  public: /** Public Parameters */

    T startwatch = 0,
      stopwatch  = 0;

  public: /** Public Function */

    void start() { startwatch = millis(); }
    void stop()  { startwatch = 0; }

    bool isRunning() { return startwatch; }

    bool elapsed(const T period=0) {
      const T now = millis(),
              end = period ? period : stopwatch;

      if (startwatch == 0 || now >= startwatch + end || now < startwatch) {
        startwatch = 0;
        return true;
      }
      return false;
    }

};

using watch_l = Watch<uint32_t>;
using watch_s = Watch<uint16_t>;
