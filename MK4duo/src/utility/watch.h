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

/**
 * Class Watch
 */
template <class T>
class Watch {

  public: /** Constructor */

    Watch(const bool _start=false) {
      if (_start) start();
      else         stop();
    }

  private: /** Private Parameters */

    T startwatch = 0;

    bool running = false;

  public: /** Public Function */

    void start() { startwatch = millis(); running = true; }
    void stop()  { running = false; }

    bool isRunning() { return running; }

    bool elapsed(const T period) {

      if (!running || !period) return false;

      bool elapsed = false;
      const T now = millis();

      if (startwatch <= startwatch + period) {
        if (now >= startwatch + period || now < startwatch)
          elapsed = true;
      }
      else {
        if (now >= startwatch + period && now < startwatch)
          elapsed = true;
      }

      if (elapsed) running = false;

      return elapsed;
    }

};

using watch_l = Watch<uint32_t>;  // Maximum period is at least 49 days.
using watch_s = Watch<uint16_t>;  // Maximum period is at least 65 seconds.

/**
 * Class StopWatch
 */
class StopWatch {

  private: /** Private Parameters */

    enum State : char { STOPPED, RUNNING, PAUSED };

    static StopWatch::State state;

    static millis_t startwatch,
                    stopwatch,
                    accumulator;

  public: /** Public Function */

    FORCE_INLINE static void init() { reset(); }

    static bool start();
    static bool stop();
    static bool pause();

    static void resume(const millis_t this_time);
    static void reset();

    FORCE_INLINE static bool isRunning()  { return state == RUNNING;  }
    FORCE_INLINE static bool isPaused()   { return state == PAUSED;   }

    static millis_t duration();

};
