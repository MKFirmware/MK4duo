/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
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
class Watch {

  private: /** Private Parameters */

    enum State : char { STOPPED, RUNNING, PAUSED };

    static Watch::State state;

    static millis_l startwatch,
                    stopwatch,
                    accumulator;

  public: /** Public Function */

    FORCE_INLINE static void init() { reset(); }

    static bool start();
    static bool stop();
    static bool pause();

    static void resume(const millis_l this_time);
    static void reset();

    FORCE_INLINE static bool isRunning()  { return state == RUNNING;  }
    FORCE_INLINE static bool isPaused()   { return state == PAUSED;   }

    static millis_l duration();

};
