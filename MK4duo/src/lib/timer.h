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
 * timer.h - simple timer
 * Copyright (c) 2018 Alberto Cotronei @MagoKimbra
 */

template <class T>
class timer {

  public: /** Constructor */

    timer()                   { running = false; }
    timer(const millis_l now) { ms = T(now); running = true; }

  private: /** Private Parameters */

    T ms = 0;
    bool running;

  protected: /** Protected Function */

    T started() { return ms; }

  public: /** Public Function */
  
    void start()                    { ms = millis(); running = true; }
    void start(const T period_ms)   { ms = millis() + period_ms; running = true; }
    void stop()                     { running = false; }
    bool isStopped()                { return !running; }
    bool isRunning()                { return running; }
    bool expired(const T period_ms, const bool renew=true) {
      if (!running || !period_ms) return false;
      bool expired = false;
      const T now = millis();
      if (ms <= T(ms + period_ms)) {
        if ((now >= T(ms + period_ms)) || (now < ms)) expired = true;
      }
      else if ((now >= T(ms + period_ms)) && (now < ms)) expired = true;
      if (expired) {
        renew ? ms = now : running = false;
      }
      return expired;
    }
    bool pending(const T period_ms) { return !expired(period_ms); }

};

/**
 * Long Timer max period 49 days
 */
typedef timer<millis_l> long_timer_t;

/**
 * Short Timer max period 65 seconds
 */
typedef timer<millis_s> short_timer_t;
