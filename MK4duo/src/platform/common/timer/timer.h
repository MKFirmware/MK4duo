/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2019 Alberto Cotronei @MagoKimbra
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

    timer(const bool auto_start=false);

  private: /** Private Parameters */

    bool  _running;
    T     _started;

  protected: /** Protected Function */

    T started() { return _started; }

  public: /** Public Function */
  
    void start();
    void start(const T period_ms);
    void stop()     { _running = false; }
    bool isRunning()  { return _running; }
    bool expired(const T period_ms, const bool renew=true);
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
