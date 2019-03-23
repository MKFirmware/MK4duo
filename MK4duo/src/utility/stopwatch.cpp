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

#include "../../MK4duo.h"

Stopwatch::State Stopwatch::state = STOPPED;

watch_t Stopwatch::Timestamp;

millis_t Stopwatch::accumulator = 0;

bool Stopwatch::stop() {
  if (isRunning() || isPaused()) {
    state = STOPPED;
    Timestamp.stopwatch = millis();
    return true;
  }
  else
    return false;
}

bool Stopwatch::pause() {
  if (isRunning()) {
    state = PAUSED;
    Timestamp.stopwatch = millis();
    return true;
  }
  else
    return false;
}

bool Stopwatch::start() {
  if (!isRunning()) {

    if (isPaused())
      accumulator = duration();
    else
      reset();

    state = RUNNING;
    Timestamp.start();
    return true;
  }
  else
    return false;
}

void Stopwatch::resume(const millis_t this_time) {
  reset();
  if ((accumulator = this_time)) state = RUNNING;
}

void Stopwatch::reset() {
  state = STOPPED;
  Timestamp.stop();
  accumulator = 0;
}

millis_t Stopwatch::duration() {
  return (((isRunning()) ? millis() : Timestamp.stopwatch)
          - Timestamp.startwatch) / 1000UL + accumulator;
}
