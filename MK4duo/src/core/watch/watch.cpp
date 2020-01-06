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

#include "../../../MK4duo.h"
#include "sanitycheck.h"

Watch::State Watch::state = STOPPED;

millis_l  Watch::startwatch   = 0,
          Watch::stopwatch    = 0,
          Watch::accumulator  = 0;

bool Watch::start() {
  if (!isRunning()) {

    if (isPaused())
      accumulator = duration();
    else
      reset();

    state = RUNNING;
    startwatch = millis();
    return true;
  }
  else
    return false;
}

bool Watch::stop() {
  if (isRunning() || isPaused()) {
    state = STOPPED;
    stopwatch = millis();
    return true;
  }
  else
    return false;
}

bool Watch::pause() {
  if (isRunning()) {
    state = PAUSED;
    stopwatch = millis();
    return true;
  }
  else
    return false;
}

void Watch::resume(const millis_l this_time) {
  reset();
  if ((accumulator = this_time)) state = RUNNING;
}

void Watch::reset() {
  state = STOPPED;
  startwatch  = 0;
  stopwatch   = 0;
  accumulator = 0;
}

millis_l Watch::duration() {
  return (((isRunning()) ? millis() : stopwatch) - startwatch) / 1000UL + accumulator;
}
