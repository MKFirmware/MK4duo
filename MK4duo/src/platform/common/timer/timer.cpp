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

/**
 * timer.cpp - simple timer
 * Copyright (c) 2018 Alberto Cotronei @MagoKimbra
 */

#include "../../../../MK4duo.h"

/**
 * Constructor timer
 */
template<typename T>
timer<T>::timer(const bool auto_start/*=false*/) {
  if (auto_start) start();
  else _running = false;
}

/**
 * Start Timer
 */
template<typename T>
void timer<T>::start() {
  _started = millis();
  _running = true;
}

template<typename T>
void timer<T>::start(const T period_ms) {
  _started = millis() + period_ms;
  _running = true;
}

template<typename T>
bool timer<T>::expired(const T period_ms, const bool renew/*=true*/) {

  if (!_running || !period_ms) return false;

  bool expired = false;
  const T now = millis();

  if (_started <= _started + period_ms) {
    if ((now >= _started + period_ms) || (now < _started)) expired = true;
  }
  else if ((now >= _started + period_ms) && (now < _started)) expired = true;

  if (expired) {
    renew ? _started = now : _running = false;
  }

  return expired;
}

template class timer<millis_l>;
template class timer<millis_s>;
