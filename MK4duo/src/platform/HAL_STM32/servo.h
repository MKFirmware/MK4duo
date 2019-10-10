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

#include <Servo.h>

#define SERVOS_PER_TIMER    12 // the maximum number of servos controlled by one timer
#define MAX_SERVOS          (_Nbr_16timers  * SERVOS_PER_TIMER)

// Inherit and expand on the official library
class MKServo : public Servo {

  public: /** Public Parameters */

    int angle[2];

  private: /** Private Parameters */

    uint8_t   servoIndex; // index into the channel data for this servo
    uint16_t  min_ticks,
              max_ticks;

  public: /** Public Function */

    int8_t attach(const int pin);
    int8_t attach(const int pin, const int min, const int max);
    void move(const int value);
    void print_M281();

};
