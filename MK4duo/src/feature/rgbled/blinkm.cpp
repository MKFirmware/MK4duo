/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 Alberto Cotronei @MagoKimbra
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
 * blinkm.cpp - Library for controlling a BlinkM over i2c
 * Created by Tim Koster, August 21 2013.
 */

#include "../../../MK4duo.h"

#if ENABLED(BLINKM)

  #include "Arduino.h"
  #include <Wire.h>

  void set_led_color(const uint8_t r, const uint8_t g, const uint8_t b, const uint8_t w/*=0*/, const uint8_t p/*=255*/) {

    UNUSED(w);
    UNUSED(p);

    Wire.begin();
    Wire.beginTransmission(0x09);
    Wire.write('o');                    //to disable ongoing script, only needs to be used once
    Wire.write('n');
    Wire.write(r);
    Wire.write(g);
    Wire.write(b);
    Wire.endTransmission();
  }

#endif // BLINKM
