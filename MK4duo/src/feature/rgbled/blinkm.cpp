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

/**
 * blinkm.cpp - Library for controlling a BlinkM over i2c
 * Created by Tim Koster, August 21 2013.
 */

#include "../../../MK4duo.h"

#if ENABLED(BLINKM)

  #include "Arduino.h"
  #include <Wire.h>
  #include "blinkm.h"

  void blinkm_set_led_color(const LEDColor &color) {
    Wire.begin();
    Wire.beginTransmission(0x09);
    Wire.write('o');                    // to disable ongoing script, only needs to be used once
    Wire.write('n');
    Wire.write(color.r);
    Wire.write(color.g);
    Wire.write(color.b);
    Wire.endTransmission();
  }

#endif // BLINKM
