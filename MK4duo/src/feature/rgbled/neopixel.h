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
 * neopixel.h
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#if ENABLED(NEOPIXEL_LED)

#include <Adafruit_NeoPixel.h>

#define NEOPIXEL_IS_RGB  (NEOPIXEL_TYPE == NEO_RGB || NEOPIXEL_TYPE == NEO_RBG || NEOPIXEL_TYPE == NEO_GRB || NEOPIXEL_TYPE == NEO_GBR || NEOPIXEL_TYPE == NEO_BRG || NEOPIXEL_TYPE == NEO_BGR)
#define NEOPIXEL_IS_RGBW !NEOPIXEL_IS_RGB

#if NEOPIXEL_IS_RGB
  #define NEO_WHITE 255, 255, 255, 0
#else
  #define NEO_WHITE 0, 0, 0, 255
#endif

#define NEO_BLACK   0, 0, 0, 0

class Neopixel {

  public: /** Constructor */

    Neopixel() {}

  public: /** Public Parameters */

    static Adafruit_NeoPixel strip;

  public: /** Public Function */

    static void setup();
    static void set_color(const uint32_t color);

};

extern Neopixel neopixel;

#endif // ENABLED(NEOPIXEL_LED)
