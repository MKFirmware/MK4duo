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
 * neopixel.cpp
 *
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"

#if ENABLED(NEOPIXEL_LED)

  #include "neopixel.h"

  Adafruit_NeoPixel strip = Adafruit_NeoPixel(NEOPIXEL_PIXELS, NEOPIXEL_PIN, NEOPIXEL_TYPE + NEO_KHZ800);

  void set_neopixel_color(const uint32_t color) {
    for (uint16_t i = 0; i < strip.numPixels(); ++i)
      strip.setPixelColor(i, color);
    strip.show();
  }

  void setup_neopixel() {

    SET_OUTPUT(NEOPIXEL_PIN);

    strip.setBrightness(NEOPIXEL_BRIGHTNESS); // 0 - 255 range
    strip.begin();
    strip.show(); // initialize to all off

    #if ENABLED(NEOPIXEL_STARTUP_TEST)
      printer.safe_delay(2000);
      set_neopixel_color(strip.Color(255, 0, 0, 0));  // red
      printer.safe_delay(2000);
      set_neopixel_color(strip.Color(0, 255, 0, 0));  // green
      printer.safe_delay(2000);
      set_neopixel_color(strip.Color(0, 0, 255, 0));  // blue
      printer.safe_delay(2000);
    #endif
    set_neopixel_color(strip.Color(NEO_BLACK));       // black
  }

#endif // ENABLED(NEOPIXEL_LED)
