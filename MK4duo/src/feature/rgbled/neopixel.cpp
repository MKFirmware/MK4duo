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
 * neopixel.cpp
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#include "../../../base.h"

#if HAS_NEOPIXEL

  #include "library/Adafruit_NeoPixel.h"

  #if ENABLED(NEOPIXEL_RGB_LED)
    Adafruit_NeoPixel strip = Adafruit_NeoPixel(NEOPIXEL_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
  #else
    Adafruit_NeoPixel strip = Adafruit_NeoPixel(NEOPIXEL_PIXELS, NEOPIXEL_PIN, NEO_GRBW + NEO_KHZ800);
  #endif

  void Printer::set_neopixel_color(const uint32_t color) {
    for (uint16_t i = 0; i < strip.numPixels(); ++i)
      strip.setPixelColor(i, color);
    strip.show();
  }

  void Printer::setup_neopixel() {
    strip.setBrightness(255); // 0 - 255 range
    strip.begin();
    strip.show(); // initialize to all off

    #if ENABLED(NEOPIXEL_STARTUP_TEST)
      delay(2000);
      set_neopixel_color(strip.Color(255, 0, 0, 0));  // red
      delay(2000);
      set_neopixel_color(strip.Color(0, 255, 0, 0));  // green
      delay(2000);
      set_neopixel_color(strip.Color(0, 0, 255, 0));  // blue
      delay(2000);
    #endif
    set_neopixel_color(strip.Color(0, 0, 0, 255));    // white
  }

  void set_led_color(const uint8_t r, const uint8_t g, const uint8_t b, const uint8_t w/*=0*/, const bool isSequence/*=false*/) {

    #if ENABLED(NEOPIXEL_RGBW_LED)
      const uint32_t color = strip.Color(r, g, b, w);
    #else
      UNUSED(w);
      const uint32_t color = strip.Color(r, g, b);
    #endif

    static uint16_t nextLed = 0;

    if (!isSequence)
      set_neopixel_color(color);
    else {
      strip.setPixelColor(nextLed, color);
      strip.show();
      if (++nextLed >= strip.numPixels()) nextLed = 0;
      return true;
    }
    return false;
  }

#endif // HAS_NEOPIXEL
