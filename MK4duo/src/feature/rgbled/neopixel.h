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
 * neopixel.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#ifndef __NEOPIXEL_H__
#define __NEOPIXEL_H__

#if ENABLED(NEOPIXEL_LED)

  #include <Adafruit_NeoPixel.h>

  #define NEOPIXEL_IS_RGB  (NEOPIXEL_TYPE == NEO_RGB || NEOPIXEL_TYPE == NEO_RBG || NEOPIXEL_TYPE == NEO_GRB || NEOPIXEL_TYPE == NEO_GBR || NEOPIXEL_TYPE == NEO_BRG || NEOPIXEL_TYPE == NEO_BGR)
  #define NEOPIXEL_IS_RGBW !NEOPIXEL_IS_RGB

  #if NEOPIXEL_IS_RGB
    #define NEO_WHITE 255, 255, 255
  #else
    #define NEO_WHITE 0, 0, 0, 255
  #endif

  void setup_neopixel();
  void set_led_color(const uint8_t r, const uint8_t g, const uint8_t b, const uint8_t w=0, const uint8_t brightness=255);

  extern Adafruit_NeoPixel strip;

#endif // ENABLED(NEOPIXEL_LED)

#endif /* __NEOPIXEL_H__ */
