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
 * led.cpp
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"

#if ENABLED(RGB_LED) || ENABLED(RGBW_LED)

  void set_led_color(const uint8_t r, const uint8_t g, const uint8_t b, const uint8_t w/*=0*/, const uint8_t p/*=255*/) {

    UNUSED(p);

    // This variant uses 3 separate pins for the RGB components.
    // If the pins can do PWM then their intensity will be set.
    WRITE(RGB_LED_R_PIN, r ? HIGH : LOW);
    WRITE(RGB_LED_G_PIN, g ? HIGH : LOW);
    WRITE(RGB_LED_B_PIN, b ? HIGH : LOW);
    analogWrite(RGB_LED_R_PIN, r);
    analogWrite(RGB_LED_G_PIN, g);
    analogWrite(RGB_LED_B_PIN, b);

    #if ENABLED(RGBW_LED)
      WRITE(RGB_LED_W_PIN, w ? HIGH : LOW);
      analogWrite(RGB_LED_W_PIN, w);
    #else
      UNUSED(w);
    #endif
  }

#endif // ENABLED(RGB_LED) || ENABLED(RGBW_LED)
