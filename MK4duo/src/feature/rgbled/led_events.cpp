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
 * led_events.cpp
 *
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"

#if ENABLED(PRINTER_EVENT_LEDS)

  LedEvents ledevents;

  /** Private Parameters */
  uint8_t LedEvents::old_intensity = 0;

  #if HAS_LEDS_OFF_FLAG
    bool LedEvents::leds_off_after_print = false;
  #endif

  /** Public Function */
  void LedEvents::onHeating(const bool Hotend, const float &start, const float &current, const float &target) {
    if (Hotend) {
      const uint8_t blue = get_intensity(start, current, target);
      if (blue != old_intensity) {
        old_intensity = blue;
        set_rgb(255, 0, 255 - blue);
      }
    }
    else {
      const uint8_t red = get_intensity(start, current, target);
      if (red != old_intensity) {
        old_intensity = red;
        set_rgb(red, 0, 255);
      }
    }
  }

  /** Private Function */
  uint8_t LedEvents::get_intensity(const float &start, const float &current, const float &target) {
    return (uint8_t)map(constrain(current, start, target), start, target, 0.f, 255.f);
    //return (uint8_t)((((current - start) * 255) + target - start + 1) / (target - start + 1));
  }

  inline void LedEvents::set_rgb(const uint8_t r, const uint8_t g, const uint8_t b) {
    leds.set_color(
      MakeLEDColor(r, g, b, 0, leds.getBrightness())
      #if ENABLED(NEOPIXEL_LED) && ENABLED(NEOPIXEL_IS_SEQUENTIAL)
        , true
      #endif
    );
  }

#endif // PRINTER_EVENT_LEDS
