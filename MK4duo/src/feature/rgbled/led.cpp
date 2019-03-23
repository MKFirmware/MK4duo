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
 * led.cpp
 *
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"

#if HAS_COLOR_LEDS

  #if ENABLED(BLINKM)
    #include "blinkm.h"
  #endif

  #if ENABLED(PCA9632)
    #include "pca9632.h"
  #endif

  #if ENABLED(NEOPIXEL_LED)
    #include "neopixel.h"
  #endif

  LEDLights leds;

  /** Public Parameters */
  #if ENABLED(LED_COLOR_PRESETS)
    const LEDColor LEDLights::defaultLEDColor = MakeLEDColor(
      LED_USER_PRESET_RED,
      LED_USER_PRESET_GREEN,
      LED_USER_PRESET_BLUE,
      LED_USER_PRESET_WHITE,
      LED_USER_PRESET_BRIGHTNESS
    );
  #endif

  #if ENABLED(LED_CONTROL_MENU) || ENABLED(PRINTER_EVENT_LEDS)
    LEDColor LEDLights::color;
    bool LEDLights::lights_on = false;
  #endif

  /** Public Function */
  void LEDLights::setup() {

    #if ENABLED(NEOPIXEL_LED)
      setup_neopixel();
    #endif

    #if ENABLED(RGB_LED) || ENABLED(RGBW_LED)
      SET_OUTPUT(RGB_LED_R_PIN);
      SET_OUTPUT(RGB_LED_G_PIN);
      SET_OUTPUT(RGB_LED_B_PIN);
      #if ENABLED(RGBW_LED)
        SET_OUTPUT(RGB_LED_W_PIN);
      #endif
    #endif

    #if ENABLED(LED_USER_PRESET_STARTUP)
      set_default();
    #endif
  }

  void LEDLights::set_color(const LEDColor &incol
    #if ENABLED(NEOPIXEL_LED)
      , bool isSequence/*=false*/
    #endif
  ) {

    #if ENABLED(NEOPIXEL_LED)

      const uint32_t neocolor = LEDColorWhite() == incol
                              ? strip.Color(NEO_WHITE)
                              : strip.Color(incol.r, incol.g, incol.b, incol.w);
      static uint16_t nextLed = 0;

      strip.setBrightness(incol.i);
      if (!isSequence)
        set_neopixel_color(neocolor);
      else {
        strip.setPixelColor(nextLed, neocolor);
        strip.show();
        if (++nextLed >= strip.numPixels()) nextLed = 0;
        return;
      }

    #endif

    #if ENABLED(BLINKM)

      // This variant uses i2c to send the RGB components to the device.
      blinkm_set_led_color(incol);

    #endif

    #if ENABLED(RGB_LED) || ENABLED(RGBW_LED)

      // This variant uses 3-4 separate pins for the RGB(W) components.
      // If the pins can do PWM then their intensity will be set.
      WRITE(RGB_LED_R_PIN, incol.r ? HIGH : LOW);
      WRITE(RGB_LED_G_PIN, incol.g ? HIGH : LOW);
      WRITE(RGB_LED_B_PIN, incol.b ? HIGH : LOW);
      HAL::analogWrite(RGB_LED_R_PIN, incol.r);
      HAL::analogWrite(RGB_LED_G_PIN, incol.g);
      HAL::analogWrite(RGB_LED_B_PIN, incol.b);

      #if ENABLED(RGBW_LED)
        WRITE(RGB_LED_W_PIN, incol.w ? HIGH : LOW);
        HAL::analogWrite(RGB_LED_W_PIN, incol.w);
      #endif

    #endif

    #if ENABLED(PCA9632)
      // Update I2C LED driver
      pca9632_set_led_color(incol);
    #endif

    #if ENABLED(LED_CONTROL_MENU)
      // Don't update the color when OFF
      lights_on = !incol.is_off();
      if (lights_on) color = incol;
    #endif
  }

  uint8_t LEDLights::getBrightness() {
    #if ENABLED(NEOPIXEL_LED)
      return strip.getBrightness();
    #else
      return 255;
    #endif
  }

  void LEDLights::set_white() {
    #if ENABLED(RGB_LED) || ENABLED(RGBW_LED) || ENABLED(BLINKM) || ENABLED(PCA9632)
      set_color(LEDColorWhite());
    #endif
    #if ENABLED(NEOPIXEL_LED)
      set_neopixel_color(strip.Color(NEO_WHITE));
    #endif
  }

  #if ENABLED(LED_CONTROL_MENU)
    void LEDLights::toggle() { if (lights_on) set_off(); else update(); }
  #endif

#endif // ENABLED(RGB_LED) || ENABLED(RGBW_LED)
