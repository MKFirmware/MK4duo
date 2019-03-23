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
 * sanitycheck.h
 *
 * Test configuration values for errors at compile-time.
 */

#ifndef _RGBLED_SANITYCHECK_H_
#define _RGBLED_SANITYCHECK_H_

// RGB_LED Requirements
#define _RGB_TEST (PIN_EXISTS(RGB_LED_R) && PIN_EXISTS(RGB_LED_G) && PIN_EXISTS(RGB_LED_B))

#if ENABLED(RGB_LED)
  #if !_RGB_TEST
    #error "DEPENDENCY ERROR: RGB_LED requires RGB_LED_R_PIN, RGB_LED_G_PIN, and RGB_LED_B_PIN."
  #elif ENABLED(RGBW_LED)
    #error "DEPENDENCY ERROR: Please enable only one of RGB_LED and RGBW_LED."
  #endif
#elif ENABLED(RGBW_LED)
  #if !(_RGB_TEST && PIN_EXISTS(RGB_LED_W))
    #error "DEPENDENCY ERROR: RGBW_LED requires RGB_LED_R_PIN, RGB_LED_G_PIN, RGB_LED_B_PIN, and RGB_LED_W_PIN."
  #endif
#elif ENABLED(NEOPIXEL_LED)
  #if !(PIN_EXISTS(NEOPIXEL) && NEOPIXEL_PIXELS > 0)
    #error "DEPENDENCY ERROR: NEOPIXEL_LED requires NEOPIXEL_PIN and NEOPIXEL_PIXELS."
  #endif
#elif ENABLED(PRINTER_EVENT_LEDS) && DISABLED(BLINKM) && DISABLED(PCA9632) && DISABLED(NEOPIXEL_LED)
  #error "DEPENDENCY ERROR: PRINTER_EVENT_LEDS requires BLINKM, PCA9632, RGB_LED, RGBW_LED or NEOPIXEL_LED."
#endif

#if ENABLED(LED_CONTROL_MENU) && DISABLED(ULTRA_LCD)
  #error "DEPENDENCY ERROR: LED_CONTROL_MENU requires an LCD controller."
#endif

#if ENABLED(CASE_LIGHT_USE_NEOPIXEL) && DISABLED(NEOPIXEL_LED)
  #error "DEPENDENCY ERROR: CASE_LIGHT_USE_NEOPIXEL requires NEOPIXEL_LED."
#endif

#endif /* _RGBLED_SANITYCHECK_H_ */
