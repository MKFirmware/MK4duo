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
 * led_events.h
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#if ENABLED(PRINTER_EVENT_LEDS)

  class LedEvents {

    public: /** Constructor */

      LedEvents() {}

    private: /** Private Parameters */

      static uint8_t old_intensity;

      #if HAS_LEDS_OFF_FLAG
        static bool leds_off_after_print;
      #endif

    public: /** Public Function */

      static void onHeating(const bool Hotend, const float &start, const float &current, const float &target);

      static inline void set_done() {
        #if ENABLED(LED_COLOR_PRESETS)
          leds.set_default();
        #else
          leds.set_off();
        #endif
      }

      static inline LEDColor onHeatingStart(const bool Hotend) {
        old_intensity = Hotend ? 0 : 127;
        return leds.get_color();
      }

      static inline void onHeatingDone() { leds.set_color(LEDColorWhite()); }
      static inline void onPidTuningDone(LEDColor c) { leds.set_color(c); }

      #if HAS_SD_SUPPORT

        static inline void onPrintCompleted() {
          leds.set_green();
          #if HAS_LEDS_OFF_FLAG
            leds_off_after_print = true;
          #else
            HAL::delayMilliseconds(2000);
            set_done();
          #endif
        }

        static inline void onResumeAfterWait() {
          #if HAS_LEDS_OFF_FLAG
            if (leds_off_after_print) {
              set_done();
              leds_off_after_print = false;
            }
          #endif
        }

      #endif // HAS_SD_SUPPORT

    private: /** Private Function */

      static uint8_t get_intensity(const float &start, const float &current, const float &target);

      static void set_rgb(const uint8_t r, const uint8_t g, const uint8_t b);

  };

  extern LedEvents ledevents;

#endif // PRINTER_EVENT_LEDS
