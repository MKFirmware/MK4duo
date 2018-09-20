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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef __BUZZER_H__
#define __BUZZER_H__

#if ENABLED(LCD_USE_I2C_BUZZER)

  #define BUZZ(duration,freq) lcd.buzz(duration,freq)

#elif PIN_EXISTS(BEEPER)

  #define TONE_QUEUE_LENGTH 4

  struct tone_t {
    uint16_t duration;
    uint16_t frequency;
  };

  class Buzzer {

    public: /** Constructor */

      Buzzer() {
        SET_OUTPUT(BEEPER_PIN);
        off();
      }

    protected: /** Protected Parameters */

      static Circular_Queue<tone_t, TONE_QUEUE_LENGTH> buffer;

      FORCE_INLINE static void off()  { WRITE(BEEPER_PIN, LOW);   }
      FORCE_INLINE static void on()   { WRITE(BEEPER_PIN, HIGH);  }

    public: /** Public Function */

      static void playTone(const uint16_t duration, const uint16_t freq);
      static void tick();

  };

  extern Buzzer buzzer;

  #define BUZZ(duration,freq) buzzer.playTone(duration,freq)

#else // No buzz capability

  #define BUZZ(duration,freq) NOOP

#endif

#endif // __BUZZER_H__
