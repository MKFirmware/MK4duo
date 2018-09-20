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

#include "../../../MK4duo.h"

#if DISABLED(LCD_USE_I2C_BUZZER) && PIN_EXISTS(BEEPER)

  Circular_Queue<tone_t, TONE_QUEUE_LENGTH> Buzzer::buffer;
  Buzzer buzzer;

  void Buzzer::playTone(const uint16_t duration, const uint16_t frequency/*=0*/) {
    while (buffer.isFull()) printer.idle(true);
    tone_t tone = { duration, frequency };
    buffer.enqueue(tone);
  }

  void Buzzer::tick() {
    static tone_t tone = { 0, 0 };
    static watch_t tone_watch;

    if (tone_watch.elapsed(tone.duration)) {

      #if ENABLED(SPEAKER)
        CRITICAL_SECTION_START
          ::noTone(BEEPER_PIN);
        CRITICAL_SECTION_END
      #else
        off();
      #endif

      tone_watch.start();

      if (buffer.isEmpty()) return;

      tone = buffer.dequeue();

      if (tone.frequency > 0) {
        #if ENABLED(SPEAKER)
          CRITICAL_SECTION_START
            ::tone(BEEPER_PIN, tone.frequency, tone.duration);
          CRITICAL_SECTION_END
        #else
          on();
        #endif
      }
    }
  }

#endif // DISABLED(LCD_USE_I2C_BUZZER) && PIN_EXISTS(BEEPER)
