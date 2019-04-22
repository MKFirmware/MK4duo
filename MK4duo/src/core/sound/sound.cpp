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

#include "../../../MK4duo.h"

Sound sound;

/** Public Parameters */
SoundModeEnum Sound::mode = SOUND_MODE_ON;

/** Private Parameters */
millis_s Sound::tone_ms;

/** Protected Parameters */
Circular_Queue<tone_t, TONE_QUEUE_LENGTH> Sound::buffer;

/** Public Function */
void Sound::playtone(const uint16_t duration, const uint16_t frequency/*=0*/) {
  if (mode == SOUND_MODE_MUTE) return;
  while (buffer.isFull()) printer.idle(true);
  tone_t tone = { duration, frequency };
  buffer.enqueue(tone);
}

void Sound::spin() {

  static tone_t tone = { 0, 0 };

  if (!tone_ms) {
    if (buffer.isEmpty()) return;

    tone = buffer.dequeue();
    tone_ms = millis();

    if (tone.frequency > 0) {
      #if ENABLED(LCD_USE_I2C_BUZZER)
        lcd.buzz(tone.duration, tone.frequency);
      #elif ENABLED(SPEAKER)
        CRITICAL_SECTION_START
          ::tone(BEEPER_PIN, tone.frequency, tone.duration);
        CRITICAL_SECTION_END
      #elif PIN_EXISTS(BEEPER)
        on();
      #endif
    }
  }
  else if (expired(&tone_ms, tone.duration)) reset();

}

void Sound::cyclestate() {
  switch (mode) {
    case SOUND_MODE_ON:
      mode = SOUND_MODE_SILENT;
      break;
    case SOUND_MODE_SILENT:
      mode = SOUND_MODE_MUTE;
      break;
    case SOUND_MODE_MUTE:
      mode = SOUND_MODE_ON;
      break;
    default:
      mode = SOUND_MODE_ON;
  }
}

void Sound::feedback(const bool good/*=true*/) {
  if (mode != SOUND_MODE_ON) return;

  if (good) {
    playtone(100, NOTE_E5);
    playtone(100, NOTE_F5);
  }
  else {
    playtone(250, NOTE_G4);
    playtone(500, NOTE_C4);
  }
}
