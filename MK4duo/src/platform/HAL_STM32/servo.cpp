/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2019 Alberto Cotronei @MagoKimbra
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

#ifdef ARDUINO_ARCH_STM32 && !defined(STM32GENERIC)

#include "../../../MK4duo.h"

#if HAS_SERVOS

#include "servo.h"

uint8_t servoPin[MAX_SERVOS] = { 0 };

int8_t MKServo::attach(const int pin) {
  if (this->servoIndex >= MAX_SERVOS) return -1;
  if (pin > 0) servoPin[this->servoIndex] = pin;
  return Servo::attach(servoPin[this->servoIndex]);
}

int8_t MKServo::attach(const int pin, const int min, const int max) {
  if (pin > 0) servoPin[this->servoIndex] = pin;
  return Servo::attach(servoPin[this->servoIndex], min, max);
}

void MKServo::move(const int value) {
  constexpr uint16_t servo_delay[] = SERVO_DELAY;
  static_assert(COUNT(servo_delay) == NUM_SERVOS, "SERVO_DELAY must be an array NUM_SERVOS long.");
  if (this->attach(0) >= 0) {
    this->write(value);
    safe_delay(servo_delay[this->servoIndex]);
    #if ENABLED(DEACTIVATE_SERVOS_AFTER_MOVE)
      this->detach();
    #endif
  }
}

void MKServo::print_M281() {
  SERIAL_LM(CFG, "Servo Angles: P<Servo> L<Low> U<Up>:");
  SERIAL_SMV(CFG, "  M281 P", (int)this->servoIndex);
  SERIAL_MV(" L", this->angle[0]);
  SERIAL_MV(" U", this->angle[1]);
  SERIAL_EOL();
}

#endif // HAS_SERVOS

#endif // ARDUINO_ARCH_STM32 && !defined(STM32GENERIC)
