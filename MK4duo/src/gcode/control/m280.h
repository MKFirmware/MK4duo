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
 * mcode
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#if HAS_SERVOS

  #define CODE_M280

  /**
   * M280: Get or set servo position. P<index> S<angle>
   */
  inline void gcode_M280(void) {
    if (!parser.seen('P')) return;
    const int servo_index = parser.value_int();

    #if HAS_DONDOLO
      int servo_position = 0;
      if (parser.seenval('S')) {
        servo_position = parser.value_int();
        if (servo_index >= 0 && servo_index < NUM_SERVOS && servo_index != DONDOLO_SERVO_INDEX) {
          MOVE_SERVO(servo_index, servo_position);
        }
        else if (servo_index == DONDOLO_SERVO_INDEX) {
          Servo *srv = &servo[servo_index];
          srv->attach(0);
          srv->write(servo_position);
          #if (DONDOLO_SERVO_DELAY > 0)
            printer.safe_delay(DONDOLO_SERVO_DELAY);
            srv->detach();
          #endif
        }
        else {
          SERIAL_SMV(ER, "Servo ", servo_index);
          SERIAL_EM(" out of range");
        }
      }
    #else
      if (WITHIN(servo_index, 0, NUM_SERVOS - 1)) {
        if (parser.seenval('S'))
          MOVE_SERVO(servo_index, parser.value_int());
        else {
          SERIAL_SMV(ECHO, " Servo ", servo_index);
          SERIAL_EMV(": ", servo[servo_index].read());
        }
      }
      else {
        SERIAL_SMV(ER, "Servo ", servo_index);
        SERIAL_EM(" out of range");
      }
    #endif
  }

#endif // NUM_SERVOS > 0
