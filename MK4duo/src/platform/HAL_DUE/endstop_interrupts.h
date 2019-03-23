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
 * Endstop Interrupts
 *
 * Without endstop interrupts the endstop pins must be polled continually in
 * the stepper-ISR via endstop_ISR(), most of the time finding no change.
 * With this feature endstop_ISR() is called only when we know that at
 * least one endstop has changed state, saving valuable CPU cycles.
 *
 * This feature only works when all used endstop pins can generate an 'external interrupt'.
 */

/**
 *  Endstop interrupts for Due based targets.
 *  On Due, all pins support external interrupt capability.
 */

void Endstops::setup_interrupts(void) {

  #if HAS_X_MAX
    attachInterrupt(digitalPinToInterrupt(X_MAX_PIN), endstop_ISR, CHANGE); // assign it
  #endif

  #if HAS_X_MIN
    attachInterrupt(digitalPinToInterrupt(X_MIN_PIN), endstop_ISR, CHANGE); // assign it
  #endif

  #if HAS_Y_MAX
    attachInterrupt(digitalPinToInterrupt(Y_MAX_PIN), endstop_ISR, CHANGE); // assign it
  #endif

  #if HAS_Y_MIN
    attachInterrupt(digitalPinToInterrupt(Y_MIN_PIN), endstop_ISR, CHANGE); // assign it
  #endif

  #if HAS_Z_MAX
    attachInterrupt(digitalPinToInterrupt(Z_MAX_PIN), endstop_ISR, CHANGE); // assign it
  #endif

  #if HAS_Z_MIN
    attachInterrupt(digitalPinToInterrupt(Z_MIN_PIN), endstop_ISR, CHANGE); // assign it
  #endif

  #if HAS_Z2_MAX
    attachInterrupt(digitalPinToInterrupt(Z2_MAX_PIN), endstop_ISR, CHANGE); // assign it
  #endif

  #if HAS_Z2_MIN
    attachInterrupt(digitalPinToInterrupt(Z2_MIN_PIN), endstop_ISR, CHANGE); // assign it
  #endif

  #if HAS_Z3_MAX
    attachInterrupt(digitalPinToInterrupt(Z3_MAX_PIN), endstop_ISR, CHANGE); // assign it
  #endif

  #if HAS_Z3_MIN
    attachInterrupt(digitalPinToInterrupt(Z3_MIN_PIN), endstop_ISR, CHANGE); // assign it
  #endif

  #if HAS_Z_PROBE_PIN
    attachInterrupt(digitalPinToInterrupt(Z_PROBE_PIN), endstop_ISR, CHANGE); // assign it
  #endif
}
