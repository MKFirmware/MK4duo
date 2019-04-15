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

#if ENABLED(HYSTERESIS_FEATURE)

Hysteresis hysteresis;

/** Public Parameters */
float Hysteresis::mm[XYZ]     = { 0.0 },
      Hysteresis::correction  = 0.0;

/** Public Function */
void Hysteresis::factory_parameters() {
  static const float tmp[] PROGMEM = HYSTERESIS_AXIS_MM;
  LOOP_XYZ(i) mm[i] = pgm_read_float(&tmp[i]);
  correction  = HYSTERESIS_CORRECTION;
}

void Hysteresis::add_correction_step(block_t * const block) {

  static uint8_t last_direction_bits = 0;
  uint8_t direction_change_bits = last_direction_bits ^ block->direction_bits;

  LOOP_XYZ(axis)
    if (!block->steps[axis]) CBI(direction_change_bits, axis);

  last_direction_bits ^= direction_change_bits;

  if (correction == 0.0f || !direction_change_bits) return;

  LOOP_XYZ(axis) {
    if (mm[axis]) {
      // When an axis changes direction, add axis hysteresis
      if (TEST(direction_change_bits, axis)) {
        const uint32_t fix = correction * mm[axis] * mechanics.data.axis_steps_per_mm[axis];
        block->steps[axis] += fix;
      }
    }
  }
}

void Hysteresis::print_M99() {
  SERIAL_LM(CFG, "Hysteresis Correction");
  SERIAL_SMV(CFG, "  M99 X", mm[X_AXIS]);
  SERIAL_MV(" Y", mm[Y_AXIS]);
  SERIAL_MV(" Z", mm[Z_AXIS]);
  SERIAL_MV(" F", correction);
  SERIAL_EOL();
}

#endif // ENABLED(HYSTERESIS_FEATURE)
