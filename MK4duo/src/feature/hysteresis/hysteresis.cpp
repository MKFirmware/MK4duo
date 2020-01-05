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

#include "../../../MK4duo.h"
#include "sanitycheck.h"

#if ENABLED(HYSTERESIS_FEATURE)

Hysteresis hysteresis;

/** Public Parameters */
hysteresis_data_t Hysteresis::data;

/** Public Function */
void Hysteresis::factory_parameters() {
  constexpr float tmp[] = HYSTERESIS_AXIS_MM;
  LOOP_XYZ(i) data.mm[i] = tmp[i];
  data.correction = HYSTERESIS_CORRECTION;
}

void Hysteresis::add_correction_step(block_t * const block) {

  static uint8_t last_direction_bits = 0;
  uint8_t direction_change_bits = last_direction_bits ^ block->direction_bits;

  LOOP_XYZ(axis)
    if (!block->steps[axis]) CBI(direction_change_bits, axis);

  last_direction_bits ^= direction_change_bits;

  if (data.correction == 0.0f || !direction_change_bits) return;

  LOOP_XYZ(axis) {
    if (data.mm[axis]) {
      // When an axis changes direction, add axis hysteresis
      if (TEST(direction_change_bits, axis)) {
        const uint32_t fix = data.correction * data.mm[axis] * mechanics.data.axis_steps_per_mm[axis];
        block->steps[axis] += fix;
      }
    }
  }
}

void Hysteresis::print_M99() {
  SERIAL_LM(CFG, "Hysteresis Correction");
  SERIAL_SMV(CFG, "  M99 X", data.mm[X_AXIS]);
  SERIAL_MV(" Y", data.mm[Y_AXIS]);
  SERIAL_MV(" Z", data.mm[Z_AXIS]);
  SERIAL_MV(" F", data.correction);
  SERIAL_EOL();
}

#endif // ENABLED(HYSTERESIS_FEATURE)
