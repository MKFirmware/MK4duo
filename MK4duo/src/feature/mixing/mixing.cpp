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
 * mixing.cpp
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"

#if ENABLED(COLOR_MIXING_EXTRUDER)

  Mixer mixer;

  // Used up to Planner level
  uint_fast8_t  Mixer::selected_v_tool = 0;
  float         Mixer::M163_collector[MIXING_STEPPERS]; // mix proportion. 0.0 = off, otherwise <= COLOR_A_MASK.
  mixer_color_t Mixer::color[MIXING_VIRTUAL_TOOLS][MIXING_STEPPERS];

  // Used in Stepper
  int_fast8_t   Mixer::runner = 0;
  mixer_color_t Mixer::s_color[MIXING_STEPPERS];
  mixer_accu_t  Mixer::accu[MIXING_STEPPERS] = { 0 };

  void Mixer::normalize(const uint8_t tool_index) {
    float cmax = 0;

    MIXER_STEPPER_LOOP(i)
      cmax = max(cmax, M163_collector[i]);

    // Scale all values so their maximum is COLOR_A_MASK
    const float inverse_max = RECIPROCAL(cmax);
    MIXER_STEPPER_LOOP(i)
      color[tool_index][i] = M163_collector[i] * COLOR_A_MASK * inverse_max;

  }

  // called at boot
  void Mixer::init(void) {
    // Virtual Tools 0, 1, 2, 3 = Filament 1, 2, 3, 4, etc.
    // Every virtual tool gets a pure filament
    for (uint8_t t = 0; t < MIXING_VIRTUAL_TOOLS && t < MIXING_STEPPERS; t++)
      MIXER_STEPPER_LOOP(i)
        color[t][i] = (t == i) ? COLOR_A_MASK : 0;

    // Remaining virtual tools are 100% filament 1
    #if MIXING_STEPPERS < MIXING_VIRTUAL_TOOLS
      for (uint8_t t = MIXING_STEPPERS; t < MIXING_VIRTUAL_TOOLS; t++)
        MIXER_STEPPER_LOOP(i)
          color[t][i] = (i == 0) ? COLOR_A_MASK : 0;
    #endif

    ZERO(M163_collector);
  }

#endif // ENABLED(COLOR_MIXING_EXTRUDER)
