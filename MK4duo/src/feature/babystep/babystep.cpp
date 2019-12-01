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

/**
 * babystep.cpp
 *
 * Copyright (c) 2019 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"
#include "sanitycheck.h"

#if ENABLED(BABYSTEPPING)

Babystep babystep;

/** Public Parameters */
volatile int16_t Babystep::steps[BS_TODO_AXIS(Z_AXIS) + 1];

#if HAS_LCD_MENU
  int16_t Babystep::accum;
  #if ENABLED(BABYSTEP_DISPLAY_TOTAL)
    int16_t Babystep::axis_total[BS_TOTAL_AXIS(Z_AXIS) + 1];
  #endif
#endif

/** Public Function */
void Babystep::spin() {
  #if ENABLED(BABYSTEP_XY)
    LOOP_XYZ(axis) step_axis((AxisEnum)axis);
  #else
    step_axis(Z_AXIS);
  #endif
}

void Babystep::add_mm(const AxisEnum axis, const float &mm) {
  add_steps(axis, mm * mechanics.data.axis_steps_per_mm[axis]);
}

void Babystep::add_steps(const AxisEnum axis, const int16_t distance) {

  if (!mechanics.isAxisHomed(axis)) return;

  #if HAS_LCD_MENU
    accum += distance; // Count up babysteps for the LCDUI
    #if ENABLED(BABYSTEP_DISPLAY_TOTAL)
      axis_total[BS_TOTAL_AXIS(axis)] += distance;
    #endif
  #endif

  #if IS_CORE
    #if ENABLED(BABYSTEP_XY)
      switch (axis) {
        case CORE_AXIS_1: // X on CoreXY and CoreXZ, Y on CoreYZ
          steps[CORE_AXIS_1] += distance * 2;
          steps[CORE_AXIS_2] += distance * 2;
          break;
        case CORE_AXIS_2: // Y on CoreXY, Z on CoreXZ and CoreYZ
          steps[CORE_AXIS_1] += CORESIGN(distance * 2);
          steps[CORE_AXIS_2] -= CORESIGN(distance * 2);
          break;
        case NORMAL_AXIS: // Z on CoreXY, Y on CoreXZ, X on CoreYZ
        default:
          steps[NORMAL_AXIS] += distance;
          break;
      }
    #elif CORE_IS_XZ || CORE_IS_YZ
      // Only Z stepping needs to be handled here
      steps[CORE_AXIS_1] += CORESIGN(distance * 2);
      steps[CORE_AXIS_2] -= CORESIGN(distance * 2);
    #else
      steps[Z_AXIS] += distance;
    #endif
  #else
    steps[BS_TODO_AXIS(axis)] += distance;
  #endif

}

/** Private Function */
void Babystep::step_axis(const AxisEnum axis) {
  const int16_t curTodo = steps[BS_TODO_AXIS(axis)]; // get rid of volatile for performance
  if (curTodo) {
    stepper.babystep((AxisEnum)axis, curTodo > 0);
    if (curTodo > 0)  steps[BS_TODO_AXIS(axis)]--;
    else              steps[BS_TODO_AXIS(axis)]++;
  }
}

#endif // BABYSTEPPING
