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

/**
 * babystep.cpp
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"
#include "sanitycheck.h"

#if ENABLED(BABYSTEPPING)

Babystep babystep;

/** Public Parameters */
volatile int16_t Babystep::steps[BS_AXIS_IND(Z_AXIS) + 1];

int16_t Babystep::accum;

#if HAS_LCD_MENU && ENABLED(BABYSTEP_DISPLAY_TOTAL)
  int16_t Babystep::axis_total[BS_TOTAL_IND(Z_AXIS) + 1];
#endif

/** Public Function */
void Babystep::add_mm(const AxisEnum axis, const float &mm) {
  add_steps(axis, mm * mechanics.data.axis_steps_per_mm[axis]);
}

void Babystep::add_steps(const AxisEnum axis, const int16_t distance) {

  if (!mechanics.isAxisHomed(axis)) return;

  accum += distance; // Count up babysteps for the LCDUI

  #if HAS_LCD_MENU && ENABLED(BABYSTEP_DISPLAY_TOTAL)
    axis_total[BS_TOTAL_IND(axis)] += distance;
  #endif

  #define BSA_ENABLE(AXIS) do{ switch (AXIS) { case X_AXIS: stepper.enable_X(); break; case Y_AXIS: stepper.enable_Y(); break; case Z_AXIS: stepper.enable_Z(); break; default: break; } }while(0)

  #if IS_CORE
    #if ENABLED(BABYSTEP_XY)
      switch (axis) {
        case CORE_AXIS_1: // X on CoreXY and CoreXZ, Y on CoreYZ
          BSA_ENABLE(CORE_AXIS_1);
          BSA_ENABLE(CORE_AXIS_2);
          steps[CORE_AXIS_1] += distance * 2;
          steps[CORE_AXIS_2] += distance * 2;
          break;
        case CORE_AXIS_2: // Y on CoreXY, Z on CoreXZ and CoreYZ
          BSA_ENABLE(CORE_AXIS_1);
          BSA_ENABLE(CORE_AXIS_2);
          steps[CORE_AXIS_1] += CORESIGN(distance * 2);
          steps[CORE_AXIS_2] -= CORESIGN(distance * 2);
          break;
        case NORMAL_AXIS: // Z on CoreXY, Y on CoreXZ, X on CoreYZ
        default:
          BSA_ENABLE(NORMAL_AXIS);
          steps[NORMAL_AXIS] += distance;
          break;
      }
    #elif CORE_IS_XZ || CORE_IS_YZ
      // Only Z stepping needs to be handled here
      BSA_ENABLE(CORE_AXIS_1);
      BSA_ENABLE(CORE_AXIS_2);
      steps[CORE_AXIS_1] += CORESIGN(distance * 2);
      steps[CORE_AXIS_2] -= CORESIGN(distance * 2);
    #else
      BSA_ENABLE(Z_AXIS);
      steps[Z_AXIS] += distance;
    #endif
  #else
    #if ENABLED(BABYSTEP_XY)
      BSA_ENABLE(axis);
    #else
      BSA_ENABLE(Z_AXIS);
    #endif
    steps[BS_AXIS_IND(axis)] += distance;
  #endif

}

/** Private Function */
void Babystep::step_axis(const AxisEnum axis) {
  const int16_t curTodo = steps[BS_AXIS_IND(axis)]; // get rid of volatile for performance
  if (curTodo) {
    stepper.do_babystep((AxisEnum)axis, curTodo > 0);
    if (curTodo > 0)  steps[BS_AXIS_IND(axis)]--;
    else              steps[BS_AXIS_IND(axis)]++;
  }
}

#endif // BABYSTEPPING
