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
 * filrunout.cpp
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"

#if HAS_FIL_RUNOUT

  FilamentRunOut filamentrunout;

  void FilamentRunOut::init() {
    SET_INPUT(FIL_RUNOUT0_PIN);
    #if HAS_FIL_RUNOUT1
      SET_INPUT(FIL_RUNOUT1_PIN);
      #if HAS_FIL_RUNOUT2
        SET_INPUT(FIL_RUNOUT2_PIN);
        #if HAS_FIL_RUNOUT3
          SET_INPUT(FIL_RUNOUT3_PIN);
          #if HAS_FIL_RUNOUT4
            SET_INPUT(FIL_RUNOUT4_PIN);
            #if HAS_FIL_RUNOUT5
              SET_INPUT(FIL_RUNOUT5_PIN);
            #endif
          #endif
        #endif
      #endif
    #endif
  }

  void FilamentRunOut::setup_pullup(const bool onoff) {
    HAL::setInputPullup(FIL_RUNOUT0_PIN, onoff);
    #if HAS_FIL_RUNOUT1
      HAL::setInputPullup(FIL_RUNOUT1_PIN, onoff);
      #if HAS_FIL_RUNOUT2
        HAL::setInputPullup(FIL_RUNOUT2_PIN, onoff);
        #if HAS_FIL_RUNOUT3
          HAL::setInputPullup(FIL_RUNOUT3_PIN, onoff);
          #if HAS_FIL_RUNOUT4
            HAL::setInputPullup(FIL_RUNOUT4_PIN, onoff);
            #if HAS_FIL_RUNOUT5
              HAL::setInputPullup(FIL_RUNOUT5_PIN, onoff);
            #endif
          #endif
        #endif
      #endif
    #endif
  }

  void FilamentRunOut::spin() {

    #if FILAMENT_RUNOUT_DOUBLE_CHECK > 0
      static bool filament_double_check = false;
      static millis_t filament_switch_time = 0;
      if ((IS_SD_PRINTING || print_job_counter.isRunning()) && read()) {
        if (filament_double_check) {
          if (ELAPSED(millis(), filament_switch_time)) {
            printer.setInterruptEvent(INTERRUPT_EVENT_FIL_RUNOUT);
            filament_double_check = false;
          }
        }
        else {
          filament_double_check = true;
          filament_switch_time = millis() + FILAMENT_RUNOUT_DOUBLE_CHECK;
        }
      }
    #else
      if ((IS_SD_PRINTING || print_job_counter.isRunning()) && read())
        printer.setInterruptEvent(INTERRUPT_EVENT_FIL_RUNOUT);
    #endif

  }

#endif // HAS_FIL_RUNOUT