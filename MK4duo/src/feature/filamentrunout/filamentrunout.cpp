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

#if HAS_FIL_RUNOUT_0

  FilamentRunOut filamentrunout;

  // Public Parameters
  flagbyte_t  FilamentRunOut::logic_flag,
              FilamentRunOut::pullup_flag;

  // Public Function
  void FilamentRunOut::init() {
    SET_INPUT(FIL_RUNOUT_0_PIN);
    #if HAS_FIL_RUNOUT_1
      SET_INPUT(FIL_RUNOUT_1_PIN);
      #if HAS_FIL_RUNOUT_2
        SET_INPUT(FIL_RUNOUT_2_PIN);
        #if HAS_FIL_RUNOUT_3
          SET_INPUT(FIL_RUNOUT_3_PIN);
          #if HAS_FIL_RUNOUT_4
            SET_INPUT(FIL_RUNOUT_4_PIN);
            #if HAS_FIL_RUNOUT_5
              SET_INPUT(FIL_RUNOUT_5_PIN);
            #endif
          #endif
        #endif
      #endif
    #endif
  }

  void FilamentRunOut::factory_parameters() {
    setLogic(FIL_RUNOUT_0, FIL_RUNOUT_0_LOGIC);
    setPullup(FIL_RUNOUT_0, FIL_RUNOUT_0_PULLUP);
    #if HAS_FIL_RUNOUT_1
      setLogic(FIL_RUNOUT_1, FIL_RUNOUT_1_LOGIC);
      setPullup(FIL_RUNOUT_1, FIL_RUNOUT_1_PULLUP);
      #if HAS_FIL_RUNOUT_2
        setLogic(FIL_RUNOUT_2, FIL_RUNOUT_2_LOGIC);
        setPullup(FIL_RUNOUT_2, FIL_RUNOUT_2_PULLUP);
        #if HAS_FIL_RUNOUT_3
          setLogic(FIL_RUNOUT_3, FIL_RUNOUT_3_LOGIC);
          setPullup(FIL_RUNOUT_3, FIL_RUNOUT_3_PULLUP);
          #if HAS_FIL_RUNOUT_4
            setLogic(FIL_RUNOUT_4, FIL_RUNOUT_4_LOGIC);
            setPullup(FIL_RUNOUT_4, FIL_RUNOUT_4_PULLUP);
            #if HAS_FIL_RUNOUT_5
              setLogic(FIL_RUNOUT_5, FIL_RUNOUT_5_LOGIC);
              setPullup(FIL_RUNOUT_5, FIL_RUNOUT_5_PULLUP);
            #endif
          #endif
        #endif
      #endif
    #endif
  }

  void FilamentRunOut::setup_pullup() {
    HAL::setInputPullup(FIL_RUNOUT_0_PIN, isPullup(FIL_RUNOUT_0));
    #if HAS_FIL_RUNOUT_1
      HAL::setInputPullup(FIL_RUNOUT_1_PIN, isPullup(FIL_RUNOUT_1));
      #if HAS_FIL_RUNOUT_2
        HAL::setInputPullup(FIL_RUNOUT_2_PIN, isPullup(FIL_RUNOUT_2));
        #if HAS_FIL_RUNOUT_3
          HAL::setInputPullup(FIL_RUNOUT_3_PIN, isPullup(FIL_RUNOUT_3));
          #if HAS_FIL_RUNOUT_4
            HAL::setInputPullup(FIL_RUNOUT_4_PIN, isPullup(FIL_RUNOUT_4));
            #if HAS_FIL_RUNOUT_5
              HAL::setInputPullup(FIL_RUNOUT_5_PIN, isPullup(FIL_RUNOUT_5));
            #endif
          #endif
        #endif
      #endif
    #endif
  }

  void FilamentRunOut::report() {
    SERIAL_LOGIC("FIL_RUNOUT_0 Logic", isLogic(FIL_RUNOUT_0));
    SERIAL_EMT(" Pullup", isPullup(FIL_RUNOUT_0));
    #if HAS_FIL_RUNOUT_1
      SERIAL_MT("FIL_RUNOUT_1 Logic", isLogic(FIL_RUNOUT_1));
      SERIAL_EMT(" Pullup", isPullup(FIL_RUNOUT_1));
      #if HAS_FIL_RUNOUT_2
        SERIAL_MT("FIL_RUNOUT_2 Logic", isLogic(FIL_RUNOUT_2));
        SERIAL_EMT(" Pullup", isPullup(FIL_RUNOUT_2));
        #if HAS_FIL_RUNOUT_3
          SERIAL_MT("FIL_RUNOUT_3 Logic", isLogic(FIL_RUNOUT_3));
          SERIAL_EMT(" Pullup", isPullup(FIL_RUNOUT_3));
          #if HAS_FIL_RUNOUT_4
            SERIAL_MT("FIL_RUNOUT_4 Logic", isLogic(FIL_RUNOUT_4));
            SERIAL_EMT(" Pullup", isPullup(FIL_RUNOUT_4));
            #if HAS_FIL_RUNOUT_5
              SERIAL_MT("FIL_RUNOUT_5 Logic", isLogic(FIL_RUNOUT_5));
              SERIAL_EMT(" Pullup:", isPullup(FIL_RUNOUT_5));
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
      if (printer.isPrinting() && read()) {
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
      else
        filament_double_check = false;
    #else
      if (printer.isPrinting() && read())
        printer.setInterruptEvent(INTERRUPT_EVENT_FIL_RUNOUT);
    #endif

  }

  // Private Function
  static bool FilamentRunOut::read() {

    #if (PIN_EXISTS(FIL_RUNOUT_1))

      // Read the sensor for the active extruder
      #if ENABLED(DUAL_X_CARRIAGE)
        bool is_out;
        const bool out1 = READ(FIL_RUNOUT_0_PIN) ^ isLogic(FIL_RUNOUT_0),
                   out2 = READ(FIL_RUNOUT_1_PIN) ^ isLogic(FIL_RUNOUT_1);
        if (mechanics.extruder_duplication_enabled)
          is_out = out1 || out2;
        else
          is_out = tools.active_extruder ? out2 : out1;
        return is_out;
      #else
        switch (tools.active_extruder) {
          case 0: return READ(FIL_RUNOUT_0_PIN) ^ isLogic(FIL_RUNOUT_0);
          case 1: return READ(FIL_RUNOUT_1_PIN) ^ isLogic(FIL_RUNOUT_1);
          #if HAS_FIL_RUNOUT_2
            case 2: return READ(FIL_RUNOUT_2_PIN) ^ isLogic(FIL_RUNOUT_2);
            #if HAS_FIL_RUNOUT_3
              case 3: return READ(FIL_RUNOUT_3_PIN) ^ isLogic(FIL_RUNOUT_3);
              #if HAS_FIL_RUNOUT_4
                case 4: return READ(FIL_RUNOUT_4_PIN) ^ isLogic(FIL_RUNOUT_4);
                #if HAS_FIL_RUNOUT_5
                  case 5: return READ(FIL_RUNOUT_5_PIN) ^ isLogic(FIL_RUNOUT_5);
                #endif
              #endif
            #endif
          #endif
          default: return false;
        }
      #endif

    #else

      // A single sensor applying to all extruders
      return READ(FIL_RUNOUT_0_PIN) ^ isLogic(FIL_RUNOUT_0);

    #endif

  }
      
#endif // HAS_FIL_RUNOUT_0