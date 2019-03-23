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
 * filrunout.cpp
 *
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"

#if HAS_FILAMENT_SENSOR

FilamentRunout filamentrunout;

/** Public Parameters */
flagfilament_t  FilamentRunoutBase::flag;

flagbyte_t      FilamentSensorBase::logic_flag,
                FilamentSensorBase::pullup_flag;

#if FILAMENT_RUNOUT_DISTANCE_MM > 0
  float RunoutResponseDelayed::runout_distance_mm = FILAMENT_RUNOUT_DISTANCE_MM;
  volatile float RunoutResponseDelayed::runout_mm_countdown[EXTRUDERS] = { 0 };
#endif

/** Private Parameters */
#if ENABLED(EXTRUDER_ENCODER_CONTROL)
  uint8_t FilamentSensorEncoder::motion_detected = 0;
#endif

#if FILAMENT_RUNOUT_DISTANCE_MM == 0
  int8_t RunoutResponseDebounced::runout_count = 0;
#endif

/** Public Function */
void FilamentSensorBase::factory_parameters() {
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

void FilamentSensorBase::setup_pullup() {
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

void FilamentSensorBase::report() {
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

void FilamentSensorBase::init() {
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

/**
 * Called by FilamentSensorSwitch::run when filament is detected.
 * Called by FilamentSensorEncoder::block_completed when motion is detected.
 */
void FilamentSensorBase::filament_present(const uint8_t extruder) {
  filamentrunout.filament_present(extruder);
}

#if FILAMENT_RUNOUT_DISTANCE_MM > 0
  void RunoutResponseDelayed::block_completed(const block_t* const b) {
    if (b->steps[X_AXIS] || b->steps[Y_AXIS] || b->steps[Z_AXIS]
      #if ENABLED(ADVANCED_PAUSE_FEATURE)
        || advancedpause.did_pause_print // Allow pause purge move to re-trigger runout state
      #endif
    ) {
      // Only trigger on extrusion with XYZ movement to allow filament change and retract/recover.
      const uint8_t e = b->active_extruder;
      const int32_t steps = b->steps[E_AXIS];
      runout_mm_countdown[e] -= (TEST(b->direction_bits, E_AXIS) ? -steps : steps) * mechanics.steps_to_mm[E_AXIS_N(e)];
    }
  }
#endif

#endif // HAS_FILAMENT_SENSOR
