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
 * endstops.cpp - A singleton object to manage endstops hardware and software
 */

#include "../../../MK4duo.h"
#include "sanitycheck.h"
#include "../../platform/common/endstop_interrupts.h"

#define _ENDSTOP(AXIS, MINMAX)      AXIS ##_## MINMAX
#define _ENDSTOP_PIN(AXIS, MINMAX)  AXIS ##_## MINMAX ##_PIN

Endstops endstops;

/** Public Parameters */
endstop_data_t  Endstops::data;
endstop_flag_t  Endstops::flag;

#if MECH(DELTA)
  float Endstops::soft_endstop_radius_2 = 0.0;
#else
  xyz_limit_float_t Endstops::soft_endstop{0};
#endif

uint16_t Endstops::live_state = 0;

#if ENABLED(SPI_ENDSTOPS)
  tmc_spi_flag_t Endstops::tmc_spi_homing;
#endif

/** Private Parameters */
volatile uint8_t Endstops::hit_state = 0;

/** Public Function */
void Endstops::init() {

  #if HAS_X_MIN
    SET_INPUT(X_MIN_PIN);
  #endif
  #if HAS_X2_MIN
    SET_INPUT(X2_MIN_PIN);
  #endif

  #if HAS_Y_MIN
    SET_INPUT(Y_MIN_PIN);
  #endif
  #if HAS_Y2_MIN
    SET_INPUT(Y2_MIN_PIN);
  #endif

  #if HAS_Z_MIN
    SET_INPUT(Z_MIN_PIN);
  #endif
  #if HAS_Z2_MIN
    SET_INPUT(Z2_MIN_PIN);
  #endif
  #if HAS_Z3_MIN
    SET_INPUT(Z3_MIN_PIN);
  #endif

  #if HAS_X_MAX
    SET_INPUT(X_MAX_PIN);
  #endif
  #if HAS_X2_MAX
    SET_INPUT(X2_MAX_PIN);
  #endif

  #if HAS_Y_MAX
    SET_INPUT(Y_MAX_PIN);
  #endif
  #if HAS_Y2_MAX
    SET_INPUT(Y2_MAX_PIN);
  #endif

  #if HAS_Z_MAX
    SET_INPUT(Z_MAX_PIN);
  #endif
  #if HAS_Z2_MAX
    SET_INPUT(Z2_MAX_PIN);
  #endif
  #if HAS_Z3_MAX
    SET_INPUT(Z3_MAX_PIN);
  #endif

  #if HAS_Z_PROBE_PIN
    SET_INPUT(Z_PROBE_PIN);
  #endif

  #if HAS_DOOR_OPEN
    SET_INPUT(DOOR_OPEN_PIN);
  #endif

  #if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
    setup_interrupts();
  #endif

  setGlobally(
    #if ENABLED(ENDSTOPS_ONLY_FOR_HOMING)
      false
    #else
      true
    #endif
  );

}

void Endstops::factory_parameters() {

  setGlobally(
    #if ENABLED(ENDSTOPS_ONLY_FOR_HOMING)
      false
    #else
      true
    #endif
  );

  #if NOMECH(DELTA)
    soft_endstop.min = mechanics.data.base_pos.min;
    soft_endstop.max = mechanics.data.base_pos.max;
  #endif

  #if ENABLED(X_TWO_ENDSTOPS)
    data.x2_endstop_adj = 0.0f;
  #endif
  #if ENABLED(Y_TWO_ENDSTOPS)
    data.y2_endstop_adj = 0.0f;
  #endif
  #if ENABLED(Z_THREE_ENDSTOPS)
    data.z2_endstop_adj = 0.0f;
    data.z3_endstop_adj = 0.0f;
  #elif ENABLED(Z_TWO_ENDSTOPS)
    data.z2_endstop_adj = 0.0f;
  #endif

  #if MB(ALLIGATOR_R2) || MB(ALLIGATOR_R3)

    setLogic(X_MIN, !X_MIN_ENDSTOP_LOGIC);
    setLogic(Y_MIN, !Y_MIN_ENDSTOP_LOGIC);
    setLogic(Z_MIN, !Z_MIN_ENDSTOP_LOGIC);
    setLogic(X_MAX, !X_MAX_ENDSTOP_LOGIC);
    setLogic(Y_MAX, !Y_MAX_ENDSTOP_LOGIC);
    setLogic(Z_MAX, !Z_MAX_ENDSTOP_LOGIC);
    setLogic(X2_MIN, !X2_MIN_ENDSTOP_LOGIC);
    setLogic(Y2_MIN, !Y2_MIN_ENDSTOP_LOGIC);
    setLogic(Z2_MIN, !Z2_MIN_ENDSTOP_LOGIC);
    setLogic(Z3_MIN, !Z3_MIN_ENDSTOP_LOGIC);
    setLogic(X2_MAX, !X2_MAX_ENDSTOP_LOGIC);
    setLogic(Y2_MAX, !Y2_MAX_ENDSTOP_LOGIC);
    setLogic(Z2_MAX, !Z2_MAX_ENDSTOP_LOGIC);
    setLogic(Z3_MAX, !Z3_MAX_ENDSTOP_LOGIC);
    setLogic(Z_PROBE, !Z_PROBE_ENDSTOP_LOGIC);
    setLogic(DOOR_OPEN, !DOOR_OPEN_LOGIC);

  #else

    setLogic(X_MIN, X_MIN_ENDSTOP_LOGIC);
    setLogic(Y_MIN, Y_MIN_ENDSTOP_LOGIC);
    setLogic(Z_MIN, Z_MIN_ENDSTOP_LOGIC);
    setLogic(X_MAX, X_MAX_ENDSTOP_LOGIC);
    setLogic(Y_MAX, Y_MAX_ENDSTOP_LOGIC);
    setLogic(Z_MAX, Z_MAX_ENDSTOP_LOGIC);
    setLogic(X2_MIN, X2_MIN_ENDSTOP_LOGIC);
    setLogic(Y2_MIN, Y2_MIN_ENDSTOP_LOGIC);
    setLogic(Z2_MIN, Z2_MIN_ENDSTOP_LOGIC);
    setLogic(Z3_MIN, Z3_MIN_ENDSTOP_LOGIC);
    setLogic(X2_MAX, X2_MAX_ENDSTOP_LOGIC);
    setLogic(Y2_MAX, Y2_MAX_ENDSTOP_LOGIC);
    setLogic(Z2_MAX, Z2_MAX_ENDSTOP_LOGIC);
    setLogic(Z3_MAX, Z3_MAX_ENDSTOP_LOGIC);
    setLogic(Z_PROBE, Z_PROBE_ENDSTOP_LOGIC);
    setLogic(DOOR_OPEN, DOOR_OPEN_LOGIC);

  #endif

  setPullup(X_MIN, ENDSTOPPULLUP_XMIN);
  setPullup(Y_MIN, ENDSTOPPULLUP_YMIN);
  setPullup(Z_MIN, ENDSTOPPULLUP_ZMIN);
  setPullup(X_MAX, ENDSTOPPULLUP_XMAX);
  setPullup(Y_MAX, ENDSTOPPULLUP_YMAX);
  setPullup(Z_MAX, ENDSTOPPULLUP_ZMAX);
  setPullup(X2_MIN, ENDSTOPPULLUP_X2MIN);
  setPullup(Y2_MIN, ENDSTOPPULLUP_Y2MIN);
  setPullup(Z2_MIN, ENDSTOPPULLUP_Z2MIN);
  setPullup(Z3_MIN, ENDSTOPPULLUP_Z3MIN);
  setPullup(X2_MAX, ENDSTOPPULLUP_X2MAX);
  setPullup(Y2_MAX, ENDSTOPPULLUP_Y2MAX);
  setPullup(Z2_MAX, ENDSTOPPULLUP_Z2MAX);
  setPullup(Z3_MAX, ENDSTOPPULLUP_Z3MAX);
  setPullup(Z_PROBE, ENDSTOPPULLUP_ZPROBE);
  setPullup(DOOR_OPEN, PULLUP_DOOR_OPEN);

}

void Endstops::print_parameters() {

  //Endstop logic
  SERIAL_LM(CFG, "Endstops logic:");
  #if HAS_X_MIN
    SERIAL_SMV(CFG, "  M123 X", (int)isLogic(X_MIN));
  #elif HAS_X_MAX
    SERIAL_SMV(CFG, "  M123 X", (int)isLogic(X_MAX));
  #endif

  #if HAS_Y_MIN
    SERIAL_MV(" Y", (int)isLogic(Y_MIN));
  #elif HAS_Y_MAX
    SERIAL_MV(" Y", (int)isLogic(Y_MAX));
  #endif

  #if HAS_Y_MIN
    SERIAL_MV(" Z", (int)isLogic(Z_MIN));
  #elif HAS_Y_MAX
    SERIAL_MV(" Z", (int)isLogic(Z_MAX));
  #endif

  #if HAS_X2_MIN
    SERIAL_MV(" I", (int)isLogic(X2_MIN));
  #elif HAS_X2_MAX
    SERIAL_MV(" I", (int)isLogic(X2_MAX));
  #endif

  #if HAS_Y2_MIN
    SERIAL_MV(" J", (int)isLogic(Y2_MIN));
  #elif HAS_Y2_MAX
    SERIAL_MV(" J", (int)isLogic(Y2_MAX));
  #endif  

  #if HAS_Z2_MIN
    SERIAL_MV(" K", (int)isLogic(Z2_MIN));
  #elif HAS_Z2_MAX
    SERIAL_MV(" K", (int)isLogic(Z2_MAX));
  #endif

  #if HAS_Z3_MIN
    SERIAL_MV(" L", (int)isLogic(Z3_MIN));
  #elif HAS_Z3_MAX
    SERIAL_MV(" L", (int)isLogic(Z3_MAX));
  #endif

  #if HAS_Z_PROBE_PIN
    SERIAL_MV(" P", (int)isLogic(Z_PROBE));
  #endif

  #if HAS_DOOR_OPEN
    SERIAL_MV(" D", (int)isLogic(DOOR_OPEN));
  #endif

  SERIAL_EOL();

  //Endstop pullup
  SERIAL_LM(CFG, "Endstops pullup:");
  #if HAS_X_MIN
    SERIAL_SMV(CFG, "  M124 X", (int)isPullup(X_MIN));
  #elif HAS_X_MAX
    SERIAL_SMV(CFG, "  M124 X", (int)isPullup(X_MAX));
  #endif

  #if HAS_Y_MIN
    SERIAL_MV(" Y", (int)isPullup(Y_MIN));
  #elif HAS_Y_MAX
    SERIAL_MV(" Y", (int)isPullup(Y_MAX));
  #endif

  #if HAS_Y_MIN
    SERIAL_MV(" Z", (int)isPullup(Z_MIN));
  #elif HAS_Y_MAX
    SERIAL_MV(" Z", (int)isPullup(Z_MAX));
  #endif

  #if HAS_X2_MIN
    SERIAL_MV(" I", (int)isPullup(X2_MIN));
  #elif HAS_X2_MAX
    SERIAL_MV(" I", (int)isPullup(X2_MAX));
  #endif

  #if HAS_Y2_MIN
    SERIAL_MV(" J", (int)isPullup(Y2_MIN));
  #elif HAS_Y2_MAX
    SERIAL_MV(" J", (int)isPullup(Y2_MAX));
  #endif  

  #if HAS_Z2_MIN
    SERIAL_MV(" K", (int)isPullup(Z2_MIN));
  #elif HAS_Z2_MAX
    SERIAL_MV(" K", (int)isPullup(Z2_MAX));
  #endif

  #if HAS_Z3_MIN
    SERIAL_MV(" L", (int)isPullup(Z3_MIN));
  #elif HAS_Z3_MAX
    SERIAL_MV(" L", (int)isPullup(Z3_MAX));
  #endif

  #if HAS_Z_PROBE_PIN
    SERIAL_MV(" P", (int)isPullup(Z_PROBE));
  #endif

  #if HAS_DOOR_OPEN
    SERIAL_MV(" D", (int)isPullup(DOOR_OPEN));
  #endif

  SERIAL_EOL();
}

void Endstops::setup_pullup() {

  #if HAS_X_MIN
    HAL::setInputPullup(X_MIN_PIN, isPullup(X_MIN));
  #endif
  #if HAS_X2_MIN
    HAL::setInputPullup(X2_MIN_PIN, isPullup(X2_MIN));
  #endif

  #if HAS_Y_MIN
    HAL::setInputPullup(Y_MIN_PIN, isPullup(Y_MIN));
  #endif
  #if HAS_Y2_MIN
    HAL::setInputPullup(Y2_MIN_PIN, isPullup(Y2_MIN));
  #endif

  #if HAS_Z_MIN
    HAL::setInputPullup(Z_MIN_PIN, isPullup(Z_MIN));
  #endif
  #if HAS_Z2_MIN
    HAL::setInputPullup(Z2_MIN_PIN, isPullup(Z2_MIN));
  #endif
  #if HAS_Z3_MIN
    HAL::setInputPullup(Z3_MIN_PIN, isPullup(Z3_MIN));
  #endif

  #if HAS_X_MAX
    HAL::setInputPullup(X_MAX_PIN, isPullup(X_MAX));
  #endif
  #if HAS_X2_MAX
    HAL::setInputPullup(X2_MAX_PIN, isPullup(X2_MAX));
  #endif

  #if HAS_Y_MAX
    HAL::setInputPullup(Y_MAX_PIN, isPullup(Y_MAX));
  #endif
  #if HAS_Y2_MAX
    HAL::setInputPullup(Y2_MAX_PIN, isPullup(Y2_MAX));
  #endif

  #if HAS_Z_MAX
    HAL::setInputPullup(Z_MAX_PIN, isPullup(Z_MAX));
  #endif
  #if HAS_Z2_MAX
    HAL::setInputPullup(Z2_MAX_PIN, isPullup(Z2_MAX));
  #endif
  #if HAS_Z3_MAX
    HAL::setInputPullup(Z3_MAX_PIN, isPullup(Z2_MAX));
  #endif

  #if HAS_Z_PROBE_PIN
    HAL::setInputPullup(Z_PROBE_PIN, isPullup(Z_PROBE));
  #endif

  #if HAS_DOOR_OPEN
    HAL::setInputPullup(DOOR_OPEN_PIN, isPullup(DOOR_OPEN));
  #endif

}

// Called from HAL::Tick. Check endstop state if required
void Endstops::Tick() {
  #if ENABLED(PINS_DEBUGGING)
    run_monitor();  // report changes in endstop status
  #endif

  #if DISABLED(ENDSTOP_INTERRUPTS_FEATURE)
    update();
  #endif
}

// Update endstops
void Endstops::update() {

  if (!abort_enabled()) return;

  #define UPDATE_ENDSTOP_BIT(AXIS, MINMAX)  SET_BIT_TO(live_state, _ENDSTOP(AXIS, MINMAX), (READ(_ENDSTOP_PIN(AXIS, MINMAX)) != isLogic(AXIS ##_## MINMAX)))
  #define COPY_LIVE_STATE(SRC_BIT, DST_BIT) SET_BIT_TO(live_state, DST_BIT, TEST(live_state, SRC_BIT))

  #if ENABLED(G38_PROBE_TARGET) && HAS_Z_PROBE_PIN && !(CORE_IS_XY || CORE_IS_XZ)
    // If G38 command is active check Z_MIN_PROBE for ALL movement
    if (printer.IsG38Move()) UPDATE_ENDSTOP_BIT(Z, PROBE);
  #endif

  // With Dual X, endstops are only checked in the homing direction for the active extruder
  #if ENABLED(DUAL_X_CARRIAGE)
    #define E0_ACTIVE   stepper.movement_extruder() == 0
    #define X_MIN_TEST  ((X_HOME_DIR < 0 && E0_ACTIVE) || (X2_HOME_DIR < 0 && !E0_ACTIVE))
    #define X_MAX_TEST  ((X_HOME_DIR > 0 && E0_ACTIVE) || (X2_HOME_DIR > 0 && !E0_ACTIVE))
  #else
    #define X_MIN_TEST  true
    #define X_MAX_TEST  true
  #endif

  // Use HEAD for core axes, AXIS for others
  #if CORE_IS_XY || CORE_IS_XZ
    #define X_AXIS_HEAD X_HEAD
  #else
    #define X_AXIS_HEAD X_AXIS
  #endif
  #if CORE_IS_XY || CORE_IS_YZ
    #define Y_AXIS_HEAD Y_HEAD
  #else
    #define Y_AXIS_HEAD Y_AXIS
  #endif
  #if CORE_IS_XZ || CORE_IS_YZ
    #define Z_AXIS_HEAD Z_HEAD
  #else
    #define Z_AXIS_HEAD Z_AXIS
  #endif

  /**
   * Check and update endstops
   */
  #if HAS_X_MIN && !X_SPI_SENSORLESS
    #if ENABLED(X_TWO_ENDSTOPS)
      UPDATE_ENDSTOP_BIT(X, MIN);
      #if HAS_X2_MIN
        UPDATE_ENDSTOP_BIT(X2, MIN);
      #else
        COPY_LIVE_STATE(X_MIN, X2_MIN);
      #endif
    #else
      UPDATE_ENDSTOP_BIT(X, MIN);
    #endif
  #endif

  #if HAS_X_MAX && !X_SPI_SENSORLESS
    #if ENABLED(X_TWO_ENDSTOPS)
      UPDATE_ENDSTOP_BIT(X, MAX);
      #if HAS_X2_MAX
        UPDATE_ENDSTOP_BIT(X2, MAX);
      #else
        COPY_LIVE_STATE(X_MAX, X2_MAX);
      #endif
    #else
      UPDATE_ENDSTOP_BIT(X, MAX);
    #endif
  #endif

  #if HAS_Y_MIN && !Y_SPI_SENSORLESS
    #if ENABLED(Y_TWO_ENDSTOPS)
      UPDATE_ENDSTOP_BIT(Y, MIN);
      #if HAS_Y2_MIN
        UPDATE_ENDSTOP_BIT(Y2, MIN);
      #else
        COPY_LIVE_STATE(Y_MIN, Y2_MIN);
      #endif
    #else
      UPDATE_ENDSTOP_BIT(Y, MIN);
    #endif
  #endif

  #if HAS_Y_MAX && !Y_SPI_SENSORLESS
    #if ENABLED(Y_TWO_ENDSTOPS)
      UPDATE_ENDSTOP_BIT(Y, MAX);
      #if HAS_Y2_MAX
        UPDATE_ENDSTOP_BIT(Y2, MAX);
      #else
        COPY_LIVE_STATE(Y_MAX, Y2_MAX);
      #endif
    #else
      UPDATE_ENDSTOP_BIT(Y, MAX);
    #endif
  #endif

  #if HAS_Z_MIN && !Z_SPI_SENSORLESS
    #if ENABLED(Z_THREE_ENDSTOPS)
      UPDATE_ENDSTOP_BIT(Z, MIN);
      #if HAS_Z3_MIN
        UPDATE_ENDSTOP_BIT(Z3, MIN);
      #else
        COPY_LIVE_STATE(Z_MIN, Z3_MIN);
      #endif
      #if HAS_Z2_MIN
        UPDATE_ENDSTOP_BIT(Z2, MIN);
      #else
        COPY_LIVE_STATE(Z_MIN, Z2_MIN);
      #endif
    #elif ENABLED(Z_TWO_ENDSTOPS)
      UPDATE_ENDSTOP_BIT(Z, MIN);
      #if HAS_Z2_MIN
        UPDATE_ENDSTOP_BIT(Z2, MIN);
      #else
        COPY_LIVE_STATE(Z_MIN, Z2_MIN);
      #endif
    #else
      UPDATE_ENDSTOP_BIT(Z, MIN);
    #endif
  #endif

  // When closing the gap check the enabled probe
  #if HAS_BED_PROBE && HAS_Z_PROBE_PIN
    UPDATE_ENDSTOP_BIT(Z, PROBE);
  #endif

  #if HAS_Z_MAX && !Z_SPI_SENSORLESS
    #if ENABLED(Z_THREE_ENDSTOPS)
      UPDATE_ENDSTOP_BIT(Z, MAX);
      #if HAS_Z3_MAX
        UPDATE_ENDSTOP_BIT(Z3, MAX);
      #else
        COPY_LIVE_STATE(Z_MAX, Z3_MAX);
      #endif
      #if HAS_Z2_MAX
        UPDATE_ENDSTOP_BIT(Z2, MAX);
      #else
        COPY_LIVE_STATE(Z_MAX, Z2_MAX);
      #endif
    #elif ENABLED(Z_TWO_ENDSTOPS)
      UPDATE_ENDSTOP_BIT(Z, MAX);
      #if HAS_Z2_MAX
        UPDATE_ENDSTOP_BIT(Z2, MAX);
      #else
        COPY_LIVE_STATE(Z_MAX, Z2_MAX);
      #endif
    #else
      UPDATE_ENDSTOP_BIT(Z, MAX);
    #endif
  #endif

  // Test the current status of an endstop
  #define TEST_ENDSTOP(ENDSTOP)       (TEST(live_state, ENDSTOP))

  // Record endstop was hit
  #define _ENDSTOP_HIT(AXIS, MINMAX)  SBI(hit_state, _ENDSTOP(AXIS, MINMAX))

  // Call the endstop triggered routine for single endstops
  #define PROCESS_ENDSTOP(AXIS,MINMAX) do { \
    if (TEST_ENDSTOP(_ENDSTOP(AXIS, MINMAX))) { \
      _ENDSTOP_HIT(AXIS, MINMAX); \
      planner.endstop_triggered(_AXIS(AXIS)); \
    } \
  }while(0)

  // Call the endstop triggered routine for dual endstops
  #define PROCESS_DUAL_ENDSTOP(AXIS1, AXIS2, MINMAX) do { \
    const byte dual_hit = TEST_ENDSTOP(_ENDSTOP(AXIS1, MINMAX)) | (TEST_ENDSTOP(_ENDSTOP(AXIS2, MINMAX)) << 1); \
    if (dual_hit) { \
      _ENDSTOP_HIT(AXIS1, MINMAX); \
      if (!stepper.separate_multi_axis || dual_hit == 0b11) \
        planner.endstop_triggered(_AXIS(AXIS1)); \
    } \
  }while(0)

  #define PROCESS_TRIPLE_ENDSTOP(AXIS1, AXIS2, AXIS3, MINMAX) do { \
    const byte triple_hit = TEST_ENDSTOP(_ENDSTOP(AXIS1, MINMAX)) | (TEST_ENDSTOP(_ENDSTOP(AXIS2, MINMAX)) << 1) | (TEST_ENDSTOP(_ENDSTOP(AXIS3, MINMAX)) << 2); \
    if (triple_hit) { \
      _ENDSTOP_HIT(AXIS1, MINMAX); \
      /* if not performing home or if both endstops were trigged during homing... */ \
      if (!stepper.separate_multi_axis || triple_hit == 0x7) \
        planner.endstop_triggered(_AXIS(AXIS1)); \
    } \
  }while(0)

  #if ENABLED(G38_PROBE_TARGET) && HAS_Z_PROBE_PIN && !(CORE_IS_XY || CORE_IS_XZ)
    // If G38 command is active check Z_MIN_PROBE for ALL movement
    if (printer.IsG38Move()) {
      if (TEST_ENDSTOP(_ENDSTOP(Z, PROBE))) {
        if      (stepper.axis_is_moving(X_AXIS)) { _ENDSTOP_HIT(X, MIN); planner.endstop_triggered(X_AXIS); }
        else if (stepper.axis_is_moving(Y_AXIS)) { _ENDSTOP_HIT(Y, MIN); planner.endstop_triggered(Y_AXIS); }
        else if (stepper.axis_is_moving(Z_AXIS)) { _ENDSTOP_HIT(Z, MIN); planner.endstop_triggered(Z_AXIS); }
        setG38EndstopHit(true);
      }
    }
  #endif

  // Now, we must signal, after validation, if an endstop limit is pressed or not
  if (stepper.axis_is_moving(X_AXIS)) {
    if (stepper.motor_direction(X_AXIS_HEAD)) { // -direction
      #if HAS_X_MIN || (X_SPI_SENSORLESS && X_HOME_DIR < 0)
        #if ENABLED(X_TWO_ENDSTOPS)
          PROCESS_DUAL_ENDSTOP(X, X2, MIN);
        #else
          if (X_MIN_TEST) PROCESS_ENDSTOP(X, MIN);
        #endif
      #elif MECH(DELTA) && ENABLED(PROBE_SENSORLESS)
        PROCESS_ENDSTOP(X, MAX);
      #endif
    }
    else {  // +direction
      #if HAS_X_MAX || (X_SPI_SENSORLESS && X_HOME_DIR > 0)
        #if ENABLED(X_TWO_ENDSTOPS)
          PROCESS_DUAL_ENDSTOP(X, X2, MAX);
        #else
          if (X_MAX_TEST) PROCESS_ENDSTOP(X, MAX);
        #endif
      #endif
    }
  }

  if (stepper.axis_is_moving(Y_AXIS)) {
    if (stepper.motor_direction(Y_AXIS_HEAD)) { // -direction
      #if HAS_Y_MIN || (Y_SPI_SENSORLESS && Y_HOME_DIR < 0)
        #if ENABLED(Y_TWO_ENDSTOPS)
          PROCESS_DUAL_ENDSTOP(Y, Y2, MIN);
        #else
          PROCESS_ENDSTOP(Y, MIN);
        #endif
      #elif MECH(DELTA) && ENABLED(PROBE_SENSORLESS)
        PROCESS_ENDSTOP(Y, MAX);
      #endif
    }
    else {  // +direction
      #if HAS_Y_MAX || (Y_SPI_SENSORLESS && Y_HOME_DIR > 0)
        #if ENABLED(Y_TWO_ENDSTOPS)
          PROCESS_DUAL_ENDSTOP(Y, Y2, MAX);
        #else
          PROCESS_ENDSTOP(Y, MAX);
        #endif
      #endif
    }
  }

  if (stepper.axis_is_moving(Z_AXIS)) {
    if (stepper.motor_direction(Z_AXIS_HEAD)) { // Z -direction. Gantry down, bed up.
      #if HAS_Z_MIN || (Z_SPI_SENSORLESS && Z_HOME_DIR < 0)
        #if ENABLED(Z_THREE_ENDSTOPS)
          PROCESS_TRIPLE_ENDSTOP(Z, Z2, Z3, MIN);
        #elif ENABLED(Z_TWO_ENDSTOPS)
          PROCESS_DUAL_ENDSTOP(Z, Z2, MIN);
        #else
          PROCESS_ENDSTOP(Z, MIN);
        #endif
      #elif MECH(DELTA) && ENABLED(PROBE_SENSORLESS)
        PROCESS_ENDSTOP(Z, MAX);
      #endif

      // When closing the gap check the enabled probe
      #if HAS_BED_PROBE && HAS_Z_PROBE_PIN
        if (isProbeEnabled()) PROCESS_ENDSTOP(Z, PROBE);
      #endif
    }
    else { // Z +direction. Gantry up, bed down.
      #if HAS_Z_MAX || (Z_SPI_SENSORLESS && Z_HOME_DIR > 0)
        #if ENABLED(Z_THREE_ENDSTOPS)
          PROCESS_TRIPLE_ENDSTOP(Z, Z2, Z3, MAX);
        #elif ENABLED(Z_TWO_ENDSTOPS)
          PROCESS_DUAL_ENDSTOP(Z, Z2, MAX);
        #else
          PROCESS_ENDSTOP(Z, MAX);
        #endif
      #endif
    }
  }

} // Endstops::update()

void Endstops::report() {

  SERIAL_EM("Reporting endstop logic and pullup");

  // X Endstop
  SERIAL_MSG("Endstop");
  if (mechanics.home_dir.x == -1) {
    SERIAL_LOGIC(" X Logic",  isLogic(X_MIN));
    SERIAL_LOGIC(" Pullup",   isPullup(X_MIN));
    #if HAS_X2_MIN
      SERIAL_LOGIC(" X2 Logic", isLogic(X2_MIN));
      SERIAL_LOGIC(" Pullup",   isPullup(X2_MIN));
    #endif
  }
  else {
    SERIAL_LOGIC(" X Logic",  isLogic(X_MAX));
    SERIAL_LOGIC(" Pullup",   isPullup(X_MAX));
    #if HAS_X2_MAX
      SERIAL_LOGIC(" X2 Logic", isLogic(X2_MAX));
      SERIAL_LOGIC(" Pullup",   isPullup(X2_MAX));
    #endif
  }
  SERIAL_EOL();

  // Y Endstop
  SERIAL_MSG("Endstop");
  if (mechanics.home_dir.y == -1) {
    SERIAL_LOGIC(" Y Logic",  isLogic(Y_MIN));
    SERIAL_LOGIC(" Pullup",   isPullup(Y_MIN));
    #if HAS_Y2_MIN
      SERIAL_LOGIC(" Y2 Logic", isLogic(Y2_MIN));
      SERIAL_LOGIC(" Pullup",   isPullup(Y2_MIN));
    #endif
  }
  else {
    SERIAL_LOGIC(" Y Logic",  isLogic(Y_MAX));
    SERIAL_LOGIC(" Pullup",   isPullup(Y_MAX));
    #if HAS_Y2_MAX
      SERIAL_LOGIC(" Y2 Logic", isLogic(Y2_MAX));
      SERIAL_LOGIC(" Pullup",   isPullup(Y2_MAX));
    #endif
  }
  SERIAL_EOL();

  // Z Endstop
  SERIAL_MSG("Endstop");
  if (mechanics.home_dir.z == -1) {
    SERIAL_LOGIC(" Z Logic",  isLogic(Z_MIN));
    SERIAL_LOGIC(" Pullup",   isPullup(Z_MIN));
    #if HAS_Z2_MIN
      SERIAL_LOGIC(" Z2 Logic", isLogic(Z2_MIN));
      SERIAL_LOGIC(" Pullup",   isPullup(Z2_MIN));
    #endif
    #if HAS_Z3_MIN
      SERIAL_LOGIC(" Z3 Logic", isLogic(Z3_MIN));
      SERIAL_LOGIC(" Pullup",   isPullup(Z3_MIN));
    #endif
  }
  else {
    SERIAL_LOGIC(" Z Logic",  isLogic(Z_MAX));
    SERIAL_LOGIC(" Pullup",   isPullup(Z_MAX));
    #if HAS_Z2_MAX
      SERIAL_LOGIC(" Z2 Logic", isLogic(Z2_MAX));
      SERIAL_LOGIC(" Pullup",   isPullup(Z2_MAX));
    #endif
    #if HAS_Z3_MAX
      SERIAL_LOGIC(" Z3 Logic", isLogic(Z3_MAX));
      SERIAL_LOGIC(" Pullup",   isPullup(Z3_MAX));
    #endif
  }
  SERIAL_EOL();

  #if HAS_Z_PROBE_PIN
    // Probe Endstop
    SERIAL_LOGIC("Endstop PROBE Logic", isLogic(Z_PROBE));
    SERIAL_LOGIC(" Pullup", isPullup(Z_PROBE));
    SERIAL_EOL();
  #endif

  #if HAS_DOOR_OPEN
    // Door Open
    SERIAL_LOGIC("Endstop DOOR OPEN Logic", isLogic(DOOR_OPEN));
    SERIAL_LOGIC(" Pullup", isPullup(DOOR_OPEN));
    SERIAL_EOL();
  #endif

}

void Endstops::report_state() {

  static uint8_t prev_hit_state = 0;

  if (hit_state == prev_hit_state) return;

  prev_hit_state = hit_state;

  if (hit_state) {

    #if HAS_LCD
      char chrX = ' ', chrY = ' ', chrZ = ' ', chrP = ' ';
    #endif

    SERIAL_SM(ECHO, STR_ENDSTOPS_HIT);
    if (TEST(hit_state, X_MIN) || TEST(hit_state, X_MAX)) {
      SERIAL_MV("X:", planner.triggered_position_mm(X_AXIS));
      #if HAS_LCD
        chrX = 'X';
      #endif
    }
    if (TEST(hit_state, Y_MIN) || TEST(hit_state, Y_MAX)) {
      SERIAL_MV("Y:", planner.triggered_position_mm(Y_AXIS));
      #if HAS_LCD
        chrY = 'Y';
      #endif
    }
    if (TEST(hit_state, Z_MIN) || TEST(hit_state, Z_MAX)) {
      SERIAL_MV("Z:", planner.triggered_position_mm(Z_AXIS));
      #if HAS_LCD
        chrZ = 'Z';
      #endif
    }
  
    #if HAS_Z_PROBE_PIN
      #define P_AXIS Z_AXIS
      if (TEST(hit_state, Z_PROBE)) {
        SERIAL_MV("P:", planner.triggered_position_mm(Z_AXIS));
        #if HAS_LCD
          chrP = 'P';
        #endif
      }
    #endif
    SERIAL_EOL();

    #if HAS_LCD
      lcdui.status_printf_P(0, PSTR(S_FMT " %c %c %c %c"), GET_TEXT(MSG_LCD_ENDSTOPS), chrX, chrY, chrZ, chrP);
    #endif

    #if ENABLED(SD_ABORT_ON_ENDSTOP_HIT) && HAS_SD_SUPPORT
      if (planner.flag.abort_on_endstop_hit) {
        card.endFilePrint();
        printer.quickstop_stepper();
        tempManager.disable_all_heaters();
      }
    #endif
  }

} // Endstops::report_state

// If the last move failed to trigger an endstop, call kill
void Endstops::validate_homing_move() {
  if (trigger_state()) hit_on_purpose();
  else {
    sound.feedback(false);
    SERIAL_LM(REQUESTPAUSE, " Homing Failed");
  }
}

/**
 * Constrain the given coordinates to the software endstops.
 */
void Endstops::apply_motion_limits(xyz_pos_t &target) {

  if (!isSoftEndstop()) return;

  #if MECH(DELTA)

    if (!mechanics.isHomedAll()) return;

    const float dist_2 = HYPOT2(target.x, target.y);
    if (dist_2 > soft_endstop_radius_2) {
      const float ratio = mechanics.data.print_radius / SQRT(dist_2);
      target.x *= ratio;
      target.y *= ratio;
    }
    NOLESS(target.z, 0);
    NOMORE(target.z, mechanics.data.height);

  #else

    if (mechanics.isAxisHomed(X_AXIS)) {
      #if ENABLED(MIN_SOFTWARE_ENDSTOPS)
        NOLESS(target.x, soft_endstop.min.x);
      #endif
      #if ENABLED(MAX_SOFTWARE_ENDSTOPS)
        NOMORE(target.x, soft_endstop.max.x);
      #endif
    }
    if (mechanics.isAxisHomed(Y_AXIS)) {
      #if ENABLED(MIN_SOFTWARE_ENDSTOPS)
        NOLESS(target.y, soft_endstop.min.y);
      #endif
      #if ENABLED(MAX_SOFTWARE_ENDSTOPS)
        NOMORE(target.y, soft_endstop.max.y);
      #endif
    }
    if (mechanics.isAxisHomed(Z_AXIS)) {
      #if ENABLED(MIN_SOFTWARE_ENDSTOPS)
        NOLESS(target.z, soft_endstop.min.z);
      #endif
      #if ENABLED(MAX_SOFTWARE_ENDSTOPS)
        NOMORE(target.z, soft_endstop.max.z);
      #endif
    }

  #endif
}

/**
 * Software endstops can be used to monitor the open end of
 * an axis that has a hardware endstop on the other end. Or
 * they can prevent axes from moving past endstops and grinding.
 *
 * To keep doing their job as the coordinate system changes,
 * the software endstop positions must be refreshed to remain
 * at the same positions relative to the machine.
 */
void Endstops::update_software_endstops(const AxisEnum axis) {

  #if ENABLED(DUAL_X_CARRIAGE)

    if (axis == X_AXIS) {

      // In Dual X mode nozzle.data.hotend_offset[X] is T1's home position
      float dual_max_x = MAX(nozzle.data.hotend_offset[1].x, X2_MAX_POS);

      if (toolManager.extruder.target != 0) {
        // T1 can move from X2_MIN_POS to X2_MAX_POS or X2 home position (whichever is larger)
        soft_endstop.min.x = X2_MIN_POS;
        soft_endstop.max.x = dual_max_x;
      }
      else if (mechanics.dxc_is_duplicating()) {
        // In Duplication Mode, T0 can move as far left as X1_MIN_POS
        // but not so far to the right that T1 would move past the end
        soft_endstop.min.x = X1_MIN_POS;
        soft_endstop.max.x = MIN(X1_MAX_POS, dual_max_x - mechanics.duplicate_extruder_x_offset);
      }
      else {
        // In other modes, T0 can move from X1_MIN_POS to X1_MAX_POS
        soft_endstop.min[axis] = X1_MIN_POS;
        soft_endstop.max[axis] = X1_MAX_POS;
      }
    }

  #elif MECH(DELTA)

    soft_endstop_radius_2 = sq(mechanics.data.print_radius);

  #else

    if (tempManager.heater.hotends > 1) {
      if (toolManager.extruder.active != toolManager.extruder.target) {
        const float offs = nozzle.data.hotend_offset[toolManager.target_hotend()][axis] - nozzle.data.hotend_offset[toolManager.active_hotend()][axis];
        soft_endstop.min[axis] += offs;
        soft_endstop.max[axis] += offs;
      }
      else {
        const float offs = nozzle.data.hotend_offset[toolManager.active_hotend()][axis];
        soft_endstop.min[axis] = mechanics.data.base_pos.min[axis] + offs;
        soft_endstop.max[axis] = mechanics.data.base_pos.max[axis] + offs;
      }
    }
    else {
      soft_endstop.min[axis] = mechanics.data.base_pos.min[axis];
      soft_endstop.max[axis] = mechanics.data.base_pos.max[axis];

      #if ENABLED(WORKSPACE_OFFSETS)
        if (printer.debugFeature()) {
          DEBUG_MV("For ", axis_codes[axis]);
          DEBUG_MV(" axis:\n data.home_offset = ", mechanics.data.home_offset[axis]);
          DEBUG_MV("\n position_shift = ", mechanics.position_shift[axis]);
          DEBUG_MV("\n soft_endstop_min = ", soft_endstop.min[axis]);
          DEBUG_EMV("\n soft_endstop_max = ", soft_endstop.max[axis]);
        }
      #endif
    }

  #endif

}

#if ENABLED(SPI_ENDSTOPS)

  #define X_STOP (X_HOME_DIR < 0 ? X_MIN : X_MAX)
  #define Y_STOP (Y_HOME_DIR < 0 ? Y_MIN : Y_MAX)
  #define Z_STOP (Z_HOME_DIR < 0 ? Z_MIN : Z_MAX)

  bool Endstops::tmc_spi_homing_check() {

    bool hit = false;

    #if X_SPI_SENSORLESS
      if (tmc_spi_homing.x && driver.x->tmc->test_stall_status()) {
        SBI(live_state, X_STOP);
        hit = true;
      }
    #endif

    #if Y_SPI_SENSORLESS
      if (tmc_spi_homing.y && driver.y->tmc->test_stall_status()) {
        SBI(live_state, Y_STOP);
        hit = true;
      }
    #endif

    #if Z_SPI_SENSORLESS
      if (tmc_spi_homing.z && driver.z->tmc->test_stall_status()) {
        SBI(live_state, Z_STOP);
        hit = true;
      }
    #endif

    return hit;

  }

  void Endstops::clear_state() {
    #if X_SPI_SENSORLESS
      CBI(live_state, X_STOP);
    #endif
    #if Y_SPI_SENSORLESS
      CBI(live_state, Y_STOP);
    #endif
    #if Z_SPI_SENSORLESS
      CBI(live_state, Z_STOP);
    #endif
  }

#endif // SPI_ENDSTOPS

/** Private Function */
void Endstops::resync() {

  if (!abort_enabled()) return;     // If endstops/probes are disabled the loop below can hang

  #if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
    update();
  #else
    HAL::delayMilliseconds(2);
  #endif
}

#if ENABLED(PINS_DEBUGGING)

  void Endstops::run_monitor() {
    if (!isMonitorEnabled()) return;
    static uint8_t monitor_count = 16;  // offset this check from the others
    monitor_count += _BV(1);            //  15 Hz
    monitor_count &= 0x7F;
    if (!monitor_count) monitor();      // report changes in endstop status
  }

  /**
   * monitors endstops & Z probe for changes
   *
   * If a change is detected then the LED is toggled and
   * a message is sent out the serial port
   *
   * Yes, we could miss a rapid back & forth change but
   * that won't matter because this is all manual.
   *
   */
  void Endstops::monitor() {

    static uint16_t old_bits_local = 0;
    uint16_t current_bits_local = 0;
    #if HAS_X_MIN
      if (READ(X_MIN_PIN)) SBI(current_bits_local, X_MIN);
    #endif
    #if HAS_X_MAX
      if (READ(X_MAX_PIN)) SBI(current_bits_local, X_MAX);
    #endif
    #if HAS_Y_MIN
      if (READ(Y_MIN_PIN)) SBI(current_bits_local, Y_MIN);
    #endif
    #if HAS_Y_MAX
      if (READ(Y_MAX_PIN)) SBI(current_bits_local, Y_MAX);
    #endif
    #if HAS_Z_MIN
      if (READ(Z_MIN_PIN)) SBI(current_bits_local, Z_MIN);
    #endif
    #if HAS_Z_MAX
      if (READ(Z_MAX_PIN)) SBI(current_bits_local, Z_MAX);
    #endif
    #if HAS_Z_PROBE_PIN
      if (READ(Z_PROBE_PIN)) SBI(current_bits_local, Z_PROBE);
    #endif
    #if HAS_X2_MIN
      if (READ(X2_MIN_PIN)) SBI(current_bits_local, X2_MIN);
    #endif
    #if HAS_X2_MAX
      if (READ(X2_MAX_PIN)) SBI(current_bits_local, X2_MAX);
    #endif
    #if HAS_Y2_MIN
      if (READ(Y2_MIN_PIN)) SBI(current_bits_local, Y2_MIN);
    #endif
    #if HAS_Y2_MAX
      if (READ(Y2_MAX_PIN)) SBI(current_bits_local, Y2_MAX);
    #endif
    #if HAS_Z2_MIN
      if (READ(Z2_MIN_PIN)) SBI(current_bits_local, Z2_MIN);
    #endif
    #if HAS_Z2_MAX
      if (READ(Z2_MAX_PIN)) SBI(current_bits_local, Z2_MAX);
    #endif
    #if HAS_Z3_MIN
      if (READ(Z3_MIN_PIN)) SBI(current_bits_local, Z3_MIN);
    #endif
    #if HAS_Z3_MAX
      if (READ(Z3_MAX_PIN)) SBI(current_bits_local, Z3_MAX);
    #endif

    uint16_t endstop_change = current_bits_local ^ old_bits_local;

    if (endstop_change) {
      #if HAS_X_MIN
        if (TEST(endstop_change, X_MIN)) SERIAL_MV("  X_MIN:", TEST(current_bits_local, X_MIN));
      #endif
      #if HAS_X_MAX
        if (TEST(endstop_change, X_MAX)) SERIAL_MV("  X_MAX:", TEST(current_bits_local, X_MAX));
      #endif
      #if HAS_Y_MIN
        if (TEST(endstop_change, Y_MIN)) SERIAL_MV("  Y_MIN:", TEST(current_bits_local, Y_MIN));
      #endif
      #if HAS_Y_MAX
        if (TEST(endstop_change, Y_MAX)) SERIAL_MV("  Y_MAX:", TEST(current_bits_local, Y_MAX));
      #endif
      #if HAS_Z_MIN
        if (TEST(endstop_change, Z_MIN)) SERIAL_MV("  Z_MIN:", TEST(current_bits_local, Z_MIN));
      #endif
      #if HAS_Z_MAX
        if (TEST(endstop_change, Z_MAX)) SERIAL_MV("  Z_MAX:", TEST(current_bits_local, Z_MAX));
      #endif
      #if HAS_Z_PROBE_PIN
        if (TEST(endstop_change, Z_PROBE)) SERIAL_MV("  PROBE:", TEST(current_bits_local, Z_PROBE));
      #endif
      #if HAS_X2_MIN
        if (TEST(endstop_change, X2_MIN)) SERIAL_MV("  X2_MIN:", TEST(current_bits_local, X2_MIN));
      #endif
      #if HAS_X2_MAX
        if (TEST(endstop_change, X2_MAX)) SERIAL_MV("  X2_MAX:", TEST(current_bits_local, X2_MAX));
      #endif
      #if HAS_Y2_MIN
        if (TEST(endstop_change, Y2_MIN)) SERIAL_MV("  Y2_MIN:", TEST(current_bits_local, Y2_MIN));
      #endif
      #if HAS_Y2_MAX
        if (TEST(endstop_change, Y2_MAX)) SERIAL_MV("  Y2_MAX:", TEST(current_bits_local, Y2_MAX));
      #endif
      #if HAS_Z2_MIN
        if (TEST(endstop_change, Z2_MIN)) SERIAL_MV("  Z2_MIN:", TEST(current_bits_local, Z2_MIN));
      #endif
      #if HAS_Z2_MAX
        if (TEST(endstop_change, Z2_MAX)) SERIAL_MV("  Z2_MAX:", TEST(current_bits_local, Z2_MAX));
      #endif
      #if HAS_Z3_MIN
        if (TEST(endstop_change, Z3_MIN)) SERIAL_MV("  Z3_MIN:", TEST(current_bits_local, Z3_MIN));
      #endif
      #if HAS_Z3_MAX
        if (TEST(endstop_change, Z3_MAX)) SERIAL_MV("  Z3_MAX:", TEST(current_bits_local, Z3_MAX));
      #endif
      SERIAL_MSG("\n\n");
      old_bits_local = current_bits_local;
    }
  }

#endif // PINS_DEBUGGING
