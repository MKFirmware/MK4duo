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
 * endstops.cpp - A singleton object to manage endstops hardware and software
 */

#include "../../../MK4duo.h"

#if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
  #include "../../HAL/HAL_endstop_interrupts.h"
#endif

// TEST_ENDSTOP: test the old and the current status of an endstop
#define TEST_ENDSTOP(ENDSTOP) (TEST(current_bits & old_bits, ENDSTOP))

Endstops endstops;

// public:

#if IS_KINEMATIC
  float Endstops::soft_endstop_radius_2 = 0.0;
#else
  float Endstops::soft_endstop_min[XYZ] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS },
        Endstops::soft_endstop_max[XYZ] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };
#endif

#if ENABLED(X_TWO_ENDSTOPS)
  float Endstops::x_endstop_adj = 0.0;
#endif
#if ENABLED(Y_TWO_ENDSTOPS)
  float Endstops::y_endstop_adj = 0.0;
#endif
#if ENABLED(Z_TWO_ENDSTOPS)
  float Endstops::z_endstop_adj = 0.0;
#endif

volatile char Endstops::hit_bits; // use X_MIN, Y_MIN, Z_MIN and Z_MIN_PROBE as BIT value

volatile uint8_t Endstops::e_hit = 0; // Different from 0 when the endstops shall be tested in detail.
                                      // Must be reset to 0 by the test function when the tests are finished.

uint16_t  Endstops::logic_bits    = 0,
          Endstops::pullup_bits   = 0,
          Endstops::current_bits  = 0,
          Endstops::old_bits      = 0;

// Private
uint8_t   Endstops::flag1_bits    = 0;

/**
 * Class and Instance Methods
 */

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

  #if HAS_Z_PROBE_PIN
    SET_INPUT(Z_PROBE_PIN);
  #endif

  #if HAS_FIL_RUNOUT
    filamentrunout.init();
  #endif

  #if HAS_DOOR_OPEN
    SET_INPUT(DOOR_OPEN_PIN);
  #endif

  #if HAS_POWER_CHECK && HAS_SDSUPPORT
    SET_INPUT(POWER_CHECK_PIN);
  #endif

  #if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
    setup_endstop_interrupts();
  #endif

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

  #if HAS_Z_PROBE_PIN
    HAL::setInputPullup(Z_PROBE_PIN, isPullup(Z_PROBE));
  #endif

  #if HAS_FIL_RUNOUT
    filamentrunout.setup_pullup(isPullup(FIL_RUNOUT));
  #endif

  #if HAS_DOOR_OPEN
    HAL::setInputPullup(DOOR_OPEN_PIN, isPullup(DOOR_OPEN_SENSOR));
  #endif

  #if HAS_POWER_CHECK && HAS_SDSUPPORT
    HAL::setInputPullup(POWER_CHECK_PIN, isPullup(POWER_CHECK_SENSOR));
  #endif

}

void Endstops::report() {

  SERIAL_EM("Reporting endstop logic and pullup");

  // X Endstop
  SERIAL_MSG("Endstop");
  if (mechanics.home_dir[X_AXIS] == -1) {
    SERIAL_MT(" X Logic:",  isLogic(X_MIN)  ? "true" : "false");
    SERIAL_MT(" Pullup:",   isPullup(X_MIN) ? "true" : "false");
    #if HAS_X2_MIN
      SERIAL_MT(" X2 Logic:",   isLogic(X2_MIN)   ? "true" : "false");
      SERIAL_MT(" X2 Pullup:",  isPullup(X2_MIN)  ? "true" : "false");
    #endif
  }
  else {
    SERIAL_MT(" X Logic:",  isLogic(X_MAX)  ? "true" : "false");
    SERIAL_MT(" Pullup:",   isPullup(X_MAX) ? "true" : "false");
    #if HAS_X2_MAX
      SERIAL_MT(" X2 Logic:",   isLogic(X2_MAX)   ? "true" : "false");
      SERIAL_MT(" X2 Pullup:",  isPullup(X2_MAX)  ? "true" : "false");
    #endif
  }
  SERIAL_EOL();

  // Y Endstop
  SERIAL_MSG("Endstop");
  if (mechanics.home_dir[Y_AXIS] == -1) {
    SERIAL_MT(" Y Logic:",  isLogic(Y_MIN)  ? "true" : "false");
    SERIAL_MT(" Pullup:",   isPullup(Y_MIN) ? "true" : "false");
    #if HAS_Y2_MIN
      SERIAL_MT(" Y2 Logic:",   isLogic(Y2_MIN)   ? "true" : "false");
      SERIAL_MT(" Y2 Pullup:",  isPullup(Y2_MIN)  ? "true" : "false");
    #endif
  }
  else {
    SERIAL_MT(" Y Logic:",  isLogic(Y_MAX)  ? "true" : "false");
    SERIAL_MT(" Pullup:",   isPullup(Y_MAX) ? "true" : "false");
    #if HAS_Y2_MAX
      SERIAL_MT(" Y2 Logic:",   isLogic(Y2_MAX)   ? "true" : "false");
      SERIAL_MT(" Y2 Pullup:",  isPullup(Y2_MAX)  ? "true" : "false");
    #endif
  }
  SERIAL_EOL();

  // Z Endstop
  SERIAL_MSG("Endstop");
  if (mechanics.home_dir[Z_AXIS] == -1) {
    SERIAL_MT(" Z Logic:",  isLogic(Z_MIN)  ? "true" : "false");
    SERIAL_MT(" Pullup:",   isPullup(Z_MIN) ? "true" : "false");
    #if HAS_Z2_MIN
      SERIAL_MT(" Z2 Logic:",   isLogic(Z2_MIN)   ? "true" : "false");
      SERIAL_MT(" Z2 Pullup:",  isPullup(Z2_MIN)  ? "true" : "false");
    #endif
  }
  else {
    SERIAL_MT(" Z Logic:",  isLogic(Z_MAX)  ? "true" : "false");
    SERIAL_MT(" Pullup:",   isPullup(Z_MAX) ? "true" : "false");
    #if HAS_Z2_MAX
      SERIAL_MT(" Z2 Logic:",   isLogic(Z2_MAX)   ? "true" : "false");
      SERIAL_MT(" Z2 Pullup:",  isPullup(Z2_MAX)  ? "true" : "false");
    #endif
  }
  SERIAL_EOL();

  #if HAS_Z_PROBE_PIN
    // Probe Endstop
    SERIAL_MV("Endstop PROBE Logic:", isLogic(Z_PROBE) ? "true" : "false");
    SERIAL_EMV(" Pullup:", isPullup(Z_PROBE) ? "true" : "false");
  #endif

  #if HAS_FIL_RUNOUT
    // FIL RUNOUT
    SERIAL_MV("Endstop FIL_RUNOUT Logic:", isLogic(FIL_RUNOUT) ? "true" : "false");
    SERIAL_EMV(" Pullup:", isPullup(FIL_RUNOUT) ? "true" : "false");
  #endif

  #if HAS_DOOR_OPEN
    // Door Open
    SERIAL_MV("Endstop DOOR OPEN Logic:", isLogic(DOOR_OPEN_SENSOR) ? "true" : "false");
    SERIAL_EMV(" Pullup:", isPullup(DOOR_OPEN_SENSOR) ? "true" : "false");
  #endif

  #if HAS_POWER_CHECK && HAS_SDSUPPORT
    // Power Check
    SERIAL_MV("Endstop Power Check Logic:", isLogic(POWER_CHECK_SENSOR) ? "true" : "false");
    SERIAL_EMV(" Pullup:", isPullup(POWER_CHECK_SENSOR) ? "true" : "false");
  #endif

}

void Endstops::report_state() {
  if (hit_bits) {
    #if ENABLED(ULTRA_LCD)
      char chrX = ' ', chrY = ' ', chrZ = ' ', chrP = ' ';
      #define _SET_STOP_CHAR(A,C) (chr## A = C)
    #else
      #define _SET_STOP_CHAR(A,C) ;
    #endif

    #define _ENDSTOP_HIT_ECHO(A,C) do{ \
      SERIAL_MV(STRINGIFY(A) ":", stepper.triggered_position_mm(A ##_AXIS)); \
      _SET_STOP_CHAR(A,C); }while(0)

    #define _ENDSTOP_HIT_TEST(A,C) \
      if (TEST(hit_bits, A ##_MIN) || TEST(hit_bits, A ##_MAX)) \
        _ENDSTOP_HIT_ECHO(A,C)

    SERIAL_SM(ECHO, MSG_ENDSTOPS_HIT);
    _ENDSTOP_HIT_TEST(X, 'X');
    _ENDSTOP_HIT_TEST(Y, 'Y');
    _ENDSTOP_HIT_TEST(Z, 'Z');

    #if HAS_Z_PROBE_PIN
      #define P_AXIS Z_AXIS
      if (TEST(hit_bits, Z_PROBE)) _ENDSTOP_HIT_ECHO(P, 'P');
    #endif
    SERIAL_EOL();

    #if ENABLED(ULTRA_LCD)
      lcd_status_printf_P(0, PSTR(MSG_LCD_ENDSTOPS " %c %c %c %c"), chrX, chrY, chrZ, chrP);
    #endif

    hit_on_purpose();

    #if ENABLED(ABORT_ON_ENDSTOP_HIT)
      if (stepper.abort_on_endstop_hit && !printer.isHoming()) {
        stepper.quickstop_stepper();
        thermalManager.disable_all_heaters();
        #if HAS_SDSUPPORT
          // Stop printing, close file and save restart.gcode
          card.stopSDPrint(); // same as executing M33
        #endif
        SERIAL_LM(ER, MSG_PRINT_ABORTED);
        printer.kill(MSG_PRINT_ABORTED);
      }
    #endif
  }
} // Endstops::report_state

#if ENABLED(X_TWO_ENDSTOPS)
  void Endstops::test_two_x_endstops(const EndstopEnum es1, const EndstopEnum es2) {
    const byte x_test = TEST_ENDSTOP(es1) | (TEST_ENDSTOP(es2) << 1); // bit 0 for X, bit 1 for X2
    if (x_test && stepper.current_block->steps[X_AXIS] > 0) {
      SBI(hit_bits, X_MIN);
      if (!printer.isHoming() || (x_test == 0x3))  // if not performing home or if both endstops were trigged during homing...
        stepper.kill_current_block();
    }
  }
#endif
#if ENABLED(Y_TWO_ENDSTOPS)
  void Endstops::test_two_y_endstops(const EndstopEnum es1, const EndstopEnum es2) {
    const byte y_test = TEST_ENDSTOP(es1) | (TEST_ENDSTOP(es2) << 1); // bit 0 for Y, bit 1 for Y2
    if (y_test && stepper.current_block->steps[Y_AXIS] > 0) {
      SBI(hit_bits, Y_MIN);
      if (!printer.isHoming() || (y_test == 0x3))  // if not performing home or if both endstops were trigged during homing...
        stepper.kill_current_block();
    }
  }
#endif
#if ENABLED(Z_TWO_ENDSTOPS)
  void Endstops::test_two_z_endstops(const EndstopEnum es1, const EndstopEnum es2) {
    const byte z_test = TEST_ENDSTOP(es1) | (TEST_ENDSTOP(es2) << 1); // bit 0 for Z, bit 1 for Z2
    if (z_test && stepper.current_block->steps[Z_AXIS] > 0) {
      SBI(hit_bits, Z_MIN);
      if (!printer.isHoming() || (z_test == 0x3))  // if not performing home or if both endstops were trigged during homing...
        stepper.kill_current_block();
    }
  }
#endif

/**
 * Constrain the given coordinates to the software endstops.
 */
void Endstops::clamp_to_software_endstops(float target[XYZ]) {

  if (!isSoftEndstop()) return;

  #if IS_KINEMATIC
    const float dist_2 = HYPOT2(target[X_AXIS], target[Y_AXIS]);
    if (dist_2 > soft_endstop_radius_2) {
      const float ratio = mechanics.delta_print_radius / SQRT(dist_2);
      target[X_AXIS] *= ratio;
      target[Y_AXIS] *= ratio;
    }
    NOLESS(target[Z_AXIS], 0);
    NOMORE(target[Z_AXIS], mechanics.delta_height);
  #else
    #if ENABLED(MIN_SOFTWARE_ENDSTOPS)
      NOLESS(target[X_AXIS], soft_endstop_min[X_AXIS]);
      NOLESS(target[Y_AXIS], soft_endstop_min[Y_AXIS]);
      NOLESS(target[Z_AXIS], soft_endstop_min[Z_AXIS]);
    #endif
    #if ENABLED(MAX_SOFTWARE_ENDSTOPS)
      NOMORE(target[X_AXIS], soft_endstop_max[X_AXIS]);
      NOMORE(target[Y_AXIS], soft_endstop_max[Y_AXIS]);
      NOMORE(target[Z_AXIS], soft_endstop_max[Z_AXIS]);
    #endif
  #endif
}

#if ENABLED(WORKSPACE_OFFSETS) || ENABLED(DUAL_X_CARRIAGE)

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
    const float offs = mechanics.home_offset[axis] + mechanics.position_shift[axis];

    mechanics.workspace_offset[axis] = offs;

    #if ENABLED(DUAL_X_CARRIAGE)
      if (axis == X_AXIS) {

        // In Dual X mode tools.hotend_offset[X] is T1's home position
        float dual_max_x = max(tools.hotend_offset[X_AXIS][1], X2_MAX_POS);

        if (tools.active_extruder != 0) {
          // T1 can move from X2_MIN_POS to X2_MAX_POS or X2 home position (whichever is larger)
          soft_endstop_min[X_AXIS] = X2_MIN_POS + offs;
          soft_endstop_max[X_AXIS] = dual_max_x + offs;
        }
        else if (mechanics.dual_x_carriage_mode == DXC_DUPLICATION_MODE) {
          // In Duplication Mode, T0 can move as far left as X_MIN_POS
          // but not so far to the right that T1 would move past the end
          soft_endstop_min[X_AXIS] = mechanics.base_min_pos[X_AXIS] + offs;
          soft_endstop_max[X_AXIS] = min(mechanics.base_max_pos[X_AXIS], dual_max_x - mechanics.duplicate_hotend_x_offset) + offs;
        }
        else {
          // In other modes, T0 can move from X_MIN_POS to X_MAX_POS
          soft_endstop_min[axis] = mechanics.base_min_pos[axis] + offs;
          soft_endstop_max[axis] = mechanics.base_max_pos[axis] + offs;
        }
      }
    #else
      soft_endstop_min[axis] = mechanics.base_min_pos[axis] + offs;
      soft_endstop_max[axis] = mechanics.base_max_pos[axis] + offs;
    #endif

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) {
        SERIAL_MV("For ", axis_codes[axis]);
        SERIAL_MV(" axis:\n home_offset = ", mechanics.home_offset[axis]);
        SERIAL_MV("\n position_shift = ", mechanics.position_shift[axis]);
        SERIAL_MV("\n soft_endstop_min = ", soft_endstop_min[axis]);
        SERIAL_EMV("\n soft_endstop_max = ", soft_endstop_max[axis]);
      }
    #endif

  }

#endif // ENABLED(WORKSPACE_OFFSETS) || DUAL_X_CARRIAGE

#if ENABLED(PINS_DEBUGGING)
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
  void Endstops::endstop_monitor() {
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
      SERIAL_MSG("\n\n");
      old_bits_local = current_bits_local;
    }
  }
#endif // PINS_DEBUGGING

// Check endstops - Called from ISR!
void Endstops::update() {

  #define _ENDSTOP(AXIS, MINMAX) AXIS ##_## MINMAX
  #define _ENDSTOP_PIN(AXIS, MINMAX) AXIS ##_## MINMAX ##_PIN
  #define _ENDSTOP_HIT(AXIS, MINMAX) SBI(hit_bits, _ENDSTOP(AXIS, MINMAX))

  // UPDATE_ENDSTOP_BIT: set the current endstop bits for an endstop to its status
  #define UPDATE_ENDSTOP_BIT(AXIS, MINMAX) SET_BIT(current_bits, _ENDSTOP(AXIS, MINMAX), (READ(_ENDSTOP_PIN(AXIS, MINMAX)) != isLogic(AXIS ##_## MINMAX)))
  // COPY_BIT: copy the value of SRC_BIT to DST_BIT in DST
  #define COPY_BIT(DST, SRC_BIT, DST_BIT) SET_BIT(DST, DST_BIT, TEST(DST, SRC_BIT))

  #define UPDATE_ENDSTOP(AXIS,MINMAX) do { \
      UPDATE_ENDSTOP_BIT(AXIS, MINMAX); \
      if (TEST_ENDSTOP(_ENDSTOP(AXIS, MINMAX))) { \
        _ENDSTOP_HIT(AXIS, MINMAX); \
        stepper.endstop_triggered(AXIS ##_AXIS); \
      } \
    } while(0)

  #if ENABLED(G38_PROBE_TARGET) && HAS_Z_PROBE_PIN && !(CORE_IS_XY || CORE_IS_XZ)
    // If G38 command is active check Z_MIN_PROBE for ALL movement
    if (printer.IsG38Move()) {
      UPDATE_ENDSTOP_BIT(Z, PROBE);
      if (TEST_ENDSTOP(_ENDSTOP(Z, PROBE))) {
        if      (stepper.current_block->steps[X_AXIS] > 0) { _ENDSTOP_HIT(X, MIN); stepper.endstop_triggered(X_AXIS); }
        else if (stepper.current_block->steps[Y_AXIS] > 0) { _ENDSTOP_HIT(Y, MIN); stepper.endstop_triggered(Y_AXIS); }
        else if (stepper.current_block->steps[Z_AXIS] > 0) { _ENDSTOP_HIT(Z, MIN); stepper.endstop_triggered(Z_AXIS); }
        setG38EndstopHit(true);
      }
    }
  #endif

  /**
   * Define conditions for checking endstops
   */

  #if IS_CORE
    #define S_(N) stepper.current_block->steps[CORE_AXIS_##N]
    #define D_(N) stepper.motor_direction(CORE_AXIS_##N)
  #endif

  #if CORE_IS_XY || CORE_IS_XZ
    /**
     * Head direction in -X axis for CoreXY and CoreXZ bots.
     *
     * If steps differ, both axes are moving.
     * If DeltaA == -DeltaB, the movement is only in the 2nd axis (Y or Z, handled below)
     * If DeltaA ==  DeltaB, the movement is only in the 1st axis (X)
     */
    #if MECH(COREXY) || MECH(COREXZ)
      #define X_CMP ==
    #else
      #define X_CMP !=
    #endif
    #define X_MOVE_TEST ( S_(1) != S_(2) || (S_(1) > 0 && D_(1) X_CMP D_(2)) )
    #define X_AXIS_HEAD X_HEAD
  #else
    #define X_MOVE_TEST stepper.current_block->steps[X_AXIS] > 0
    #define X_AXIS_HEAD X_AXIS
  #endif

  #if CORE_IS_XY || CORE_IS_YZ
    /**
     * Head direction in -Y axis for CoreXY / CoreYZ bots.
     *
     * If steps differ, both axes are moving
     * If DeltaA ==  DeltaB, the movement is only in the 1st axis (X or Y)
     * If DeltaA == -DeltaB, the movement is only in the 2nd axis (Y or Z)
     */
    #if MECH(COREYX) || MECH(COREYZ)
      #define Y_CMP ==
    #else
      #define Y_CMP !=
    #endif
    #define Y_MOVE_TEST ( S_(1) != S_(2) || (S_(1) > 0 && D_(1) Y_CMP D_(2)) )
    #define Y_AXIS_HEAD Y_HEAD
  #else
    #define Y_MOVE_TEST stepper.current_block->steps[Y_AXIS] > 0
    #define Y_AXIS_HEAD Y_AXIS
  #endif

  #if CORE_IS_XZ || CORE_IS_YZ
    /**
     * Head direction in -Z axis for CoreXZ or CoreYZ bots.
     *
     * If steps differ, both axes are moving
     * If DeltaA ==  DeltaB, the movement is only in the 1st axis (X or Y, already handled above)
     * If DeltaA == -DeltaB, the movement is only in the 2nd axis (Z)
     */
    #if MECH(COREZX) || MECH(COREZY)
      #define Z_CMP ==
    #else
      #define Z_CMP !=
    #endif
    #define Z_MOVE_TEST ( S_(1) != S_(2) || (S_(1) > 0 && D_(1) Z_CMP D_(2)) )
    #define Z_AXIS_HEAD Z_HEAD
  #else
    #define Z_MOVE_TEST stepper.current_block->steps[Z_AXIS] > 0
    #define Z_AXIS_HEAD Z_AXIS
  #endif

  // With Dual X, endstops are only checked in the homing direction for the active extruder
  #if ENABLED(DUAL_X_CARRIAGE)
    #define E0_ACTIVE stepper.current_block->active_extruder == 0
    #define X_MIN_TEST ((X_HOME_DIR < 0 && E0_ACTIVE) || (X2_HOME_DIR < 0 && !E0_ACTIVE))
    #define X_MAX_TEST ((X_HOME_DIR > 0 && E0_ACTIVE) || (X2_HOME_DIR > 0 && !E0_ACTIVE))
  #else
    #define X_MIN_TEST true
    #define X_MAX_TEST true
  #endif

  /**
   * Check and update endstops according to conditions
   */
  if (stepper.current_block) {

    if (X_MOVE_TEST) {
      if (stepper.motor_direction(X_AXIS_HEAD)) { // -direction
        #if HAS_X_MIN
          #if ENABLED(X_TWO_ENDSTOPS)
            UPDATE_ENDSTOP_BIT(X, MIN);
            #if HAS_X2_MIN
              UPDATE_ENDSTOP_BIT(X2, MIN);
            #else
              COPY_BIT(current_bits, X_MIN, X2_MIN);
            #endif
            test_two_x_endstops(X_MIN, X2_MIN);
          #else
            if (X_MIN_TEST) UPDATE_ENDSTOP(X, MIN);
          #endif
        #endif
      }
      else {  // +direction
        #if HAS_X_MAX
          #if ENABLED(X_TWO_ENDSTOPS)
            UPDATE_ENDSTOP_BIT(X, MAX);
            #if HAS_X2_MAX
              UPDATE_ENDSTOP_BIT(X2, MAX);
            #else
              COPY_BIT(current_bits, X_MAX, X2_MAX);
            #endif
            test_two_x_endstops(X_MAX, X2_MAX);
          #else
            if (X_MAX_TEST) UPDATE_ENDSTOP(X, MAX);
          #endif
        #endif
      }
    }

    if (Y_MOVE_TEST) {
      if (stepper.motor_direction(Y_AXIS_HEAD)) { // -direction
        #if HAS_Y_MIN
          #if ENABLED(Y_TWO_ENDSTOPS)
            UPDATE_ENDSTOP_BIT(Y, MIN);
            #if HAS_Y2_MIN
              UPDATE_ENDSTOP_BIT(Y2, MIN);
            #else
              COPY_BIT(current_bits, Y_MIN, Y2_MIN);
            #endif
            test_two_y_endstops(Y_MIN, Y2_MIN);
          #else
            UPDATE_ENDSTOP(Y, MIN);
          #endif
        #endif
      }
      else {  // +direction
        #if HAS_Y_MAX
          #if ENABLED(Y_TWO_ENDSTOPS)
            UPDATE_ENDSTOP_BIT(Y, MAX);
            #if HAS_Y2_MAX
              UPDATE_ENDSTOP_BIT(Y2, MAX);
            #else
              COPY_BIT(current_bits, Y_MAX, Y2_MAX);
            #endif
            test_two_y_endstops(Y_MAX, Y2_MAX);
          #else
            UPDATE_ENDSTOP(Y, MAX);
          #endif
        #endif
      }
    }

    if (Z_MOVE_TEST) {
      if (stepper.motor_direction(Z_AXIS_HEAD)) { // Z -direction. Gantry down, bed up.
        #if HAS_Z_MIN
          #if ENABLED(Z_TWO_ENDSTOPS)
            UPDATE_ENDSTOP_BIT(Z, MIN);
            #if HAS_Z2_MIN
              UPDATE_ENDSTOP_BIT(Z2, MIN);
            #else
              COPY_BIT(current_bits, Z_MIN, Z2_MIN);
            #endif
            test_two_z_endstops(Z_MIN, Z2_MIN);
          #else
            #if HAS_BED_PROBE && !HAS_Z_PROBE_PIN
              if (isProbeEndstop()) UPDATE_ENDSTOP(Z, MIN);
            #else
              UPDATE_ENDSTOP(Z, MIN);
            #endif
          #endif
        #endif

        // When closing the gap check the enabled probe
        #if HAS_BED_PROBE && HAS_Z_PROBE_PIN
          if (isProbeEndstop()) {
            UPDATE_ENDSTOP(Z, PROBE);
            if (TEST_ENDSTOP(Z_PROBE)) SBI(hit_bits, Z_PROBE);
          }
        #endif
      }
      else { // Z +direction. Gantry up, bed down.
        #if HAS_Z_MAX
          // Check both Z two endstops
          #if ENABLED(Z_TWO_ENDSTOPS)
            UPDATE_ENDSTOP_BIT(Z, MAX);
            #if HAS_Z2_MAX
              UPDATE_ENDSTOP_BIT(Z2, MAX);
            #else
              COPY_BIT(current_bits, Z_MAX, Z2_MAX);
            #endif
            test_two_z_endstops(Z_MAX, Z2_MAX);
          #else
            UPDATE_ENDSTOP(Z, MAX);
          #endif
        #endif
      }
    }

  } // stepper.current_block

  old_bits = current_bits;

} // Endstops::update()
