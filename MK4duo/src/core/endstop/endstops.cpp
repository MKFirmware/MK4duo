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
#include "../../platform/common/endstop_interrupts.h"

Endstops endstops;

// public:

#if IS_DELTA
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

uint16_t  Endstops::logic_bits  = 0,
          Endstops::pullup_bits = 0,
          Endstops::live_state  = 0;

// Private
uint8_t   Endstops::flag_bits = 0;

volatile uint8_t Endstops::hit_state = 0;

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

  #if HAS_POWER_CHECK && HAS_SD_SUPPORT
    SET_INPUT(POWER_CHECK_PIN);
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
      (false)
    #else
      (true)
    #endif
  );

  #if ENABLED(X_TWO_ENDSTOPS)
    x_endstop_adj = 0.0f;
  #endif
  #if ENABLED(Y_TWO_ENDSTOPS)
    y_endstop_adj = 0.0f;
  #endif
  #if ENABLED(Z_TWO_ENDSTOPS)
    z_endstop_adj = 0.0f;
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
    setLogic(X2_MAX, !X2_MAX_ENDSTOP_LOGIC);
    setLogic(Y2_MAX, !Y2_MAX_ENDSTOP_LOGIC);
    setLogic(Z2_MAX, !Z2_MAX_ENDSTOP_LOGIC);
    setLogic(Z_PROBE, !Z_PROBE_ENDSTOP_LOGIC);
    setLogic(FIL_RUNOUT, !FIL_RUNOUT_LOGIC);
    setLogic(DOOR_OPEN_SENSOR, !DOOR_OPEN_LOGIC);
    setLogic(POWER_CHECK_SENSOR, !POWER_CHECK_LOGIC);

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
    setLogic(X2_MAX, X2_MAX_ENDSTOP_LOGIC);
    setLogic(Y2_MAX, Y2_MAX_ENDSTOP_LOGIC);
    setLogic(Z2_MAX, Z2_MAX_ENDSTOP_LOGIC);
    setLogic(Z_PROBE, Z_PROBE_ENDSTOP_LOGIC);
    setLogic(FIL_RUNOUT, FIL_RUNOUT_LOGIC);
    setLogic(DOOR_OPEN_SENSOR, DOOR_OPEN_LOGIC);
    setLogic(POWER_CHECK_SENSOR, POWER_CHECK_LOGIC);

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
  setPullup(X2_MAX, ENDSTOPPULLUP_X2MAX);
  setPullup(Y2_MAX, ENDSTOPPULLUP_Y2MAX);
  setPullup(Z2_MAX, ENDSTOPPULLUP_Z2MAX);
  setPullup(Z_PROBE, ENDSTOPPULLUP_ZPROBE);
  setPullup(FIL_RUNOUT, PULLUP_FIL_RUNOUT);
  setPullup(DOOR_OPEN_SENSOR, PULLUP_DOOR_OPEN);
  setPullup(POWER_CHECK_SENSOR, PULLUP_POWER_CHECK);

}

// Called from HAL::Tick or HAL_temp_isr. Check endstop state if required
void Endstops::Tick() {
  #if ENABLED(PINS_DEBUGGING)
    run_monitor();  // report changes in endstop status
  #endif

  #if DISABLED(ENDSTOP_INTERRUPTS_FEATURE)
    update();
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

  #if HAS_POWER_CHECK && HAS_SD_SUPPORT
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

  #if HAS_POWER_CHECK && HAS_SD_SUPPORT
    // Power Check
    SERIAL_MV("Endstop Power Check Logic:", isLogic(POWER_CHECK_SENSOR) ? "true" : "false");
    SERIAL_EMV(" Pullup:", isPullup(POWER_CHECK_SENSOR) ? "true" : "false");
  #endif

}

void Endstops::report_state() {

  static uint8_t prev_hit_state = 0;

  if (hit_state && hit_state != prev_hit_state) {

    #if ENABLED(ULTRA_LCD)
      char chrX = ' ', chrY = ' ', chrZ = ' ', chrP = ' ';
      #define _SET_STOP_CHAR(A,C) (chr## A = C)
    #else
      #define _SET_STOP_CHAR(A,C) ;
    #endif

    #define _ENDSTOP_HIT_ECHO(A,C) do{ \
      SERIAL_MV(STRINGIFY(A) ":", planner.triggered_position_mm(_AXIS(A))); \
      _SET_STOP_CHAR(A,C); }while(0)

    #define _ENDSTOP_HIT_TEST(A,C) \
      if (TEST(hit_state, A##_MIN) || TEST(hit_state, A##_MAX)) \
        _ENDSTOP_HIT_ECHO(A,C)

    SERIAL_SM(ECHO, MSG_ENDSTOPS_HIT);
    _ENDSTOP_HIT_TEST(X, 'X');
    _ENDSTOP_HIT_TEST(Y, 'Y');
    _ENDSTOP_HIT_TEST(Z, 'Z');

    #if HAS_Z_PROBE_PIN
      #define P_AXIS Z_AXIS
      if (TEST(hit_state, Z_PROBE)) _ENDSTOP_HIT_ECHO(P, 'P');
    #endif
    SERIAL_EOL();

    #if ENABLED(ULTRA_LCD)
      lcd_status_printf_P(0, PSTR(MSG_LCD_ENDSTOPS " %c %c %c %c"), chrX, chrY, chrZ, chrP);
    #endif

    #if ENABLED(ABORT_ON_ENDSTOP_HIT) && HAS_SD_SUPPORT
      if (planner.abort_on_endstop_hit) {
        card.setSDprinting(false);
        card.closeFile();
        printer.quickstop_stepper();
        thermalManager.disable_all_heaters();
      }
    #endif
  }

  prev_hit_state = hit_state;

} // Endstops::report_state

// If the last move failed to trigger an endstop, call kill
void Endstops::validate_homing_move() {
  if (trigger_state()) hit_on_purpose();
  else printer.kill(PSTR(MSG_ERR_HOMING_FAILED));
}

/**
 * Constrain the given coordinates to the software endstops.
 */
void Endstops::clamp_to_software(float target[XYZ]) {

  if (!isSoftEndstop()) return;

  #if IS_DELTA
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

    mechanics.workspace_offset[axis] = mechanics.home_offset[axis] + mechanics.position_shift[axis];

    #if ENABLED(DUAL_X_CARRIAGE)
      if (axis == X_AXIS) {

        // In Dual X mode tools.hotend_offset[X] is T1's home position
        float dual_max_x = MAX(tools.hotend_offset[X_AXIS][1], X2_MAX_POS);

        if (tools.active_extruder != 0) {
          // T1 can move from X2_MIN_POS to X2_MAX_POS or X2 home position (whichever is larger)
          soft_endstop_min[X_AXIS] = X2_MIN_POS;
          soft_endstop_max[X_AXIS] = dual_max_x;
        }
        else if (mechanics.dual_x_carriage_mode == DXC_DUPLICATION_MODE) {
          // In Duplication Mode, T0 can move as far left as X_MIN_POS
          // but not so far to the right that T1 would move past the end
          soft_endstop_min[X_AXIS] = mechanics.base_min_pos[X_AXIS];
          soft_endstop_max[X_AXIS] = MIN(mechanics.base_max_pos[X_AXIS], dual_max_x - mechanics.duplicate_hotend_x_offset);
        }
        else {
          // In other modes, T0 can move from X_MIN_POS to X_MAX_POS
          soft_endstop_min[axis] = mechanics.base_min_pos[axis];
          soft_endstop_max[axis] = mechanics.base_max_pos[axis];
        }
      }
    #else
      soft_endstop_min[axis] = mechanics.base_min_pos[axis];
      soft_endstop_max[axis] = mechanics.base_max_pos[axis];
    #endif

    #if ENABLED(DEBUG_FEATURE)
      if (printer.debugFeature()) {
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

#define _ENDSTOP(AXIS, MINMAX)      AXIS ##_## MINMAX
#define _ENDSTOP_PIN(AXIS, MINMAX)  AXIS ##_## MINMAX ##_PIN

// update endstops - Called from ISR!
void Endstops::update() {

  if (!isEnabled() && !isProbeEnabled()) return;

  #define UPDATE_ENDSTOP_BIT(AXIS, MINMAX)  SET_BIT(live_state, _ENDSTOP(AXIS, MINMAX), (READ(_ENDSTOP_PIN(AXIS, MINMAX)) != isLogic(AXIS ##_## MINMAX)))
  #define COPY_LIVE_STATE(SRC_BIT, DST_BIT) SET_BIT(live_state, DST_BIT, TEST(live_state, SRC_BIT))

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
  #if HAS_X_MIN
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

  #if HAS_X_MAX
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

  #if HAS_Y_MIN
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

  #if HAS_Y_MAX
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

  #if HAS_Z_MIN
    #if ENABLED(Z_TWO_ENDSTOPS)
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

  #if HAS_Z_MAX
    // Check both Z two endstops
    #if ENABLED(Z_TWO_ENDSTOPS)
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
      if (!stepper.homing_dual_axis || dual_hit == 0b11) \
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
      #if HAS_X_MIN
        #if ENABLED(X_TWO_ENDSTOPS)
          PROCESS_DUAL_ENDSTOP(X, X2, MIN);
        #else
          if (X_MIN_TEST) PROCESS_ENDSTOP(X, MIN);
        #endif
      #endif
    }
    else {  // +direction
      #if HAS_X_MAX
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
      #if HAS_Y_MIN
        #if ENABLED(Y_TWO_ENDSTOPS)
          PROCESS_DUAL_ENDSTOP(Y, Y2, MIN);
        #else
          PROCESS_ENDSTOP(Y, MIN);
        #endif
      #endif
    }
    else {  // +direction
      #if HAS_Y_MAX
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
      #if HAS_Z_MIN
        #if ENABLED(Z_TWO_ENDSTOPS)
          PROCESS_DUAL_ENDSTOP(Z, Z2, MIN);
        #else
          PROCESS_ENDSTOP(Z, MIN);
        #endif
      #endif

      // When closing the gap check the enabled probe
      #if HAS_BED_PROBE && HAS_Z_PROBE_PIN
        if (isProbeEnabled()) PROCESS_ENDSTOP(Z, PROBE);
      #endif
    }
    else { // Z +direction. Gantry up, bed down.
      #if HAS_Z_MAX
        #if ENABLED(Z_TWO_ENDSTOPS)
          PROCESS_DUAL_ENDSTOP(Z, Z2, MAX);
        #else
          PROCESS_ENDSTOP(Z, MAX);
        #endif
      #endif
    }
  }

} // Endstops::update()
