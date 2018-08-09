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
 * stepper.cpp - A singleton object to execute motion plans using stepper motors
 *
 * Derived from Grbl
 * Copyright (c) 2009-2011 Simen Svale Skogsrud
 *
 * Grbl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Grbl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * The timer calculations of this module informed by the 'RepRap cartesian firmware' by Zack Smith
 * and Philipp Tiefenbacher.
 */

/**
 *         __________________________
 *        /|                        |\     _________________         ^
 *       / |                        | \   /|               |\        |
 *      /  |                        |  \ / |               | \       s
 *     /   |                        |   |  |               |  \      p
 *    /    |                        |   |  |               |   \     e
 *   +-----+------------------------+---+--+---------------+----+    e
 *   |               BLOCK 1            |      BLOCK 2          |    d
 *
 *                           time ----->
 *
 *  The trapezoid is the shape the speed curve over time. It starts at block->initial_rate, accelerates
 *  first block->accelerate_until step_events_completed, then keeps going at constant speed until
 *  step_events_completed reaches block->decelerate_after after which it decelerates until the trapezoid generator is reset.
 *  The slope of acceleration is calculated using v = u + at where t is the accumulated timer values of the steps so far.
 */

/**
 * MK4duo uses the Bresenham algorithm. For a detailed explanation of theory and
 * method see https://www.cs.helsinki.fi/group/goa/mallinnus/lines/bresenh.html
 */

/**
 * Jerk controlled movements planner added Apr 2018 by Eduardo José Tagle.
 * Equations based on Synthethos TinyG2 sources, but the fixed-point
 * implementation is new, as we are running the ISR with a variable period.
 * Also implemented the Bézier velocity curve evaluation in ARM assembler,
 * to avoid impacting ISR speed.
 */

#include "../../../MK4duo.h"
#include "stepper.h"

Stepper stepper;

/** Public Parameters */
uint16_t Stepper::direction_flag = 0;

#if ENABLED(X_TWO_ENDSTOPS) || ENABLED(Y_TWO_ENDSTOPS) || ENABLED(Z_TWO_ENDSTOPS)
  bool Stepper::homing_dual_axis = false;
#endif

uint8_t   Stepper::minimum_pulse    = 0;
uint32_t  Stepper::maximum_rate     = 0,
          Stepper::direction_delay  = 0;

/** Private Parameters */
block_t* Stepper::current_block = NULL;  // A pointer to the block currently being traced

uint8_t Stepper::last_direction_bits  = 0,
        Stepper::axis_did_move        = 0;

bool    Stepper::abort_current_block  = false;

#if DISABLED(COLOR_MIXING_EXTRUDER)
  uint8_t Stepper::last_moved_extruder = 0xFF;
#endif

#if ENABLED(X_TWO_ENDSTOPS)
  bool Stepper::locked_X_motor = false, Stepper::locked_X2_motor = false;
#endif
#if ENABLED(Y_TWO_ENDSTOPS)
  bool Stepper::locked_Y_motor = false, Stepper::locked_Y2_motor = false;
#endif
#if ENABLED(Z_TWO_ENDSTOPS)
  bool Stepper::locked_Z_motor = false, Stepper::locked_Z2_motor = false;
#endif

uint32_t  Stepper::acceleration_time  = 0,
          Stepper::deceleration_time  = 0;

uint8_t   Stepper::steps_per_isr = 0;

int32_t   Stepper::delta_error[XYZE] = { 0 };

uint32_t  Stepper::advance_dividend[XYZE] = { 0 },
          Stepper::advance_divisor        = 0,
          Stepper::step_events_completed  = 0,  // The number of step events executed in the current block
          Stepper::accelerate_until       = 0,  // The point from where we need to stop acceleration
          Stepper::decelerate_after       = 0,  // The point from where we need to start decelerating
          Stepper::step_event_count       = 0;  // The total event count for the current block

#if ENABLED(COLOR_MIXING_EXTRUDER)
  int32_t   Stepper::delta_error_m[MIXING_STEPPERS]       = { 0 };
  uint32_t  Stepper::advance_dividend_m[MIXING_STEPPERS]  = { 0 },
            Stepper::advance_divisor_m                    = 0;
#else
  uint8_t Stepper::active_extruder        = 0,
          Stepper::active_extruder_driver = 0;
#endif

#if ENABLED(BEZIER_JERK_CONTROL)
  int32_t __attribute__((used)) Stepper::bezier_A __asm__("bezier_A");    // A coefficient in Bézier speed curve with alias for assembler
  int32_t __attribute__((used)) Stepper::bezier_B __asm__("bezier_B");    // B coefficient in Bézier speed curve with alias for assembler
  int32_t __attribute__((used)) Stepper::bezier_C __asm__("bezier_C");    // C coefficient in Bézier speed curve with alias for assembler
  uint32_t __attribute__((used)) Stepper::bezier_F __asm__("bezier_F");   // F coefficient in Bézier speed curve with alias for assembler
  uint32_t __attribute__((used)) Stepper::bezier_AV __asm__("bezier_AV"); // AV coefficient in Bézier speed curve with alias for assembler
  #if ENABLED(__AVR__)
    bool __attribute__((used)) Stepper::A_negative __asm__("A_negative"); // If A coefficient was negative
  #endif
  bool Stepper::bezier_2nd_half = false;  // =false If Bézier curve has been initialized or not
#endif

uint32_t Stepper::nextMainISR = 0;

#if ENABLED(ADAPTIVE_STEP_SMOOTHING)
  static uint8_t oversampling_factor = 0;
#else
  constexpr uint8_t oversampling_factor = 0;
#endif

#if ENABLED(LIN_ADVANCE)

  constexpr uint32_t LA_ADV_NEVER         = 0xFFFFFFFF;
  uint32_t  Stepper::nextAdvanceISR       = LA_ADV_NEVER,
            Stepper::LA_isr_rate          = LA_ADV_NEVER;

  uint16_t  Stepper::LA_current_adv_steps = 0,
            Stepper::LA_final_adv_steps   = 0,
            Stepper::LA_max_adv_steps     = 0;

  int8_t    Stepper::LA_steps             = 0;

  bool      Stepper::LA_use_advance_lead  = false;

#endif // LIN_ADVANCE

int32_t Stepper::ticks_nominal = -1;
#if DISABLED(BEZIER_JERK_CONTROL)
  uint32_t Stepper::acc_step_rate = 0; // needed for deceleration start point
#endif

volatile int32_t Stepper::endstops_trigsteps[XYZ] = { 0 };

volatile int32_t  Stepper::count_position[NUM_AXIS]   = { 0 };

int8_t  Stepper::count_direction[NUM_AXIS]  = { 1, 1, 1, 1 };

#if ENABLED(LASER)
  int32_t Stepper::counter_L = 0;
  #if ENABLED(LASER_RASTER)
    int Stepper::counter_raster = 0;
  #endif // LASER_RASTER
#endif // LASER

/** Public Function */
void Stepper::init() {

  // Init Digipot Motor Current
  #if HAS_DIGIPOTSS || HAS_MOTOR_CURRENT_PWM
    digipot_init();
  #endif

  // Init Microstepping Pins
  #if HAS_MICROSTEPS
    microstep_init();
  #endif

  // Init TMC26x Steppers
  #if HAVE_DRV(TMC26X)
    tmc26x_init_to_defaults();
  #endif

  // Init TMC2130 Steppers
  #if HAVE_DRV(TMC2130)
    tmc2130_init_to_defaults();
  #endif

  // Init TMC2208 Steppers
  #if HAVE_DRV(TMC2208)
    tmc2208_init_to_defaults();
  #endif

  // TRAMS, TMC2130 and TMC2208 advanced settings
  #if HAS_TRINAMIC
    TMC_ADV()
  #endif

  // Init L6470 Steppers
  #if HAVE_DRV(L6470)
    L6470_init_to_defaults();
  #endif

  // Init Dir Pins
  #if HAS_X_DIR
    X_DIR_INIT;
  #endif
  #if HAS_X2_DIR
    X2_DIR_INIT;
  #endif
  #if HAS_Y_DIR
    Y_DIR_INIT;
    #if ENABLED(Y_TWO_STEPPER_DRIVERS) && HAS_Y2_DIR
      Y2_DIR_INIT;
    #endif
  #endif
  #if HAS_Z_DIR
    Z_DIR_INIT;
    #if ENABLED(Z_TWO_STEPPER_DRIVERS) && HAS_Z2_DIR
      Z2_DIR_INIT;
    #endif
  #endif
  #if HAS_E0_DIR
    E0_DIR_INIT;
  #endif
  #if HAS_E1_DIR
    E1_DIR_INIT;
  #endif
  #if HAS_E2_DIR
    E2_DIR_INIT;
  #endif
  #if HAS_E3_DIR
    E3_DIR_INIT;
  #endif
  #if HAS_E4_DIR
    E4_DIR_INIT;
  #endif
  #if HAS_E5_DIR
    E5_DIR_INIT;
  #endif

  // Init Enable Pins - steppers default to disabled.
  #if HAS_X_ENABLE
    X_ENABLE_INIT;
    if (!X_ENABLE_ON) X_ENABLE_WRITE(HIGH);
    #if (ENABLED(DUAL_X_CARRIAGE) || ENABLED(X_TWO_STEPPER_DRIVERS)) && HAS_X2_ENABLE
      X2_ENABLE_INIT;
      if (!X_ENABLE_ON) X2_ENABLE_WRITE(HIGH);
    #endif
  #endif
  #if HAS_Y_ENABLE
    Y_ENABLE_INIT;
    if (!Y_ENABLE_ON) Y_ENABLE_WRITE(HIGH);
    #if ENABLED(Y_TWO_STEPPER_DRIVERS) && HAS_Y2_ENABLE
      Y2_ENABLE_INIT;
      if (!Y_ENABLE_ON) Y2_ENABLE_WRITE(HIGH);
    #endif
  #endif
  #if HAS_Z_ENABLE
    Z_ENABLE_INIT;
    if (!Z_ENABLE_ON) Z_ENABLE_WRITE(HIGH);
    #if ENABLED(Z_TWO_STEPPER_DRIVERS) && HAS_Z2_ENABLE
      Z2_ENABLE_INIT;
      if (!Z_ENABLE_ON) Z2_ENABLE_WRITE(HIGH);
    #endif
  #endif
  #if HAS_E0_ENABLE
    E0_ENABLE_INIT;
    if (!E_ENABLE_ON) E0_ENABLE_WRITE(HIGH);
  #endif
  #if HAS_E1_ENABLE
    E1_ENABLE_INIT;
    if (!E_ENABLE_ON) E1_ENABLE_WRITE(HIGH);
  #endif
  #if HAS_E2_ENABLE
    E2_ENABLE_INIT;
    if (!E_ENABLE_ON) E2_ENABLE_WRITE(HIGH);
  #endif
  #if HAS_E3_ENABLE
    E3_ENABLE_INIT;
    if (!E_ENABLE_ON) E3_ENABLE_WRITE(HIGH);
  #endif
  #if HAS_E4_ENABLE
    E4_ENABLE_INIT;
    if (!E_ENABLE_ON) E4_ENABLE_WRITE(HIGH);
  #endif
  #if HAS_E5_ENABLE
    E5_ENABLE_INIT;
    if (!E_ENABLE_ON) E5_ENABLE_WRITE(HIGH);
  #endif

  #if ENABLED(MKR4) // MKR4 System
    #if HAS_E0E1
      OUT_WRITE_RELE(E0E1_CHOICE_PIN, LOW);
    #endif
    #if HAS_E0E2
      OUT_WRITE_RELE(E0E2_CHOICE_PIN, LOW);
    #endif
    #if HAS_E1E3
      OUT_WRITE_RELE(E1E3_CHOICE_PIN, LOW);
    #endif
  #elif ENABLED(MKR6) || ENABLED(MKR12) // MKR6 or MKR12 System
    #if HAS_EX1
      OUT_WRITE_RELE(EX1_CHOICE_PIN, LOW);
    #endif
    #if HAS_EX2
      OUT_WRITE_RELE(EX2_CHOICE_PIN, LOW);
    #endif
  #endif

  // Init Step Pins
  #if HAS_X_STEP
    X_STEP_INIT;
    X_STEP_WRITE(INVERT_X_STEP_PIN);
    #if ENABLED(X_TWO_STEPPER_DRIVERS) || ENABLED(DUAL_X_CARRIAGE)
      X2_STEP_INIT;
      X2_STEP_WRITE(INVERT_X_STEP_PIN);
    #endif
    disable_X();
  #endif

  #if HAS_Y_STEP
    Y_STEP_INIT;
    Y_STEP_WRITE(INVERT_Y_STEP_PIN);
    #if ENABLED(Y_TWO_STEPPER_DRIVERS) && HAS_Y2_STEP
      Y2_STEP_INIT;
      Y2_STEP_WRITE(INVERT_Y_STEP_PIN);
    #endif
    disable_Y();
  #endif

  #if HAS_Z_STEP
    Z_STEP_INIT;
    Z_STEP_WRITE(INVERT_Y_STEP_PIN);
    #if ENABLED(Z_TWO_STEPPER_DRIVERS) && HAS_Z2_STEP
      Z2_STEP_INIT;
      Z2_STEP_WRITE(INVERT_Z_STEP_PIN);
    #endif
    disable_Z();
  #endif

  #if DRIVER_EXTRUDERS > 0 && HAS_E0_STEP
    E0_STEP_INIT;
    E_STEP_WRITE(0, INVERT_E_STEP_PIN);
    disable_E0();
  #endif
  #if DRIVER_EXTRUDERS > 1 && HAS_E1_STEP
    E1_STEP_INIT;
    E_STEP_WRITE(1, INVERT_E_STEP_PIN);
    disable_E1();
  #endif
  #if DRIVER_EXTRUDERS > 2 && HAS_E2_STEP
    E2_STEP_INIT;
    E_STEP_WRITE(2, INVERT_E_STEP_PIN);
    disable_E2();
  #endif
  #if DRIVER_EXTRUDERS > 3 && HAS_E3_STEP
    E3_STEP_INIT;
    E_STEP_WRITE(3, INVERT_E_STEP_PIN);
    disable_E3();
  #endif
  #if DRIVER_EXTRUDERS > 4 && HAS_E4_STEP
    E4_STEP_INIT;
    E_STEP_WRITE(4, INVERT_E_STEP_PIN);
    disable_E4();
  #endif
  #if DRIVER_EXTRUDERS > 5 && HAS_E5_STEP
    E5_STEP_INIT;
    E_STEP_WRITE(5, INVERT_E_STEP_PIN);
    disable_E5();
  #endif

  #if HAS_EXT_ENCODER
    // Initialize enc sensors
    #if HAS_E0_ENC
      #if ENABLED(E0_ENC_PULLUP)
        SET_INPUT_PULLUP(E0_ENC_PIN);
      #else
        SET_INPUT(E0_ENC_PIN);
      #endif
    #endif
    #if HAS_E1_ENC
      #if ENABLED(E1_ENC_PULLUP)
        SET_INPUT_PULLUP(E1_ENC_PIN);
      #else
        SET_INPUT(E1_ENC_PIN);
      #endif
    #endif
    #if HAS_E2_ENC
      #if ENABLED(E2_ENC_PULLUP)
        SET_INPUT_PULLUP(E2_ENC_PIN);
      #else
        SET_INPUT(E2_ENC_PIN);
      #endif
    #endif
    #if HAS_E3_ENC
      #if ENABLED(E3_ENC_PULLUP)
        SET_INPUT_PULLUP(E3_ENC_PIN);
      #else
        SET_INPUT(E3_ENC_PIN);
      #endif
    #endif
    #if HAS_E4_ENC
      #if ENABLED(E4_ENC_PULLUP)
        SET_INPUT_PULLUP(E4_ENC_PIN);
      #else
        SET_INPUT(E4_ENC_PIN);
      #endif
    #endif
    #if HAS_E5_ENC
      #if ENABLED(E5_ENC_PULLUP)
        SET_INPUT_PULLUP(E5_ENC_PIN);
      #else
        SET_INPUT(E5_ENC_PIN);
      #endif
    #endif

    HAL::delayMilliseconds(1);

    #if HAS_E0_ENC
      tools.encLastSignal[0] = READ_ENCODER(E0_ENC_PIN);
    #endif
    #if HAS_E1_ENC
      tools.encLastSignal[1] = READ_ENCODER(E1_ENC_PIN);
    #endif
    #if HAS_E2_ENC
      tools.encLastSignal[2] = READ_ENCODER(E2_ENC_PIN);
    #endif
    #if HAS_E3_ENC
      tools.encLastSignal[3] = READ_ENCODER(E3_ENC_PIN);
    #endif
    #if HAS_E4_ENC
      tools.encLastSignal[4] = READ_ENCODER(E4_ENC_PIN);
    #endif
    #if HAS_E5_ENC
      tools.encLastSignal[5] = READ_ENCODER(E5_ENC_PIN);
    #endif

  #endif // HAS_EXT_ENCODER

  // Init Stepper ISR to 128 Hz for quick starting
  HAL_timer_start(STEPPER_TIMER, 128);

  ENABLE_STEPPER_INTERRUPT();

  endstops.setEnabled(true); // Start with endstops active. After homing they can be disabled
  sei();

  set_directions(); // Init directions to last_direction_bits = 0
}

void Stepper::factory_parameters() {
  constexpr bool tmpdir[] = { INVERT_X_DIR, INVERT_Y_DIR, INVERT_Z_DIR, INVERT_E0_DIR, INVERT_E1_DIR, INVERT_E2_DIR, INVERT_E3_DIR, INVERT_E4_DIR, INVERT_E5_DIR };
  LOOP_XYZE_N(axis) setStepDir((AxisEnum)axis, tmpdir[axis]);

  direction_delay = DIRECTION_STEPPER_DELAY;
  minimum_pulse   = MINIMUM_STEPPER_PULSE;
  maximum_rate    = MAXIMUM_STEPPER_RATE;
}

/**
 * This is called by the interrupt service routine to execute steps.
 */
void Stepper::Step() {

  #if DISABLED(__AVR__)
    // Disable interrupts, to avoid ISR preemption while we reprogram the period
    // (AVR enters the ISR with global interrupts disabled, so no need to do it here)
    DISABLE_ISRS();
  #endif

  // Program timer compare for the maximum period, so it does NOT
  // flag an interrupt while this ISR is running - So changes from small
  // periods to big periods are respected and the timer does not reset to 0
  HAL_timer_set_count(STEPPER_TIMER, HAL_TIMER_TYPE_MAX);

  // Count of ticks for the next ISR
  hal_timer_t next_isr_ticks = 0;

  // Limit the amount of iterations
  uint8_t max_loops = 10;

  // We need this variable here to be able to use it in the following loop
  hal_timer_t min_ticks;

  do {

    // Enable ISRs to reduce USART processing latency
    ENABLE_ISRS();

    // Run main stepping pulse phase ISR if we have to
    if (!nextMainISR) pulse_phase_step();

    #if ENABLED(LIN_ADVANCE)
      // Run linear advance stepper ISR
      if (!nextAdvanceISR) nextAdvanceISR = lin_advance_step();
    #endif

    // Run main stepping block phase ISR
    if (!nextMainISR) nextMainISR = block_phase_step();

    #if ENABLED(LIN_ADVANCE)
      uint32_t interval = MIN(nextAdvanceISR, nextMainISR); // Nearest time interval
    #else
      uint32_t interval = nextMainISR;                      // Remaining stepper ISR time
    #endif

    // Limit the value to the maximum possible value of the timer
    NOMORE(interval, HAL_TIMER_TYPE_MAX);

    // Compute the time remaining for the main isr
    nextMainISR -= interval;

    #if ENABLED(LIN_ADVANCE)
      // Compute the time remaining for the advance isr
      if (nextAdvanceISR != LA_ADV_NEVER) nextAdvanceISR -= interval;
    #endif

    /**
     * This needs to avoid a race-condition caused by interleaving
     * of interrupts required by both the LA and Stepper algorithms.
     *
     * Assume the following tick times for stepper pulses:
     *   Stepper ISR (S):  1 1000 2000 3000 4000
     *   Linear Adv. (E): 10 1010 2010 3010 4010
     *
     * The current algorithm tries to interleave them, giving:
     *  1:S 10:E 1000:S 1010:E 2000:S 2010:E 3000:S 3010:E 4000:S 4010:E
     *
     * Ideal timing would yield these delta periods:
     *  1:S  9:E  990:S   10:E  990:S   10:E  990:S   10:E  990:S   10:E
     *
     * But, since each event must fire an ISR with a minimum duration, the
     * minimum delta might be 900, so deltas under 900 get rounded up:
     *  900:S d900:E d990:S d900:E d990:S d900:E d990:S d900:E d990:S d900:E
     *
     * It works, but divides the speed of all motors by half, leading to a sudden
     * reduction to 1/2 speed! Such jumps in speed lead to lost steps (not even
     * accounting for double/quad stepping, which makes it even worse).
     */

    // Compute the tick count for the next ISR
    next_isr_ticks += interval;

    /**
     * The following section must be done with global interrupts disabled.
     * We want nothing to interrupt it, as that could mess the calculations
     * we do for the next value to program in the period register of the
     * stepper timer and lead to skipped ISRs (if the value we happen to program
     * is less than the current count due to something preempting between the
     * read and the write of the new period value).
     */
    DISABLE_ISRS();

    /**
     * Get the current tick value + margin
     * Assuming at least 6µs between calls to this ISR...
     * On AVR the ISR epilogue+prologue is estimated at 100 instructions - Give 8µs as margin
     * On ARM the ISR epilogue+prologue is estimated at 20 instructions - Give 1µs as margin
     */
    min_ticks = HAL_timer_get_current_count(STEPPER_TIMER) + hal_timer_t(STEPPER_TIMER_MAX_INTERVAL); // ISR never takes more than 1ms, so this shouldn't cause trouble

    /**
     * NB: If for some reason the stepper monopolizes the MPU, eventually the
     * timer will wrap around (and so will 'next_isr_ticks'). So, limit the
     * loop to 10 iterations. Beyond that, there's no way to ensure correct pulse
     * timing, since the MCU isn't fast enough.
     */
    if (!--max_loops) next_isr_ticks = min_ticks;

    // Advance pulses if not enough time to wait for the next ISR
  } while (next_isr_ticks < min_ticks);

  // Schedule next interrupt
  HAL_timer_set_count(STEPPER_TIMER, hal_timer_t(next_isr_ticks));

  // Don't forget to finally reenable interrupts
  ENABLE_ISRS();

}

/**
 * Check if the given block is busy or not - Must not be called from ISR contexts
 * The current_block could change in the middle of the read by an Stepper ISR, so
 * we must explicitly prevent that!
 */
bool Stepper::is_block_busy(const block_t* const block) {

  #if ENABLED(__AVR__)

    // Keep reading until 2 consecutive reads return the same value,
    // meaning there was no update in-between caused by an interrupt.
    // This works because stepper ISRs happen at a slower rate than
    // successive reads of a variable, so 2 consecutive reads with
    // the same value means no interrupt updated it.
    block_t* vold, *vnew = current_block;
    sw_barrier();
    do {
      vold = vnew;
      vnew = current_block;
      sw_barrier();
    } while (vold != vnew);

  #else // CPU 32 BIT

    block_t *vnew = current_block;

  #endif

  // Return if the block is busy or not
  return block == vnew;

}

/**
 * Get a stepper's position in steps.
 */
int32_t Stepper::position(const AxisEnum axis) {

  #if ENABLED(__AVR__)
    // Protect the access to the position. Only required for AVR, as
    //  any 32bit CPU offers atomic access to 32bit variables
    const bool isr_enabled = STEPPER_ISR_ENABLED();
    if (isr_enabled) DISABLE_STEPPER_INTERRUPT();
  #endif

  const int32_t v = count_position[axis];

  #if ENABLED(__AVR__)
    // Reenable Stepper ISR
    if (isr_enabled) ENABLE_STEPPER_INTERRUPT();
  #endif

  return v;
}

void Stepper::report_positions() {

  // Disable stepper ISR
  const bool isr_enabled = STEPPER_ISR_ENABLED();
  if (isr_enabled) DISABLE_STEPPER_INTERRUPT();

  const int32_t xpos = count_position[X_AXIS],
                ypos = count_position[Y_AXIS],
                zpos = count_position[Z_AXIS];

  // Reenable Stepper ISR
  if (isr_enabled) ENABLE_STEPPER_INTERRUPT();

  #if CORE_IS_XY || CORE_IS_XZ || IS_SCARA
    SERIAL_MSG(MSG_COUNT_A);
  #elif MECH(DELTA)
    SERIAL_MSG(MSG_COUNT_ALPHA);
  #else
    SERIAL_MSG(MSG_COUNT_X);
  #endif
  SERIAL_VAL(xpos);

  #if CORE_IS_XY || CORE_IS_YZ || IS_SCARA
    SERIAL_MSG(" B:");
  #elif MECH(DELTA)
    SERIAL_MSG(" Beta:");
  #else
    SERIAL_MSG(" Y:");
  #endif
  SERIAL_VAL(ypos);

  #if CORE_IS_XZ || CORE_IS_YZ
    SERIAL_MSG(" C:");
  #elif MECH(DELTA)
    SERIAL_MSG(" Teta:");
  #else
    SERIAL_MSG(" Z:");
  #endif
  SERIAL_VAL(zpos);

  SERIAL_EOL();
}

/**
 * Set the stepper direction of each axis
 *
 *   COREXY: X_AXIS=A_AXIS and Y_AXIS=B_AXIS
 *   COREYX: X_AXIS=A_AXIS and Y_AXIS=B_AXIS
 *   COREXZ: X_AXIS=A_AXIS and Z_AXIS=C_AXIS
 *   COREZX: X_AXIS=A_AXIS and Z_AXIS=C_AXIS
 */
void Stepper::set_directions() {

  #if HAS_X_DIR
    if (motor_direction(X_AXIS)) {
      set_X_dir(isStepDir(X_AXIS));
      count_direction[X_AXIS] = -1;
    }
    else {
      set_X_dir(!isStepDir(X_AXIS));
      count_direction[X_AXIS] = 1;
    }
  #endif

  #if HAS_Y_DIR
    if (motor_direction(Y_AXIS)) {
      set_Y_dir(isStepDir(Y_AXIS));
      count_direction[Y_AXIS] = -1;
    }
    else {
      set_Y_dir(!isStepDir(Y_AXIS));
      count_direction[Y_AXIS] = 1;
    }
  #endif

  #if HAS_Z_DIR
    if (motor_direction(Z_AXIS)) {
      set_Z_dir(isStepDir(Z_AXIS));
      count_direction[Z_AXIS] = -1;
    }
    else {
      set_Z_dir(!isStepDir(Z_AXIS));
      count_direction[Z_AXIS] = 1;
    }
  #endif

  #if HAS_EXTRUDERS && DISABLED(LIN_ADVANCE)
    #if ENABLED(COLOR_MIXING_EXTRUDER)
      if (motor_direction(E_AXIS)) {
        MIXING_STEPPERS_LOOP(j) REV_E_DIR(j);
        count_direction[E_AXIS] = -1;
      }
      else {
        MIXING_STEPPERS_LOOP(j) NORM_E_DIR(j);
        count_direction[E_AXIS] = 1;
      }
    #else
      if (motor_direction(E_AXIS)) {
        REV_E_DIR(active_extruder_driver);
        count_direction[E_AXIS] = -1;
      }
      else {
        NORM_E_DIR(active_extruder_driver);
        count_direction[E_AXIS] = 1;
      }
    #endif
  #endif // HAS_EXTRUDERS && DISABLED(LIN_ADVANCE)

  #if HAS_EXT_ENCODER
    tools.encLastDir[active_extruder] = count_direction[E_AXIS];
  #endif

  // After changing directions, an small delay could be needed.
  // Min delay is 50 Nanoseconds
  if (direction_delay >= 50) HAL::delayNanoseconds(direction_delay);

}

/**
 * Enabled or Disable one axis or all stepper driver
 */
void Stepper::enable_X() {
  #if HAS_X2_ENABLE
    X_ENABLE_WRITE( X_ENABLE_ON);
    X2_ENABLE_WRITE(X_ENABLE_ON);
  #elif HAS_X_ENABLE
    X_ENABLE_WRITE(X_ENABLE_ON);
  #endif
}
void Stepper::disable_X() {
  #if HAS_X2_ENABLE
    X_ENABLE_WRITE( !X_ENABLE_ON);
    X2_ENABLE_WRITE(!X_ENABLE_ON);
    printer.setXHomed(false);
  #elif HAS_X_ENABLE
    X_ENABLE_WRITE(!X_ENABLE_ON);
    printer.setXHomed(false);
  #endif
}

void Stepper::enable_Y() {
  #if HAS_Y2_ENABLE
    Y_ENABLE_WRITE( Y_ENABLE_ON);
    Y2_ENABLE_WRITE(Y_ENABLE_ON);
  #elif HAS_Y_ENABLE
    Y_ENABLE_WRITE(Y_ENABLE_ON);
  #endif
}
void Stepper::disable_Y() {
  #if HAS_Y2_ENABLE
    Y_ENABLE_WRITE( !Y_ENABLE_ON);
    Y2_ENABLE_WRITE(!Y_ENABLE_ON);
    printer.setYHomed(false);
  #elif HAS_Y_ENABLE
    Y_ENABLE_WRITE(!Y_ENABLE_ON);
    printer.setYHomed(false);
  #endif
}

void Stepper::enable_Z() {
  #if HAS_Z4_ENABLE
    Z_ENABLE_WRITE( Z_ENABLE_ON);
    Z2_ENABLE_WRITE(Z_ENABLE_ON);
    Z3_ENABLE_WRITE(Z_ENABLE_ON);
    Z4_ENABLE_WRITE(Z_ENABLE_ON);
  #elif HAS_Z3_ENABLE
    Z_ENABLE_WRITE( Z_ENABLE_ON);
    Z2_ENABLE_WRITE(Z_ENABLE_ON);
    Z3_ENABLE_WRITE(Z_ENABLE_ON);
  #elif HAS_Z2_ENABLE
    Z_ENABLE_WRITE( Z_ENABLE_ON);
    Z2_ENABLE_WRITE(Z_ENABLE_ON);
  #elif HAS_Z_ENABLE
    Z_ENABLE_WRITE( Z_ENABLE_ON);
  #endif
}
void Stepper::disable_Z() {
  #if HAS_Z4_ENABLE
    Z_ENABLE_WRITE( !Z_ENABLE_ON);
    Z2_ENABLE_WRITE(!Z_ENABLE_ON);
    Z3_ENABLE_WRITE(!Z_ENABLE_ON);
    Z4_ENABLE_WRITE(!Z_ENABLE_ON);
    printer.setZHomed(false);
  #elif HAS_Z3_ENABLE
    Z_ENABLE_WRITE( !Z_ENABLE_ON);
    Z2_ENABLE_WRITE(!Z_ENABLE_ON);
    Z3_ENABLE_WRITE(!Z_ENABLE_ON);
    printer.setZHomed(false);
  #elif HAS_Z2_ENABLE
    Z_ENABLE_WRITE( !Z_ENABLE_ON);
    Z2_ENABLE_WRITE(!Z_ENABLE_ON);
    printer.setZHomed(false);
  #elif HAS_Z_ENABLE
    Z_ENABLE_WRITE( !Z_ENABLE_ON);
    printer.setZHomed(false);
  #endif
}

void Stepper::enable_E() {
  enable_E0();
  enable_E1();
  enable_E2();
  enable_E3();
  enable_E4();
  enable_E5();
}
void Stepper::disable_E() {
  disable_E0();
  disable_E1();
  disable_E2();
  disable_E3();
  disable_E4();
  disable_E5();
}

void Stepper::enable_all() {
  #if HAS_POWER_SWITCH
    powerManager.power_on();
  #endif
  enable_X();
  enable_Y();
  enable_Z();
  enable_E();
}
void Stepper::disable_all() {
  disable_X();
  disable_Y();
  disable_Z();
  disable_E();
}

void Stepper::enable_E0() {
  #if ENABLED(COLOR_MIXING_EXTRUDER)
    #if MIXING_STEPPERS > 5
      E0_ENABLE_WRITE(E_ENABLE_ON);
      E1_ENABLE_WRITE(E_ENABLE_ON);
      E2_ENABLE_WRITE(E_ENABLE_ON);
      E3_ENABLE_WRITE(E_ENABLE_ON);
      E4_ENABLE_WRITE(E_ENABLE_ON);
      E5_ENABLE_WRITE(E_ENABLE_ON);
    #elif MIXING_STEPPERS > 4
      E0_ENABLE_WRITE(E_ENABLE_ON);
      E1_ENABLE_WRITE(E_ENABLE_ON);
      E2_ENABLE_WRITE(E_ENABLE_ON);
      E3_ENABLE_WRITE(E_ENABLE_ON);
      E4_ENABLE_WRITE(E_ENABLE_ON);
    #elif MIXING_STEPPERS > 3
      E0_ENABLE_WRITE(E_ENABLE_ON);
      E1_ENABLE_WRITE(E_ENABLE_ON);
      E2_ENABLE_WRITE(E_ENABLE_ON);
      E3_ENABLE_WRITE(E_ENABLE_ON);
    #elif MIXING_STEPPERS > 2
      E0_ENABLE_WRITE(E_ENABLE_ON);
      E1_ENABLE_WRITE(E_ENABLE_ON);
      E2_ENABLE_WRITE(E_ENABLE_ON);
    #else
      E0_ENABLE_WRITE(E_ENABLE_ON);
      E1_ENABLE_WRITE(E_ENABLE_ON);
    #endif

  #else // !COLOR_MIXING_EXTRUDER

    #if (DRIVER_EXTRUDERS > 0) && HAS_E0_ENABLE
      E0_ENABLE_WRITE(E_ENABLE_ON);
    #endif

  #endif

}
void Stepper::disable_E0() {
  #if ENABLED(COLOR_MIXING_EXTRUDER)
    #if MIXING_STEPPERS > 5
      E0_ENABLE_WRITE(!E_ENABLE_ON);
      E1_ENABLE_WRITE(!E_ENABLE_ON);
      E2_ENABLE_WRITE(!E_ENABLE_ON);
      E3_ENABLE_WRITE(!E_ENABLE_ON);
      E4_ENABLE_WRITE(!E_ENABLE_ON);
      E5_ENABLE_WRITE(!E_ENABLE_ON);
    #elif MIXING_STEPPERS > 4
      E0_ENABLE_WRITE(!E_ENABLE_ON);
      E1_ENABLE_WRITE(!E_ENABLE_ON);
      E2_ENABLE_WRITE(!E_ENABLE_ON);
      E3_ENABLE_WRITE(!E_ENABLE_ON);
      E4_ENABLE_WRITE(!E_ENABLE_ON);
    #elif MIXING_STEPPERS > 3
      E0_ENABLE_WRITE(!E_ENABLE_ON);
      E1_ENABLE_WRITE(!E_ENABLE_ON);
      E2_ENABLE_WRITE(!E_ENABLE_ON);
      E3_ENABLE_WRITE(!E_ENABLE_ON);
    #elif MIXING_STEPPERS > 2
      E0_ENABLE_WRITE(!E_ENABLE_ON);
      E1_ENABLE_WRITE(!E_ENABLE_ON);
      E2_ENABLE_WRITE(!E_ENABLE_ON);
    #else
      E0_ENABLE_WRITE(!E_ENABLE_ON);
      E1_ENABLE_WRITE(!E_ENABLE_ON);
    #endif

  #else // !COLOR_MIXING_EXTRUDER

    #if (DRIVER_EXTRUDERS > 0) && HAS_E0_ENABLE
      E0_ENABLE_WRITE(!E_ENABLE_ON);
    #endif

  #endif
}

/**
 * Handle a triggered endstop
 */
void Stepper::endstop_triggered(const AxisEnum axis) {

  // Disable stepper ISR
  const bool isr_enabled = STEPPER_ISR_ENABLED();
  if (isr_enabled) DISABLE_STEPPER_INTERRUPT();

  #if IS_CORE

    endstops_trigsteps[axis] = 0.5f * (
      axis == CORE_AXIS_2 ? CORESIGN(count_position[CORE_AXIS_1] - count_position[CORE_AXIS_2])
                          : count_position[CORE_AXIS_1] + count_position[CORE_AXIS_2]
    );

  #else // !COREXY && !COREXZ && !COREYZ

    endstops_trigsteps[axis] = count_position[axis];

  #endif // !COREXY && !COREXZ && !COREYZ

  // Discard the rest of the move if there is a current block
  quick_stop();

  // Reenable Stepper ISR
  if (isr_enabled) ENABLE_STEPPER_INTERRUPT();

}

/**
 * Triggered position of an axis in steps
 */
int32_t Stepper::triggered_position(const AxisEnum axis) {
  #if ENABLED(__AVR__)
    // Protect the access to the position. Only required for AVR, as
    //  any 32bit CPU offers atomic access to 32bit variables
    const bool isr_enabled = STEPPER_ISR_ENABLED();
    if (isr_enabled) DISABLE_STEPPER_INTERRUPT();
  #endif

  const int32_t v = endstops_trigsteps[axis];

  #if ENABLED(__AVR__)
    // Reenable Stepper ISR
    if (isr_enabled) ENABLE_STEPPER_INTERRUPT();
  #endif

  return v;
}

/**
 * Software-controlled Stepper Motor Current
 */
#if HAS_DIGIPOTSS

  void Stepper::digitalPotWrite(int address, int value) {
    WRITE(DIGIPOTSS_PIN, LOW);  // take the SS pin low to select the chip
    SPI.transfer(address);      //  send in the address and value via SPI:
    SPI.transfer(value);
    WRITE(DIGIPOTSS_PIN, HIGH); // take the SS pin high to de-select the chip:
    //HAL::delayMilliseconds(10);
  }
#endif

#if HAS_DIGIPOTSS || HAS_MOTOR_CURRENT_PWM

  void Stepper::digipot_current(uint8_t driver, int current) {
    #if HAS_DIGIPOTSS
      const uint8_t digipot_ch[] = DIGIPOT_CHANNELS;
      digitalPotWrite(digipot_ch[driver], current);
    #elif HAS_MOTOR_CURRENT_PWM
      #define _WRITE_CURRENT_PWM(P) analogWrite(P, 255L * current / (MOTOR_CURRENT_PWM_RANGE))
      switch (driver) {
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_XY)
          case 0: _WRITE_CURRENT_PWM(MOTOR_CURRENT_PWM_XY_PIN); break;
        #endif
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_Z)
          case 1: _WRITE_CURRENT_PWM(MOTOR_CURRENT_PWM_Z_PIN); break;
        #endif
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_E)
          case 2: _WRITE_CURRENT_PWM(MOTOR_CURRENT_PWM_E_PIN); break;
        #endif
      }
    #endif
  }

#endif

#if HAS_MICROSTEPS

  /**
   * Software-controlled Microstepping
   */
  void Stepper::microstep_ms(uint8_t driver, int8_t ms1, int8_t ms2) {
    if (ms1 >= 0) switch (driver) {
      #if HAS_X_MICROSTEPS
        case 0: WRITE(X_MS1_PIN, ms1); break;
      #endif
      #if HAS_Y_MICROSTEPS
        case 1: WRITE(Y_MS1_PIN, ms1); break;
      #endif
      #if HAS_Z_MICROSTEPS
        case 2: WRITE(Z_MS1_PIN, ms1); break;
      #endif
      #if HAS_E0_MICROSTEPS
        case 3: WRITE(E0_MS1_PIN, ms1); break;
      #endif
      #if HAS_E1_MICROSTEPS
        case 4: WRITE(E1_MS1_PIN, ms1); break;
      #endif
      #if HAS_E2_MICROSTEPS
        case 5: WRITE(E2_MS1_PIN, ms1); break;
      #endif
      #if HAS_E3_MICROSTEPS
        case 6: WRITE(E3_MS1_PIN, ms1); break;
      #endif
      #if HAS_E4_MICROSTEPS
        case 7: WRITE(E4_MS1_PIN, ms1); break;
      #endif
      #if HAS_E5_MICROSTEPS
        case 8: WRITE(E5_MS1_PIN, ms1); break;
      #endif
    }
    #if !MB(ALLIGATOR_R2) && !MB(ALLIGATOR_R3)
      if (ms2 >= 0) switch (driver) {
        #if HAS_X_MICROSTEPS
          case 0: WRITE(X_MS2_PIN, ms2); break;
        #endif
        #if HAS_Y_MICROSTEPS
          case 1: WRITE(Y_MS2_PIN, ms2); break;
        #endif
        #if HAS_Z_MICROSTEPS
          case 2: WRITE(Z_MS2_PIN, ms2); break;
        #endif
        #if HAS_E0_MICROSTEPS
          case 3: WRITE(E0_MS2_PIN, ms2); break;
        #endif
        #if HAS_E1_MICROSTEPS
          case 4: WRITE(E1_MS2_PIN, ms2); break;
        #endif
        #if HAS_E2_MICROSTEPS
          case 5: WRITE(E2_MS2_PIN, ms2); break;
        #endif
        #if HAS_E3_MICROSTEPS
          case 6: WRITE(E3_MS2_PIN, ms2); break;
        #endif
        #if HAS_E4_MICROSTEPS
          case 7: WRITE(E4_MS2_PIN, ms2); break;
        #endif
        #if HAS_E5_MICROSTEPS
          case 8: WRITE(E5_MS2_PIN, ms2); break;
        #endif
      }
    #else
      UNUSED(ms2);
    #endif
  }

  void Stepper::microstep_mode(uint8_t driver, uint8_t stepping_mode) {
    switch (stepping_mode) {
      case 1: microstep_ms(driver,  MICROSTEP1); break;
      case 2: microstep_ms(driver,  MICROSTEP2); break;
      case 4: microstep_ms(driver,  MICROSTEP4); break;
      case 8: microstep_ms(driver,  MICROSTEP8); break;
      case 16: microstep_ms(driver, MICROSTEP16); break;
      #if MB(ALLIGATOR_R2) || MB(ALLIGATOR_R3)
        case 32: microstep_ms(driver, MICROSTEP32); break;
      #endif
    }
  }

  void Stepper::microstep_readings() {
    SERIAL_MSG(MSG_MICROSTEP_MS1_MS2);
    #if HAS_X_MICROSTEPS
      SERIAL_MSG(MSG_MICROSTEP_X);
      SERIAL_VAL(READ(X_MS1_PIN));
      #if PIN_EXISTS(X_MS2)
        SERIAL_EV(READ(X_MS2_PIN));
      #else
        SERIAL_EOL();
      #endif
    #endif
    #if HAS_Y_MICROSTEPS
      SERIAL_MSG(MSG_MICROSTEP_Y);
      SERIAL_VAL(READ(Y_MS1_PIN));
      #if PIN_EXISTS(Y_MS2)
        SERIAL_EV(READ(Y_MS2_PIN));
      #else
        SERIAL_EOL();
      #endif
    #endif
    #if HAS_Z_MICROSTEPS
      SERIAL_MSG(MSG_MICROSTEP_Z);
      SERIAL_VAL(READ(Z_MS1_PIN));
      #if PIN_EXISTS(Z_MS2)
        SERIAL_EV(READ(Z_MS2_PIN));
      #else
        SERIAL_EOL();
      #endif
    #endif
    #if HAS_E0_MICROSTEPS
      SERIAL_MSG(MSG_MICROSTEP_E0);
      SERIAL_VAL(READ(E0_MS1_PIN));
      #if PIN_EXISTS(E0_MS2)
        SERIAL_EV(READ(E0_MS2_PIN));
      #else
        SERIAL_EOL();
      #endif
    #endif
    #if HAS_E1_MICROSTEPS
      SERIAL_MSG(MSG_MICROSTEP_E1);
      SERIAL_VAL(READ(E1_MS1_PIN));
      #if PIN_EXISTS(E1_MS2)
        SERIAL_EV(READ(E1_MS2_PIN));
      #else
        SERIAL_EOL();
      #endif
    #endif
    #if HAS_E2_MICROSTEPS
      SERIAL_MSG(MSG_MICROSTEP_E2);
      SERIAL_VAL(READ(E2_MS1_PIN));
      #if PIN_EXISTS(E2_MS2)
        SERIAL_EV(READ(E2_MS2_PIN));
      #else
        SERIAL_EOL();
      #endif
    #endif
    #if HAS_E3_MICROSTEPS
      SERIAL_MSG(MSG_MICROSTEP_E3);
      SERIAL_VAL(READ(E3_MS1_PIN));
      #if PIN_EXISTS(E3_MS2)
        SERIAL_EV(READ(E3_MS2_PIN));
      #else
        SERIAL_EOL();
      #endif
    #endif
    #if HAS_E4_MICROSTEPS
      SERIAL_MSG(MSG_MICROSTEP_E4);
      SERIAL_VAL(READ(E4_MS1_PIN));
      #if PIN_EXISTS(E4_MS2)
        SERIAL_EV(READ(E4_MS2_PIN));
      #else
        SERIAL_EOL();
      #endif
    #endif
    #if HAS_E5_MICROSTEPS
      SERIAL_MSG(MSG_MICROSTEP_E5);
      SERIAL_VAL(READ(E5_MS1_PIN));
      #if PIN_EXISTS(E5_MS2)
        SERIAL_EV(READ(E5_MS2_PIN));
      #else
        SERIAL_EOL();
      #endif
    #endif
  }

#endif // HAS_MICROSTEPS

/**
 * Set the current position in steps
 */
void Stepper::set_position(const int32_t &a, const int32_t &b, const int32_t &c, const int32_t &e) {
  planner.synchronize();
  const bool isr_enabled = STEPPER_ISR_ENABLED();
  if (isr_enabled) DISABLE_STEPPER_INTERRUPT();
  _set_position(a, b, c, e);
  if (isr_enabled) ENABLE_STEPPER_INTERRUPT();
}
void Stepper::set_position(const AxisEnum a, const int32_t &v) {
  planner.synchronize();

  #if ENABLED(__AVR__)
    // Protect the access to the variable. Only required for AVR.
    // Disable stepper ISR
    const bool isr_enabled = STEPPER_ISR_ENABLED();
    if (isr_enabled) DISABLE_STEPPER_INTERRUPT();
  #endif

  count_position[a] = v;

  #if ENABLED(__AVR__)
    // Reenable Stepper ISR
    if (isr_enabled) ENABLE_STEPPER_INTERRUPT();
  #endif
}

/** Private Function */
/**
 * This phase of the ISR should ONLY create the pulses for the steppers.
 * This prevents jitter caused by the interval between the start of the
 * interrupt and the start of the pulses. DON'T add any logic ahead of the
 * call to this method that might cause variation in the timing. The aim
 * is to keep pulse timing as regular as possible.
 */
void Stepper::pulse_phase_step() {

  // If we must abort the current block, do so!
  if (abort_current_block) {
    abort_current_block = false;
    if (current_block) {
      axis_did_move = 0;
      current_block = NULL;
      planner.discard_current_block();
    }
  }

  // If there is no current block, do nothing
  if (!current_block) return;

  // Compute the count of pending loops
  const uint32_t pending_events = step_event_count - step_events_completed;
  uint8_t events_to_do = MIN(pending_events, steps_per_isr);

  // Just update the value we will get at the end of the loop
  step_events_completed += events_to_do;

  // Get the timer count and estimate the end of the pulse
  hal_timer_t pulse_end = HAL_timer_get_current_count(STEPPER_TIMER) + HAL_min_pulse_tick;

  // Take multiple steps per interrupt (For high speed moves)
  do {

    // Start an active pulse
    pulse_tick_start();

    // Test Encoder if exist
    #if HAS_EXT_ENCODER
      test_extruder_encoder();
    #endif

    if (minimum_pulse) {
      // Just wait for the requested pulse time.
      while (HAL_timer_get_current_count(STEPPER_TIMER) < pulse_end) { /* nada */ }
    }

    // Add to the value, the value needed for the pulse end and ensuring the maximum driver rate is enforced
    if (signed(HAL_add_pulse_ticks) > 0) pulse_end += HAL_add_pulse_ticks;

    // Stop an active pulse
    pulse_tick_stop();

    #if ENABLED(LASER)
      counter_L += current_block->steps_l;
      if (counter_L >= 0) {
        if (current_block->laser_mode == PULSED && current_block->laser_status == LASER_ON) { // Pulsed Firing Mode
          laser.fire(current_block->laser_intensity);
          if (laser.diagnostics) {
            SERIAL_MV("X: ", delta_error[X_AXIS]);
            SERIAL_MV("Y: ", delta_error[Y_AXIS]);
            SERIAL_MV("L: ", counter_L);
          }
        }
        #if ENABLED(LASER_RASTER)
          if (current_block->laser_mode == RASTER && current_block->laser_status == LASER_ON) { // Raster Firing Mode
            // For some reason, when comparing raster power to ppm line burns the rasters were around 2% more powerful
            // going from darkened paper to burning through paper.
            laser.fire(current_block->laser_raster_data[counter_raster]);
            if (laser.diagnostics) SERIAL_MV("Pixel: ", (float)current_block->laser_raster_data[counter_raster]);
            counter_raster++;
          }
        #endif // LASER_RASTER

        counter_L -= current_block->step_event_count;
      }
      if (current_block->laser_duration != 0 && (laser.last_firing + current_block->laser_duration < micros())) {
        if (laser.diagnostics)
          SERIAL_EM("Laser firing duration elapsed, in interrupt fast loop");
        laser.extinguish();
      }
    #endif // LASER

    // Decrement the count of pending pulses to do
    --events_to_do;

    // For minimum pulse time wait after stopping pulses also
    // Just wait for the requested pulse time.
    if (events_to_do) {
      while (HAL_timer_get_current_count(STEPPER_TIMER) < pulse_end) { /* nada */ }
      if (minimum_pulse) {
        // Add to the value, the time that the pulse must be active (to be used on the next loop)
        pulse_end += HAL_min_pulse_tick;
      }
    }

  } while (events_to_do);
}

uint32_t Stepper::block_phase_step() {

  // If no queued movements, just wait 1ms for the next move
  uint32_t interval = (STEPPER_TIMER_RATE / 1000);

  // If there is a current block
  if (current_block) {

    // If current block is finished, reset pointer
    if (step_events_completed >= step_event_count) {
      axis_did_move = 0;
      current_block = NULL;
      planner.discard_current_block();

      #if ENABLED(LASER)
        laser.extinguish();
      #endif
    }
    // Are we in acceleration phase
    else if (step_events_completed <= accelerate_until) {

      #if ENABLED(BEZIER_JERK_CONTROL)
        // Get the next speed to use (Jerk limited!)
        uint32_t acc_step_rate =
          acceleration_time < current_block->acceleration_time
            ? _eval_bezier_curve(acceleration_time)
            : current_block->cruise_rate;
      #else
        acc_step_rate = HAL_MULTI_ACC(acceleration_time, current_block->acceleration_rate) + current_block->initial_rate;
        NOMORE(acc_step_rate, current_block->nominal_rate);
      #endif

      // acc_step_rate is in steps/second

      // step_rate to timer interval
      interval = HAL_calc_timer_interval(acc_step_rate, &steps_per_isr, oversampling_factor);
      acceleration_time += interval;

      #if ENABLED(LIN_ADVANCE)
        if (LA_use_advance_lead) {
          // Wake up eISR on first acceleration loop and fire ISR if final adv_rate is reached
          if (step_events_completed == steps_per_isr || (LA_steps && LA_isr_rate != current_block->advance_speed)) {
            nextAdvanceISR = 0;
            LA_isr_rate = current_block->advance_speed;
          }
        }
        else {
          LA_isr_rate = LA_ADV_NEVER;
          if (LA_steps) nextAdvanceISR = 0;
        }
      #endif // ENABLED(LIN_ADVANCE)
    }
    // Are we in deceleration phase
    else if (step_events_completed > decelerate_after) {
      uint32_t step_rate;

      #if ENABLED(BEZIER_JERK_CONTROL)
        // If this is the 1st time we process the 2nd half of the trapezoid...
        if (!bezier_2nd_half) {
          // Initialize the Bézier speed curve
          _calc_bezier_curve_coeffs(current_block->cruise_rate, current_block->final_rate, current_block->deceleration_time_inverse);
          bezier_2nd_half = true;
          // The first point starts at cruise rate. Just save evaluation of the Bézier curve
          step_rate = current_block->cruise_rate;
        }
        else {
          // Calculate the next speed to use
          step_rate = deceleration_time < current_block->deceleration_time
            ? _eval_bezier_curve(deceleration_time)
            : current_block->final_rate;
        }
      #else

        // Using the old trapezoidal control
        step_rate = HAL_MULTI_ACC(deceleration_time, current_block->acceleration_rate);

        if (step_rate < acc_step_rate) { // Still decelerating?
          step_rate = acc_step_rate - step_rate;
          NOLESS(step_rate, current_block->final_rate);
        }
        else
          step_rate = current_block->final_rate;
      #endif

      // step_rate is in steps/second

      // step_rate to timer interval
      interval = HAL_calc_timer_interval(step_rate, &steps_per_isr, oversampling_factor);
      deceleration_time += interval;

      #if ENABLED(LIN_ADVANCE)
        if (LA_use_advance_lead) {
          if (step_events_completed <= decelerate_after + steps_per_isr ||
             (LA_steps && LA_isr_rate != current_block->advance_speed)
          ) {
            nextAdvanceISR = 0; // Wake up eISR on first deceleration loop
            LA_isr_rate = current_block->advance_speed;
          }
        }
        else {
          LA_isr_rate = LA_ADV_NEVER;
          if (LA_steps) nextAdvanceISR = 0;
        }
      #endif // LIN_ADVANCE
    }
    // We must be in cruise phase otherwise
    else {

      #if ENABLED(LIN_ADVANCE)
        // If there are any esteps, fire the next advance_isr "now"
        if (LA_steps && LA_isr_rate != current_block->advance_speed) nextAdvanceISR = 0;
      #endif

      // Calculate the ticks_nominal for this nominal speed, if not done yet
      if (ticks_nominal < 0) {
        // step_rate to timer interval and loops for the nominal speed
        ticks_nominal = HAL_calc_timer_interval(current_block->nominal_rate, &steps_per_isr, oversampling_factor);
      }

      // The timer interval is just the nominal value for the nominal speed
      interval = ticks_nominal;
    }
  }

  // If there is no current block at this point, attempt to pop one from the buffer
  // and prepare its movement
  if (!current_block) {

    // Anything in the buffer?
    if ((current_block = planner.get_current_block())) {

      // Sync block? Sync the stepper counts and return
      while (TEST(current_block->flag, BLOCK_BIT_SYNC_POSITION)) {
        _set_position(
          current_block->position[A_AXIS], current_block->position[B_AXIS],
          current_block->position[C_AXIS], current_block->position[E_AXIS]
        );
        planner.discard_current_block();

        // Try to get a new block
        if (!(current_block = planner.get_current_block()))
          return interval; // No more queued movements!
      }

      // Flag all moving axes for proper endstop handling

      #if IS_CORE
        // Define conditions for checking endstops
        #define S_(N) current_block->steps[CORE_AXIS_##N]
        #define D_(N) TEST(current_block->direction_bits, CORE_AXIS_##N)
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
      #else
        #define X_MOVE_TEST !!current_block->steps[A_AXIS]
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
      #else
        #define Y_MOVE_TEST !!current_block->steps[B_AXIS]
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
      #else
        #define Z_MOVE_TEST !!current_block->steps[C_AXIS]
      #endif

      axis_did_move = 0;
      if (X_MOVE_TEST) SBI(axis_did_move, A_AXIS);
      if (Y_MOVE_TEST) SBI(axis_did_move, B_AXIS);
      if (Z_MOVE_TEST) SBI(axis_did_move, C_AXIS);
      //if (!!current_block->steps[E_AXIS]) SBI(axis_did_move, E_AXIS);
      //if (!!current_block->steps[A_AXIS]) SBI(axis_did_move, X_HEAD);
      //if (!!current_block->steps[B_AXIS]) SBI(axis_did_move, Y_HEAD);
      //if (!!current_block->steps[C_AXIS]) SBI(axis_did_move, Z_HEAD);

      // No acceleration / deceleration time elapsed so far
      acceleration_time = deceleration_time = 0;

      #if ENABLED(ADAPTIVE_STEP_SMOOTHING)
        oversampling_factor = 0;
        // At this point, we must decide if we can use Stepper movement axis smoothing.
        uint32_t max_rate = current_block->nominal_rate;  // Get the maximum rate (maximum event speed)
        while (max_rate < HAL_min_isr_frequency) {
          max_rate <<= 1;
          if (max_rate >= HAL_min_isr_frequency) break;
          ++oversampling_factor;
        }
      #endif

      // Based on the oversampling factor, do the calculations
      step_event_count = current_block->step_event_count << oversampling_factor;

      // Initialize Bresenham delta errors to 1/2
      delta_error[X_AXIS] = delta_error[Y_AXIS] = delta_error[Z_AXIS] = delta_error[E_AXIS] = -int32_t(step_event_count);

      // Calculate Bresenham dividends
      advance_dividend[X_AXIS] = current_block->steps[X_AXIS] << 1;
      advance_dividend[Y_AXIS] = current_block->steps[Y_AXIS] << 1;
      advance_dividend[Z_AXIS] = current_block->steps[Z_AXIS] << 1;
      advance_dividend[E_AXIS] = current_block->steps[E_AXIS] << 1;

      // Calculate Bresenham divisor
      advance_divisor = step_event_count << 1;

      // No step events completed so far
      step_events_completed = 0;

      // Compute the acceleration and deceleration points
      accelerate_until = current_block->accelerate_until << oversampling_factor;
      decelerate_after = current_block->decelerate_after << oversampling_factor;

      #if ENABLED(COLOR_MIXING_EXTRUDER)
        const uint32_t e_steps = (
          #if ENABLED(LIN_ADVANCE)
            current_block->steps[E_AXIS]
          #else
            step_event_count
          #endif
        );
        MIXING_STEPPERS_LOOP(i) {
          delta_error_m[i] = -int32_t(e_steps);
          advance_dividend_m[i] = current_block->mix_steps[i] << 1;
        }
        advance_divisor_m = e_steps << 1;
      #else
        active_extruder = current_block->active_extruder;
        active_extruder_driver = get_active_extruder_driver();
      #endif

      // Initialize the trapezoid generator from the current block.
      #if ENABLED(LIN_ADVANCE)
        #if DISABLED(COLOR_MIXING_EXTRUDER) && DRIVER_EXTRUDERS > 1
          // If the now active extruder wasn't in use during the last move, its pressure is most likely gone.
          if (active_extruder != last_moved_extruder) LA_current_adv_steps = 0;
        #endif

        if ((LA_use_advance_lead = current_block->use_advance_lead)) {
          LA_final_adv_steps = current_block->final_adv_steps;
          LA_max_adv_steps = current_block->max_adv_steps;
        }
      #endif

      if (current_block->direction_bits != last_direction_bits
        #if DISABLED(COLOR_MIXING_EXTRUDER)
          || active_extruder != last_moved_extruder
        #endif
      ) {
        last_direction_bits = current_block->direction_bits;
        #if DISABLED(COLOR_MIXING_EXTRUDER)
          last_moved_extruder = active_extruder;
        #endif
        set_directions();
      }

      // At this point, we must ensure the movement about to execute isn't
      // trying to force the head against a limit switch. If using interrupt-
      // driven change detection, and already against a limit then no call to
      // the endstop_triggered method will be done and the movement will be
      // done against the endstop. So, check the limits here: If the movement
      // is against the limits, the block will be marked as to be killed, and
      // on the next call to this ISR, will be discarded.
      endstops.update();

      #if ENABLED(Z_LATE_ENABLE)
        // If delayed Z enable, enable it now. This option will severely interfere with
        // timing between pulses when chaining motion between blocks, and it could lead
        // to lost steps in both X and Y axis, so avoid using it unless strictly necessary!!
        if (current_block->steps[Z_AXIS]) enable_Z();
      #endif

      // Mark the time_nominal as not calculated yet
      ticks_nominal = -1;

      #if DISABLED(BEZIER_JERK_CONTROL)
        // Set as deceleration point the initial rate of the block
        acc_step_rate = current_block->initial_rate;
      #endif

      #if ENABLED(BEZIER_JERK_CONTROL)
        // Initialize the Bézier speed curve
        _calc_bezier_curve_coeffs(current_block->initial_rate, current_block->cruise_rate, current_block->acceleration_time_inverse);
        // We haven't started the 2nd half of the trapezoid
        bezier_2nd_half = false;
      #endif

      #if ENABLED(LASER)
        counter_L = delta_error[X_AXIS];
        laser.dur = current_block->laser_duration;
      #endif

      #if ENABLED(LASER) && ENABLED(LASER_RASTER)
         if (current_block->laser_mode == RASTER) counter_raster = 0;
      #endif

      // Calculate the initial timer interval
      interval = HAL_calc_timer_interval(current_block->initial_rate, &steps_per_isr, oversampling_factor);
    }
  }

  // Continuous firing of the laser during a move happens here, PPM and raster happen further down
  #if ENABLED(LASER)
    if (current_block->laser_mode == CONTINUOUS && current_block->laser_status == LASER_ON)
      laser.fire(current_block->laser_intensity);

    if (current_block->laser_status == LASER_OFF) {
      if (laser.diagnostics)
        SERIAL_EM("Laser status set to off, in interrupt handler");
      laser.extinguish();
    }
  #endif

  #if ENABLED(BABYSTEPPING)
    LOOP_XYZ(axis) {
      const int curTodo = mechanics.babystepsTodo[axis]; // get rid of volatile for performance

      if (curTodo) {
        babystep((AxisEnum)axis, curTodo > 0);
        if (curTodo > 0) mechanics.babystepsTodo[axis]--;
                    else mechanics.babystepsTodo[axis]++;
      }
    }
  #endif // ENABLED(BABYSTEPPING)

  // Return the interval to wait
  return interval;
}

void Stepper::pulse_tick_start() {

  #if HAS_X_STEP
    delta_error[X_AXIS] += advance_dividend[X_AXIS];
    if (delta_error[X_AXIS] >= 0) {
      start_X_step();
      count_position[X_AXIS] += count_direction[X_AXIS];
    }
  #endif

  #if HAS_Y_STEP
    delta_error[Y_AXIS] += advance_dividend[Y_AXIS];
    if (delta_error[Y_AXIS] >= 0) {
      start_Y_step();
      count_position[Y_AXIS] += count_direction[Y_AXIS];
    }
  #endif

  #if HAS_Z_STEP
    delta_error[Z_AXIS] += advance_dividend[Z_AXIS];
    if (delta_error[Z_AXIS] >= 0) {
      start_Z_step();
      count_position[Z_AXIS] += count_direction[Z_AXIS];
    }
  #endif

  // Pulse E/Mixing extruders
  #if ENABLED(LIN_ADVANCE)

    // Tick the E axis, correct error term and update position
    delta_error[E_AXIS] += advance_dividend[E_AXIS];
    if (delta_error[E_AXIS] >= 0) {
      count_position[E_AXIS] += count_direction[E_AXIS];
      delta_error[E_AXIS] -= advance_divisor;

      // Don't step E here - But remember the number of steps to perform
      motor_direction(E_AXIS) ? --LA_steps : ++LA_steps;
    }

  #elif HAS_EXTRUDERS

    #if ENABLED(COLOR_MIXING_EXTRUDER)

      // Tick the E axis
      delta_error[E_AXIS] += advance_dividend[E_AXIS];
      if (delta_error[E_AXIS] >= 0) {
        count_position[E_AXIS] += count_direction[E_AXIS];
        delta_error[E_AXIS] -= advance_divisor;
      }

      // Tick the counters used for this mix in proper proportion
      MIXING_STEPPERS_LOOP(j) {
        // Step mixing steppers (proportionally)
        delta_error_m[j] += advance_dividend_m[j];
        // Step when the counter goes over zero
        if (delta_error_m[j] >= 0)
          E_STEP_WRITE(j, !INVERT_E_STEP_PIN);
      }

    #else

      delta_error[E_AXIS] += advance_dividend[E_AXIS];
      if (delta_error[E_AXIS] >= 0) {
        E_STEP_WRITE(active_extruder_driver, !INVERT_E_STEP_PIN);
        count_position[E_AXIS] += count_direction[E_AXIS];
      }

    #endif

  #endif

}

void Stepper::pulse_tick_stop() {

  #if HAS_X_STEP
    if (delta_error[X_AXIS] >= 0) {
      delta_error[X_AXIS] -= advance_divisor;
      stop_X_step();
    }
  #endif

  #if HAS_Y_STEP
    if (delta_error[Y_AXIS] >= 0) {
      delta_error[Y_AXIS] -= advance_divisor;
      stop_Y_step();
    }
  #endif

  #if HAS_Z_STEP
    if (delta_error[Z_AXIS] >= 0) {
      delta_error[Z_AXIS] -= advance_divisor;
      stop_Z_step();
    }
  #endif

  #if HAS_EXTRUDERS && DISABLED(LIN_ADVANCE)
    #if ENABLED(COLOR_MIXING_EXTRUDER)
      MIXING_STEPPERS_LOOP(j) {
        if (delta_error_m[j] >= 0) {
          delta_error_m[j] -= advance_divisor_m;
          E_STEP_WRITE(j, INVERT_E_STEP_PIN);
        }
      }
    #elif HAS_EXTRUDERS
      if (delta_error[E_AXIS] >= 0) {
        delta_error[E_AXIS] -= advance_divisor;
        E_STEP_WRITE(active_extruder_driver, INVERT_E_STEP_PIN);
      }
    #endif
  #endif

}

/**
 * Start X Y Z Step
 */
void Stepper::start_X_step() {

  #if ENABLED(X_TWO_STEPPER_DRIVERS)
    #if ENABLED(X_TWO_ENDSTOPS)
      if (homing_dual_axis) {
        if (X_HOME_DIR < 0) {
          if (!(TEST(endstops.live_state, X_MIN)  && count_direction[X_AXIS] < 0) && !locked_X_motor) X_STEP_WRITE(!INVERT_X_STEP_PIN);
          if (!(TEST(endstops.live_state, X2_MIN) && count_direction[X_AXIS] < 0) && !locked_X2_motor) X2_STEP_WRITE(!INVERT_X_STEP_PIN);
        }
        else {
          if (!(TEST(endstops.live_state, X_MAX)  && count_direction[X_AXIS] > 0) && !locked_X_motor) X_STEP_WRITE(!INVERT_X_STEP_PIN);
          if (!(TEST(endstops.live_state, X2_MAX) && count_direction[X_AXIS] > 0) && !locked_X2_motor) X2_STEP_WRITE(!INVERT_X_STEP_PIN);
        }
      }
      else {
        X_STEP_WRITE(!INVERT_X_STEP_PIN);
        X2_STEP_WRITE(!INVERT_X_STEP_PIN);
      }
    #else
      X_STEP_WRITE(!INVERT_X_STEP_PIN);
      X2_STEP_WRITE(!INVERT_X_STEP_PIN);
    #endif
  #elif ENABLED(DUAL_X_CARRIAGE)
    if (mechanics.hotend_duplication_enabled) {
      X_STEP_WRITE(!INVERT_X_STEP_PIN);
      X2_STEP_WRITE(!INVERT_X_STEP_PIN);
    }
    else {
      if (movement_extruder())
        X2_STEP_WRITE(!INVERT_X_STEP_PIN);
      else
        X_STEP_WRITE(!INVERT_X_STEP_PIN);
    }
  #else
    X_STEP_WRITE(!INVERT_X_STEP_PIN);
  #endif

}
void Stepper::start_Y_step() {

  #if ENABLED(Y_TWO_STEPPER_DRIVERS)
    #if ENABLED(Y_TWO_ENDSTOPS)
      if (homing_dual_axis) {
        if (Y_HOME_DIR < 0) {
          if (!(TEST(endstops.live_state, Y_MIN)  && count_direction[Y_AXIS] < 0) && !locked_Y_motor) Y_STEP_WRITE(!INVERT_Y_STEP_PIN);
          if (!(TEST(endstops.live_state, Y2_MIN) && count_direction[Y_AXIS] < 0) && !locked_Y2_motor) Y2_STEP_WRITE(!INVERT_Y_STEP_PIN);
        }
        else {
          if (!(TEST(endstops.live_state, Y_MAX)  && count_direction[Y_AXIS] > 0) && !locked_Y_motor) Y_STEP_WRITE(!INVERT_Y_STEP_PIN);
          if (!(TEST(endstops.live_state, Y2_MAX) && count_direction[Y_AXIS] > 0) && !locked_Y2_motor) Y2_STEP_WRITE(!INVERT_Y_STEP_PIN);
        }
      }
      else {
        Y_STEP_WRITE(!INVERT_Y_STEP_PIN);
        Y2_STEP_WRITE(!INVERT_Y_STEP_PIN);
      }
    #else
      Y_STEP_WRITE(!INVERT_Y_STEP_PIN);
      Y2_STEP_WRITE(!INVERT_Y_STEP_PIN);
    #endif
  #else
    Y_STEP_WRITE(!INVERT_Y_STEP_PIN);
  #endif

}
void Stepper::start_Z_step() {

  #if ENABLED(Z_TWO_STEPPER_DRIVERS)
    #if ENABLED(Z_TWO_ENDSTOPS)
      if (homing_dual_axis) {
        if (Z_HOME_DIR < 0) {
          if (!(TEST(endstops.live_state, Z_MIN)  && count_direction[Z_AXIS] < 0) && !locked_Z_motor) Z_STEP_WRITE(!INVERT_Z_STEP_PIN);
          if (!(TEST(endstops.live_state, Z2_MIN) && count_direction[Z_AXIS] < 0) && !locked_Z2_motor) Z2_STEP_WRITE(!INVERT_Z_STEP_PIN);
        }
        else {
          if (!(TEST(endstops.live_state, Z_MAX)  && count_direction[Z_AXIS] > 0) && !locked_Z_motor) Z_STEP_WRITE(!INVERT_Z_STEP_PIN);
          if (!(TEST(endstops.live_state, Z2_MAX) && count_direction[Z_AXIS] > 0) && !locked_Z2_motor) Z2_STEP_WRITE(!INVERT_Z_STEP_PIN);
        }
      }
      else {
        Z_STEP_WRITE(!INVERT_Z_STEP_PIN);
        Z2_STEP_WRITE(!INVERT_Z_STEP_PIN);
      }
    #else
      Z_STEP_WRITE(!INVERT_Z_STEP_PIN);
      Z2_STEP_WRITE(!INVERT_Z_STEP_PIN);
    #endif
  #else
    Z_STEP_WRITE(!INVERT_Z_STEP_PIN);
  #endif

}

/**
 * End X Y Z Step
 */
void Stepper::stop_X_step() {
  X_STEP_WRITE(INVERT_X_STEP_PIN);
  #if ENABLED(X_TWO_STEPPER_DRIVERS) || ENABLED(DUAL_X_CARRIAGE)
    X2_STEP_WRITE(INVERT_X_STEP_PIN);
  #endif
}
void Stepper::stop_Y_step() {
  Y_STEP_WRITE(INVERT_Y_STEP_PIN);
  #if ENABLED(Y_TWO_STEPPER_DRIVERS)
    Y2_STEP_WRITE(INVERT_Y_STEP_PIN);
  #endif
}
void Stepper::stop_Z_step() {
  Z_STEP_WRITE(INVERT_Z_STEP_PIN);
  #if ENABLED(Z_TWO_STEPPER_DRIVERS)
    Z2_STEP_WRITE(INVERT_Z_STEP_PIN);
  #endif
}

/**
 * Set X Y Z direction
 */
void Stepper::set_X_dir(const bool dir) {
  #if ENABLED(X_TWO_STEPPER_DRIVERS)
    X_DIR_WRITE(dir);
    X2_DIR_WRITE((dir) != INVERT_X2_VS_X_DIR);
  #elif ENABLED(DUAL_X_CARRIAGE)
    if (mechanics.hotend_duplication_enabled) {
      X_DIR_WRITE(dir);
      X2_DIR_WRITE(dir);
    }
    else {
      if (movement_extruder())
        X2_DIR_WRITE(dir);
      else
        X_DIR_WRITE(dir);
    }
  #else
    X_DIR_WRITE(dir);
  #endif
}
void Stepper::set_Y_dir(const bool dir) {
  #if ENABLED(Y_TWO_STEPPER_DRIVERS)
    Y_DIR_WRITE(dir);
    Y2_DIR_WRITE((dir) != INVERT_Y2_VS_Y_DIR);
  #else
    Y_DIR_WRITE(dir);
  #endif
}
void Stepper::set_Z_dir(const bool dir) {
  #if ENABLED(Z_TWO_STEPPER_DRIVERS)
    Z_DIR_WRITE(dir);
    Z2_DIR_WRITE((dir) != INVERT_Z2_VS_Z_DIR);
  #else
    Z_DIR_WRITE(dir);
  #endif
}

/**
 * Set the stepper positions directly in steps
 *
 * The input is based on the typical per-axis XYZ steps.
 * For CORE machines XYZ needs to be translated to ABC.
 *
 * This allows get_axis_position_mm to correctly
 * derive the current XYZ position later on.
 */
void Stepper::_set_position(const int32_t &a, const int32_t &b, const int32_t &c, const int32_t &e) {

  #if CORE_IS_XY
    // corexy positioning
    count_position[A_AXIS] = a + (CORE_FACTOR) * b;
    count_position[B_AXIS] = CORESIGN(a - (CORE_FACTOR) * b);
    count_position[Z_AXIS] = c;
  #elif CORE_IS_XZ
    // corexz planning
    count_position[A_AXIS] = a + (CORE_FACTOR) * c;
    count_position[Y_AXIS] = b;
    count_position[C_AXIS] = CORESIGN(a - (CORE_FACTOR) * c);
  #elif CORE_IS_YZ
    // coreyz planning
    count_position[X_AXIS] = a;
    count_position[B_AXIS] = b + (CORE_FACTOR) * c;
    count_position[C_AXIS] = CORESIGN(b - (CORE_FACTOR) * c);
  #else
    // default non-h-bot planning
    count_position[X_AXIS] = a;
    count_position[Y_AXIS] = b;
    count_position[Z_AXIS] = c;
  #endif

  count_position[E_AXIS] = e;

}

#if DISABLED(COLOR_MIXING_EXTRUDER)

  /**
   * Get active driver for Multitools MKr, MKs o Dondolo
   */
  uint8_t Stepper::get_active_extruder_driver() {

    #if HAS_MKMULTI_TOOLS
      #if ENABLED(MKSE6)
        return 0;
      #elif ENABLED(MKR4)
        const uint8_t driver = active_extruder & 1;
        if (driver)
          return 1;
        else 
          return 0;
      #elif ENABLED(MKR6) || ENABLED(MKR12)
        if WITHIN(active_extruder, 0, 2)
          return 0;
        if WITHIN(active_extruder, 3, 5)
          return 1;
        if WITHIN(active_extruder, 6, 8)
          return 2;
        if WITHIN(active_extruder, 9, 11)
          return 3;
      #endif
    #elif ENABLED(DONDOLO_SINGLE_MOTOR)
      return 0;
    #else
      return active_extruder;
    #endif

  }

#endif

/**
 * This is the last half of the stepper interrupt: This one processes and
 * properly schedules blocks from the planner. This is executed after creating
 * the step pulses, so it is not time critical, as pulses are already done.
 */
#if ENABLED(LIN_ADVANCE)

  // Timer interrupt for E. LA_steps is set in the main routine
  uint32_t Stepper::lin_advance_step() {
    uint32_t interval;

    if (LA_use_advance_lead) {
      if (step_events_completed > decelerate_after && LA_current_adv_steps > LA_final_adv_steps) {
        LA_steps--;
        LA_current_adv_steps--;
        interval = LA_isr_rate;
      }
      else if (step_events_completed < decelerate_after && LA_current_adv_steps < LA_max_adv_steps) {
             //step_events_completed <= (uint32_t)accelerate_until) {
        LA_steps++;
        LA_current_adv_steps++;
        interval = LA_isr_rate;
      }
      else
        interval = LA_isr_rate = LA_ADV_NEVER;
    }
    else
      interval = LA_ADV_NEVER;

    #if ENABLED(COLOR_MIXING_EXTRUDER)
      if (LA_steps >= 0)
        MIXING_STEPPERS_LOOP(j) NORM_E_DIR(j);
      else
        MIXING_STEPPERS_LOOP(j) REV_E_DIR(j);
    #else
      if (LA_steps >= 0)
        NORM_E_DIR(active_extruder_driver);
      else
        REV_E_DIR(active_extruder_driver);
    #endif

    // Get the timer count and estimate the end of the pulse
    hal_timer_t pulse_end = HAL_timer_get_current_count(STEPPER_TIMER) + HAL_min_pulse_tick;

    // Step E stepper if we have steps
    while (LA_steps) {

      #if ENABLED(COLOR_MIXING_EXTRUDER)
        MIXING_STEPPERS_LOOP(j) {
          // Step mixing steppers (proportionally)
          delta_error_m[j] += advance_dividend_m[j];
          // Step when the counter goes over zero
          if (delta_error_m[j] >= 0) E_STEP_WRITE(j, !INVERT_E_STEP_PIN);
        }
      #else
        E_STEP_WRITE(active_extruder_driver, !INVERT_E_STEP_PIN);
      #endif

      if (minimum_pulse) {
        // Just wait for the requested pulse time.
        while (HAL_timer_get_current_count(STEPPER_TIMER) < pulse_end) { /* nada */ }
      }

      // Add the delay needed to ensure the maximum driver rate is enforced
      if (signed(HAL_add_pulse_ticks) > 0) pulse_end += HAL_add_pulse_ticks;

      LA_steps < 0 ? ++LA_steps : --LA_steps;

      #if ENABLED(COLOR_MIXING_EXTRUDER)
        MIXING_STEPPERS_LOOP(j) {
          if (delta_error_m[j] >= 0) {
            delta_error_m[j] -= advance_divisor_m;
            E_STEP_WRITE(j, INVERT_E_STEP_PIN);
          }
        }
      #else
        E_STEP_WRITE(active_extruder_driver, INVERT_E_STEP_PIN);
      #endif

      // For minimum pulse time wait before looping
      // Just wait for the requested pulse time.
      if (LA_steps) {
        while (HAL_timer_get_current_count(STEPPER_TIMER) < pulse_end) { /* nada */ }
        if (minimum_pulse) {
          // Add to the value, the time that the pulse must be active (to be used on the next loop)
          pulse_end += HAL_min_pulse_tick;
        }
      }
    } // LA_steps

    return interval;
  }

#endif // ENABLED(LIN_ADVANCE)

/**
 *  This uses a quintic (fifth-degree) Bézier polynomial for the velocity curve, giving
 *  a "linear pop" velocity curve; with pop being the sixth derivative of position:
 *  velocity - 1st, acceleration - 2nd, jerk - 3rd, snap - 4th, crackle - 5th, pop - 6th
 *
 *  The Bézier curve takes the form:
 *
 *  V(t) = P_0 * B_0(t) + P_1 * B_1(t) + P_2 * B_2(t) + P_3 * B_3(t) + P_4 * B_4(t) + P_5 * B_5(t)
 *
 *  Where 0 <= t <= 1, and V(t) is the velocity. P_0 through P_5 are the control points, and B_0(t)
 *  through B_5(t) are the Bernstein basis as follows:
 *
 *        B_0(t) =   (1-t)^5        =   -t^5 +  5t^4 - 10t^3 + 10t^2 -  5t   +   1
 *        B_1(t) =  5(1-t)^4 * t    =   5t^5 - 20t^4 + 30t^3 - 20t^2 +  5t
 *        B_2(t) = 10(1-t)^3 * t^2  = -10t^5 + 30t^4 - 30t^3 + 10t^2
 *        B_3(t) = 10(1-t)^2 * t^3  =  10t^5 - 20t^4 + 10t^3
 *        B_4(t) =  5(1-t)   * t^4  =  -5t^5 +  5t^4
 *        B_5(t) =             t^5  =    t^5
 *                                      ^       ^       ^       ^       ^       ^
 *                                      |       |       |       |       |       |
 *                                      A       B       C       D       E       F
 *
 *  Unfortunately, we cannot use forward-differencing to calculate each position through
 *  the curve, as Marlin uses variable timer periods. So, we require a formula of the form:
 *
 *        V_f(t) = A*t^5 + B*t^4 + C*t^3 + D*t^2 + E*t + F
 *
 *  Looking at the above B_0(t) through B_5(t) expanded forms, if we take the coefficients of t^5
 *  through t of the Bézier form of V(t), we can determine that:
 *
 *        A =    -P_0 +  5*P_1 - 10*P_2 + 10*P_3 -  5*P_4 +  P_5
 *        B =   5*P_0 - 20*P_1 + 30*P_2 - 20*P_3 +  5*P_4
 *        C = -10*P_0 + 30*P_1 - 30*P_2 + 10*P_3
 *        D =  10*P_0 - 20*P_1 + 10*P_2
 *        E = - 5*P_0 +  5*P_1
 *        F =     P_0
 *
 *  Now, since we will (currently) *always* want the initial acceleration and jerk values to be 0,
 *  We set P_i = P_0 = P_1 = P_2 (initial velocity), and P_t = P_3 = P_4 = P_5 (target velocity),
 *  which, after simplification, resolves to:
 *
 *        A = - 6*P_i +  6*P_t =  6*(P_t - P_i)
 *        B =  15*P_i - 15*P_t = 15*(P_i - P_t)
 *        C = -10*P_i + 10*P_t = 10*(P_t - P_i)
 *        D = 0
 *        E = 0
 *        F = P_i
 *
 *  As the t is evaluated in non uniform steps here, there is no other way rather than evaluating
 *  the Bézier curve at each point:
 *
 *        V_f(t) = A*t^5 + B*t^4 + C*t^3 + F          [0 <= t <= 1]
 *
 * Floating point arithmetic execution time cost is prohibitive, so we will transform the math to
 * use fixed point values to be able to evaluate it in realtime. Assuming a maximum of 250000 steps
 * per second (driver pulses should at least be 2µS hi/2µS lo), and allocating 2 bits to avoid
 * overflows on the evaluation of the Bézier curve, means we can use
 *
 *   t: unsigned Q0.32 (0 <= t < 1) |range 0 to 0xFFFFFFFF unsigned
 *   A:   signed Q24.7 ,            |range = +/- 250000 * 6 * 128 = +/- 192000000 = 0x0B71B000 | 28 bits + sign
 *   B:   signed Q24.7 ,            |range = +/- 250000 *15 * 128 = +/- 480000000 = 0x1C9C3800 | 29 bits + sign
 *   C:   signed Q24.7 ,            |range = +/- 250000 *10 * 128 = +/- 320000000 = 0x1312D000 | 29 bits + sign
 *   F:   signed Q24.7 ,            |range = +/- 250000     * 128 =      32000000 = 0x01E84800 | 25 bits + sign
 *
 * The trapezoid generator state contains the following information, that we will use to create and evaluate
 * the Bézier curve:
 *
 *  blk->step_event_count [TS] = The total count of steps for this movement. (=distance)
 *  blk->initial_rate     [VI] = The initial steps per second (=velocity)
 *  blk->final_rate       [VF] = The ending steps per second  (=velocity)
 *  and the count of events completed (step_events_completed) [CS] (=distance until now)
 *
 *  Note the abbreviations we use in the following formulae are between []s
 *
 *  For Any 32bit CPU:
 *
 *    At the start of each trapezoid, calculate the coefficients A,B,C,F and Advance [AV], as follows:
 *
 *      A =  6*128*(VF - VI) =  768*(VF - VI)
 *      B = 15*128*(VI - VF) = 1920*(VI - VF)
 *      C = 10*128*(VF - VI) = 1280*(VF - VI)
 *      F =    128*VI        =  128*VI
 *     AV = (1<<32)/TS      ~= 0xFFFFFFFF / TS (To use ARM UDIV, that is 32 bits) (this is computed at the planner, to offload expensive calculations from the ISR)
 *
 *    And for each point, evaluate the curve with the following sequence:
 *
 *      void lsrs(uint32_t& d, uint32_t s, int cnt) {
 *        d = s >> cnt;
 *      }
 *      void lsls(uint32_t& d, uint32_t s, int cnt) {
 *        d = s << cnt;
 *      }
 *      void lsrs(int32_t& d, uint32_t s, int cnt) {
 *        d = uint32_t(s) >> cnt;
 *      }
 *      void lsls(int32_t& d, uint32_t s, int cnt) {
 *        d = uint32_t(s) << cnt;
 *      }
 *      void umull(uint32_t& rlo, uint32_t& rhi, uint32_t op1, uint32_t op2) {
 *        uint64_t res = uint64_t(op1) * op2;
 *        rlo = uint32_t(res & 0xFFFFFFFF);
 *        rhi = uint32_t((res >> 32) & 0xFFFFFFFF);
 *      }
 *      void smlal(int32_t& rlo, int32_t& rhi, int32_t op1, int32_t op2) {
 *        int64_t mul = int64_t(op1) * op2;
 *        int64_t s = int64_t(uint32_t(rlo) | ((uint64_t(uint32_t(rhi)) << 32U)));
 *        mul += s;
 *        rlo = int32_t(mul & 0xFFFFFFFF);
 *        rhi = int32_t((mul >> 32) & 0xFFFFFFFF);
 *      }
 *      int32_t _eval_bezier_curve_arm(uint32_t curr_step) {
 *        register uint32_t flo = 0;
 *        register uint32_t fhi = bezier_AV * curr_step;
 *        register uint32_t t = fhi;
 *        register int32_t alo = bezier_F;
 *        register int32_t ahi = 0;
 *        register int32_t A = bezier_A;
 *        register int32_t B = bezier_B;
 *        register int32_t C = bezier_C;
 *
 *        lsrs(ahi, alo, 1);          // a  = F << 31
 *        lsls(alo, alo, 31);         //
 *        umull(flo, fhi, fhi, t);    // f *= t
 *        umull(flo, fhi, fhi, t);    // f>>=32; f*=t
 *        lsrs(flo, fhi, 1);          //
 *        smlal(alo, ahi, flo, C);    // a+=(f>>33)*C
 *        umull(flo, fhi, fhi, t);    // f>>=32; f*=t
 *        lsrs(flo, fhi, 1);          //
 *        smlal(alo, ahi, flo, B);    // a+=(f>>33)*B
 *        umull(flo, fhi, fhi, t);    // f>>=32; f*=t
 *        lsrs(flo, fhi, 1);          // f>>=33;
 *        smlal(alo, ahi, flo, A);    // a+=(f>>33)*A;
 *        lsrs(alo, ahi, 6);          // a>>=38
 *
 *        return alo;
 *      }
 *
 *  This is rewritten in ARM assembly for optimal performance (43 cycles to execute).
 *
 *  For AVR, the precision of coefficients is scaled so the Bézier curve can be evaluated in real-time:
 *  Let's reduce precision as much as possible. After some experimentation we found that:
 *
 *    Assume t and AV with 24 bits is enough
 *       A =  6*(VF - VI)
 *       B = 15*(VI - VF)
 *       C = 10*(VF - VI)
 *       F =     VI
 *      AV = (1<<24)/TS   (this is computed at the planner, to offload expensive calculations from the ISR)
 *
 *    Instead of storing sign for each coefficient, we will store its absolute value,
 *    and flag the sign of the A coefficient, so we can save to store the sign bit.
 *    It always holds that sign(A) = - sign(B) = sign(C)
 *
 *     So, the resulting range of the coefficients are:
 *
 *       t: unsigned (0 <= t < 1) |range 0 to 0xFFFFFF unsigned
 *       A:   signed Q24 , range = 250000 * 6 = 1500000 = 0x16E360 | 21 bits
 *       B:   signed Q24 , range = 250000 *15 = 3750000 = 0x393870 | 22 bits
 *       C:   signed Q24 , range = 250000 *10 = 2500000 = 0x1312D0 | 21 bits
 *       F:   signed Q24 , range = 250000     =  250000 = 0x0ED090 | 20 bits
 *
 *    And for each curve, estimate its coefficients with:
 *
 *      void _calc_bezier_curve_coeffs(int32_t v0, int32_t v1, uint32_t av) {
 *       // Calculate the Bézier coefficients
 *       if (v1 < v0) {
 *         A_negative = true;
 *         bezier_A = 6 * (v0 - v1);
 *         bezier_B = 15 * (v0 - v1);
 *         bezier_C = 10 * (v0 - v1);
 *       }
 *       else {
 *         A_negative = false;
 *         bezier_A = 6 * (v1 - v0);
 *         bezier_B = 15 * (v1 - v0);
 *         bezier_C = 10 * (v1 - v0);
 *       }
 *       bezier_F = v0;
 *      }
 *
 *    And for each point, evaluate the curve with the following sequence:
 *
 *      // unsigned multiplication of 24 bits x 24bits, return upper 16 bits
 *      void umul24x24to16hi(uint16_t& r, uint24_t op1, uint24_t op2) {
 *        r = (uint64_t(op1) * op2) >> 8;
 *      }
 *      // unsigned multiplication of 16 bits x 16bits, return upper 16 bits
 *      void umul16x16to16hi(uint16_t& r, uint16_t op1, uint16_t op2) {
 *        r = (uint32_t(op1) * op2) >> 16;
 *      }
 *      // unsigned multiplication of 16 bits x 24bits, return upper 24 bits
 *      void umul16x24to24hi(uint24_t& r, uint16_t op1, uint24_t op2) {
 *        r = uint24_t((uint64_t(op1) * op2) >> 16);
 *      }
 *
 *      int32_t _eval_bezier_curve(uint32_t curr_step) {
 *        // To save computing, the first step is always the initial speed
 *        if (!curr_step)
 *          return bezier_F;
 *
 *        uint16_t t;
 *        umul24x24to16hi(t, bezier_AV, curr_step);   // t: Range 0 - 1^16 = 16 bits
 *        uint16_t f = t;
 *        umul16x16to16hi(f, f, t);           // Range 16 bits (unsigned)
 *        umul16x16to16hi(f, f, t);           // Range 16 bits : f = t^3  (unsigned)
 *        uint24_t acc = bezier_F;            // Range 20 bits (unsigned)
 *        if (A_negative) {
 *          uint24_t v;
 *          umul16x24to24hi(v, f, bezier_C);  // Range 21bits
 *          acc -= v;
 *          umul16x16to16hi(f, f, t);         // Range 16 bits : f = t^4  (unsigned)
 *          umul16x24to24hi(v, f, bezier_B);  // Range 22bits
 *          acc += v;
 *          umul16x16to16hi(f, f, t);         // Range 16 bits : f = t^5  (unsigned)
 *          umul16x24to24hi(v, f, bezier_A);  // Range 21bits + 15 = 36bits (plus sign)
 *          acc -= v;
 *        }
 *        else {
 *          uint24_t v;
 *          umul16x24to24hi(v, f, bezier_C);  // Range 21bits
 *          acc += v;
 *          umul16x16to16hi(f, f, t);         // Range 16 bits : f = t^4  (unsigned)
 *          umul16x24to24hi(v, f, bezier_B);  // Range 22bits
 *          acc -= v;
 *          umul16x16to16hi(f, f, t);         // Range 16 bits : f = t^5  (unsigned)
 *          umul16x24to24hi(v, f, bezier_A);  // Range 21bits + 15 = 36bits (plus sign)
 *          acc += v;
 *        }
 *        return acc;
 *      }
 *    These functions are translated to assembler for optimal performance.
 *    Coefficient calculation takes 70 cycles. Bezier point evaluation takes 150 cycles.
 */
#if ENABLED(BEZIER_JERK_CONTROL)

  #if ENABLED(__AVR__)

    // For AVR we use assembly to maximize speed
    void Stepper::_calc_bezier_curve_coeffs(const int32_t v0, const int32_t v1, const uint32_t av) {

      // Store advance
      bezier_AV = av;

      // Calculate the rest of the coefficients
      register uint8_t r2 = v0 & 0xFF;
      register uint8_t r3 = (v0 >> 8) & 0xFF;
      register uint8_t r12 = (v0 >> 16) & 0xFF;
      register uint8_t r5 = v1 & 0xFF;
      register uint8_t r6 = (v1 >> 8) & 0xFF;
      register uint8_t r7 = (v1 >> 16) & 0xFF;
      register uint8_t r4,r8,r9,r10,r11;

      __asm__ __volatile__(
        /* Calculate the Bézier coefficients */
        /*  %10:%1:%0 = v0*/
        /*  %5:%4:%3 = v1*/
        /*  %7:%6:%10 = temporary*/
        /*  %9 = val (must be high register!)*/
        /*  %10 (must be high register!)*/

        /* Store initial velocity*/
        A("sts bezier_F, %0")
        A("sts bezier_F+1, %1")
        A("sts bezier_F+2, %10")    /* bezier_F = %10:%1:%0 = v0 */

        /* Get delta speed */
        A("ldi %2,-1")              /* %2 = 0xFF, means A_negative = true */
        A("clr %8")                 /* %8 = 0 */
        A("sub %0,%3")
        A("sbc %1,%4")
        A("sbc %10,%5")             /*  v0 -= v1, C=1 if result is negative */
        A("brcc 1f")                /* branch if result is positive (C=0), that means v0 >= v1 */

        /*  Result was negative, get the absolute value*/
        A("com %10")
        A("com %1")
        A("neg %0")
        A("sbc %1,%2")
        A("sbc %10,%2")             /* %10:%1:%0 +1  -> %10:%1:%0 = -(v0 - v1) = (v1 - v0) */
        A("clr %2")                 /* %2 = 0, means A_negative = false */

        /*  Store negative flag*/
        L("1")
        A("sts A_negative, %2")     /* Store negative flag */

        /*  Compute coefficients A,B and C   [20 cycles worst case]*/
        A("ldi %9,6")               /* %9 = 6 */
        A("mul %0,%9")              /* r1:r0 = 6*LO(v0-v1) */
        A("sts bezier_A, r0")
        A("mov %6,r1")
        A("clr %7")                 /* %7:%6:r0 = 6*LO(v0-v1) */
        A("mul %1,%9")              /* r1:r0 = 6*MI(v0-v1) */
        A("add %6,r0")
        A("adc %7,r1")              /* %7:%6:?? += 6*MI(v0-v1) << 8 */
        A("mul %10,%9")             /* r1:r0 = 6*HI(v0-v1) */
        A("add %7,r0")              /* %7:%6:?? += 6*HI(v0-v1) << 16 */
        A("sts bezier_A+1, %6")
        A("sts bezier_A+2, %7")     /* bezier_A = %7:%6:?? = 6*(v0-v1) [35 cycles worst] */

        A("ldi %9,15")              /* %9 = 15 */
        A("mul %0,%9")              /* r1:r0 = 5*LO(v0-v1) */
        A("sts bezier_B, r0")
        A("mov %6,r1")
        A("clr %7")                 /* %7:%6:?? = 5*LO(v0-v1) */
        A("mul %1,%9")              /* r1:r0 = 5*MI(v0-v1) */
        A("add %6,r0")
        A("adc %7,r1")              /* %7:%6:?? += 5*MI(v0-v1) << 8 */
        A("mul %10,%9")             /* r1:r0 = 5*HI(v0-v1) */
        A("add %7,r0")              /* %7:%6:?? += 5*HI(v0-v1) << 16 */
        A("sts bezier_B+1, %6")
        A("sts bezier_B+2, %7")     /* bezier_B = %7:%6:?? = 5*(v0-v1) [50 cycles worst] */

        A("ldi %9,10")              /* %9 = 10 */
        A("mul %0,%9")              /* r1:r0 = 10*LO(v0-v1) */
        A("sts bezier_C, r0")
        A("mov %6,r1")
        A("clr %7")                 /* %7:%6:?? = 10*LO(v0-v1) */
        A("mul %1,%9")              /* r1:r0 = 10*MI(v0-v1) */
        A("add %6,r0")
        A("adc %7,r1")              /* %7:%6:?? += 10*MI(v0-v1) << 8 */
        A("mul %10,%9")             /* r1:r0 = 10*HI(v0-v1) */
        A("add %7,r0")              /* %7:%6:?? += 10*HI(v0-v1) << 16 */
        A("sts bezier_C+1, %6")
        " sts bezier_C+2, %7"       /* bezier_C = %7:%6:?? = 10*(v0-v1) [65 cycles worst] */
        : "+r" (r2),
          "+d" (r3),
          "=r" (r4),
          "+r" (r5),
          "+r" (r6),
          "+r" (r7),
          "=r" (r8),
          "=r" (r9),
          "=r" (r10),
          "=d" (r11),
          "+r" (r12)
        :
        : "r0", "r1", "cc", "memory"
      );
    }

    FORCE_INLINE int32_t Stepper::_eval_bezier_curve(const uint32_t curr_step) {

      // If dealing with the first step, save expensive computing and return the initial speed
      if (!curr_step)
        return bezier_F;

      register uint8_t r0 = 0; /* Zero register */
      register uint8_t r2 = (curr_step) & 0xFF;
      register uint8_t r3 = (curr_step >> 8) & 0xFF;
      register uint8_t r4 = (curr_step >> 16) & 0xFF;
      register uint8_t r1,r5,r6,r7,r8,r9,r10,r11; /* Temporary registers */

      __asm__ __volatile(
        /* umul24x24to16hi(t, bezier_AV, curr_step);  t: Range 0 - 1^16 = 16 bits*/
        A("lds %9,bezier_AV")       /* %9 = LO(AV)*/
        A("mul %9,%2")              /* r1:r0 = LO(bezier_AV)*LO(curr_step)*/
        A("mov %7,r1")              /* %7 = LO(bezier_AV)*LO(curr_step) >> 8*/
        A("clr %8")                 /* %8:%7  = LO(bezier_AV)*LO(curr_step) >> 8*/
        A("lds %10,bezier_AV+1")    /* %10 = MI(AV)*/
        A("mul %10,%2")             /* r1:r0  = MI(bezier_AV)*LO(curr_step)*/
        A("add %7,r0")
        A("adc %8,r1")              /* %8:%7 += MI(bezier_AV)*LO(curr_step)*/
        A("lds r1,bezier_AV+2")     /* r11 = HI(AV)*/
        A("mul r1,%2")              /* r1:r0  = HI(bezier_AV)*LO(curr_step)*/
        A("add %8,r0")              /* %8:%7 += HI(bezier_AV)*LO(curr_step) << 8*/
        A("mul %9,%3")              /* r1:r0 =  LO(bezier_AV)*MI(curr_step)*/
        A("add %7,r0")
        A("adc %8,r1")              /* %8:%7 += LO(bezier_AV)*MI(curr_step)*/
        A("mul %10,%3")             /* r1:r0 =  MI(bezier_AV)*MI(curr_step)*/
        A("add %8,r0")              /* %8:%7 += LO(bezier_AV)*MI(curr_step) << 8*/
        A("mul %9,%4")              /* r1:r0 =  LO(bezier_AV)*HI(curr_step)*/
        A("add %8,r0")              /* %8:%7 += LO(bezier_AV)*HI(curr_step) << 8*/
        /* %8:%7 = t*/

        /* uint16_t f = t;*/
        A("mov %5,%7")              /* %6:%5 = f*/
        A("mov %6,%8")
        /* %6:%5 = f*/

        /* umul16x16to16hi(f, f, t); / Range 16 bits (unsigned) [17] */
        A("mul %5,%7")              /* r1:r0 = LO(f) * LO(t)*/
        A("mov %9,r1")              /* store MIL(LO(f) * LO(t)) in %9, we need it for rounding*/
        A("clr %10")                /* %10 = 0*/
        A("clr %11")                /* %11 = 0*/
        A("mul %5,%8")              /* r1:r0 = LO(f) * HI(t)*/
        A("add %9,r0")              /* %9 += LO(LO(f) * HI(t))*/
        A("adc %10,r1")             /* %10 = HI(LO(f) * HI(t))*/
        A("adc %11,%0")             /* %11 += carry*/
        A("mul %6,%7")              /* r1:r0 = HI(f) * LO(t)*/
        A("add %9,r0")              /* %9 += LO(HI(f) * LO(t))*/
        A("adc %10,r1")             /* %10 += HI(HI(f) * LO(t)) */
        A("adc %11,%0")             /* %11 += carry*/
        A("mul %6,%8")              /* r1:r0 = HI(f) * HI(t)*/
        A("add %10,r0")             /* %10 += LO(HI(f) * HI(t))*/
        A("adc %11,r1")             /* %11 += HI(HI(f) * HI(t))*/
        A("mov %5,%10")             /* %6:%5 = */
        A("mov %6,%11")             /* f = %10:%11*/

        /* umul16x16to16hi(f, f, t); / Range 16 bits : f = t^3  (unsigned) [17]*/
        A("mul %5,%7")              /* r1:r0 = LO(f) * LO(t)*/
        A("mov %1,r1")              /* store MIL(LO(f) * LO(t)) in %1, we need it for rounding*/
        A("clr %10")                /* %10 = 0*/
        A("clr %11")                /* %11 = 0*/
        A("mul %5,%8")              /* r1:r0 = LO(f) * HI(t)*/
        A("add %1,r0")              /* %1 += LO(LO(f) * HI(t))*/
        A("adc %10,r1")             /* %10 = HI(LO(f) * HI(t))*/
        A("adc %11,%0")             /* %11 += carry*/
        A("mul %6,%7")              /* r1:r0 = HI(f) * LO(t)*/
        A("add %1,r0")              /* %1 += LO(HI(f) * LO(t))*/
        A("adc %10,r1")             /* %10 += HI(HI(f) * LO(t))*/
        A("adc %11,%0")             /* %11 += carry*/
        A("mul %6,%8")              /* r1:r0 = HI(f) * HI(t)*/
        A("add %10,r0")             /* %10 += LO(HI(f) * HI(t))*/
        A("adc %11,r1")             /* %11 += HI(HI(f) * HI(t))*/
        A("mov %5,%10")             /* %6:%5 =*/
        A("mov %6,%11")             /* f = %10:%11*/
        /* [15 +17*2] = [49]*/

        /* %4:%3:%2 will be acc from now on*/

        /* uint24_t acc = bezier_F; / Range 20 bits (unsigned)*/
        A("clr %9")                 /* "decimal place we get for free"*/
        A("lds %2,bezier_F")
        A("lds %3,bezier_F+1")
        A("lds %4,bezier_F+2")      /* %4:%3:%2 = acc*/

        /* if (A_negative) {*/
        A("lds r0,A_negative")
        A("or r0,%0")               /* Is flag signalling negative? */
        A("brne 3f")                /* If yes, Skip next instruction if A was negative*/
        A("rjmp 1f")                /* Otherwise, jump */

        /* uint24_t v; */
        /* umul16x24to24hi(v, f, bezier_C); / Range 21bits [29] */
        /* acc -= v; */
        L("3")
        A("lds %10, bezier_C")      /* %10 = LO(bezier_C)*/
        A("mul %10,%5")             /* r1:r0 = LO(bezier_C) * LO(f)*/
        A("sub %9,r1")
        A("sbc %2,%0")
        A("sbc %3,%0")
        A("sbc %4,%0")              /* %4:%3:%2:%9 -= HI(LO(bezier_C) * LO(f))*/
        A("lds %11, bezier_C+1")    /* %11 = MI(bezier_C)*/
        A("mul %11,%5")             /* r1:r0 = MI(bezier_C) * LO(f)*/
        A("sub %9,r0")
        A("sbc %2,r1")
        A("sbc %3,%0")
        A("sbc %4,%0")              /* %4:%3:%2:%9 -= MI(bezier_C) * LO(f)*/
        A("lds %1, bezier_C+2")     /* %1 = HI(bezier_C)*/
        A("mul %1,%5")              /* r1:r0 = MI(bezier_C) * LO(f)*/
        A("sub %2,r0")
        A("sbc %3,r1")
        A("sbc %4,%0")              /* %4:%3:%2:%9 -= HI(bezier_C) * LO(f) << 8*/
        A("mul %10,%6")             /* r1:r0 = LO(bezier_C) * MI(f)*/
        A("sub %9,r0")
        A("sbc %2,r1")
        A("sbc %3,%0")
        A("sbc %4,%0")              /* %4:%3:%2:%9 -= LO(bezier_C) * MI(f)*/
        A("mul %11,%6")             /* r1:r0 = MI(bezier_C) * MI(f)*/
        A("sub %2,r0")
        A("sbc %3,r1")
        A("sbc %4,%0")              /* %4:%3:%2:%9 -= MI(bezier_C) * MI(f) << 8*/
        A("mul %1,%6")              /* r1:r0 = HI(bezier_C) * LO(f)*/
        A("sub %3,r0")
        A("sbc %4,r1")              /* %4:%3:%2:%9 -= HI(bezier_C) * LO(f) << 16*/

        /* umul16x16to16hi(f, f, t); / Range 16 bits : f = t^3  (unsigned) [17]*/
        A("mul %5,%7")              /* r1:r0 = LO(f) * LO(t)*/
        A("mov %1,r1")              /* store MIL(LO(f) * LO(t)) in %1, we need it for rounding*/
        A("clr %10")                /* %10 = 0*/
        A("clr %11")                /* %11 = 0*/
        A("mul %5,%8")              /* r1:r0 = LO(f) * HI(t)*/
        A("add %1,r0")              /* %1 += LO(LO(f) * HI(t))*/
        A("adc %10,r1")             /* %10 = HI(LO(f) * HI(t))*/
        A("adc %11,%0")             /* %11 += carry*/
        A("mul %6,%7")              /* r1:r0 = HI(f) * LO(t)*/
        A("add %1,r0")              /* %1 += LO(HI(f) * LO(t))*/
        A("adc %10,r1")             /* %10 += HI(HI(f) * LO(t))*/
        A("adc %11,%0")             /* %11 += carry*/
        A("mul %6,%8")              /* r1:r0 = HI(f) * HI(t)*/
        A("add %10,r0")             /* %10 += LO(HI(f) * HI(t))*/
        A("adc %11,r1")             /* %11 += HI(HI(f) * HI(t))*/
        A("mov %5,%10")             /* %6:%5 =*/
        A("mov %6,%11")             /* f = %10:%11*/

        /* umul16x24to24hi(v, f, bezier_B); / Range 22bits [29]*/
        /* acc += v; */
        A("lds %10, bezier_B")      /* %10 = LO(bezier_B)*/
        A("mul %10,%5")             /* r1:r0 = LO(bezier_B) * LO(f)*/
        A("add %9,r1")
        A("adc %2,%0")
        A("adc %3,%0")
        A("adc %4,%0")              /* %4:%3:%2:%9 += HI(LO(bezier_B) * LO(f))*/
        A("lds %11, bezier_B+1")    /* %11 = MI(bezier_B)*/
        A("mul %11,%5")             /* r1:r0 = MI(bezier_B) * LO(f)*/
        A("add %9,r0")
        A("adc %2,r1")
        A("adc %3,%0")
        A("adc %4,%0")              /* %4:%3:%2:%9 += MI(bezier_B) * LO(f)*/
        A("lds %1, bezier_B+2")     /* %1 = HI(bezier_B)*/
        A("mul %1,%5")              /* r1:r0 = MI(bezier_B) * LO(f)*/
        A("add %2,r0")
        A("adc %3,r1")
        A("adc %4,%0")              /* %4:%3:%2:%9 += HI(bezier_B) * LO(f) << 8*/
        A("mul %10,%6")             /* r1:r0 = LO(bezier_B) * MI(f)*/
        A("add %9,r0")
        A("adc %2,r1")
        A("adc %3,%0")
        A("adc %4,%0")              /* %4:%3:%2:%9 += LO(bezier_B) * MI(f)*/
        A("mul %11,%6")             /* r1:r0 = MI(bezier_B) * MI(f)*/
        A("add %2,r0")
        A("adc %3,r1")
        A("adc %4,%0")              /* %4:%3:%2:%9 += MI(bezier_B) * MI(f) << 8*/
        A("mul %1,%6")              /* r1:r0 = HI(bezier_B) * LO(f)*/
        A("add %3,r0")
        A("adc %4,r1")              /* %4:%3:%2:%9 += HI(bezier_B) * LO(f) << 16*/

        /* umul16x16to16hi(f, f, t); / Range 16 bits : f = t^5  (unsigned) [17]*/
        A("mul %5,%7")              /* r1:r0 = LO(f) * LO(t)*/
        A("mov %1,r1")              /* store MIL(LO(f) * LO(t)) in %1, we need it for rounding*/
        A("clr %10")                /* %10 = 0*/
        A("clr %11")                /* %11 = 0*/
        A("mul %5,%8")              /* r1:r0 = LO(f) * HI(t)*/
        A("add %1,r0")              /* %1 += LO(LO(f) * HI(t))*/
        A("adc %10,r1")             /* %10 = HI(LO(f) * HI(t))*/
        A("adc %11,%0")             /* %11 += carry*/
        A("mul %6,%7")              /* r1:r0 = HI(f) * LO(t)*/
        A("add %1,r0")              /* %1 += LO(HI(f) * LO(t))*/
        A("adc %10,r1")             /* %10 += HI(HI(f) * LO(t))*/
        A("adc %11,%0")             /* %11 += carry*/
        A("mul %6,%8")              /* r1:r0 = HI(f) * HI(t)*/
        A("add %10,r0")             /* %10 += LO(HI(f) * HI(t))*/
        A("adc %11,r1")             /* %11 += HI(HI(f) * HI(t))*/
        A("mov %5,%10")             /* %6:%5 =*/
        A("mov %6,%11")             /* f = %10:%11*/

        /* umul16x24to24hi(v, f, bezier_A); / Range 21bits [29]*/
        /* acc -= v; */
        A("lds %10, bezier_A")      /* %10 = LO(bezier_A)*/
        A("mul %10,%5")             /* r1:r0 = LO(bezier_A) * LO(f)*/
        A("sub %9,r1")
        A("sbc %2,%0")
        A("sbc %3,%0")
        A("sbc %4,%0")              /* %4:%3:%2:%9 -= HI(LO(bezier_A) * LO(f))*/
        A("lds %11, bezier_A+1")    /* %11 = MI(bezier_A)*/
        A("mul %11,%5")             /* r1:r0 = MI(bezier_A) * LO(f)*/
        A("sub %9,r0")
        A("sbc %2,r1")
        A("sbc %3,%0")
        A("sbc %4,%0")              /* %4:%3:%2:%9 -= MI(bezier_A) * LO(f)*/
        A("lds %1, bezier_A+2")     /* %1 = HI(bezier_A)*/
        A("mul %1,%5")              /* r1:r0 = MI(bezier_A) * LO(f)*/
        A("sub %2,r0")
        A("sbc %3,r1")
        A("sbc %4,%0")              /* %4:%3:%2:%9 -= HI(bezier_A) * LO(f) << 8*/
        A("mul %10,%6")             /* r1:r0 = LO(bezier_A) * MI(f)*/
        A("sub %9,r0")
        A("sbc %2,r1")
        A("sbc %3,%0")
        A("sbc %4,%0")              /* %4:%3:%2:%9 -= LO(bezier_A) * MI(f)*/
        A("mul %11,%6")             /* r1:r0 = MI(bezier_A) * MI(f)*/
        A("sub %2,r0")
        A("sbc %3,r1")
        A("sbc %4,%0")              /* %4:%3:%2:%9 -= MI(bezier_A) * MI(f) << 8*/
        A("mul %1,%6")              /* r1:r0 = HI(bezier_A) * LO(f)*/
        A("sub %3,r0")
        A("sbc %4,r1")              /* %4:%3:%2:%9 -= HI(bezier_A) * LO(f) << 16*/
        A("jmp 2f")                 /* Done!*/

        L("1")

        /* uint24_t v; */
        /* umul16x24to24hi(v, f, bezier_C); / Range 21bits [29]*/
        /* acc += v; */
        A("lds %10, bezier_C")      /* %10 = LO(bezier_C)*/
        A("mul %10,%5")             /* r1:r0 = LO(bezier_C) * LO(f)*/
        A("add %9,r1")
        A("adc %2,%0")
        A("adc %3,%0")
        A("adc %4,%0")              /* %4:%3:%2:%9 += HI(LO(bezier_C) * LO(f))*/
        A("lds %11, bezier_C+1")    /* %11 = MI(bezier_C)*/
        A("mul %11,%5")             /* r1:r0 = MI(bezier_C) * LO(f)*/
        A("add %9,r0")
        A("adc %2,r1")
        A("adc %3,%0")
        A("adc %4,%0")              /* %4:%3:%2:%9 += MI(bezier_C) * LO(f)*/
        A("lds %1, bezier_C+2")     /* %1 = HI(bezier_C)*/
        A("mul %1,%5")              /* r1:r0 = MI(bezier_C) * LO(f)*/
        A("add %2,r0")
        A("adc %3,r1")
        A("adc %4,%0")              /* %4:%3:%2:%9 += HI(bezier_C) * LO(f) << 8*/
        A("mul %10,%6")             /* r1:r0 = LO(bezier_C) * MI(f)*/
        A("add %9,r0")
        A("adc %2,r1")
        A("adc %3,%0")
        A("adc %4,%0")              /* %4:%3:%2:%9 += LO(bezier_C) * MI(f)*/
        A("mul %11,%6")             /* r1:r0 = MI(bezier_C) * MI(f)*/
        A("add %2,r0")
        A("adc %3,r1")
        A("adc %4,%0")              /* %4:%3:%2:%9 += MI(bezier_C) * MI(f) << 8*/
        A("mul %1,%6")              /* r1:r0 = HI(bezier_C) * LO(f)*/
        A("add %3,r0")
        A("adc %4,r1")              /* %4:%3:%2:%9 += HI(bezier_C) * LO(f) << 16*/

        /* umul16x16to16hi(f, f, t); / Range 16 bits : f = t^3  (unsigned) [17]*/
        A("mul %5,%7")              /* r1:r0 = LO(f) * LO(t)*/
        A("mov %1,r1")              /* store MIL(LO(f) * LO(t)) in %1, we need it for rounding*/
        A("clr %10")                /* %10 = 0*/
        A("clr %11")                /* %11 = 0*/
        A("mul %5,%8")              /* r1:r0 = LO(f) * HI(t)*/
        A("add %1,r0")              /* %1 += LO(LO(f) * HI(t))*/
        A("adc %10,r1")             /* %10 = HI(LO(f) * HI(t))*/
        A("adc %11,%0")             /* %11 += carry*/
        A("mul %6,%7")              /* r1:r0 = HI(f) * LO(t)*/
        A("add %1,r0")              /* %1 += LO(HI(f) * LO(t))*/
        A("adc %10,r1")             /* %10 += HI(HI(f) * LO(t))*/
        A("adc %11,%0")             /* %11 += carry*/
        A("mul %6,%8")              /* r1:r0 = HI(f) * HI(t)*/
        A("add %10,r0")             /* %10 += LO(HI(f) * HI(t))*/
        A("adc %11,r1")             /* %11 += HI(HI(f) * HI(t))*/
        A("mov %5,%10")             /* %6:%5 =*/
        A("mov %6,%11")             /* f = %10:%11*/

        /* umul16x24to24hi(v, f, bezier_B); / Range 22bits [29]*/
        /* acc -= v;*/
        A("lds %10, bezier_B")      /* %10 = LO(bezier_B)*/
        A("mul %10,%5")             /* r1:r0 = LO(bezier_B) * LO(f)*/
        A("sub %9,r1")
        A("sbc %2,%0")
        A("sbc %3,%0")
        A("sbc %4,%0")              /* %4:%3:%2:%9 -= HI(LO(bezier_B) * LO(f))*/
        A("lds %11, bezier_B+1")    /* %11 = MI(bezier_B)*/
        A("mul %11,%5")             /* r1:r0 = MI(bezier_B) * LO(f)*/
        A("sub %9,r0")
        A("sbc %2,r1")
        A("sbc %3,%0")
        A("sbc %4,%0")              /* %4:%3:%2:%9 -= MI(bezier_B) * LO(f)*/
        A("lds %1, bezier_B+2")     /* %1 = HI(bezier_B)*/
        A("mul %1,%5")              /* r1:r0 = MI(bezier_B) * LO(f)*/
        A("sub %2,r0")
        A("sbc %3,r1")
        A("sbc %4,%0")              /* %4:%3:%2:%9 -= HI(bezier_B) * LO(f) << 8*/
        A("mul %10,%6")             /* r1:r0 = LO(bezier_B) * MI(f)*/
        A("sub %9,r0")
        A("sbc %2,r1")
        A("sbc %3,%0")
        A("sbc %4,%0")              /* %4:%3:%2:%9 -= LO(bezier_B) * MI(f)*/
        A("mul %11,%6")             /* r1:r0 = MI(bezier_B) * MI(f)*/
        A("sub %2,r0")
        A("sbc %3,r1")
        A("sbc %4,%0")              /* %4:%3:%2:%9 -= MI(bezier_B) * MI(f) << 8*/
        A("mul %1,%6")              /* r1:r0 = HI(bezier_B) * LO(f)*/
        A("sub %3,r0")
        A("sbc %4,r1")              /* %4:%3:%2:%9 -= HI(bezier_B) * LO(f) << 16*/

        /* umul16x16to16hi(f, f, t); / Range 16 bits : f = t^5  (unsigned) [17]*/
        A("mul %5,%7")              /* r1:r0 = LO(f) * LO(t)*/
        A("mov %1,r1")              /* store MIL(LO(f) * LO(t)) in %1, we need it for rounding*/
        A("clr %10")                /* %10 = 0*/
        A("clr %11")                /* %11 = 0*/
        A("mul %5,%8")              /* r1:r0 = LO(f) * HI(t)*/
        A("add %1,r0")              /* %1 += LO(LO(f) * HI(t))*/
        A("adc %10,r1")             /* %10 = HI(LO(f) * HI(t))*/
        A("adc %11,%0")             /* %11 += carry*/
        A("mul %6,%7")              /* r1:r0 = HI(f) * LO(t)*/
        A("add %1,r0")              /* %1 += LO(HI(f) * LO(t))*/
        A("adc %10,r1")             /* %10 += HI(HI(f) * LO(t))*/
        A("adc %11,%0")             /* %11 += carry*/
        A("mul %6,%8")              /* r1:r0 = HI(f) * HI(t)*/
        A("add %10,r0")             /* %10 += LO(HI(f) * HI(t))*/
        A("adc %11,r1")             /* %11 += HI(HI(f) * HI(t))*/
        A("mov %5,%10")             /* %6:%5 =*/
        A("mov %6,%11")             /* f = %10:%11*/

        /* umul16x24to24hi(v, f, bezier_A); / Range 21bits [29]*/
        /* acc += v; */
        A("lds %10, bezier_A")      /* %10 = LO(bezier_A)*/
        A("mul %10,%5")             /* r1:r0 = LO(bezier_A) * LO(f)*/
        A("add %9,r1")
        A("adc %2,%0")
        A("adc %3,%0")
        A("adc %4,%0")              /* %4:%3:%2:%9 += HI(LO(bezier_A) * LO(f))*/
        A("lds %11, bezier_A+1")    /* %11 = MI(bezier_A)*/
        A("mul %11,%5")             /* r1:r0 = MI(bezier_A) * LO(f)*/
        A("add %9,r0")
        A("adc %2,r1")
        A("adc %3,%0")
        A("adc %4,%0")              /* %4:%3:%2:%9 += MI(bezier_A) * LO(f)*/
        A("lds %1, bezier_A+2")     /* %1 = HI(bezier_A)*/
        A("mul %1,%5")              /* r1:r0 = MI(bezier_A) * LO(f)*/
        A("add %2,r0")
        A("adc %3,r1")
        A("adc %4,%0")              /* %4:%3:%2:%9 += HI(bezier_A) * LO(f) << 8*/
        A("mul %10,%6")             /* r1:r0 = LO(bezier_A) * MI(f)*/
        A("add %9,r0")
        A("adc %2,r1")
        A("adc %3,%0")
        A("adc %4,%0")              /* %4:%3:%2:%9 += LO(bezier_A) * MI(f)*/
        A("mul %11,%6")             /* r1:r0 = MI(bezier_A) * MI(f)*/
        A("add %2,r0")
        A("adc %3,r1")
        A("adc %4,%0")              /* %4:%3:%2:%9 += MI(bezier_A) * MI(f) << 8*/
        A("mul %1,%6")              /* r1:r0 = HI(bezier_A) * LO(f)*/
        A("add %3,r0")
        A("adc %4,r1")              /* %4:%3:%2:%9 += HI(bezier_A) * LO(f) << 16*/
        L("2")
        " clr __zero_reg__"         /* C runtime expects r1 = __zero_reg__ = 0 */
        : "+r"(r0),
          "+r"(r1),
          "+r"(r2),
          "+r"(r3),
          "+r"(r4),
          "+r"(r5),
          "+r"(r6),
          "+r"(r7),
          "+r"(r8),
          "+r"(r9),
          "+r"(r10),
          "+r"(r11)
        :
        :"cc","r0","r1"
      );
      return (r2 | (uint16_t(r3) << 8)) | (uint32_t(r4) << 16);
    }

  #else // !ENABLED(__AVR__)

    FORCE_INLINE void Stepper::_calc_bezier_curve_coeffs(const int32_t v0, const int32_t v1, const uint32_t av) {
      // Calculate the Bézier coefficients
      bezier_A =  768 * (v1 - v0);
      bezier_B = 1920 * (v0 - v1);
      bezier_C = 1280 * (v1 - v0);
      bezier_F =  128 * v0;
      bezier_AV = av;
    }

    FORCE_INLINE int32_t Stepper::_eval_bezier_curve(const uint32_t curr_step) {
      uint32_t t = bezier_AV * curr_step;               // t: Range 0 - 1^32 = 32 bits
      uint64_t f = t;
      f *= t;                                           // Range 32*2 = 64 bits (unsigned)
      f >>= 32;                                         // Range 32 bits  (unsigned)
      f *= t;                                           // Range 32*2 = 64 bits  (unsigned)
      f >>= 32;                                         // Range 32 bits : f = t^3  (unsigned)
      int64_t acc = (int64_t) bezier_F << 31;           // Range 63 bits (signed)
      acc += ((uint32_t) f >> 1) * (int64_t) bezier_C;  // Range 29bits + 31 = 60bits (plus sign)
      f *= t;                                           // Range 32*2 = 64 bits
      f >>= 32;                                         // Range 32 bits : f = t^3  (unsigned)
      acc += ((uint32_t) f >> 1) * (int64_t) bezier_B;  // Range 29bits + 31 = 60bits (plus sign)
      f *= t;                                           // Range 32*2 = 64 bits
      f >>= 32;                                         // Range 32 bits : f = t^3  (unsigned)
      acc += ((uint32_t) f >> 1) * (int64_t) bezier_A;  // Range 28bits + 31 = 59bits (plus sign)
      acc >>= (31 + 7);                                 // Range 24bits (plus sign)
      return (int32_t) acc;
    }

  #endif // !ENABLED(__AVR__)

#endif // BEZIER_JERK_CONTROL

#if ENABLED(BABYSTEPPING)

  #if MECH(DELTA)
    #define CYCLES_EATEN_BABYSTEP (2 * 15)
  #else
    #define CYCLES_EATEN_BABYSTEP 0
  #endif
  #define EXTRA_CYCLES_BABYSTEP (HAL_min_pulse_tick - (CYCLES_EATEN_BABYSTEP))

  #if ENABLED(X_TWO_ENDSTOPS) || ENABLED(Y_TWO_ENDSTOPS) || ENABLED(Z_TWO_ENDSTOPS)
    #define TWO_ENDSTOP_APPLY_STEP(A,V)                                                                                        \
      if (homing_dual_axis) {                                                                                                   \
        if (A##_HOME_DIR < 0) {                                                                                                 \
          if (!(TEST(endstops.live_state, A##_MIN) && count_direction[_AXIS(A)] < 0) && !locked_##A##_motor) A##_STEP_WRITE(V);    \
          if (!(TEST(endstops.live_state, A##2_MIN) && count_direction[_AXIS(A)] < 0) && !locked_##A##2_motor) A##2_STEP_WRITE(V); \
        }                                                                                                                       \
        else {                                                                                                                  \
          if (!(TEST(endstops.live_state, A##_MAX) && count_direction[_AXIS(A)] > 0) && !locked_##A##_motor) A##_STEP_WRITE(V);    \
          if (!(TEST(endstops.live_state, A##2_MAX) && count_direction[_AXIS(A)] > 0) && !locked_##A##2_motor) A##2_STEP_WRITE(V); \
        }                                                                                                                       \
      }                                                                                                                         \
      else {                                                                                                                    \
        A##_STEP_WRITE(V);                                                                                                      \
        A##2_STEP_WRITE(V);                                                                                                     \
      }
  #endif

  #if ENABLED(X_TWO_STEPPER_DRIVERS)
    #if ENABLED(X_TWO_ENDSTOPS)
      #define X_APPLY_STEP(v,Q) TWO_ENDSTOP_APPLY_STEP(X,v)
    #else
      #define X_APPLY_STEP(v,Q) do{ X_STEP_WRITE(v); X2_STEP_WRITE(v); }while(0)
    #endif
  #elif ENABLED(DUAL_X_CARRIAGE)
    #define X_APPLY_STEP(v,ALWAYS) \
      if (mechanics.hotend_duplication_enabled || ALWAYS) { \
        X_STEP_WRITE(v); \
        X2_STEP_WRITE(v); \
      } \
      else { \
        if (movement_extruder()) X2_STEP_WRITE(v); else X_STEP_WRITE(v); \
      }
  #else
    #define X_APPLY_STEP(v,Q) X_STEP_WRITE(v)
  #endif

  #if ENABLED(Y_TWO_STEPPER_DRIVERS)
    #if ENABLED(Y_TWO_ENDSTOPS)
      #define Y_APPLY_STEP(v,Q) TWO_ENDSTOP_APPLY_STEP(Y,v)
    #else
      #define Y_APPLY_STEP(v,Q) do{ Y_STEP_WRITE(v); Y2_STEP_WRITE(v); }while(0)
    #endif
  #else
    #define Y_APPLY_DIR(v,Q) Y_DIR_WRITE(v)
    #define Y_APPLY_STEP(v,Q) Y_STEP_WRITE(v)
  #endif

  #if ENABLED(Z_TWO_STEPPER_DRIVERS)
    #if ENABLED(Z_TWO_ENDSTOPS)
      #define Z_APPLY_STEP(v,Q) TWO_ENDSTOP_APPLY_STEP(Z,v)
    #else
      #define Z_APPLY_STEP(v,Q) do{ Z_STEP_WRITE(v); Z2_STEP_WRITE(v); }while(0)
    #endif
  #else
    #define Z_APPLY_DIR(v,Q) Z_DIR_WRITE(v)
    #define Z_APPLY_STEP(v,Q) Z_STEP_WRITE(v)
  #endif

  #define _INVERT_STEP_PIN(PIN)     INVERT_## PIN ##_STEP_PIN
  #define _APPLY_STEP(AXIS) AXIS    ##_APPLY_STEP
  #define _ENABLE(AXIS)             enable_## AXIS()
  #define _READ_DIR(AXIS)           AXIS ##_DIR_READ
  #define _INVERT_DIR(AXIS)         isStepDir(AXIS ##_AXIS)
  #define _APPLY_DIR(AXIS, INVERT)  set_##AXIS##_dir(INVERT)

  #if EXTRA_CYCLES_BABYSTEP > 20
    #define _SAVE_START const hal_timer_t pulse_start = HAL_timer_get_current_count(STEPPER_TIMER)
    #define _PULSE_WAIT while (EXTRA_CYCLES_BABYSTEP > (uint32_t)(HAL_timer_get_current_count(STEPPER_TIMER) - pulse_start) * (STEPPER_TIMER_PRESCALE)) { /* nada */ }
  #else
    #define _SAVE_START NOOP
    #if EXTRA_CYCLES_BABYSTEP > 0
      #define _PULSE_WAIT DELAY_NS(EXTRA_CYCLES_BABYSTEP * NS_PER_CYCLE)
    #elif HAL_min_pulse_tick > 0
      #define _PULSE_WAIT NOOP
    #elif MECH(DELTA)
      #define _PULSE_WAIT DELAY_US(2);
    #else
      #define _PULSE_WAIT DELAY_US(4);
    #endif
  #endif

  #define BABYSTEP_AXIS(AXIS, INVERT, DIR) {            \
      const uint8_t old_dir = _READ_DIR(AXIS);          \
      _ENABLE(AXIS);                                    \
      _APPLY_DIR(AXIS, _INVERT_DIR(AXIS)^DIR^INVERT);   \
      if (direction_delay >= 50)                        \
        HAL::delayNanoseconds(direction_delay);         \
      _SAVE_START;                                      \
      _APPLY_STEP(AXIS)(!_INVERT_STEP_PIN(AXIS), true); \
      _PULSE_WAIT;                                      \
      _APPLY_STEP(AXIS)(_INVERT_STEP_PIN(AXIS), true);  \
      _APPLY_DIR(AXIS, old_dir);                        \
    }

  // MUST ONLY BE CALLED BY AN ISR,
  // No other ISR should ever interrupt this!
  void Stepper::babystep(const AxisEnum axis, const bool direction) {
    cli();

    switch (axis) {

      #if ENABLED(BABYSTEP_XY)

        case X_AXIS:
          #if CORE_IS_XY
            BABYSTEP_AXIS(X, false, direction);
            BABYSTEP_AXIS(Y, false, direction);
          #elif CORE_IS_XZ
            BABYSTEP_AXIS(X, false, direction);
            BABYSTEP_AXIS(Z, false, direction);
          #else
            BABYSTEP_AXIS(X, false, direction);
          #endif
          break;

        case Y_AXIS:
          #if CORE_IS_XY
            BABYSTEP_AXIS(X, false, direction);
            BABYSTEP_AXIS(Y, false, direction^(CORESIGN(1)<0));
          #elif CORE_IS_YZ
            BABYSTEP_AXIS(Y, false, direction);
            BABYSTEP_AXIS(Z, false, direction^(CORESIGN(1)<0));
          #else
            BABYSTEP_AXIS(Y, false, direction);
          #endif
          break;

      #endif

      case Z_AXIS: {

        #if CORE_IS_XZ
          BABYSTEP_AXIS(X, BABYSTEP_INVERT_Z, direction);
          BABYSTEP_AXIS(Z, BABYSTEP_INVERT_Z, direction^(CORESIGN(1)<0));

        #elif CORE_IS_YZ
          BABYSTEP_AXIS(Y, BABYSTEP_INVERT_Z, direction);
          BABYSTEP_AXIS(Z, BABYSTEP_INVERT_Z, direction^(CORESIGN(1)<0));

        #elif DISABLED(DELTA)
          BABYSTEP_AXIS(Z, BABYSTEP_INVERT_Z, direction);

        #else // DELTA

          const bool z_direction = direction ^ BABYSTEP_INVERT_Z;

          enable_X();
          enable_Y();
          enable_Z();

          const uint8_t old_x_dir_pin = X_DIR_READ,
                        old_y_dir_pin = Y_DIR_READ,
                        old_z_dir_pin = Z_DIR_READ;

          X_DIR_WRITE(isStepDir(X_AXIS) ^ z_direction);
          Y_DIR_WRITE(isStepDir(Y_AXIS) ^ z_direction);
          Z_DIR_WRITE(isStepDir(Z_AXIS) ^ z_direction);

          if (direction_delay >= 50)
            HAL::delayNanoseconds(direction_delay);

          _SAVE_START;

          X_STEP_WRITE(!INVERT_X_STEP_PIN);
          Y_STEP_WRITE(!INVERT_Y_STEP_PIN);
          Z_STEP_WRITE(!INVERT_Z_STEP_PIN);

          _PULSE_WAIT;

          X_STEP_WRITE(INVERT_X_STEP_PIN);
          Y_STEP_WRITE(INVERT_Y_STEP_PIN);
          Z_STEP_WRITE(INVERT_Z_STEP_PIN);

          // Restore direction bits
          X_DIR_WRITE(old_x_dir_pin);
          Y_DIR_WRITE(old_y_dir_pin);
          Z_DIR_WRITE(old_z_dir_pin);

        #endif

      } break;

      default: break;
    }
    sei();
  }

#endif //BABYSTEPPING

#if HAS_DIGIPOTSS || HAS_MOTOR_CURRENT_PWM

  void Stepper::digipot_init() {
    #if HAS_DIGIPOTSS
      static const uint8_t digipot_motor_current[] = DIGIPOT_MOTOR_CURRENT;

      SPI.begin();
      SET_OUTPUT(DIGIPOTSS_PIN);

      for (uint8_t i = 0; i < COUNT(digipot_motor_current); i++) {
        //digitalPotWrite(digipot_ch[i], digipot_motor_current[i]);
        digipot_current(i, digipot_motor_current[i]);
      }

    #elif HAS_MOTOR_CURRENT_PWM

      #if PIN_EXISTS(MOTOR_CURRENT_PWM_XY)
        SET_OUTPUT(MOTOR_CURRENT_PWM_XY_PIN);
        digipot_current(0, motor_current_setting[0]);
      #endif
      #if PIN_EXISTS(MOTOR_CURRENT_PWM_Z)
        SET_OUTPUT(MOTOR_CURRENT_PWM_Z_PIN);
        digipot_current(1, motor_current_setting[1]);
      #endif
      #if PIN_EXISTS(MOTOR_CURRENT_PWM_E)
        SET_OUTPUT(MOTOR_CURRENT_PWM_E_PIN);
        digipot_current(2, motor_current_setting[2]);
      #endif
      //Set timer5 to 31khz so the PWM of the motor power is as constant as possible. (removes a buzzing noise)
      TCCR5B = (TCCR5B & ~(_BV(CS50) | _BV(CS51) | _BV(CS52))) | _BV(CS50);
    #endif
  }
  
#endif

#if HAS_MICROSTEPS

  void Stepper::microstep_init() {

    #if HAS_X_MICROSTEPS
      SET_OUTPUT(X_MS1_PIN);
    #endif
    #if HAS_Y_MICROSTEPS
      SET_OUTPUT(Y_MS1_PIN);
    #endif
    #if HAS_Z_MICROSTEPS
      SET_OUTPUT(Z_MS1_PIN);
    #endif
    #if HAS_E0_MICROSTEPS
      SET_OUTPUT(E0_MS1_PIN);
    #endif
    #if HAS_E1_MICROSTEPS
      SET_OUTPUT(E1_MS1_PIN);
    #endif
    #if HAS_E2_MICROSTEPS
      SET_OUTPUT(E2_MS1_PIN);
    #endif
    #if HAS_E3_MICROSTEPS
      SET_OUTPUT(E3_MS1_PIN);
    #endif
    #if HAS_E4_MICROSTEPS
      SET_OUTPUT(E4_MS1_PIN);
    #endif
    #if HAS_E5_MICROSTEPS
      SET_OUTPUT(E5_MS1_PIN);
    #endif

    #if !MB(ALLIGATOR_R2) && !MB(ALLIGATOR_R3)
      #if HAS_X_MICROSTEPS
        SET_OUTPUT(X_MS2_PIN);
      #endif
      #if HAS_Y_MICROSTEPS
        SET_OUTPUT(Y_MS2_PIN);
      #endif
      #if HAS_Z_MICROSTEPS
        SET_OUTPUT(Z_MS2_PIN);
      #endif
      #if HAS_E0_MICROSTEPS
        SET_OUTPUT(E0_MS2_PIN);
      #endif
      #if HAS_E1_MICROSTEPS
        SET_OUTPUT(E1_MS2_PIN);
      #endif
      #if HAS_E2_MICROSTEPS
        SET_OUTPUT(E2_MS2_PIN);
      #endif
      #if HAS_E3_MICROSTEPS
        SET_OUTPUT(E3_MS2_PIN);
      #endif
      #if HAS_E4_MICROSTEPS
        SET_OUTPUT(E4_MS2_PIN);
      #endif
      #if HAS_E5_MICROSTEPS
        SET_OUTPUT(E5_MS2_PIN);
      #endif
    #endif

    static const uint8_t microstep_modes[] = { X_MICROSTEPS, Y_MICROSTEPS, Z_MICROSTEPS, E0_MICROSTEPS };
    for (uint16_t i = 0; i < COUNT(microstep_modes); i++)
      microstep_mode(i, microstep_modes[i]);
  }

#endif

#if HAS_EXT_ENCODER

  void Stepper::test_extruder_encoder() {

    static const pin_t encoder_pin[] = { E0_ENC_PIN, E1_ENC_PIN, E2_ENC_PIN, E3_ENC_PIN, E4_ENC_PIN, E5_ENC_PIN };

    if (delta_error[E_AXIS] >= 0 && encoder_pin[active_extruder] > 0) {
      const uint8_t sig = READ_ENCODER(encoder_pin[active_extruder]);
      tools.encStepsSinceLastSignal[active_extruder] += tools.encLastDir[active_extruder];
      if (tools.encLastSignal[active_extruder] != sig && ABS(tools.encStepsSinceLastSignal[active_extruder] - tools.encLastChangeAt[active_extruder]) > ENC_MIN_STEPS) {
        if (sig) tools.encStepsSinceLastSignal[active_extruder] = 0;
        tools.encLastSignal[active_extruder] = sig;
        tools.encLastChangeAt[active_extruder] = tools.encStepsSinceLastSignal[active_extruder];
      }
      else if (ABS(tools.encStepsSinceLastSignal[active_extruder]) > tools.encErrorSteps[active_extruder]) {
        if (tools.encLastDir[active_extruder] > 0)
          printer.setInterruptEvent(INTERRUPT_EVENT_ENC_DETECT);
      }
    }

  }

#endif
