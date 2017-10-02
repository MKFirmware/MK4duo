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

#include "../../base.h"
#include "stepper.h"

#if ENABLED(__AVR__)
  #include "speed_lookuptable.h"
#endif

#if HAS_DIGIPOTSS
  #include <SPI.h>
#endif

Stepper stepper; // Singleton

// public:

block_t* Stepper::current_block = NULL;  // A pointer to the block currently being traced

#if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
  #if ENABLED(ABORT_ON_ENDSTOP_HIT_INIT)
    bool Stepper::abort_on_endstop_hit = ABORT_ON_ENDSTOP_HIT_INIT;
  #else
    bool Stepper::abort_on_endstop_hit = false;
  #endif
#endif

#if ENABLED(Z_TWO_ENDSTOPS)
  bool Stepper::performing_homing = false;
#endif

millis_t Stepper::stepper_inactive_time  = (DEFAULT_STEPPER_DEACTIVE_TIME) * 1000UL;

// private:

unsigned char Stepper::last_direction_bits = 0;        // The next stepping-bits to be output
unsigned int Stepper::cleaning_buffer_counter = 0;

#if ENABLED(Z_FOUR_ENDSTOPS)
  bool Stepper::locked_z_motor = false;
  bool Stepper::locked_z2_motor = false;
  bool Stepper::locked_z3_motor = false;
  bool Stepper::locked_z4_motor = false;
#elif ENABLED(Z_THREE_ENDSTOPS)
  bool Stepper::locked_z_motor = false;
  bool Stepper::locked_z2_motor = false;
  bool Stepper::locked_z3_motor = false;
#elif ENABLED(Z_TWO_ENDSTOPS)
  bool Stepper::locked_z_motor = false;
  bool Stepper::locked_z2_motor = false;
#endif

long  Stepper::counter_X = 0,
      Stepper::counter_Y = 0,
      Stepper::counter_Z = 0,
      Stepper::counter_E = 0;

volatile uint32_t Stepper::step_events_completed = 0; // The number of step events executed in the current block

#if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)

  HAL_TIMER_TYPE  Stepper::nextMainISR = 0,
                  Stepper::nextAdvanceISR = ADV_NEVER,
                  Stepper::eISR_Rate = ADV_NEVER;

  #if ENABLED(LIN_ADVANCE)
    int Stepper::e_steps[DRIVER_EXTRUDERS],
        Stepper::final_estep_rate,
        Stepper::current_estep_rate[DRIVER_EXTRUDERS],
        Stepper::current_adv_steps[DRIVER_EXTRUDERS];
  #else
    long  Stepper::e_steps[DRIVER_EXTRUDERS],
          Stepper::final_advance = 0,
          Stepper::old_advance = 0,
          Stepper::advance_rate,
          Stepper::advance;
  #endif

  FORCE_INLINE HAL_TIMER_TYPE adv_rate(const int steps, const HAL_TIMER_TYPE timer, const uint8_t loops) {
    if (steps) {
      const HAL_TIMER_TYPE rate = (timer * loops) / abs(steps);
      //return constrain(rate, 1, ADV_NEVER - 1)
      return rate ? rate : 1;
    }
    return ADV_NEVER;
  }

#endif // ADVANCE or LIN_ADVANCE

volatile long Stepper::machine_position[NUM_AXIS] = { 0 };
volatile signed char Stepper::count_direction[NUM_AXIS] = { 1, 1, 1, 1 };

#if ENABLED(COLOR_MIXING_EXTRUDER)
  long Stepper::counter_m[MIXING_STEPPERS];
#endif

#if ENABLED(LASER)
  long Stepper::counter_L;
  #if ENABLED(LASER_RASTER)
    int Stepper::counter_raster;
  #endif // LASER_RASTER
#endif // LASER

long            Stepper::acceleration_time,
                Stepper::deceleration_time;

HAL_TIMER_TYPE  Stepper::acc_step_rate, // needed for deceleration start point
                Stepper::OCR1A_nominal;

uint8_t         Stepper::step_loops,
                Stepper::step_loops_nominal;

volatile long   Stepper::endstops_trigsteps[XYZ];

#if ENABLED(X_TWO_STEPPER)
  #define X_APPLY_DIR(v,Q)  { X_DIR_WRITE(v); X2_DIR_WRITE(v != INVERT_X2_VS_X_DIR); }
  #define X_APPLY_STEP(v,Q) { X_STEP_WRITE(v); X2_STEP_WRITE(v); }
#elif ENABLED(DUAL_X_CARRIAGE)
  #define X_APPLY_DIR(v,ALWAYS) \
    if (mechanics.hotend_duplication_enabled || ALWAYS) { \
      X_DIR_WRITE(v); \
      X2_DIR_WRITE(v); \
    } \
    else { \
      if (TOOL_E_INDEX != 0) X2_DIR_WRITE(v); else X_DIR_WRITE(v); \
    }
  #define X_APPLY_STEP(v,ALWAYS) \
    if (mechanics.hotend_duplication_enabled || ALWAYS) { \
      X_STEP_WRITE(v); \
      X2_STEP_WRITE(v); \
    } \
    else { \
      if (TOOL_E_INDEX) X2_STEP_WRITE(v); else X_STEP_WRITE(v); \
    }
#else
  #define X_APPLY_DIR(v,Q) X_DIR_WRITE(v)
  #define X_APPLY_STEP(v,Q) X_STEP_WRITE(v)
#endif

#if ENABLED(Y_TWO_STEPPER)
  #define Y_APPLY_DIR(v,Q)  { Y_DIR_WRITE(v); Y2_DIR_WRITE(v != INVERT_Y2_VS_Y_DIR); }
  #define Y_APPLY_STEP(v,Q) { Y_STEP_WRITE(v); Y2_STEP_WRITE(v); }
#else
  #define Y_APPLY_DIR(v,Q) Y_DIR_WRITE(v)
  #define Y_APPLY_STEP(v,Q) Y_STEP_WRITE(v)
#endif

#if ENABLED(Z_FOUR_STEPPER)
  #define Z_APPLY_DIR(v,Q) { Z_DIR_WRITE(v); Z2_DIR_WRITE(v != INVERT_Z2_VS_Z_DIR); Z3_DIR_WRITE(v != INVERT_Z3_VS_Z_DIR); Z4_DIR_WRITE(v != INVERT_Z4_VS_Z_DIR); }
  #if ENABLED(Z_FOUR_ENDSTOPS)
    #define Z_APPLY_STEP(v,Q) \
    if (performing_homing) { \
      if (Z_HOME_DIR > 0) {\
        if (!(TEST(endstops.old_endstop_bits, Z_MAX) && (count_direction[Z_AXIS] > 0)) && !locked_z_motor) Z_STEP_WRITE(v); \
        if (!(TEST(endstops.old_endstop_bits, Z2_MAX) && (count_direction[Z_AXIS] > 0)) && !locked_z2_motor) Z2_STEP_WRITE(v); \
        if (!(TEST(endstops.old_endstop_bits, Z3_MAX) && (count_direction[Z_AXIS] > 0)) && !locked_z3_motor) Z3_STEP_WRITE(v); \
        if (!(TEST(endstops.old_endstop_bits, Z4_MAX) && (count_direction[Z_AXIS] > 0)) && !locked_z4_motor) Z4_STEP_WRITE(v); \
      } \
      else { \
        if (!(TEST(endstops.old_endstop_bits, Z_MIN) && (count_direction[Z_AXIS] < 0)) && !locked_z_motor) Z_STEP_WRITE(v); \
        if (!(TEST(endstops.old_endstop_bits, Z2_MIN) && (count_direction[Z_AXIS] < 0)) && !locked_z2_motor) Z2_STEP_WRITE(v); \
        if (!(TEST(endstops.old_endstop_bits, Z3_MIN) && (count_direction[Z_AXIS] < 0)) && !locked_z3_motor) Z3_STEP_WRITE(v); \
        if (!(TEST(endstops.old_endstop_bits, Z4_MIN) && (count_direction[Z_AXIS] < 0)) && !locked_z4_motor) Z4_STEP_WRITE(v); \
      } \
    } \
    else { \
      Z_STEP_WRITE(v); \
      Z2_STEP_WRITE(v); \
      Z3_STEP_WRITE(v); \
      Z4_STEP_WRITE(v); \
    }
  #else
    #define Z_APPLY_STEP(v,Q) do{ Z_STEP_WRITE(v); Z2_STEP_WRITE(v); Z3_STEP_WRITE(v); Z4_STEP_WRITE(v); }while(0)
  #endif
#elif ENABLED(Z_THREE_STEPPER)
  #define Z_APPLY_DIR(v,Q) { Z_DIR_WRITE(v); Z2_DIR_WRITE(v != INVERT_Z2_VS_Z_DIR); Z3_DIR_WRITE(v != INVERT_Z3_VS_Z_DIR); }
  #if ENABLED(Z_THREE_ENDSTOPS)
    #define Z_APPLY_STEP(v,Q) \
    if (performing_homing) { \
      if (Z_HOME_DIR > 0) {\
        if (!(TEST(endstops.old_endstop_bits, Z_MAX) && (count_direction[Z_AXIS] > 0)) && !locked_z_motor) Z_STEP_WRITE(v); \
        if (!(TEST(endstops.old_endstop_bits, Z2_MAX) && (count_direction[Z_AXIS] > 0)) && !locked_z2_motor) Z2_STEP_WRITE(v); \
        if (!(TEST(endstops.old_endstop_bits, Z3_MAX) && (count_direction[Z_AXIS] > 0)) && !locked_z3_motor) Z3_STEP_WRITE(v); \
      } \
      else { \
        if (!(TEST(endstops.old_endstop_bits, Z_MIN) && (count_direction[Z_AXIS] < 0)) && !locked_z_motor) Z_STEP_WRITE(v); \
        if (!(TEST(endstops.old_endstop_bits, Z2_MIN) && (count_direction[Z_AXIS] < 0)) && !locked_z2_motor) Z2_STEP_WRITE(v); \
        if (!(TEST(endstops.old_endstop_bits, Z3_MIN) && (count_direction[Z_AXIS] < 0)) && !locked_z3_motor) Z3_STEP_WRITE(v); \
      } \
    } \
    else { \
      Z_STEP_WRITE(v); \
      Z2_STEP_WRITE(v); \
      Z3_STEP_WRITE(v); \
    }
  #else
    #define Z_APPLY_STEP(v,Q) do{ Z_STEP_WRITE(v); Z2_STEP_WRITE(v); Z3_STEP_WRITE(v); }while(0)
  #endif
#elif ENABLED(Z_TWO_STEPPER)
  #define Z_APPLY_DIR(v,Q) { Z_DIR_WRITE(v); Z2_DIR_WRITE(v != INVERT_Z2_VS_Z_DIR); }
  #if ENABLED(Z_TWO_ENDSTOPS)
    #define Z_APPLY_STEP(v,Q) \
    if (performing_homing) { \
      if (Z_HOME_DIR > 0) {\
        if (!(TEST(endstops.old_endstop_bits, Z_MAX) && (count_direction[Z_AXIS] > 0)) && !locked_z_motor) Z_STEP_WRITE(v); \
        if (!(TEST(endstops.old_endstop_bits, Z2_MAX) && (count_direction[Z_AXIS] > 0)) && !locked_z2_motor) Z2_STEP_WRITE(v); \
      } \
      else { \
        if (!(TEST(endstops.old_endstop_bits, Z_MIN) && (count_direction[Z_AXIS] < 0)) && !locked_z_motor) Z_STEP_WRITE(v); \
        if (!(TEST(endstops.old_endstop_bits, Z2_MIN) && (count_direction[Z_AXIS] < 0)) && !locked_z2_motor) Z2_STEP_WRITE(v); \
      } \
    } \
    else { \
      Z_STEP_WRITE(v); \
      Z2_STEP_WRITE(v); \
    }
  #else
    #define Z_APPLY_STEP(v,Q) do{ Z_STEP_WRITE(v); Z2_STEP_WRITE(v); }while(0)
  #endif
#else
  #define Z_APPLY_DIR(v,Q) Z_DIR_WRITE(v)
  #define Z_APPLY_STEP(v,Q) Z_STEP_WRITE(v)
#endif

#if DISABLED(COLOR_MIXING_EXTRUDER)
  #define E_APPLY_STEP(v,Q) E_STEP_WRITE(v)
#endif


/**
 * Encoder Extruder definition
 */
#if HAS_EXT_ENCODER
  #define _TEST_EXTRUDER_ENC(x,pin) { \
    const uint8_t sig = READ_ENCODER(pin); \
    tools.encStepsSinceLastSignal[x] += tools.encLastDir[x]; \
    if (tools.encLastSignal[x] != sig && abs(tools.encStepsSinceLastSignal[x] - tools.encLastChangeAt[x]) > ENC_MIN_STEPS) { \
      if (sig) tools.encStepsSinceLastSignal[x] = 0; \
      tools.encLastSignal[x] = sig; \
      tools.encLastChangeAt[x] = tools.encStepsSinceLastSignal[x]; \
    } \
    else if (abs(tools.encStepsSinceLastSignal[x]) > tools.encErrorSteps[x]) { \
      if (tools.encLastDir[x] > 0) \
        printer.setInterruptEvent(INTERRUPT_EVENT_ENC_DETECT); \
    } \
  }

  #define RESET_EXTRUDER_ENC(x,dir) tools.encLastDir[x] = dir;

  #define ___TEST_EXTRUDER_ENC(x,y) _TEST_EXTRUDER_ENC(x,y)
  #define __TEST_EXTRUDER_ENC(x)    ___TEST_EXTRUDER_ENC(x,E ##x## _ENC_PIN)
  #define TEST_EXTRUDER_ENC(x)      __TEST_EXTRUDER_ENC(x)

  #if HAS_E0_ENC
    #define TEST_EXTRUDER_ENC0      TEST_EXTRUDER_ENC(0)
  #else
    #define TEST_EXTRUDER_ENC0
  #endif
  #if HAS_E1_ENC
    #define TEST_EXTRUDER_ENC1      TEST_EXTRUDER_ENC(1)
  #else
    #define TEST_EXTRUDER_ENC1
  #endif
  #if HAS_E2_ENC
    #define TEST_EXTRUDER_ENC2      TEST_EXTRUDER_ENC(2)
  #else
    #define TEST_EXTRUDER_ENC2
  #endif
  #if HAS_E3_ENC
    #define TEST_EXTRUDER_ENC3      TEST_EXTRUDER_ENC(3)
  #else
    #define TEST_EXTRUDER_ENC3
  #endif
  #if HAS_E4_ENC
    #define TEST_EXTRUDER_ENC4      TEST_EXTRUDER_ENC(4)
  #else
    #define TEST_EXTRUDER_ENC4
  #endif
  #if HAS_E5_ENC
    #define TEST_EXTRUDER_ENC5      TEST_EXTRUDER_ENC(5)
  #else
    #define TEST_EXTRUDER_ENC5
  #endif

#endif // HAS_EXT_ENCODER

#define EXTRUDER_FLAG_RETRACTED 1
#define EXTRUDER_FLAG_WAIT_JAM_STARTCOUNT 2 ///< Waiting for the first signal to start counting


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
void Stepper::wake_up() {
  #if ENABLED(ARDUINO_ARCH_SAM)
    //
  #else
    //  TCNT1 = 0;
  #endif
  ENABLE_STEPPER_INTERRUPT();
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

  #define SET_STEP_DIR(AXIS) \
    if (motor_direction(AXIS ##_AXIS)) { \
      AXIS ##_APPLY_DIR(INVERT_## AXIS ##_DIR, false); \
      count_direction[AXIS ##_AXIS] = -1; \
    } \
    else { \
      AXIS ##_APPLY_DIR(!INVERT_## AXIS ##_DIR, false); \
      count_direction[AXIS ##_AXIS] = 1; \
    }

  #if HAS_X_DIR
    SET_STEP_DIR(X); // A
  #endif
  #if HAS_Y_DIR
    SET_STEP_DIR(Y); // B
  #endif
  #if HAS_Z_DIR
    SET_STEP_DIR(Z); // C
  #endif

  #if HAS_EXTRUDERS && DISABLED(ADVANCE) && DISABLED(LIN_ADVANCE)
    if (motor_direction(E_AXIS)) {
      REV_E_DIR();
      count_direction[E_AXIS] = -1;
    }
    else {
      NORM_E_DIR();
      count_direction[E_AXIS] = 1;
    }
  #endif // !ADVANCE && !LIN_ADVANCE

  #if HAS_EXT_ENCODER

    switch(tools.active_extruder) {
      case 0:
        RESET_EXTRUDER_ENC(0, count_direction[E_AXIS]); break;
      #if EXTRUDERS > 1
        case 1:
          RESET_EXTRUDER_ENC(1, count_direction[E_AXIS]); break;
        #if EXTRUDERS > 2
          case 2:
            RESET_EXTRUDER_ENC(2, count_direction[E_AXIS]); break;
          #if EXTRUDERS > 3
            case 3:
              RESET_EXTRUDER_ENC(3, count_direction[E_AXIS]); break;
            #if EXTRUDERS > 4
              case 4:
                RESET_EXTRUDER_ENC(4, count_direction[E_AXIS]); break;
              #if EXTRUDERS > 5
                case 5:
                  RESET_EXTRUDER_ENC(5, count_direction[E_AXIS]); break;
              #endif // EXTRUDERS > 5
            #endif // EXTRUDERS > 4
          #endif // EXTRUDERS > 3
        #endif // EXTRUDERS > 2
      #endif // EXTRUDERS > 1
    }

  #endif
}

/**
 * Stepper Driver Interrupt
 *
 * Directly pulses the stepper motors at high frequency.
 * Timer 1 runs at a base frequency of 2MHz, with this ISR using OCR1A compare mode.
 *
 * OCR1A   Frequency
 *     1     2 MHz
 *    50    40 KHz
 *   100    20 KHz - capped max rate
 *   200    10 KHz - nominal max rate
 *  2000     1 KHz - sleep rate
 *  4000   500  Hz - init rate
 */
HAL_STEP_TIMER_ISR {
  HAL_timer_isr_prologue (STEPPER_TIMER);
  #if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)
    Stepper::advance_isr_scheduler();
  #else
    Stepper::isr();
  #endif
}

void Stepper::isr() {

  HAL_TIMER_TYPE ocr_val;

  #define ENDSTOP_NOMINAL_OCR_VAL 3000    // check endstops every 1.5ms to guarantee two stepper ISRs within 5ms for BLTouch
  #define OCR_VAL_TOLERANCE 1000          // First max delay is 2.0ms, last min delay is 0.5ms, all others 1.5ms

  #if DISABLED(ADVANCE) || DISABLED(LIN_ADVANCE)
    // Allow UART ISRs
    HAL_DISABLE_ISRs();
  #endif

  #define _SPLIT(L) (ocr_val = (HAL_TIMER_TYPE)L)
  #if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
    #define SPLIT(L) _SPLIT(L)
  #else                 // sample endstops in between step pulses
    static uint32_t step_remaining = 0;
    #define SPLIT(L) do { \
      _SPLIT(L); \
      if (ENDSTOPS_ENABLED && L > ENDSTOP_NOMINAL_OCR_VAL) { \
        const HAL_TIMER_TYPE remainder = (HAL_TIMER_TYPE)L % (ENDSTOP_NOMINAL_OCR_VAL); \
        ocr_val = (remainder < OCR_VAL_TOLERANCE) ? ENDSTOP_NOMINAL_OCR_VAL + remainder : ENDSTOP_NOMINAL_OCR_VAL; \
        step_remaining = (HAL_TIMER_TYPE)L - ocr_val; \
      } \
    } while(0)

    if (step_remaining && ENDSTOPS_ENABLED) {   // just doing a check of the endstops - not yet time for a step
      endstops.update();
      ocr_val = step_remaining;
      if (step_remaining > ENDSTOP_NOMINAL_OCR_VAL) {
        step_remaining -= ENDSTOP_NOMINAL_OCR_VAL;
        ocr_val = ENDSTOP_NOMINAL_OCR_VAL;
      }
      else {
        ocr_val = step_remaining;
        step_remaining = 0;  //  last one before the ISR that does the step
      }

      _NEXT_ISR(ocr_val);

      #if ENABLED(ARDUINO_ARCH_SAM)
        HAL_TIMER_TYPE  stepper_timer_count = HAL_timer_get_count(STEPPER_TIMER),
                        stepper_timer_current_count = HAL_timer_get_current_count(STEPPER_TIMER) + 8 * STEPPER_TIMER_TICKS_PER_US;
        HAL_TIMER_SET_STEPPER_COUNT(stepper_timer_count < stepper_timer_current_count ? stepper_timer_current_count : stepper_timer_count);
      #else
        NOLESS(OCR1A, TCNT1 + 16);
      #endif

      HAL_ENABLE_ISRs(); // re-enable ISRs
      return;
    }
  #endif

  if (cleaning_buffer_counter) {
    --cleaning_buffer_counter;
    current_block = NULL;
    planner.discard_current_block();
    #if ENABLED(SD_FINISHED_RELEASECOMMAND)
      if (!cleaning_buffer_counter && (SD_FINISHED_STEPPERRELEASE)) commands.enqueue_and_echo_commands_P(PSTR(SD_FINISHED_RELEASECOMMAND));
    #endif
    _NEXT_ISR(HAL_STEPPER_TIMER_RATE / 10000); // Run at max speed - 10 KHz
    HAL_ENABLE_ISRs(); // re-enable ISRs
    return;
  }

  #if ENABLED(ARDUINO_ARCH_SAM)
    #if ENABLED(LASER)
      if (laser.firing == LASER_ON && laser.dur != 0 && (laser.last_firing + laser.dur < micros())) {
        if (laser.diagnostics)
          SERIAL_EM("Laser firing duration elapsed, in interrupt handler");
        laser.extinguish();
      }
    #endif
  #else
    #if ENABLED(LASER) && (!ENABLED(LASER_PULSE_METHOD))
      if (laser.dur != 0 && (laser.last_firing + laser.dur < micros())) {
        if (laser.diagnostics)
          SERIAL_EM("Laser firing duration elapsed, in interrupt handler");
        laser.extinguish();
      }
    #endif
  #endif

  // If there is no current block, attempt to pop one from the buffer
  if (!current_block) {
    // Anything in the buffer?
    current_block = planner.get_current_block();
    if (current_block) {
      trapezoid_generator_reset();

      #if STEPPER_DIRECTION_DELAY > 0
        HAL::delayMicroseconds(STEPPER_DIRECTION_DELAY);
      #endif

      // Initialize Bresenham counters to 1/2 the ceiling
      counter_X = counter_Y = counter_Z = counter_E = -(current_block->step_event_count >> 1);

      #if ENABLED(LASER)
        #if ENABLED(ARDUINO_ARCH_SAM)
          counter_L = 1000 * counter_X;
        #else
          counter_L = counter_X;
        #endif
        #if DISABLED(LASER_PULSE_METHOD)
          laser.dur = current_block->laser_duration;
        #endif
      #endif

      #if ENABLED(COLOR_MIXING_EXTRUDER)
        MIXING_STEPPERS_LOOP(i)
          counter_m[i] = -(current_block->mix_event_count[i] >> 1);
      #endif

      step_events_completed = 0;

      #if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
        endstops.e_hit = 2; // Needed for the case an endstop is already triggered before the new move begins.
                            // No 'change' can be detected.
      #endif

      #if ENABLED(Z_LATE_ENABLE)
        if (current_block->steps[Z_AXIS] > 0) {
          enable_Z();
          _NEXT_ISR(HAL_STEPPER_TIMER_RATE / 1000); // Run at slow speed - 1 KHz
          HAL_ENABLE_ISRs(); // re-enable ISRs
          return;
        }
      #endif

      #if ENABLED(LASER) && ENABLED(LASER_RASTER)
         if (current_block->laser_mode == RASTER) counter_raster = 0;
      #endif

      // #if ENABLED(ADVANCE)
      //   e_steps[TOOL_E_INDEX] = 0;
      // #endif
    }
    else {
      _NEXT_ISR(HAL_STEPPER_TIMER_RATE / 1000); // Run at slow speed - 1 KHz
      HAL_ENABLE_ISRs(); // re-enable ISRs
      return;
    }
  }

  // Update endstops state, if enabled
  #if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
    if (endstops.e_hit && ENDSTOPS_ENABLED) {
      endstops.update();
      endstops.e_hit--;
    }
  #else
    if (ENDSTOPS_ENABLED) endstops.update();
  #endif

  // Continuous firing of the laser during a move happens here, PPM and raster happen further down
  #if ENABLED(LASER)
    if (current_block->laser_mode == CONTINUOUS && current_block->laser_status == LASER_ON)
      laser.fire(current_block->laser_intensity);

    #if DISABLED(LASER_PULSE_METHOD)
      if (current_block->laser_status == LASER_OFF) {
        if (laser.diagnostics)
          SERIAL_EM("Laser status set to off, in interrupt handler");
        laser.extinguish();
      }
    #endif
  #endif

  #define _COUNTER(AXIS) counter_## AXIS
  #define _APPLY_STEP(AXIS) AXIS ##_APPLY_STEP
  #define _INVERT_STEP_PIN(AXIS) INVERT_## AXIS ##_STEP_PIN

  // Advance the Bresenham counter; start a pulse if the axis needs a step
  #define PULSE_START(AXIS) \
    _COUNTER(AXIS) += current_block->steps[AXIS ##_AXIS]; \
    if (_COUNTER(AXIS) > 0) _APPLY_STEP(AXIS)(!_INVERT_STEP_PIN(AXIS),0);

  // Stop an active pulse, reset the Bresenham counter, update the position
  #define PULSE_STOP(AXIS) \
    if (_COUNTER(AXIS) > 0) { \
      _COUNTER(AXIS) -= current_block->step_event_count; \
      machine_position[AXIS ##_AXIS] += count_direction[AXIS ##_AXIS]; \
      _APPLY_STEP(AXIS)(_INVERT_STEP_PIN(AXIS),0); \
    }

  #define _COUNT_STEPPERS_0 0
  #if HAS_X_STEP
    #define _COUNT_STEPPERS_1 (_COUNT_STEPPERS_0 + 1)
  #else
    #define _COUNT_STEPPERS_1 _COUNT_STEPPERS_0
  #endif
  #if HAS_Y_STEP
    #define _COUNT_STEPPERS_2 (_COUNT_STEPPERS_1 + 1)
  #else
    #define _COUNT_STEPPERS_2 _COUNT_STEPPERS_1
  #endif
  #if HAS_Z_STEP
    #define _COUNT_STEPPERS_3 (_COUNT_STEPPERS_2 + 1)
  #else
    #define _COUNT_STEPPERS_3 _COUNT_STEPPERS_2
  #endif
  #if DISABLED(ADVANCE) && DISABLED(LIN_ADVANCE)
    #define _COUNT_STEPPERS_4 (_COUNT_STEPPERS_3 + 1)
  #else
    #define _COUNT_STEPPERS_4 _COUNT_STEPPERS_3
  #endif

  #define CYCLES_EATEN_XYZE (_COUNT_STEPPERS_4 * 5)
  #define EXTRA_CYCLES_XYZE (STEP_PULSE_CYCLES - (CYCLES_EATEN_XYZE))

  // Take multiple steps per interrupt (For high speed moves)
  bool all_steps_done = false;
  for (uint8_t i = step_loops; i--;) {
    #if ENABLED(LIN_ADVANCE)

      counter_E += current_block->steps[E_AXIS];
      if (counter_E > 0) {
        counter_E -= current_block->step_event_count;
        #if DISABLED(COLOR_MIXING_EXTRUDER)
          // Don't step E here for mixing extruder
          machine_position[E_AXIS] += count_direction[E_AXIS];
          motor_direction(E_AXIS) ? --e_steps[TOOL_E_INDEX] : ++e_steps[TOOL_E_INDEX];
        #endif
      }

      #if ENABLED(COLOR_MIXING_EXTRUDER)
        // Step mixing steppers proportionally
        const bool dir = motor_direction(E_AXIS);
        MIXING_STEPPERS_LOOP(j) {
          counter_m[j] += current_block->steps[E_AXIS];
          if (counter_m[j] > 0) {
            counter_m[j] -= current_block->mix_event_count[j];
            dir ? --e_steps[j] : ++e_steps[j];
          }
        }
      #endif

    #elif ENABLED(ADVANCE)

      // Always count the unified E axis
      counter_E += current_block->steps[E_AXIS];
      if (counter_E > 0) {
        counter_E -= current_block->step_event_count;
        #if DISABLED(COLOR_MIXING_EXTRUDER)
          // Don't step E for mixing extruder
          motor_direction(E_AXIS) ? --e_steps[TOOL_E_INDEX] : ++e_steps[TOOL_E_INDEX];
        #endif
      }

      #if ENABLED(COLOR_MIXING_EXTRUDER)
        // Step mixing steppers proportionally
        const bool dir = motor_direction(E_AXIS);
        MIXING_STEPPERS_LOOP(j) {
          counter_m[j] += current_block->steps[E_AXIS];
          if (counter_m[j] > 0) {
            counter_m[j] -= current_block->mix_event_count[j];
            dir ? --e_steps[j] : ++e_steps[j];
          }
        }
      #endif

    #endif // ADVANCE or LIN_ADVANCE

    // If a minimum pulse time was specified get the CPU clock
    #if EXTRA_CYCLES_XYZE > 20
      uint32_t pulse_start = HAL_timer_get_current_count(STEPPER_TIMER);
    #endif

    #if HAS_X_STEP
      PULSE_START(X);
    #endif
    #if HAS_Y_STEP
      PULSE_START(Y);
    #endif
    #if HAS_Z_STEP
      PULSE_START(Z);
    #endif

    // For non-advance use linear interpolation for E also
    #if HAS_EXTRUDERS && DISABLED(ADVANCE) && DISABLED(LIN_ADVANCE)
      #if ENABLED(COLOR_MIXING_EXTRUDER)
        // Keep updating the single E axis
        counter_E += current_block->steps[E_AXIS];
        // Tick the counters used for this mix
        MIXING_STEPPERS_LOOP(j) {
          // Step mixing steppers (proportionally)
          counter_m[j] += current_block->steps[E_AXIS];
          // Step when the counter goes over zero
          if (counter_m[j] > 0) En_STEP_WRITE(j, !INVERT_E_STEP_PIN);
        }
      #else // !COLOR_MIXING_EXTRUDER
        PULSE_START(E);
      #endif

      #if HAS_EXT_ENCODER
        if (counter_E > 0) {
          switch(tools.active_extruder) {
            case 0:
              TEST_EXTRUDER_ENC0; break;
            #if EXTRUDERS > 1
              case 1:
                TEST_EXTRUDER_ENC1; break;
              #if EXTRUDERS > 2
                case 2:
                  TEST_EXTRUDER_ENC2; break;
                #if EXTRUDERS > 3
                  case 3:
                    TEST_EXTRUDER_ENC3; break;
                  #if EXTRUDERS > 4
                    case 4:
                      TEST_EXTRUDER_ENC4; break;
                    #if EXTRUDERS > 5
                      case 5:
                        TEST_EXTRUDER_ENC5; break;
                    #endif // EXTRUDERS > 5
                  #endif // EXTRUDERS > 4
                #endif // EXTRUDERS > 3
              #endif // EXTRUDERS > 2
            #endif // EXTRUDERS > 1
          }
        }
      #endif // HAS_EXT_ENCODER

    #endif // !ADVANCE && !LIN_ADVANCE

    // For a minimum pulse time wait before stopping pulses
    #if EXTRA_CYCLES_XYZE > 20
      while (EXTRA_CYCLES_XYZE > (uint32_t)(HAL_timer_get_current_count(STEPPER_TIMER) - pulse_start) * STEPPER_TIMER_PRESCALE) { /* noop */ }
      pulse_start = HAL_timer_get_current_count(STEPPER_TIMER);
    #elif EXTRA_CYCLES_XYZE > 0
      DELAY_NOPS(EXTRA_CYCLES_XYZE);
    #endif

    #if HAS_X_STEP
      PULSE_STOP(X);
    #endif
    #if HAS_Y_STEP
      PULSE_STOP(Y);
    #endif
    #if HAS_Z_STEP
      PULSE_STOP(Z);
    #endif

    #if HAS_EXTRUDERS && DISABLED(ADVANCE) && DISABLED(LIN_ADVANCE)
      #if ENABLED(COLOR_MIXING_EXTRUDER)
        // Always step the single E axis
        if (counter_E > 0) {
          counter_E -= current_block->step_event_count;
          machine_position[E_AXIS] += count_direction[E_AXIS];
        }
        MIXING_STEPPERS_LOOP(j) {
          if (counter_m[j] > 0) {
            counter_m[j] -= current_block->mix_event_count[j];
            En_STEP_WRITE(j, INVERT_E_STEP_PIN);
          }
        }
      #else // !COLOR_MIXING_EXTRUDER
        PULSE_STOP(E);
      #endif
    #endif // !ADVANCE && !LIN_ADVANCE

    #if ENABLED(LASER)
      counter_L += current_block->steps_l;
      if (counter_L > 0) {
        if (current_block->laser_mode == PULSED && current_block->laser_status == LASER_ON) { // Pulsed Firing Mode
          #if ENABLED(LASER_PULSE_METHOD)
            uint32_t ulValue = current_block->laser_raster_intensity_factor * 255;
            laser_pulse(ulValue, current_block->laser_duration);
            laser.time += current_block->laser_duration / 1000; 
          #else
            laser.fire(current_block->laser_intensity);
          #endif
          if (laser.diagnostics) {
            SERIAL_MV("X: ", counter_X);
            SERIAL_MV("Y: ", counter_Y);
            SERIAL_MV("L: ", counter_L);
          }
        }
        #if ENABLED(LASER_RASTER)
          if (current_block->laser_mode == RASTER && current_block->laser_status == LASER_ON) { // Raster Firing Mode
            #if ENABLED(LASER_PULSE_METHOD)
              uint32_t ulValue = current_block->laser_raster_intensity_factor * 
                                 current_block->laser_raster_data[counter_raster];
              laser_pulse(ulValue, current_block->laser_duration);
              counter_raster++;
              laser.time += current_block->laser_duration / 1000; 
            #else
              // For some reason, when comparing raster power to ppm line burns the rasters were around 2% more powerful
              // going from darkened paper to burning through paper.
              laser.fire(current_block->laser_raster_data[counter_raster]); 
            #endif
            if (laser.diagnostics) SERIAL_MV("Pixel: ", (float)current_block->laser_raster_data[counter_raster]);
            counter_raster++;
          }
        #endif // LASER_RASTER
        
        #if ENABLED(ARDUINO_ARCH_SAM)
          counter_L -= 1000 * current_block->step_event_count;
        #else
          counter_L -= current_block->step_event_count;
        #endif
      }
      #if DISABLED(LASER_PULSE_METHOD)
        if (current_block->laser_duration != 0 && (laser.last_firing + current_block->laser_duration < micros())) {
          if (laser.diagnostics)
            SERIAL_EM("Laser firing duration elapsed, in interrupt fast loop");
          laser.extinguish();
        }
      #endif // DISABLED(LASER_PULSE_METHOD)
    #endif // LASER

    if (++step_events_completed >= current_block->step_event_count) {
      all_steps_done = true;
      break;
    }

    // For minimum pulse time wait before stopping pulses
    #if EXTRA_CYCLES_XYZE > 20
      if (i) while (EXTRA_CYCLES_XYZE > (uint32_t)(HAL_timer_get_current_count(STEPPER_TIMER) - pulse_start) * (STEPPER_TIMER_PRESCALE)) { /* nada */ }
    #elif EXTRA_CYCLES_XYZE > 0
      if (i) DELAY_NOPS(EXTRA_CYCLES_XYZE);
    #endif

  } // steps_loop

  #if ENABLED(LIN_ADVANCE)
    if (current_block->use_advance_lead) {
      const int delta_adv_steps = current_estep_rate[TOOL_E_INDEX] - current_adv_steps[TOOL_E_INDEX];
      current_adv_steps[TOOL_E_INDEX] += delta_adv_steps;
      #if ENABLED(COLOR_MIXING_EXTRUDER)
        // Mixing extruders apply advance lead proportionally
        MIXING_STEPPERS_LOOP(j)
          e_steps[j] += delta_adv_steps * current_block->step_event_count / current_block->mix_event_count[j];
      #else
        // For most extruders, advance the single E stepper
        e_steps[TOOL_E_INDEX] += delta_adv_steps;
      #endif
   }
  #endif

  #if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)
    // If we have esteps to execute, fire the next advance_isr "now"
    if (e_steps[TOOL_E_INDEX]) nextAdvanceISR = 0;
  #endif

  // Calculate new timer value
  if (step_events_completed <= (uint32_t)current_block->accelerate_until) {

    HAL_MULTI_ACC(acc_step_rate, acceleration_time, current_block->acceleration_rate);
    acc_step_rate += current_block->initial_rate;

    // upper limit
    NOMORE(acc_step_rate, current_block->nominal_rate);

    // step_rate to timer interval
    const HAL_TIMER_TYPE timer = calc_timer(acc_step_rate);

    SPLIT(timer);  // split step into multiple ISRs if larger than ENDSTOP_NOMINAL_OCR_VAL
    _NEXT_ISR(ocr_val);

    acceleration_time += timer;

    #if ENABLED(LIN_ADVANCE)

      if (current_block->use_advance_lead) {
        #if ENABLED(COLOR_MIXING_EXTRUDER)
          MIXING_STEPPERS_LOOP(j)
            current_estep_rate[j] = ((uint32_t)acc_step_rate * current_block->abs_adv_steps_multiplier8 * current_block->step_event_count / current_block->mix_event_count[j]) >> 17;
        #else
          current_estep_rate[TOOL_E_INDEX] = ((uint32_t)acc_step_rate * current_block->abs_adv_steps_multiplier8) >> 17;
        #endif
      }

    #elif ENABLED(ADVANCE)

      advance += advance_rate * step_loops;
      // NOLESS(advance, current_block->advance);

      const long  advance_whole = advance >> 8,
                  advance_factor = advance_whole - old_advance;

      // Do E steps + advance steps
      #if ENABLED(COLOR_MIXING_EXTRUDER)
        // ...for mixing steppers proportionally
        MIXING_STEPPERS_LOOP(j)
          e_steps[j] += advance_factor * current_block->step_event_count / current_block->mix_event_count[j];
      #else
        // ...for the active extruder
        e_steps[TOOL_E_INDEX] += advance_factor;
      #endif

      old_advance = advance_whole;

    #endif // ADVANCE or LIN_ADVANCE

    #if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)
      eISR_Rate = adv_rate(e_steps[TOOL_E_INDEX], timer, step_loops);
    #endif
  }
  else if (step_events_completed > (uint32_t)current_block->decelerate_after) {
    HAL_TIMER_TYPE step_rate;
    HAL_MULTI_ACC(step_rate, deceleration_time, current_block->acceleration_rate);

    if (step_rate < acc_step_rate) {
      step_rate = acc_step_rate - step_rate; // Decelerate from acceleration end point.
      NOLESS(step_rate, current_block->final_rate);
    }
    else {
      step_rate = current_block->final_rate;
    }

    // step_rate to timer interval
    const HAL_TIMER_TYPE timer = calc_timer(step_rate);

    SPLIT(timer); // split step into multiple ISRs if larger than ENDSTOP_NOMINAL_OCR_VAL
    _NEXT_ISR(ocr_val);

    deceleration_time += timer;

    #if ENABLED(LIN_ADVANCE)

      if (current_block->use_advance_lead) {
        #if ENABLED(MIXING_EXTRUDER_FEATURE)
          MIXING_STEPPERS_LOOP(j)
            current_estep_rate[j] = ((uint32_t)step_rate * current_block->abs_adv_steps_multiplier8 * current_block->step_event_count / current_block->mix_event_count[j]) >> 17;
        #else
          current_estep_rate[TOOL_E_INDEX] = ((uint32_t)step_rate * current_block->abs_adv_steps_multiplier8) >> 17;
        #endif
      }

    #elif ENABLED(ADVANCE)

      advance -= advance_rate * step_loops;
      NOLESS(advance, final_advance);

      // Do E steps + advance steps
      const long  advance_whole = advance >> 8,
                  advance_factor = advance_whole - old_advance;

      #if ENABLED(MIXING_EXTRUDER_FEATURE)
        MIXING_STEPPERS_LOOP(j)
          e_steps[j] += advance_factor * current_block->step_event_count / current_block->mix_event_count[j];
      #else
        e_steps[TOOL_E_INDEX] += advance_factor;
      #endif

      old_advance = advance_whole;

    #endif // ADVANCE or LIN_ADVANCE
    
    #if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)
      eISR_Rate = adv_rate(e_steps[TOOL_E_INDEX], timer, step_loops);
    #endif
  }
  else {

    #if ENABLED(LIN_ADVANCE)

      if (current_block->use_advance_lead)
        current_estep_rate[TOOL_E_INDEX] = final_estep_rate;

      eISR_Rate = adv_rate(e_steps[TOOL_E_INDEX], OCR1A_nominal, step_loops_nominal);

    #endif

    SPLIT(OCR1A_nominal); // split step into multiple ISRs if larger than ENDSTOP_NOMINAL_OCR_VAL
    _NEXT_ISR(ocr_val);

    // ensure we're running at the correct step rate, even if we just came off an acceleration
    step_loops = step_loops_nominal;
  }

  #if DISABLED(ADVANCE) && DISABLED(LIN_ADVANCE)
    #if ENABLED(CPU_32_BIT)
      HAL_TIMER_TYPE stepper_timer_count = HAL_timer_get_count(STEPPER_TIMER);
      NOLESS(stepper_timer_count, (HAL_timer_get_current_count(STEPPER_TIMER) + 8 * STEPPER_TIMER_TICKS_PER_US));
      HAL_TIMER_SET_STEPPER_COUNT(stepper_timer_count);
    #else
      NOLESS(OCR1A, TCNT1 + 16);
    #endif
  #endif

  // If current block is finished, reset pointer
  if (all_steps_done) {
    current_block = NULL;
    planner.discard_current_block();

    #if ENABLED(ARDUINO_ARCH_SAM)
      #if ENABLED(LASER)
        laser.extinguish();
      #endif
    #else
      #if ENABLED(LASER) && ENABLED(LASER_PULSE_METHOD)
        if (current_block->laser_mode == CONTINUOUS && current_block->laser_status == LASER_ON)
          laser.extinguish();
      #endif
    #endif
  }

  #if DISABLED(ADVANCE) || DISABLED(LIN_ADVANCE)
    HAL_ENABLE_ISRs(); // re-enable ISRs
  #endif
}

#if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)

  // Timer interrupt for E. e_steps is set in the main routine;
  void Stepper::advance_isr() {

    nextAdvanceISR = eISR_Rate;

    #define SET_E_STEP_DIR(INDEX) \
      if (e_steps[INDEX]) E## INDEX ##_DIR_WRITE(e_steps[INDEX] < 0 ? INVERT_E## INDEX ##_DIR : !INVERT_E## INDEX ##_DIR)

    #define START_E_PULSE(INDEX) \
      if (e_steps[INDEX]) E## INDEX ##_STEP_WRITE(!INVERT_E_STEP_PIN)

    #define STOP_E_PULSE(INDEX) \
      if (e_steps[INDEX]) { \
        e_steps[INDEX] <= 0 ? ++e_steps[INDEX] : --e_steps[INDEX]; \
        E## INDEX ##_STEP_WRITE(INVERT_E_STEP_PIN); \
      }

    SET_E_STEP_DIR(0);
    #if DRIVER_EXTRUDERS > 1
      SET_E_STEP_DIR(1);
      #if DRIVER_EXTRUDERS > 2
        SET_E_STEP_DIR(2);
        #if DRIVER_EXTRUDERS > 3
          SET_E_STEP_DIR(3);
          #if DRIVER_EXTRUDERS > 4
            SET_E_STEP_DIR(4);
            #if DRIVER_EXTRUDERS > 5
              SET_E_STEP_DIR(5);
            #endif
          #endif
        #endif
      #endif
    #endif

    #define CYCLES_EATEN_E (DRIVER_EXTRUDERS * 5)
    #define EXTRA_CYCLES_E (STEP_PULSE_CYCLES - (CYCLES_EATEN_E))

    // Step all E steppers that have steps
    for (uint8_t i = step_loops; i--;) {

      #if EXTRA_CYCLES_E > 20
        uint32_t pulse_start = HAL_timer_get_current_count(STEPPER_TIMER);
      #endif

      START_E_PULSE(0);
      #if DRIVER_EXTRUDERS > 1
        START_E_PULSE(1);
        #if DRIVER_EXTRUDERS > 2
          START_E_PULSE(2);
          #if DRIVER_EXTRUDERS > 3
            START_E_PULSE(3);
            #if DRIVER_EXTRUDERS > 4
              START_E_PULSE(4);
              #if DRIVER_EXTRUDERS > 5
                START_E_PULSE(5);
              #endif
            #endif
          #endif
        #endif
      #endif

      // For a minimum pulse time wait before stopping pulses
      #if EXTRA_CYCLES_E > 20
        while (EXTRA_CYCLES_E > (uint32_t)(HAL_timer_get_current_count(STEPPER_TIMER) - pulse_start) * STEPPER_TIMER_PRESCALE) { /* noop */ }
        pulse_start = HAL_timer_get_current_count(STEPPER_TIMER);
      #elif EXTRA_CYCLES_E > 0
        DELAY_NOPS(EXTRA_CYCLES_E);
      #endif

      STOP_E_PULSE(0);
      #if DRIVER_EXTRUDERS > 1
        STOP_E_PULSE(1);
        #if DRIVER_EXTRUDERS > 2
          STOP_E_PULSE(2);
          #if DRIVER_EXTRUDERS > 3
            STOP_E_PULSE(3);
            #if DRIVER_EXTRUDERS > 4
              STOP_E_PULSE(4);
              #if DRIVER_EXTRUDERS > 5
                STOP_E_PULSE(5);
              #endif
            #endif
          #endif
        #endif
      #endif

      // For a minimum pulse time wait before stopping low pulses
      #if EXTRA_CYCLES_E > 20
        if (i) while (EXTRA_CYCLES_E > (uint32_t)(HAL_timer_get_current_count(STEPPER_TIMER) - pulse_start) * STEPPER_TIMER_PRESCALE) { /* noop */ }
      #elif EXTRA_CYCLES_E > 0
        if (i) DELAY_NOPS(EXTRA_CYCLES_E);
      #endif

    } // steps_loop
  }

  void Stepper::advance_isr_scheduler() {

    // Allow UART ISRs
    HAL_DISABLE_ISRs();

    // Run main stepping ISR if flagged
    if (!nextMainISR) isr();

    // Run Advance stepping ISR if flagged
    if (!nextAdvanceISR) advance_isr();
  
    // Is the next advance ISR scheduled before the next main ISR?
    if (nextAdvanceISR <= nextMainISR) {
      // Set up the next interrupt
      HAL_TIMER_SET_STEPPER_COUNT(nextAdvanceISR);
      // New interval for the next main ISR
      if (nextMainISR) nextMainISR -= nextAdvanceISR;
      // Will call Stepper::advance_isr on the next interrupt
      nextAdvanceISR = 0;
    }
    else {
      // The next main ISR comes first
      HAL_TIMER_SET_STEPPER_COUNT(nextMainISR);
      // New interval for the next advance ISR, if any
      if (nextAdvanceISR && nextAdvanceISR != ADV_NEVER)
        nextAdvanceISR -= nextMainISR;
      // Will call Stepper::isr on the next interrupt
      nextMainISR = 0;
    }
  
    // Don't run the ISR faster than possible
    #if ENABLED(ARDUINO_ARCH_SAM)
      HAL_TIMER_TYPE  stepper_timer_count = HAL_timer_get_count(STEPPER_TIMER),
                      stepper_timer_current_count = HAL_timer_get_current_count(STEPPER_TIMER) + 8 * STEPPER_TIMER_TICKS_PER_US;
      HAL_TIMER_SET_STEPPER_COUNT(stepper_timer_count < stepper_timer_current_count ? stepper_timer_current_count : stepper_timer_count);
    #else
      NOLESS(OCR1A, TCNT1 + 16);
    #endif

    HAL_ENABLE_ISRs(); // re-enable ISRs
  }

#endif // ADVANCE or LIN_ADVANCE

void Stepper::init() {

  // Init Digipot Motor Current
  #if HAS_DIGIPOTSS || HAS_MOTOR_CURRENT_PWM
    digipot_init();
  #endif

  #if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
    set_driver_current();
  #endif // MB(ALLIGATOR)

  // Init Microstepping Pins
  #if HAS_MICROSTEPS
    microstep_init();
  #endif

  // initialise TMC Steppers
  #if ENABLED(HAVE_TMCDRIVER)
    tmc_init();
  #endif

  // Init TMC2130 Steppers
  #if ENABLED(HAVE_TMC2130)
    tmc2130_init();
  #endif

  // initialise L6470 Steppers
  #if ENABLED(HAVE_L6470DRIVER)
    L6470_init();
  #endif

  // Initialize Dir Pins
  #if HAS_X_DIR
    X_DIR_INIT;
  #endif
  #if HAS_X2_DIR
    X2_DIR_INIT;
  #endif
  #if HAS_Y_DIR
    Y_DIR_INIT;
    #if ENABLED(Y_TWO_STEPPER) && HAS_Y2_DIR
      Y2_DIR_INIT;
    #endif
  #endif
  #if HAS_Z_DIR
    Z_DIR_INIT;
    #if ENABLED(Z_TWO_STEPPER) && HAS_Z2_DIR
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

  //Initialize Enable Pins - steppers default to disabled.

  #if HAS_X_ENABLE
    X_ENABLE_INIT;
    if (!X_ENABLE_ON) X_ENABLE_WRITE(HIGH);
    #if (ENABLED(DUAL_X_CARRIAGE) || ENABLED(X_TWO_STEPPER)) && HAS_X2_ENABLE
      X2_ENABLE_INIT;
      if (!X_ENABLE_ON) X2_ENABLE_WRITE(HIGH);
    #endif
  #endif

  #if HAS_Y_ENABLE
    Y_ENABLE_INIT;
    if (!Y_ENABLE_ON) Y_ENABLE_WRITE(HIGH);

    #if ENABLED(Y_TWO_STEPPER) && HAS_Y2_ENABLE
      Y2_ENABLE_INIT;
      if (!Y_ENABLE_ON) Y2_ENABLE_WRITE(HIGH);
    #endif
  #endif

  #if HAS_Z_ENABLE
    Z_ENABLE_INIT;
    if (!Z_ENABLE_ON) Z_ENABLE_WRITE(HIGH);
    #if ENABLED(Z_TWO_STEPPER) && HAS_Z2_ENABLE
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

  // Init endstops and pullups here
  endstops.init();

  #define _STEP_INIT(AXIS) AXIS ##_STEP_INIT
  #define _WRITE_STEP(AXIS, HIGHLOW) AXIS ##_STEP_WRITE(HIGHLOW)
  #define _DISABLE(AXIS) disable_## AXIS()

  #define AXIS_INIT(AXIS, PIN) \
    _STEP_INIT(AXIS); \
    _WRITE_STEP(AXIS, _INVERT_STEP_PIN(PIN)); \
    _DISABLE(AXIS)

  #define E_AXIS_INIT(NUM) AXIS_INIT(E## NUM, E)

  // Init Step Pins
  #if HAS_X_STEP
    #if ENABLED(X_TWO_STEPPER) || ENABLED(DUAL_X_CARRIAGE)
      X2_STEP_INIT;
      X2_STEP_WRITE(INVERT_X_STEP_PIN);
    #endif
    AXIS_INIT(X, X);
  #endif

  #if HAS_Y_STEP
    #if ENABLED(Y_TWO_STEPPER) && HAS_Y2_STEP
      Y2_STEP_INIT;
      Y2_STEP_WRITE(INVERT_Y_STEP_PIN);
    #endif
    AXIS_INIT(Y, Y);
  #endif

  #if HAS_Z_STEP
    #if ENABLED(Z_TWO_STEPPER) && HAS_Z2_STEP
      Z2_STEP_INIT;
      Z2_STEP_WRITE(INVERT_Z_STEP_PIN);
    #endif
    AXIS_INIT(Z, Z);
  #endif

  #if HAS_E0_STEP
    E_AXIS_INIT(0);
  #endif
  #if HAS_E1_STEP
    E_AXIS_INIT(1);
  #endif
  #if HAS_E2_STEP
    E_AXIS_INIT(2);
  #endif
  #if HAS_E3_STEP
    E_AXIS_INIT(3);
  #endif
  #if HAS_E4_STEP
    E_AXIS_INIT(4);
  #endif
  #if HAS_E5_STEP
    E_AXIS_INIT(5);
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

  // Init Stepper ISR to 122 Hz for quick starting
  HAL_STEPPER_TIMER_START();
  ENABLE_STEPPER_INTERRUPT();

  #if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)
    ZERO(e_steps);
    #if ENABLED(LIN_ADVANCE)
      ZERO(current_adv_steps);
    #endif
  #endif // ADVANCE or LIN_ADVANCE

  endstops.enable(true); // Start with endstops active. After homing they can be disabled
  sei();

  set_directions(); // Init directions to last_direction_bits = 0
}

/**
 * Block until all buffered steps are executed
 */
void Stepper::synchronize() { while (planner.blocks_queued()) printer.idle(); }

/**
 * Set the stepper positions directly in steps
 *
 * The input is based on the typical per-axis XYZ steps.
 * For CORE machines XYZ needs to be translated to ABC.
 *
 * This allows get_axis_position_mm to correctly
 * derive the current XYZ position later on.
 */
void Stepper::set_position(const long &a, const long &b, const long &c, const long &e) {

  synchronize(); // Bad to set stepper counts in the middle of a move

  CRITICAL_SECTION_START;

  #if CORE_IS_XY
    // corexy positioning
    machine_position[A_AXIS] = a + (CORE_FACTOR) * b;
    machine_position[B_AXIS] = CORESIGN(a - (CORE_FACTOR) * b);
    machine_position[Z_AXIS] = c;
  #elif CORE_IS_XZ
    // corexz planning
    machine_position[A_AXIS] = a + (CORE_FACTOR) * c;
    machine_position[Y_AXIS] = b;
    machine_position[C_AXIS] = CORESIGN(a - (CORE_FACTOR) * c);
  #elif CORE_IS_YZ
    // coreyz planning
    machine_position[X_AXIS] = a;
    machine_position[B_AXIS] = b + (CORE_FACTOR) * c;
    machine_position[C_AXIS] = CORESIGN(b - (CORE_FACTOR) * c);
  #else
    // default non-h-bot planning
    machine_position[X_AXIS] = a;
    machine_position[Y_AXIS] = b;
    machine_position[Z_AXIS] = c;
  #endif

  machine_position[E_AXIS] = e;
  CRITICAL_SECTION_END;
}

void Stepper::set_position(const AxisEnum &axis, const long &v) {
  CRITICAL_SECTION_START;
  machine_position[axis] = v;
  CRITICAL_SECTION_END;
}

void Stepper::set_e_position(const long &e) {
  CRITICAL_SECTION_START;
  machine_position[E_AXIS] = e;
  CRITICAL_SECTION_END;
}

/**
 * Get a stepper's position in steps.
 */
long Stepper::position(AxisEnum axis) {
  CRITICAL_SECTION_START;
  const long machine_pos = machine_position[axis];
  CRITICAL_SECTION_END;
  return machine_pos;
}

void Stepper::enable_all_steppers() {

  #if HAS_POWER_SWITCH 
    powerManager.power_on();
  #endif

  enable_X();
  enable_Y();
  enable_Z();
  enable_E0();
  enable_E1();
  enable_E2();
  enable_E3();
  enable_E4();
  enable_E5();
}

void Stepper::disable_e_steppers() {
  // Disable extruders steppers (only on boards that have separate ENABLE_PINS)
  #if E0_ENABLE_PIN != X_ENABLE_PIN && E1_ENABLE_PIN != Y_ENABLE_PIN
    disable_E0();
    disable_E1();
    disable_E2();
    disable_E3();
    disable_E4();
    disable_E5();
  #endif
}

void Stepper::disable_all_steppers() {
  disable_X();
  disable_Y();
  disable_Z();
  disable_e_steppers();
}

void Stepper::finish_and_disable() {
  synchronize();
  disable_all_steppers();
}

void Stepper::quick_stop() {
  cleaning_buffer_counter = 5000;
  DISABLE_STEPPER_INTERRUPT();
  while (planner.blocks_queued()) planner.discard_current_block();
  current_block = NULL;
  ENABLE_STEPPER_INTERRUPT();
  #if ENABLED(ULTRA_LCD)
    planner.clear_block_buffer_runtime();
  #endif
}

void Stepper::endstop_triggered(AxisEnum axis) {

  #if IS_CORE

    endstops_trigsteps[axis] = 0.5f * (
      axis == CORE_AXIS_2 ? CORESIGN(machine_position[CORE_AXIS_1] - machine_position[CORE_AXIS_2])
                          : machine_position[CORE_AXIS_1] + machine_position[CORE_AXIS_2]
    );

  #else // !COREXY && !COREXZ && !COREYZ

    endstops_trigsteps[axis] = machine_position[axis];

  #endif // !COREXY && !COREXZ && !COREYZ

  kill_current_block();
}

void Stepper::report_positions() {
  CRITICAL_SECTION_START;
  const long  xpos = machine_position[X_AXIS],
              ypos = machine_position[Y_AXIS],
              zpos = machine_position[Z_AXIS];
  CRITICAL_SECTION_END;

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

#if ENABLED(NPR2)
  void Stepper::colorstep(long csteps, const bool direction) {
    enable_E1();
    // setup new step
    WRITE(E1_DIR_PIN,(INVERT_E1_DIR)^direction);
    // perform step
    for (long i = 0; i <= csteps; i++){
      WRITE(E1_STEP_PIN, !INVERT_E_STEP_PIN);
      HAL::delayMicroseconds(COLOR_SLOWRATE);
      WRITE(E1_STEP_PIN, INVERT_E_STEP_PIN);
      HAL::delayMicroseconds(COLOR_SLOWRATE);
    }
  }  
#endif //NPR2

#if ENABLED(BABYSTEPPING)

  #if ENABLED(DELTA)
    #define CYCLES_EATEN_BABYSTEP (2 * 15)
  #else
    #define CYCLES_EATEN_BABYSTEP 0
  #endif
  #define EXTRA_CYCLES_BABYSTEP (STEP_PULSE_CYCLES - (CYCLES_EATEN_BABYSTEP))

  #define _ENABLE(AXIS) enable_## AXIS()
  #define _READ_DIR(AXIS) AXIS ##_DIR_READ
  #define _INVERT_DIR(AXIS) INVERT_## AXIS ##_DIR
  #define _APPLY_DIR(AXIS, INVERT) AXIS ##_APPLY_DIR(INVERT, true)

  #if EXTRA_CYCLES_BABYSTEP > 20
    #define _SAVE_START const uint32_t pulse_start = HAL_timer_get_current_count(STEPPER_TIMER)
    #define _PULSE_WAIT while (EXTRA_CYCLES_BABYSTEP > (uint32_t)(HAL_timer_get_current_count(STEPPER_TIMER) - pulse_start) * (STEPPER_TIMER_PRESCALE)) { /* nada */ }
  #else
    #define _SAVE_START NOOP
    #if EXTRA_CYCLES_BABYSTEP > 0
      #define _PULSE_WAIT DELAY_NOPS(EXTRA_CYCLES_BABYSTEP)
    #elif STEP_PULSE_CYCLES > 0
      #define _PULSE_WAIT NOOP
    #elif ENABLED(DELTA)
      #define _PULSE_WAIT HAL::delayMicroseconds(2);
    #else
      #define _PULSE_WAIT HAL::delayMicroseconds(4);
    #endif
  #endif

  #define BABYSTEP_AXIS(AXIS, INVERT) {                     \
      const uint8_t old_dir = _READ_DIR(AXIS);              \
      _ENABLE(AXIS);                                        \
      _SAVE_START;                                          \
      _APPLY_DIR(AXIS, _INVERT_DIR(AXIS)^direction^INVERT); \
      _APPLY_STEP(AXIS)(!_INVERT_STEP_PIN(AXIS), true);     \
      _PULSE_WAIT;                                          \
      _APPLY_STEP(AXIS)(_INVERT_STEP_PIN(AXIS), true);      \
      _APPLY_DIR(AXIS, old_dir);                            \
    }

  // MUST ONLY BE CALLED BY AN ISR,
  // No other ISR should ever interrupt this!
  void Stepper::babystep(const AxisEnum axis, const bool direction) {
    cli();

    switch (axis) {

      #if ENABLED(BABYSTEP_XY)

        case X_AXIS:
          BABYSTEP_AXIS(X, false);
          break;

        case Y_AXIS:
          BABYSTEP_AXIS(Y, false);
          break;

      #endif

      case Z_AXIS: {

        #if NOMECH(DELTA)

          BABYSTEP_AXIS(Z, BABYSTEP_INVERT_Z);

        #else // DELTA

          const bool z_direction = direction ^ BABYSTEP_INVERT_Z;

          enable_X();
          enable_Y();
          enable_Z();

          const uint8_t old_x_dir_pin = X_DIR_READ,
                        old_y_dir_pin = Y_DIR_READ,
                        old_z_dir_pin = Z_DIR_READ;

          X_DIR_WRITE(INVERT_X_DIR ^ z_direction);
          Y_DIR_WRITE(INVERT_Y_DIR ^ z_direction);
          Z_DIR_WRITE(INVERT_Z_DIR ^ z_direction);

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

/**
 * Software-controlled Stepper Motor Current
 */
#if HAS_DIGIPOTSS

  // From Arduino DigitalPotControl example
  void Stepper::digitalPotWrite(int address, int value) {
    WRITE(DIGIPOTSS_PIN, LOW); // take the SS pin low to select the chip
    SPI.transfer(address); //  send in the address and value via SPI:
    SPI.transfer(value);
    WRITE(DIGIPOTSS_PIN, HIGH); // take the SS pin high to de-select the chip:
    //HAL::delayMilliseconds(10);
  }

#endif

#if HAS_DIGIPOTSS || HAS_MOTOR_CURRENT_PWM

  void Stepper::digipot_init() {
    #if HAS_DIGIPOTSS
      const uint8_t digipot_motor_current[] = DIGIPOT_MOTOR_CURRENT;

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

#if MB(ALLIGATOR) || MB(ALLIGATOR_V3)

  void Stepper::set_driver_current() {
    uint8_t digipot_motor = 0;
    for (uint8_t i = 0; i < 3 + DRIVER_EXTRUDERS; i++) {
      digipot_motor = 255 * printer.motor_current[i] / 3.3;
      ExternalDac::setValue(i, digipot_motor);
    }
  }

#endif

#if HAS_MICROSTEPS

  /**
   * Software-controlled Microstepping
   */
  void Stepper::microstep_init() {

    #if HAS_X_MICROSTEPS
      SET_OUTPUT(X_MS1_PIN);
      SET_OUTPUT(X_MS2_PIN);
    #endif
    #if HAS_Y_MICROSTEPS
      SET_OUTPUT(Y_MS1_PIN);
      SET_OUTPUT(Y_MS2_PIN);
    #endif
    #if HAS_Z_MICROSTEPS
      SET_OUTPUT(Z_MS1_PIN);
      SET_OUTPUT(Z_MS2_PIN);
    #endif
    #if HAS_E0_MICROSTEPS
      SET_OUTPUT(E0_MS1_PIN);
      SET_OUTPUT(E0_MS2_PIN);
    #endif
    #if HAS_E1_MICROSTEPS
      SET_OUTPUT(E1_MS1_PIN);
      SET_OUTPUT(E1_MS2_PIN);
    #endif
    #if HAS_E2_MICROSTEPS
      SET_OUTPUT(E2_MS1_PIN);
      SET_OUTPUT(E2_MS2_PIN);
    #endif
    #if HAS_E3_MICROSTEPS
      SET_OUTPUT(E3_MS1_PIN);
      SET_OUTPUT(E3_MS2_PIN);
    #endif
    #if HAS_E4_MICROSTEPS
      SET_OUTPUT(E4_MS1_PIN);
      SET_OUTPUT(E4_MS2_PIN);
    #endif
    #if HAS_E5_MICROSTEPS
      SET_OUTPUT(E5_MS1_PIN);
      SET_OUTPUT(E5_MS2_PIN);
    #endif

    static const uint8_t microstep_modes[] = MICROSTEP_MODES;
    for (uint16_t i = 0; i < COUNT(microstep_modes); i++)
      microstep_mode(i, microstep_modes[i]);
  }

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
    #if !MB(ALLIGATOR) && !MB(ALLIGATOR_V3)
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
    #endif
  }

  void Stepper::microstep_mode(uint8_t driver, uint8_t stepping_mode) {
    switch (stepping_mode) {
      case 1: microstep_ms(driver,  MICROSTEP1); break;
      case 2: microstep_ms(driver,  MICROSTEP2); break;
      case 4: microstep_ms(driver,  MICROSTEP4); break;
      case 8: microstep_ms(driver,  MICROSTEP8); break;
      case 16: microstep_ms(driver, MICROSTEP16); break;
      #if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
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
 * Stepper Reset (RigidBoard, et.al.)
 */
#if HAS_STEPPER_RESET
  void Stepper::disableStepperDrivers() {
    OUT_WRITE(STEPPER_RESET_PIN, LOW);  // drive it down to hold in reset motor driver chips
  }
  void Stepper::enableStepperDrivers() { SET_INPUT(STEPPER_RESET_PIN); }  // set to input, which allows it to be pulled high by pullups
#endif
