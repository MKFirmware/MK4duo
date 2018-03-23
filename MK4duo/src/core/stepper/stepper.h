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
 * stepper.h - stepper motor driver: executes motion plans of planner.c using the stepper motors
 * Part of Grbl
 *
 * Copyright (c) 2009-2011 Simen Svale Skogsrud
 *
 * Grbl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Grbl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Grbl. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _STEPPER_H_
#define _STEPPER_H_

#if ENABLED(__AVR__)
  #include "speed_lookuptable.h"
#endif

#include "stepper_indirection.h"

class Stepper {

  public: /** Constructor */

    Stepper() { };

  public: /** Public Parameters */

    static block_t* current_block;  // A pointer to the block currently being traced

    static millis_t stepper_inactive_time;

    #if ENABLED(ABORT_ON_ENDSTOP_HIT)
      static bool abort_on_endstop_hit;
    #endif

    static int16_t cleaning_buffer_counter;

  private: /** Private Parameters */

    static uint8_t last_direction_bits;        // The next stepping-bits to be output

    #if ENABLED(X_TWO_ENDSTOPS)
      static bool locked_x_motor, locked_x2_motor;
    #endif
    #if ENABLED(Y_TWO_ENDSTOPS)
      static bool locked_y_motor, locked_y2_motor;
    #endif
    #if ENABLED(Z_TWO_ENDSTOPS)
      static bool locked_z_motor, locked_z2_motor;
    #endif

    // Counter variables for the Bresenham line tracer
    static long counter_X, counter_Y, counter_Z, counter_E;
    static volatile uint32_t step_events_completed; // The number of step events executed in the current block

    #if ENABLED(LIN_ADVANCE)

      static uint32_t     LA_decelerate_after;  // Copy from current executed block. Needed because current_block is set to NULL "too early".

      static hal_timer_t  nextMainISR,
                          nextAdvanceISR,
                          eISR_Rate;

      static uint16_t     current_adv_steps,
                          final_adv_steps,
                          max_adv_steps;        // Copy from current executed block. Needed because current_block is set to NULL "too early".

      #define _NEXT_ISR(T) nextMainISR = T

      static int8_t       e_steps,
                          LA_active_extruder;   // Copy from current executed block. Needed because current_block is set to NULL "too early".

      static bool         use_advance_lead;

    #else // !LIN_ADVANCE

      #define _NEXT_ISR(T) HAL_timer_set_count(STEPPER_TIMER, T);

    #endif // !LIN_ADVANCE

    static long acceleration_time, deceleration_time;

    static hal_timer_t  acc_step_rate, // needed for deceleration start point
                        OCR1A_nominal;

    static uint8_t  step_loops, step_loops_nominal;

    static volatile long endstops_trigsteps[XYZ];

    #if PIN_EXISTS(MOTOR_CURRENT_PWM_XY)
      #ifndef PWM_MOTOR_CURRENT
        #define PWM_MOTOR_CURRENT DEFAULT_PWM_MOTOR_CURRENT
      #endif
      static constexpr int motor_current_setting[3] = PWM_MOTOR_CURRENT;
    #endif

    //
    // Positions of stepper motors, in step units
    //
    static volatile long count_position[NUM_AXIS];

    //
    // Current direction of stepper motors (+1 or -1)
    //
    static volatile signed char count_direction[NUM_AXIS];

    #if ENABLED(COLOR_MIXING_EXTRUDER)
      static long counter_m[MIXING_STEPPERS];
      #define MIXING_STEPPERS_LOOP(VAR) \
        for (uint8_t VAR = 0; VAR < MIXING_STEPPERS; VAR++) \
          if (current_block->mix_event_count[VAR])
    #endif

    #if ENABLED(LASER)
      static long counter_L;
      #if ENABLED(LASER_RASTER)
        static int counter_raster;
      #endif // LASER_RASTER
    #endif // LASER

  public: /** Public Function */

    //
    // Initialize stepper hardware
    //
    static void init();

    //
    // Interrupt Service Routines
    //
    static void isr();

    #if ENABLED(LIN_ADVANCE)
      static void advance_isr();
      static void advance_isr_scheduler();
    #endif

    //
    // Block until all buffered steps are executed
    //
    static void synchronize();

    //
    // Set current position in steps
    //
    static void set_position(const long &a, const long &b, const long &c, const long &e);
    static void set_position(const AxisEnum &a, const long &v);
    static void set_e_position(const long &e);

    //
    // Set direction bits for all steppers
    //
    static void set_directions();

    //
    // Get the position of a stepper, in steps
    //
    static long position(const AxisEnum axis);

    //
    // Report the positions of the steppers, in steps
    //
    static void report_positions();

    //
    // Get the position (mm) of an axis based on stepper position(s)
    //
    static float get_axis_position_mm(const AxisEnum axis);

    //
    // SCARA AB axes are in degrees, not mm
    //
    #if IS_SCARA
      FORCE_INLINE static float get_axis_position_degrees(const AxisEnum axis) { return get_axis_position_mm(axis); }
    #endif

    //
    // The stepper subsystem goes to sleep when it runs out of things to execute. Call this
    // to notify the subsystem that it is time to go to work.
    //
    static void wake_up();

    //
    // Wait for moves to finish and disable all steppers
    //
    static void finish_and_disable();

    //
    // Quickly stop all steppers and clear the blocks queue
    //
    static void quick_stop();
    static void quickstop_stepper();

    //
    // The direction of a single motor
    //
    FORCE_INLINE static bool motor_direction(const AxisEnum axis) { return TEST(last_direction_bits, axis); }

    static void enable_all_steppers();
    static void disable_e_steppers();
    static void disable_all_steppers();

    #if HAS_DIGIPOTSS || HAS_MOTOR_CURRENT_PWM
      static void digitalPotWrite(int address, int value);
      static void digipot_current(uint8_t driver, int current);
    #endif

    #if HAS_MICROSTEPS
      static void microstep_ms(uint8_t driver, int8_t ms1, int8_t ms2);
      static void microstep_mode(uint8_t driver, uint8_t stepping);
      static void microstep_readings();
    #endif

    #if ENABLED(X_TWO_ENDSTOPS)
      FORCE_INLINE static void set_x_lock(const bool state) { locked_x_motor = state; }
      FORCE_INLINE static void set_x2_lock(const bool state) { locked_x2_motor = state; }
    #endif
    #if ENABLED(Y_TWO_ENDSTOPS)
      FORCE_INLINE static void set_y_lock(const bool state) { locked_y_motor = state; }
      FORCE_INLINE static void set_y2_lock(const bool state) { locked_y2_motor = state; }
    #endif
    #if ENABLED(Z_TWO_ENDSTOPS)
      FORCE_INLINE static void set_z_lock(const bool state) { locked_z_motor = state; }
      FORCE_INLINE static void set_z2_lock(const bool state) { locked_z2_motor = state; }
    #endif

    #if ENABLED(BABYSTEPPING)
      static void babystep(const AxisEnum axis, const bool direction); // perform a short step with a single stepper motor, outside of any convention
    #endif

    static inline void kill_current_block() {
      step_events_completed = current_block->step_event_count;
    }

    //
    // Handle a triggered endstop
    //
    static void endstop_triggered(const AxisEnum axis);

    //
    // Triggered position of an axis in mm (not core-savvy)
    //
    FORCE_INLINE static float triggered_position_mm(AxisEnum axis) {
      return endstops_trigsteps[axis] * mechanics.steps_to_mm[axis];
    }

  private: /** Private Function */

    FORCE_INLINE static hal_timer_t calc_timer_interval(hal_timer_t step_rate) {
      hal_timer_t timer;

      NOMORE(step_rate, MAX_STEP_FREQUENCY);

      #if ENABLED(DISABLE_DOUBLE_QUAD_STEPPING)
        step_loops = 1;
      #else
        if (step_rate > (2 * DOUBLE_STEP_FREQUENCY)) { // If steprate > (2 * DOUBLE_STEP_FREQUENCY) Hz >> step 4 times
          step_rate >>= 2;
          step_loops = 4;
        }
        else if (step_rate > DOUBLE_STEP_FREQUENCY) { // If steprate > DOUBLE_STEP_FREQUENCY >> step 2 times
          step_rate >>= 1;
          step_loops = 2;
        }
        else
          step_loops = 1;
      #endif

      #if ENABLED(ARDUINO_ARCH_SAM)
        // In case of high-performance processor, it is able to calculate in real-time
        constexpr uint32_t MIN_TIME_PER_STEP = (HAL_TIMER_RATE) / (MAX_STEP_FREQUENCY);
        timer = (uint32_t)HAL_TIMER_RATE / step_rate;
        NOLESS(timer, MIN_TIME_PER_STEP);
      #else
        NOLESS(step_rate, F_CPU / 500000);
        step_rate -= F_CPU / 500000; // Correct for minimal speed
        if (step_rate >= (8 * 256)) { // higher step rate
          uint16_t table_address = (uint16_t)&speed_lookuptable_fast[(unsigned char)(step_rate >> 8)][0];
          unsigned char tmp_step_rate = (step_rate & 0x00FF);
          uint16_t gain = (uint16_t)pgm_read_word_near(table_address + 2);
          MultiU16X8toH16(timer, tmp_step_rate, gain);
          timer = (uint16_t)pgm_read_word_near(table_address) - timer;
        }
        else { // lower step rates
          uint16_t table_address = (uint16_t)&speed_lookuptable_slow[0][0];
          table_address += ((step_rate) >> 1) & 0xFFFC;
          timer = (uint16_t)pgm_read_word_near(table_address);
          timer -= (((uint16_t)pgm_read_word_near(table_address + 2) * (unsigned char)(step_rate & 0x0007)) >> 3);
        }

        if (timer < 100) { // (20kHz this should never happen)
          timer = 100;
          SERIAL_EMV("Steprate too high: ", step_rate);
        }
      #endif

      return timer;
    }

    // Initializes the trapezoid generator from the current block. Called whenever a new
    // block begins.
    FORCE_INLINE static void trapezoid_generator_reset() {

      static int8_t last_extruder = -1;

      #if ENABLED(LIN_ADVANCE)
        if (current_block->active_extruder != last_extruder) {
          current_adv_steps = 0; // If the now active extruder wasn't in use during the last move, its pressure is most likely gone.
          LA_active_extruder = current_block->active_extruder;
        }

        if (current_block->use_advance_lead) {
          LA_decelerate_after = current_block->decelerate_after;
          final_adv_steps = current_block->final_adv_steps;
          max_adv_steps = current_block->max_adv_steps;
          use_advance_lead = true;
        }
        else
          use_advance_lead = false;
      #endif

      if (current_block->direction_bits != last_direction_bits || current_block->active_extruder != last_extruder) {
        last_direction_bits = current_block->direction_bits;
        last_extruder = current_block->active_extruder;
        set_directions();
      }

      deceleration_time = 0;
      // step_rate to timer interval
      OCR1A_nominal = calc_timer_interval(current_block->nominal_rate);
      // make a note of the number of step loops required at nominal speed
      step_loops_nominal = step_loops;
      acc_step_rate = current_block->initial_rate;
      acceleration_time = calc_timer_interval(acc_step_rate);
      _NEXT_ISR(acceleration_time);

    }

    #if HAS_DIGIPOTSS || HAS_MOTOR_CURRENT_PWM
      static void digipot_init();
    #endif

    #if HAS_MICROSTEPS
      static void microstep_init();
    #endif

    #if HAS_STEPPER_RESET
      /**
       * Stepper Reset (RigidBoard, et.al.)
       */
      static void disableStepperDrivers();
      static void enableStepperDrivers();
    #endif

};

extern Stepper stepper;

#endif /* _STEPPER_H_ */
