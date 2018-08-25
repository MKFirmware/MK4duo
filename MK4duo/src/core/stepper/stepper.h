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

#include "stepper_indirection.h"

class Stepper {

  public: /** Constructor */

    Stepper() { };

  public: /** Public Parameters */

    static uint16_t direction_flag; // Driver Stepper direction flag

    #if ENABLED(X_TWO_ENDSTOPS) || ENABLED(Y_TWO_ENDSTOPS) || ENABLED(Z_TWO_ENDSTOPS)
      static bool homing_dual_axis;
    #endif

    static uint8_t  minimum_pulse;
    static uint32_t maximum_rate,
                    direction_delay;

  private: /** Private Parameters */

    static block_t* current_block;          // A pointer to the block currently being traced

    static uint8_t  last_direction_bits,    // The next stepping-bits to be output
                    axis_did_move;          // Last Movement in the given direction is not null, as computed when the last movement was fetched from planner

    static bool     abort_current_block;    // Signals to the stepper that current block should be aborted

    #if DISABLED(COLOR_MIXING_EXTRUDER)
      static uint8_t last_moved_extruder;   // Last-moved extruder, as set when the last movement was fetched from planner
    #endif

    #if ENABLED(X_TWO_ENDSTOPS)
      static bool locked_X_motor, locked_X2_motor;
    #endif
    #if ENABLED(Y_TWO_ENDSTOPS)
      static bool locked_Y_motor, locked_Y2_motor;
    #endif
    #if ENABLED(Z_TWO_ENDSTOPS)
      static bool locked_Z_motor, locked_Z2_motor;
    #endif

    static uint32_t acceleration_time, deceleration_time; // time measured in Stepper Timer ticks
    static uint8_t  steps_per_isr;                        // Count of steps to perform per Stepper ISR call

    // Delta error variables for the Bresenham line tracer
    static int32_t  delta_error[XYZE];
    static uint32_t advance_dividend[XYZE],
                    advance_divisor,
                    step_events_completed,  // The number of step events executed in the current block
                    accelerate_until,       // The point from where we need to stop acceleration
                    decelerate_after,       // The point from where we need to start decelerating
                    step_event_count;       // The total event count for the current block

    #if ENABLED(COLOR_MIXING_EXTRUDER)
      static int32_t  delta_error_m[MIXING_STEPPERS];
      static uint32_t advance_dividend_m[MIXING_STEPPERS],
                      advance_divisor_m;
      #define MIXING_STEPPERS_LOOP(VAR) \
        for (uint8_t VAR = 0; VAR < MIXING_STEPPERS; VAR++)
    #else
      static uint8_t  active_extruder,        // Active extruder
                      active_extruder_driver; // Active extruder driver
    #endif

    #if ENABLED(BEZIER_JERK_CONTROL)
      static int32_t  bezier_A,     // A coefficient in Bézier speed curve
                      bezier_B,     // B coefficient in Bézier speed curve
                      bezier_C;     // C coefficient in Bézier speed curve
      static uint32_t bezier_F,     // F coefficient in Bézier speed curve
                      bezier_AV;    // AV coefficient in Bézier speed curve
      #if ENABLED(__AVR__)
        static bool A_negative;     // If A coefficient was negative
      #endif
      static bool bezier_2nd_half;  // If Bézier curve has been initialized or not
    #endif

    static uint32_t nextMainISR;    // time remaining for the next Step ISR
    #if ENABLED(LIN_ADVANCE)
      static uint32_t nextAdvanceISR, LA_isr_rate;
      static uint16_t LA_current_adv_steps, LA_final_adv_steps, LA_max_adv_steps;
      static int8_t   LA_steps;
      static bool     LA_use_advance_lead;
    #endif // !LIN_ADVANCE

    static int32_t ticks_nominal;
    #if DISABLED(BEZIER_JERK_CONTROL)
      static uint32_t acc_step_rate; // needed for deceleration start point
    #endif

    static volatile int32_t endstops_trigsteps[XYZ];

    /**
     * Positions of stepper motors, in step units
     */
    static volatile int32_t count_position[NUM_AXIS];

    /**
     * Current direction of stepper motors (+1 or -1)
     */
    static int8_t count_direction[NUM_AXIS];

    #if PIN_EXISTS(MOTOR_CURRENT_PWM_XY)
      #ifndef PWM_MOTOR_CURRENT
        #define PWM_MOTOR_CURRENT DEFAULT_PWM_MOTOR_CURRENT
      #endif
      static constexpr int motor_current_setting[3] = PWM_MOTOR_CURRENT;
    #endif

    #if ENABLED(LASER)
      static int32_t counter_L;
      #if ENABLED(LASER_RASTER)
        static int counter_raster;
      #endif
    #endif

  public: /** Public Function */

    /**
     * Initialize stepper hardware
     */
    static void init();

    /**
     * Initialize Factory parameters
     */
    static void factory_parameters();

    /**
     * This is called by the interrupt service routine to execute steps.
     */
    static void Step();

    /**
     * Check if the given block is busy or not - Must not be called from ISR contexts
     */
    static bool is_block_busy(const block_t* const block);

    /**
     * Get the position of a stepper, in steps
     */
    static int32_t position(const AxisEnum axis);

    /**
     * Report the positions of the steppers, in steps
     */
    static void report_positions();

    /**
     * The stepper subsystem goes to sleep when it runs out of things to execute. Call this
     * to notify the subsystem that it is time to go to work.
     */
    FORCE_INLINE static void wake_up() { ENABLE_STEPPER_INTERRUPT(); }

    /**
     * Set direction bits for all steppers
     */
    static void set_directions();

    /**
     * Enabled or Disable one or all stepper driver
     */
    static void enable_X();
    static void disable_X();
    static void enable_Y();
    static void disable_Y();
    static void enable_Z();
    static void disable_Z();
    static void enable_E();
    static void disable_E();
    static void enable_all();
    static void disable_all();

    /**
     * Enabled or Disable Extruder Stepper Driver
     */
    static void enable_E0();
    static void disable_E0();
    #if ENABLED(COLOR_MIXING_EXTRUDER)
      FORCE_INLINE static void enable_E1() { /* nada */ }
      FORCE_INLINE static void enable_E2() { /* nada */ }
      FORCE_INLINE static void enable_E3() { /* nada */ }
      FORCE_INLINE static void enable_E4() { /* nada */ }
      FORCE_INLINE static void enable_E5() { /* nada */ }
      FORCE_INLINE static void disable_E1() { /* nada */ }
      FORCE_INLINE static void disable_E2() { /* nada */ }
      FORCE_INLINE static void disable_E3() { /* nada */ }
      FORCE_INLINE static void disable_E4() { /* nada */ }
      FORCE_INLINE static void disable_E5() { /* nada */ }
    #else
      FORCE_INLINE static void enable_E1() {
        #if (DRIVER_EXTRUDERS > 1) && HAS_E1_ENABLE
          E1_ENABLE_WRITE(E_ENABLE_ON);
        #endif
      }
      FORCE_INLINE static void disable_E1() {
        #if (DRIVER_EXTRUDERS > 1) && HAS_E1_ENABLE
          E1_ENABLE_WRITE(!E_ENABLE_ON);
        #endif
      }
      FORCE_INLINE static void enable_E2() {
        #if (DRIVER_EXTRUDERS > 2) && HAS_E2_ENABLE
          E2_ENABLE_WRITE(E_ENABLE_ON);
        #endif
      }
      FORCE_INLINE static void disable_E2() {
        #if (DRIVER_EXTRUDERS > 2) && HAS_E2_ENABLE
          E2_ENABLE_WRITE(!E_ENABLE_ON);
        #endif
      }
      FORCE_INLINE static void enable_E3() {
        #if (DRIVER_EXTRUDERS > 3) && HAS_E3_ENABLE
          E3_ENABLE_WRITE(E_ENABLE_ON);
        #endif
      }
      FORCE_INLINE static void disable_E3() {
        #if (DRIVER_EXTRUDERS > 3) && HAS_E3_ENABLE
          E3_ENABLE_WRITE(!E_ENABLE_ON);
        #endif
      }
      FORCE_INLINE static void enable_E4() {
        #if (DRIVER_EXTRUDERS > 4) && HAS_E4_ENABLE
          E4_ENABLE_WRITE(E_ENABLE_ON);
        #endif
      }
      FORCE_INLINE static void disable_E4() {
        #if (DRIVER_EXTRUDERS > 4) && HAS_E4_ENABLE
          E4_ENABLE_WRITE(!E_ENABLE_ON);
        #endif
      }
      FORCE_INLINE static void enable_E5() {
        #if (DRIVER_EXTRUDERS > 5) && HAS_E5_ENABLE
          E5_ENABLE_WRITE(E_ENABLE_ON);
        #endif
      }
      FORCE_INLINE static void disable_E5() {
        #if (DRIVER_EXTRUDERS > 5) && HAS_E5_ENABLE
          E5_ENABLE_WRITE(!E_ENABLE_ON);
        #endif
      }
    #endif

    /**
     * Quickly stop all steppers and clear the blocks queue
     */
    FORCE_INLINE static void quick_stop() { abort_current_block = true; }

    /**
     * The direction of a single motor
     */
    FORCE_INLINE static bool motor_direction(const AxisEnum axis) { return TEST(last_direction_bits, axis); }

    /**
     * The last movement direction was not null on the specified axis. Note that motor direction is not necessarily the same.
     */
    FORCE_INLINE static bool axis_is_moving(const AxisEnum axis) { return TEST(axis_did_move, axis); }

    /**
     * The extruder associated to the last movement
     */
    FORCE_INLINE static uint8_t movement_extruder() {
      return
        #if ENABLED(COLOR_MIXING_EXTRUDER)
          0
        #else
          last_moved_extruder
        #endif
      ;
    }

    /**
     * Handle a triggered endstop
     */
    static void endstop_triggered(const AxisEnum axis);

    /**
     * Triggered position of an axis in steps
     */
    static int32_t triggered_position(const AxisEnum axis);

    #if HAS_DIGIPOTSS
      static void digitalPotWrite(int address, int value);
    #endif

    #if HAS_DIGIPOTSS || HAS_MOTOR_CURRENT_PWM
      static void digipot_current(uint8_t driver, int current);
    #endif

    #if HAS_MICROSTEPS
      static void microstep_ms(uint8_t driver, int8_t ms1, int8_t ms2);
      static void microstep_mode(uint8_t driver, uint8_t stepping);
      static void microstep_readings();
    #endif

    #if ENABLED(X_TWO_ENDSTOPS) || ENABLED(Y_TWO_ENDSTOPS) || ENABLED(Z_TWO_ENDSTOPS)
      FORCE_INLINE static void set_homing_dual_axis(const bool state) { homing_dual_axis = state; }
    #endif
    #if ENABLED(X_TWO_ENDSTOPS)
      FORCE_INLINE static void set_x_lock(const bool state) { locked_X_motor = state; }
      FORCE_INLINE static void set_x2_lock(const bool state) { locked_X2_motor = state; }
    #endif
    #if ENABLED(Y_TWO_ENDSTOPS)
      FORCE_INLINE static void set_y_lock(const bool state) { locked_Y_motor = state; }
      FORCE_INLINE static void set_y2_lock(const bool state) { locked_Y2_motor = state; }
    #endif
    #if ENABLED(Z_TWO_ENDSTOPS)
      FORCE_INLINE static void set_z_lock(const bool state) { locked_Z_motor = state; }
      FORCE_INLINE static void set_z2_lock(const bool state) { locked_Z2_motor = state; }
    #endif

    // Set the current position in steps
    static void set_position(const int32_t &a, const int32_t &b, const int32_t &c, const int32_t &e);
    static void set_position(const AxisEnum a, const int32_t &v);

    /**
     * Flag Stepper direction function
     */
    FORCE_INLINE static void setStepDir(const AxisEnum axis, const bool onoff) {
      SET_BIT(direction_flag, axis, onoff);
    }
    FORCE_INLINE static bool isStepDir(const AxisEnum axis) { return TEST(direction_flag, axis); }

  private: /** Private Function */

    /**
     * Pulse phase Step
     */
    static void pulse_phase_step();

    /**
     * Block phase Step
     */
    static uint32_t block_phase_step();

    /**
     * Pulse tick Start
     */
    static void pulse_tick_start();

    /**
     * Pulse tick Stop
     */
    static void pulse_tick_stop();

    /**
     * Start step X Y Z
     */
    static void start_X_step();
    static void start_Y_step();
    static void start_Z_step();

    /**
     * Stop step X Y Z
     */
    static void stop_X_step();
    static void stop_Y_step();
    static void stop_Z_step();

    /**
     * Set X Y Z direction
     */
    static void set_X_dir(const bool dir);
    static void set_Y_dir(const bool dir);
    static void set_Z_dir(const bool dir);

    /**
     * Set current position in steps
     */
    static void _set_position(const int32_t &a, const int32_t &b, const int32_t &c, const int32_t &e);
    
    #if DISABLED(COLOR_MIXING_EXTRUDER)
      // Get active driver
      static uint8_t get_active_extruder_driver();
    #endif

    #if ENABLED(LIN_ADVANCE)
      // The Linear advance stepper Step
      static uint32_t lin_advance_step();
    #endif

    #if ENABLED(BEZIER_JERK_CONTROL)
      static void _calc_bezier_curve_coeffs(const int32_t v0, const int32_t v1, const uint32_t av);
      static int32_t _eval_bezier_curve(const uint32_t curr_step);
    #endif

    #if ENABLED(BABYSTEPPING)
      static void babystep(const AxisEnum axis, const bool direction); // perform a short step with a single stepper motor, outside of any convention
    #endif

    #if HAS_DIGIPOTSS || HAS_MOTOR_CURRENT_PWM
      static void digipot_init();
    #endif

    #if HAS_MICROSTEPS
      static void microstep_init();
    #endif

    #if HAS_EXT_ENCODER
      static void test_extruder_encoder();
    #endif

    #if HAS_STEPPER_RESET
      /**
       * Stepper Reset (RigidBoard, et.al.)
       */
      FORCE_INLINE static void disableStepperDrivers() {
        OUT_WRITE(STEPPER_RESET_PIN, LOW);  // drive it down to hold in reset motor driver chips
      }
      FORCE_INLINE static void enableStepperDrivers() {
        SET_INPUT(STEPPER_RESET_PIN);       // set to input, which allows it to be pulled high by pullups
      }
    #endif

};

extern Stepper stepper;

#endif /* _STEPPER_H_ */
