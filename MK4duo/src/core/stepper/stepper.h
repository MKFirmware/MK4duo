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
#pragma once

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

#include "l64xx/l64xx.h"
#include "tmc/tmc.h"
#include "driver/driver.h"

// Struct Stepper data
struct stepper_data_t {
  uint32_t  maximum_rate    : 32,
            direction_delay : 32;
  uint8_t   minimum_pulse   :  8,
            drivers_e       :  3;
  bool      quad_stepping   :  1;
};
  
class Stepper {

  public: /** Constructor */

    Stepper() { };

  public: /** Public Parameters */

    static stepper_data_t data;

    #if HAS_MULTI_ENDSTOP || ENABLED(Z_STEPPER_AUTO_ALIGN)
      static bool   separate_multi_axis;
    #endif

  private: /** Private Parameters */

    static block_t* current_block;          // A pointer to the block currently being traced

    static uint8_t  last_direction_bits,    // The next stepping-bits to be output
                    axis_did_move;          // Last Movement in the given direction is not null, as computed when the last movement was fetched from planner

    static bool     abort_current_block;    // Signals to the stepper that current block should be aborted

    // Last-moved extruder, as set when the last movement was fetched from planner
    #if MAX_EXTRUDER < 2
      static constexpr uint8_t last_moved_extruder = 0;
    #elif DISABLED(COLOR_MIXING_EXTRUDER)
      static uint8_t last_moved_extruder;
    #endif

    #if ENABLED(X_TWO_ENDSTOPS)
      static bool locked_X_motor, locked_X2_motor;
    #endif
    #if ENABLED(Y_TWO_ENDSTOPS)
      static bool locked_Y_motor, locked_Y2_motor;
    #endif
    #if ENABLED(Z_THREE_ENDSTOPS) || (ENABLED(Z_STEPPER_AUTO_ALIGN) && ENABLED(Z_THREE_STEPPER_DRIVERS))
      static bool locked_Z_motor, locked_Z2_motor, locked_Z3_motor;
    #elif ENABLED(Z_TWO_ENDSTOPS) || ENABLED(Z_STEPPER_AUTO_ALIGN)
      static bool locked_Z_motor, locked_Z2_motor;
    #endif

    static uint32_t acceleration_time, deceleration_time; // time measured in Stepper Timer ticks
    static uint8_t  steps_per_isr,                        // Count of steps to perform per Stepper ISR ca
                    oversampling_factor;                  // Oversampling factor (log2(multiplier)) to increase temporal resolution of axis

    // Delta error variables for the Bresenham line tracer
    static xyze_long_t  delta_error;
    static xyze_ulong_t advance_dividend;
    static xyze_bool_t  step_needed;
    static uint32_t     advance_divisor,
                        step_events_completed,  // The number of step events executed in the current block
                        accelerate_until,       // The point from where we need to stop data.acceleration
                        decelerate_after,       // The point from where we need to start decelerating
                        step_event_count;       // The total event count for the current block

    static uint8_t      active_extruder,        // Active extruder
                        active_extruder_driver; // Active extruder driver

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

    #if ENABLED(LIN_ADVANCE)
      static constexpr uint32_t LA_ADV_NEVER = 0xFFFFFFFF;
      static uint32_t nextAdvanceISR, LA_isr_rate;
      static uint16_t LA_current_adv_steps, LA_final_adv_steps, LA_max_adv_steps;
      static int8_t   LA_steps;
      static bool     LA_use_advance_lead;
    #endif

    static int32_t ticks_nominal;
    #if DISABLED(BEZIER_JERK_CONTROL)
      static uint32_t acc_step_rate; // needed for deceleration start point
    #endif

    static xyz_long_t endstops_trigsteps;

    /**
     * Positions of stepper motors, in step units
     */
    static xyze_long_t count_position;

    /**
     * Current direction of stepper motors (+1 or -1)
     */
    static xyze_int8_t count_direction;

    #if PIN_EXISTS(MOTOR_CURRENT_PWM_XY)
      #ifndef PWM_MOTOR_CURRENT
        #define PWM_MOTOR_CURRENT DEFAULT_PWM_MOTOR_CURRENT
      #endif
      static constexpr int motor_current_setting[3] = PWM_MOTOR_CURRENT;
    #endif

    #if ENABLED(LASER)
      static int32_t delta_error_laser;
      #if ENABLED(LASER_RASTER)
        static int counter_raster;
      #endif
    #endif

  public: /** Public Function */

    /**
     * Create all Driver
     */
    static void create_driver();

    /**
     * Create xyz Driver
     */
    static void create_xyz_driver();

    /**
     * Create extruder Driver
     */
    static void create_ext_driver();

    /**
     * Change number Driver
     */
    static void change_number_driver(const uint8_t drv);

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
    static void report_positions(const xyz_long_t &pos);
    static void report_positions();

    /**
     * Called by eeprom.load / eeprom.reset
     */
    static void reset_drivers();

    /**
     * Set direction bits for all steppers
     */
    static void set_directions(const bool init=false);

    /**
     * The stepper subsystem goes to sleep when it runs out of things to execute. Call this
     * to notify the subsystem that it is time to go to work.
     */
    static inline void wake_up()  { ENABLE_STEPPER_INTERRUPT(); }

    static inline bool is_awake() { return STEPPER_ISR_ENABLED(); }

    static inline bool suspend() {
      const bool awake = is_awake();
      if (awake) DISABLE_STEPPER_INTERRUPT();
      return awake;
    }

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
    static void enable_E(const uint8_t e);
    static void disable_E(const uint8_t e);

    /**
     * Read motor is enable for fan or power
     */
    static bool driver_is_enable();

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
        #if ENABLED(COLOR_MIXING_EXTRUDER) || EXTRUDERS < 2
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

    #if HAS_MULTI_ENDSTOP || ENABLED(Z_STEPPER_AUTO_ALIGN)
      FORCE_INLINE static void set_separate_multi_axis(const bool state) { separate_multi_axis = state; }
    #endif
    #if ENABLED(X_TWO_ENDSTOPS)
      FORCE_INLINE static void set_x_lock(const bool state) { locked_X_motor = state; }
      FORCE_INLINE static void set_x2_lock(const bool state) { locked_X2_motor = state; }
    #endif
    #if ENABLED(Y_TWO_ENDSTOPS)
      FORCE_INLINE static void set_y_lock(const bool state) { locked_Y_motor = state; }
      FORCE_INLINE static void set_y2_lock(const bool state) { locked_Y2_motor = state; }
    #endif
    #if ENABLED(Z_THREE_ENDSTOPS) || (ENABLED(Z_STEPPER_AUTO_ALIGN) && ENABLED(Z_THREE_STEPPER_DRIVERS))
      FORCE_INLINE static void set_z_lock(const bool state) { locked_Z_motor = state; }
      FORCE_INLINE static void set_z2_lock(const bool state) { locked_Z2_motor = state; }
      FORCE_INLINE static void set_z3_lock(const bool state) { locked_Z3_motor = state; }
    #elif ENABLED(Z_TWO_ENDSTOPS) || ENABLED(Z_STEPPER_AUTO_ALIGN)
      FORCE_INLINE static void set_z_lock(const bool state) { locked_Z_motor = state; }
      FORCE_INLINE static void set_z2_lock(const bool state) { locked_Z2_motor = state; }
    #endif

    // Set the current position in steps
    static void set_position(const int32_t &a, const int32_t &b, const int32_t &c, const int32_t &e);
    static void set_position(const AxisEnum a, const int32_t &v);
    static inline void set_position(const xyze_long_t &pos) { set_position(pos.a, pos.b, pos.c, pos.e); }

    #if DISABLED(DISABLE_M503)
      void print_M352();
      void print_M569();
    #endif

    #if ENABLED(BABYSTEPPING)
      static void do_babystep(const AxisEnum axis, const bool direction); // perform a short step with a single stepper motor, outside of any convention
    #endif

    #if ENABLED(LASER)
      static bool laser_status();
      FORCE_INLINE static float laser_intensity() { return current_block->laser_intensity; }
    #endif

  private: /** Private Function */

    /**
     * Driver Factory parameters
     */
    static void driver_factory_parameters(Driver* act, const uint8_t index, const bool axis=true);

    /**
     * Pulse phase Step
     */
    static void pulse_phase_step();

    /**
     * Block phase Step
     */
    static uint32_t block_phase_step();

    /**
     * Direction delay
     */
    FORCE_INLINE static void direction_delay() { if (data.direction_delay >= 50) HAL::delayNanoseconds(data.direction_delay); }

    /**
     * Pulse tick prepare
     */
    FORCE_INLINE static void pulse_tick_prepare();

    /**
     * Pulse tick Start
     */
    FORCE_INLINE static void pulse_tick_start();

    /**
     * Pulse tick Stop
     */
    FORCE_INLINE static void pulse_tick_stop();

    /**
     * Start step X Y Z
     */
    FORCE_INLINE static void start_X_step();
    FORCE_INLINE static void start_Y_step();
    FORCE_INLINE static void start_Z_step();

    /**
     * Stop step X Y Z
     */
    FORCE_INLINE static void stop_X_step();
    FORCE_INLINE static void stop_Y_step();
    FORCE_INLINE static void stop_Z_step();

    /**
     * Set X Y Z direction
     */
    FORCE_INLINE static void set_X_dir(const bool dir);
    FORCE_INLINE static void set_Y_dir(const bool dir);
    FORCE_INLINE static void set_Z_dir(const bool dir);
    FORCE_INLINE static void set_nor_E_dir(const uint8_t e=0);
    FORCE_INLINE static void set_rev_E_dir(const uint8_t e=0);

    /**
     * Extruder Step for the single E axis
     */
    FORCE_INLINE static void e_step_write(const uint8_t e, const bool state) {
      #if ENABLED(DUAL_X_CARRIAGE)
        if (mechanics.extruder_duplication_enabled) {
          driver.e[0]->step_write(state);
          driver.e[1]->step_write(state);
        }
        else
      #endif
      driver.e[e]->step_write(state);
    }

    /**
     * Set current position in steps
     */
    static void _set_position(const int32_t &a, const int32_t &b, const int32_t &c, const int32_t &e);
    FORCE_INLINE static void _set_position(const abce_long_t &spos) { _set_position(spos.a, spos.b, spos.c, spos.e); }

    #if DISABLED(COLOR_MIXING_EXTRUDER)
      // Get active driver
      static uint8_t get_active_extruder_driver();
    #endif

    #if ENABLED(LIN_ADVANCE)
      // The Linear advance stepper Step
      static uint32_t lin_advance_step();
      FORCE_INLINE static void initiateLA() { nextAdvanceISR = 0; }
    #endif

    #if ENABLED(BEZIER_JERK_CONTROL)
      static void _calc_bezier_curve_coeffs(const int32_t v0, const int32_t v1, const uint32_t av);
      static int32_t _eval_bezier_curve(const uint32_t curr_step);
    #endif

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
      FORCE_INLINE static void disableStepperDrivers() {
        OUT_WRITE(STEPPER_RESET_PIN, LOW);  // drive it down to hold in reset motor driver chips
      }
      FORCE_INLINE static void enableStepperDrivers() {
        SET_INPUT(STEPPER_RESET_PIN);       // set to input, which allows it to be pulled high by pullups
      }
    #endif

    FORCE_INLINE static hal_timer_t calc_timer_interval(uint32_t step_rate, uint8_t* loops) {

      uint8_t multistep = 1;

      // Scale the frequency, as requested by the caller
      step_rate <<= oversampling_factor;

      if (data.quad_stepping) {
        // Select the proper multistepping
        uint8_t idx = 0;
        while (idx < 7 && step_rate > HAL_frequency_limit[idx]) {
          step_rate >>= 1;
          multistep <<= 1;
          ++idx;
        };
      }
      else 
        NOMORE(step_rate, HAL_frequency_limit[0]);

      *loops = multistep;

      #if ENABLED(CPU_32_BIT)
        // In case of high-performance processor, it is able to calculate in real-time
        return uint32_t(STEPPER_TIMER_RATE) / step_rate;
      #else
        hal_timer_t timer;
        constexpr uint32_t min_step_rate = F_CPU / 500000U;
        NOLESS(step_rate, min_step_rate);
        step_rate -= min_step_rate;   // Correct for minimal speed
        if (step_rate >= (8 * 256)) { // higher step rate
          const uint8_t   tmp_step_rate = (step_rate & 0x00FF);
          const uint16_t  table_address = (uint16_t)&speed_lookuptable_fast[(uint8_t)(step_rate >> 8)][0],
                          gain = (uint16_t)pgm_read_word(table_address + 2);
          timer = MultiU16X8toH16(tmp_step_rate, gain);
          timer = (uint16_t)pgm_read_word(table_address) - timer;
        }
        else { // lower step rates
          uint16_t table_address = (uint16_t)&speed_lookuptable_slow[0][0];
          table_address += ((step_rate) >> 1) & 0xFFFC;
          timer = (uint16_t)pgm_read_word(table_address)
                - (((uint16_t)pgm_read_word(table_address + 2) * (uint8_t)(step_rate & 0x0007)) >> 3);
        }

        return timer;
      #endif

    }

};

extern Stepper stepper;
