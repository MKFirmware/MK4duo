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
#pragma once

/**
 * planner.h
 *
 * Buffer movement commands and manage the acceleration profile plan
 *
 * Derived from Grbl
 * Copyright (c) 2009-2011 Simen Svale Skogsrud
 */

/**
 * struct block_t
 *
 * A single entry in the planner buffer.
 * Tracks linear movement over multiple axes.
 *
 * The "nominal" values are as-specified by gcode, and
 * may never actually be reached due to acceleration limits.
 */
typedef struct {

  volatile uint8_t flag;                    // Block flags (See BlockFlagEnum enum above) - Modified by ISR and main thread!

  // Fields used by the motion planner to manage acceleration
  float nominal_speed_sqr,                  // The nominal speed for this block in (mm/sec)^2
        entry_speed_sqr,                    // Entry speed at previous-current junction in (mm/sec)^2
        max_entry_speed_sqr,                // Maximum allowable junction entry speed in (mm/sec)^2
        millimeters,                        // The total travel of this block in mm
        acceleration;                       // acceleration mm/sec^2

  // Data used by all move blocks
  union {
    // Fields used by the Bresenham algorithm for tracing the line
    uint32_t steps[NUM_AXIS];               // Step count along each axis
    // Data used by all sync blocks
    int32_t position[NUM_AXIS];             // New position to force when this sync block is executed
  };

  uint32_t step_event_count;                // The number of step events required to complete this block

  #if EXTRUDERS > 1
    uint8_t active_extruder;                // The extruder to move (if E move)
  #else
    static constexpr uint8_t active_extruder = 0;
  #endif

  #if ENABLED(COLOR_MIXING_EXTRUDER)
    mixer_color_t b_color[MIXING_STEPPERS]; // Normalized color for the mixing steppers
  #endif

  // Settings for the trapezoid generator
  uint32_t  accelerate_until,               // The index of the step event on which to stop acceleration
            decelerate_after;               // The index of the step event on which to start decelerating

  #if ENABLED(BEZIER_JERK_CONTROL)
    uint32_t  cruise_rate,                  // The actual cruise rate to use, between end of the acceleration phase and start of deceleration phase
              acceleration_time,            // Acceleration time and deceleration time in STEP timer counts
              deceleration_time,
              acceleration_time_inverse,    // Inverse of acceleration and deceleration periods, expressed as integer. Scale depends on CPU being used
              deceleration_time_inverse;
  #else
    uint32_t  acceleration_rate;            // The acceleration rate used for acceleration calculation
  #endif

  uint8_t direction_bits;                   // The direction bit set for this block

  // Advance extrusion
  #if ENABLED(LIN_ADVANCE)
    bool      use_advance_lead;
    uint16_t  advance_speed,                // STEP timer value for extruder speed offset ISR
              max_adv_steps,                // max. advance steps to get cruising speed pressure (not always nominal_speed!)
              final_adv_steps;              // advance steps due to exit speed
    float     e_D_ratio;
  #endif

  uint32_t  nominal_rate,                   // The nominal step rate for this block in step_events/sec
            initial_rate,                   // The jerk-adjusted step rate at start of block
            final_rate,                     // The minimal rate at exit
            acceleration_steps_per_s2;      // acceleration steps/sec^2

  #if ENABLED(BARICUDA)
    uint8_t valve_pressure, e_to_p_pressure;
  #endif

  uint32_t segment_time_us;

  #if ENABLED(LASER)
    uint8_t   laser_mode;       // CONTINUOUS, PULSED, RASTER
    bool      laser_status;     // LASER_OFF, LASER_ON
    float     laser_ppm,        // pulses per millimeter, for pulsed and raster firing modes
              laser_intensity;  // Laser firing instensity in clock cycles for the PWM timer
    uint32_t  laser_duration,   // Laser firing duration in microseconds, for pulsed and raster firing modes
              steps_l;          // Step count between firings of the laser, for pulsed firing mode

    #if ENABLED(LASER_RASTER)
      unsigned char laser_raster_data[LASER_MAX_RASTER_LINE];
    #endif
  #endif

} block_t;

#define BLOCK_MOD(n) ((n)&(BLOCK_BUFFER_SIZE-1))

class Planner {

  public: /** Constructor */

    Planner() {}

  public: /** Public Parameters */

    /**
     * The move buffer, calculated in stepper steps
     *
     * block_buffer is a ring buffer...
     *
     *             head,tail : indexes for write,read
     *            head==tail : the buffer is empty
     *            head!=tail : blocks are in the buffer
     *   head==(tail-1)%size : the buffer is full
     *
     *  Writer of head is Planner::buffer_segment().
     *  Reader of tail is Stepper::isr(). Always consider tail busy / read-only
     */
    static block_t          block_buffer[BLOCK_BUFFER_SIZE];
    static volatile uint8_t block_buffer_head,        // Index of the next block to be pushed
                            block_buffer_nonbusy,     // Index of the first non busy block
                            block_buffer_planned,     // Index of the optimally planned block
                            block_buffer_tail;        // Index of the busy block, if any
    static uint8_t          delay_before_delivering;  // This counter delays delivery of blocks when queue becomes empty to allow the opportunity of merging blocks

    static bool cleaning_buffer_flag;                 // A flag to disable queuing of blocks

    #if ENABLED(LIN_ADVANCE)
      static float  extruder_advance_K;
    #endif

    #if HAS_POSITION_FLOAT
      static float  position_float[XYZE];
    #endif

    #if IS_KINEMATIC
      static float position_cart[XYZE];
    #endif

    #if ENABLED(ABORT_ON_ENDSTOP_HIT)
      static bool abort_on_endstop_hit;
    #endif

    #if ENABLED(HYSTERESIS_FEATURE)
      static float  hysteresis_mm[XYZ],
                    hysteresis_correction;
    #endif

  private: /** Private Parameters */

    /**
     * The current position of the tool in absolute steps
     * Recalculated if any data.axis_steps_per_mm are changed by gcode
     */
    static int32_t position[NUM_AXIS];

    /**
     * Speed of previous path line segment
     */
    static float previous_speed[NUM_AXIS];

    /**
     * Nominal speed of previous path line segment (mm/s)^2
     */
    static float previous_nominal_speed_sqr;

    /**
     * Limit where 64bit math is necessary for acceleration calculation
     */
    static uint32_t cutoff_long;

    #if ENABLED(DISABLE_INACTIVE_EXTRUDER)
      /**
       * Counters to manage disabling inactive extruders
       */
      static uint8_t g_uc_extruder_last_move[EXTRUDERS];
    #endif // DISABLE_INACTIVE_EXTRUDER

    #if ENABLED(XY_FREQUENCY_LIMIT)
      // Used for the frequency limit
      #define MAX_FREQ_TIME_US (uint32_t)(1000000.0 / XY_FREQUENCY_LIMIT)
      // Old direction bits. Used for speed calculations
      static uint8_t old_direction_bits;
      // Segment times (in µs). Used for speed calculations
      static uint32_t axis_segment_time_us[2][3];
    #endif

    #if HAS_SPI_LCD
      volatile static uint32_t block_buffer_runtime_us; // Theoretical block buffer runtime in µs
    #endif

  public: /** Public Function */

    static void reset_acceleration_rates();
    static void refresh_positioning();

    /**
     * Manage Axis, paste pressure, etc.
     */
    static void check_axes_activity();

    #if ENABLED(FWRETRACT)

      static void apply_retract(float &rz, float &e);
      FORCE_INLINE static void apply_retract(float (&raw)[XYZE]) { apply_retract(raw[Z_AXIS], raw[E_AXIS]); }
      static void unapply_retract(float &rz, float &e);
      FORCE_INLINE static void unapply_retract(float (&raw)[XYZE]) { unapply_retract(raw[Z_AXIS], raw[E_AXIS]); }
  
    #endif

    #if HAS_POSITION_MODIFIERS

      static void apply_modifiers(float (&pos)[XYZE]
        #if HAS_LEVELING
          , bool leveling =
          #if PLANNER_LEVELING
            true
          #else
            false
          #endif
        #endif
      );

      static void unapply_modifiers(float (&pos)[XYZE]
        #if HAS_LEVELING
          , bool leveling =
          #if PLANNER_LEVELING
            true
          #else
            false
          #endif
        #endif
      );

    #endif // HAS_POSITION_MODIFIERS

    /**
     * Number of moves currently in the planner including the busy block, if any
     */
    FORCE_INLINE static uint8_t movesplanned() { return BLOCK_MOD(block_buffer_head - block_buffer_tail); }

    /**
     * Number of nonbusy moves currently in the planner
     */
    FORCE_INLINE static uint8_t nonbusy_movesplanned() { return BLOCK_MOD(block_buffer_head - block_buffer_nonbusy); }

    /**
     * Remove all blocks from the buffer
     */
    FORCE_INLINE static void clear_block_buffer() { block_buffer_nonbusy = block_buffer_planned = block_buffer_head = block_buffer_tail = 0; }

    /**
     * Check if movement queue is full
     */
    FORCE_INLINE static bool is_full() { return block_buffer_tail == next_block_index(block_buffer_head); }

    /**
     * Get count of movement slots free
     */
    FORCE_INLINE static uint8_t moves_free() { return BLOCK_BUFFER_SIZE - 1 - movesplanned(); }

    /**
     * Planner::get_next_free_block
     *
     * - Get the next head indices (passed by reference)
     * - Wait for the number of spaces to open up in the planner
     * - Return the first head block
     */
    FORCE_INLINE static block_t* get_next_free_block(uint8_t &next_buffer_head, const uint8_t count=1) {
      // Wait until there are enough slots free
      while (moves_free() < count) { printer.idle(); }

      // Return the first available block
      next_buffer_head = next_block_index(block_buffer_head);
      return &block_buffer[block_buffer_head];
    }

    /**
     * Planner::buffer_steps
     *
     * Add a new linear movement to the buffer (in terms of steps).
     *
     *  target        - target position in steps units
     *  target_float  - target position in direct (mm, degrees) units. optional
     *  fr_mm_s       - (target) speed of the move
     *  extruder      - target extruder
     *  millimeters   - the length of the movement, if known
     *
     * Returns true if movement was properly queued, false otherwise
     */
    static bool buffer_steps(const int32_t (&target)[XYZE]
      #if HAS_POSITION_FLOAT
        , const float (&target_float)[XYZE]
      #endif
      #if IS_KINEMATIC && ENABLED(JUNCTION_DEVIATION)
        , const float (&delta_mm_cart)[XYZE]
      #endif
      , float fr_mm_s, const uint8_t extruder, const float &millimeters=0.0
    );

    /**
     * Planner::_fill_block
     *
     * Fills a new linear movement in the block (in terms of steps).
     *
     *  target      - target position in steps units
     *  fr_mm_s     - (target) speed of the move
     *  extruder    - target extruder
     *  millimeters - the length of the movement, if known
     *
     * Return true is movement is acceptable, false otherwise
     */
    static bool fill_block(block_t * const block, bool split_move,
        const int32_t (&target)[XYZE]
      #if HAS_POSITION_FLOAT
        , const float (&target_float)[XYZE]
      #endif
      #if IS_KINEMATIC && ENABLED(JUNCTION_DEVIATION)
        , const float (&delta_mm_cart)[XYZE]
      #endif
      , float fr_mm_s, const uint8_t extruder, const float &millimeters=0.0
    );

    /**
     * Planner::buffer_sync_block
     * Add a block to the buffer that just updates the position
     */
    static void buffer_sync_block();

    /**
     * Planner::buffer_segment
     *
     * Add a new linear movement to the buffer in axis units.
     *
     * Leveling and kinematics should be applied ahead of calling this.
     *
     *  a,b,c,e     - target positions in mm and/or degrees
     *  fr_mm_s     - (target) speed of the move
     *  extruder    - target extruder
     *  millimeters - the length of the movement, if known
     */
    static bool buffer_segment(const float &a, const float &b, const float &c, const float &e
      #if IS_KINEMATIC && ENABLED(JUNCTION_DEVIATION)
        , const float (&delta_mm_cart)[XYZE]
      #endif
      , const float &fr_mm_s, const uint8_t extruder, const float &millimeters=0.0
    );

    FORCE_INLINE static bool buffer_segment(const float (&abce)[ABCE]
      #if IS_KINEMATIC && ENABLED(JUNCTION_DEVIATION)
        , const float (&delta_mm_cart)[XYZE]
      #endif
      , const float &fr_mm_s, const uint8_t extruder, const float &millimeters=0.0
    ) {
      return buffer_segment(abce[A_AXIS], abce[B_AXIS], abce[C_AXIS], abce[E_AXIS]
        #if IS_KINEMATIC && ENABLED(JUNCTION_DEVIATION)
          , delta_mm_cart
        #endif
        , fr_mm_s, extruder, millimeters);
    }

    /**
     * Planner::buffer_segment
     *
     * Add a new linear movement to the buffer in axis units.
     *
     * Leveling and kinematics should be applied ahead of calling this.
     *
     *  a,b,c,e     - target positions in mm and/or degrees
     *  fr_mm_s     - (target) speed of the move
     *  extruder    - target extruder
     *  millimeters - the length of the movement, if known
     */
    static bool buffer_line(const float &rx, const float &ry, const float &rz, const float &e, const float &fr_mm_s, const uint8_t extruder, const float millimeters=0.0);

    FORCE_INLINE static bool buffer_line(const float (&cart)[XYZE], const float &fr_mm_s, const uint8_t extruder, const float millimeters=0.0
      #if ENABLED(SCARA_FEEDRATE_SCALING)
        , const float &inv_duration=0.0
      #endif
    ) {
      return buffer_line(cart[X_AXIS], cart[Y_AXIS], cart[Z_AXIS], cart[E_AXIS], fr_mm_s, extruder, millimeters
        #if ENABLED(SCARA_FEEDRATE_SCALING)
          , inv_duration
        #endif
      );
    }

    /**
     * Set the planner.position and individual stepper positions.
     * Used by G92, G28, G29, and other procedures.
     *
     * The supplied position is in the cartesian coordinate space and is
     * translated in to machine space as needed. Modifiers such as leveling
     * and skew are also applied.
     *
     * Multiplies by data.axis_steps_per_mm[] and does necessary conversion
     * for COREXY / COREXZ / COREYZ to set the corresponding stepper positions.
     *
     * Clears previous speed values.
     */
    static void set_position_mm(const float &rx, const float &ry, const float &rz, const float &e);
    FORCE_INLINE static void set_position_mm(const float (&cart)[XYZE]) { set_position_mm(cart[X_AXIS], cart[Y_AXIS], cart[Z_AXIS], cart[E_AXIS]); }
    static void set_e_position_mm(const float &e);

    /**
     * Set the planner.position and individual stepper positions.
     *
     * The supplied position is in machine space, and no additional
     * conversions are applied.
     */
    static void set_machine_position_mm(const float &a, const float &b, const float &c, const float &e);
    FORCE_INLINE static void set_machine_position_mm(const float (&abce)[ABCE]) { set_machine_position_mm(abce[A_AXIS], abce[B_AXIS], abce[C_AXIS], abce[E_AXIS]); }

    /**
     * Get an axis position according to stepper position(s)
     * For CORE machines apply translation from ABC to XYZ.
     */
    static float get_axis_position_mm(const AxisEnum axis);

    /**
     * SCARA AB axes are in degrees, not mm
     */
    #if IS_SCARA
      FORCE_INLINE static float get_axis_position_degrees(const AxisEnum axis) { return get_axis_position_mm(axis); }
    #endif

    /**
     * Block until all buffered steps are executed / cleaned
     */
    static void synchronize();

    /**
     * Wait for moves to finish and disable all steppers
     */
    static void finish_and_disable();

    /**
     * Called to force a quick stop of the machine (for example, when an emergency
     * stop is required, or when endstops are hit)
     */
    static void quick_stop();

    /**
     * Called when an endstop is triggered. Causes the machine to stop inmediately
     */
    static void endstop_triggered(const AxisEnum axis);

    /**
     * Triggered position of an axis in mm (not core-savvy)
     */
    static float triggered_position_mm(const AxisEnum axis);

    /**
     * Does the buffer have any blocks queued?
     */
    FORCE_INLINE static bool has_blocks_queued() { return (block_buffer_head != block_buffer_tail); }

    /**
     * "Discard" the block and "release" the memory.
     * Called when the current block is no longer needed.
     * NB: There MUST be a current block to call this function!!
     */
    FORCE_INLINE static void discard_current_block() {
      if (has_blocks_queued())
        block_buffer_tail = next_block_index(block_buffer_tail);
    }

    /**
     * The current block. NULL if the buffer is empty.
     * This also marks the block as busy.
     * WARNING: Called from Stepper ISR context!
     */
    static block_t* get_current_block() {

      // Get the number of moves in the planner queue so far
      const uint8_t nr_moves = movesplanned();

      // If there are any moves queued ...
      if (nr_moves) {

        // If there is still delay of delivery of blocks running, decrement it
        if (delay_before_delivering) {
          --delay_before_delivering;
          // If the number of movements queued is less than 3, and there is still time
          //  to wait, do not deliver anything
          if (nr_moves < 3 && delay_before_delivering) return NULL;
          delay_before_delivering = 0;
        }

        // If we are here, there is no excuse to deliver the block
        block_t * const block = &block_buffer[block_buffer_tail];

        // No trapezoid calculated? Don't execute yet.
        if (TEST(block->flag, BLOCK_BIT_RECALCULATE)) return NULL;

        #if HAS_SPI_LCD
          block_buffer_runtime_us -= block->segment_time_us; // We can't be sure how long an active block will take, so don't count it.
        #endif

        // As this block is busy, advance the nonbusy block pointer
        block_buffer_nonbusy = next_block_index(block_buffer_tail);

        // Push block_buffer_planned pointer, if encountered.
        if (block_buffer_tail == block_buffer_planned)
          block_buffer_planned = block_buffer_nonbusy;

        // Return the block
        return block;
      }

      // The queue became empty
      #if HAS_SPI_LCD
        clear_block_buffer_runtime(); // paranoia. Buffer is empty now - so reset accumulated time to zero.
      #endif

      return NULL;
    }

    #if ENABLED(ULTRA_LCD)

      static uint16_t block_buffer_runtime() {
        #if ENABLED(__AVR__)
          // Protect the access to the variable. Only required for AVR, as
          //  any 32bit CPU offers atomic access to 32bit variables
          bool was_enabled = STEPPER_ISR_ENABLED();
          if (was_enabled) DISABLE_STEPPER_INTERRUPT();
        #endif

        millis_t bbru = block_buffer_runtime_us;

        #if ENABLED(__AVR__)
          // Reenable Stepper ISR
          if (was_enabled) ENABLE_STEPPER_INTERRUPT();
        #endif

        // To translate µs to ms a division by 1000 would be required.
        // We introduce 2.4% error here by dividing by 1024.
        // Doesn't matter because block_buffer_runtime_us is already too small an estimation.
        bbru >>= 10;
        // limit to about a minute.
        NOMORE(bbru, 0xFFFFul);
        return bbru;
      }

      static void clear_block_buffer_runtime() {
        #if ENABLED(__AVR__)
          // Protect the access to the variable. Only required for AVR, as
          //  any 32bit CPU offers atomic access to 32bit variables
          bool was_enabled = STEPPER_ISR_ENABLED();
          if (was_enabled) DISABLE_STEPPER_INTERRUPT();
        #endif

        block_buffer_runtime_us = 0;

        #if ENABLED(__AVR__)
          // Reenable Stepper ISR
          if (was_enabled) ENABLE_STEPPER_INTERRUPT();
        #endif
      }

    #endif

    #if HAS_TEMP_HOTEND && ENABLED(AUTOTEMP)
      static float autotemp_min, autotemp_max, autotemp_factor;
      static bool autotemp_enabled;
      static void getHighESpeed();
      static void autotemp_M104_M109();
    #endif

  private: /** Private Function */

    /**
     * Get the index of the next / previous block in the ring buffer
     */
    static constexpr uint8_t next_block_index(const uint8_t block_index) { return BLOCK_MOD(block_index + 1); }
    static constexpr uint8_t prev_block_index(const uint8_t block_index) { return BLOCK_MOD(block_index - 1); }

    /**
     * Calculate the distance (not time) it takes to accelerate
     * from initial_rate to target_rate using the given acceleration:
     */
    static float estimate_acceleration_distance(const float &initial_rate, const float &target_rate, const float &accel) {
      if (accel == 0) return 0; // accel was 0, set acceleration distance to 0
      return (sq(target_rate) - sq(initial_rate)) / (accel * 2.0);
    }

    /**
     * Return the point at which you must start braking (at the rate of -'accel') if
     * you start at 'initial_rate', accelerate (until reaching the point), and want to end at
     * 'final_rate' after traveling 'distance'.
     *
     * This is used to compute the intersection point between acceleration and deceleration
     * in cases where the "trapezoid" has no plateau (i.e., never reaches maximum speed)
     */
    static float intersection_distance(const float &initial_rate, const float &final_rate, const float &accel, const float &distance) {
      if (accel == 0) return 0; // accel was 0, set intersection distance to 0
      return (accel * 2 * distance - sq(initial_rate) + sq(final_rate)) / (accel * 4.0);
    }

    /**
     * Calculate the maximum allowable speed at this point, in order
     * to reach 'target_velocity_sqr' using 'acceleration' within a given
     * 'distance'.
     */
    static float max_allowable_speed_sqr(const float &accel, const float &target_velocity_sqr, const float &distance) {
      return target_velocity_sqr - 2 * accel * distance;
    }

    #if ENABLED(BEZIER_JERK_CONTROL)
      /**
       * Calculate the speed reached given initial speed, acceleration and distance
       */
      static float final_speed(const float &initial_velocity, const float &accel, const float &distance) {
        return SQRT(sq(initial_velocity) + 2 * accel * distance);
      }
    #endif

    static void calculate_trapezoid_for_block(block_t* const block, const float &entry_factor, const float &exit_factor);

    static void reverse_pass_kernel(block_t* const current, const block_t* const next);
    static void forward_pass_kernel(const block_t* const previous, block_t* const current, const uint8_t block_index);

    static void reverse_pass();
    static void forward_pass();

    static void recalculate_trapezoids();

    static void recalculate();

    #if ENABLED(JUNCTION_DEVIATION)

      FORCE_INLINE static void normalize_junction_vector(float (&vector)[XYZE]) {
        float magnitude_sq = 0.0;
        LOOP_XYZE(idx) if (vector[idx]) magnitude_sq += sq(vector[idx]);
        const float inv_magnitude = 1.0 / SQRT(magnitude_sq);
        LOOP_XYZE(idx) vector[idx] *= inv_magnitude;
      }

      FORCE_INLINE static float limit_value_by_axis_maximum(const float &max_value, float (&unit_vec)[XYZE]) {
        float limit_value = max_value;
        LOOP_XYZE(idx) {
          if (unit_vec[idx]) // Avoid divide by zero
            NOMORE(limit_value, ABS(mechanics.data.max_acceleration_mm_per_s2[idx] / unit_vec[idx]));
        }
        return limit_value;
      }

    #endif // JUNCTION_DEVIATION

    #if ENABLED(HYSTERESIS_FEATURE)
      static void insert_hysteresis_correction(block_t * const block);
    #endif

};

#define PLANNER_XY_FEEDRATE() (MIN(mechanics.data.max_feedrate_mm_s[X_AXIS], mechanics.data.max_feedrate_mm_s[Y_AXIS]))

extern Planner planner;
