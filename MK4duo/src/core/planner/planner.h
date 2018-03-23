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
 * planner.h
 *
 * Buffer movement commands and manage the acceleration profile plan
 *
 * Derived from Grbl
 * Copyright (c) 2009-2011 Simen Svale Skogsrud
 */

#ifndef PLANNER_H
#define PLANNER_H

enum BlockFlagBit {
  // Recalculate trapezoids on entry junction. For optimization.
  BLOCK_BIT_RECALCULATE,

  // Nominal speed always reached.
  // i.e., The segment is long enough, so the nominal speed is reachable if accelerating
  // from a safe speed (in consideration of jerking from zero speed).
  BLOCK_BIT_NOMINAL_LENGTH,

  // Start from a halt at the start of this block, respecting the maximum allowed jerk.
  BLOCK_BIT_START_FROM_FULL_HALT,

  // The block is busy
  BLOCK_BIT_BUSY,

  // The block is segment 2+ of a longer move
  BLOCK_BIT_CONTINUED
};

enum BlockFlag {
  BLOCK_FLAG_RECALCULATE          = _BV(BLOCK_BIT_RECALCULATE),
  BLOCK_FLAG_NOMINAL_LENGTH       = _BV(BLOCK_BIT_NOMINAL_LENGTH),
  BLOCK_FLAG_START_FROM_FULL_HALT = _BV(BLOCK_BIT_START_FROM_FULL_HALT),
  BLOCK_FLAG_BUSY                 = _BV(BLOCK_BIT_BUSY),
  BLOCK_FLAG_CONTINUED            = _BV(BLOCK_BIT_CONTINUED)
};

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

  uint8_t flag;                             // Block flags (See BlockFlag enum above)

  unsigned char active_extruder;            // The extruder to move (if E move)
  unsigned char active_driver;              // Selects the active driver for E

  // Fields used by the Bresenham algorithm for tracing the line
  int32_t steps[NUM_AXIS];                  // Step count along each axis
  uint32_t step_event_count;                // The number of step events required to complete this block

  #if ENABLED(COLOR_MIXING_EXTRUDER)
    uint32_t mix_event_count[MIXING_STEPPERS]; // Scaled step_event_count for the mixing steppers
  #endif

  int32_t accelerate_until,                 // The index of the step event on which to stop acceleration
          decelerate_after,                 // The index of the step event on which to start decelerating
          acceleration_rate;                // The acceleration rate used for acceleration calculation

  uint8_t direction_bits;                   // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)

  // Advance extrusion
  #if ENABLED(LIN_ADVANCE)
    bool      use_advance_lead;
    uint16_t  advance_speed,                // Timer value for extruder speed offset
              max_adv_steps,                // max. advance steps to get cruising speed pressure (not always nominal_speed!)
              final_adv_steps;              // advance steps due to exit speed
    float     e_D_ratio;
  #endif

  // Fields used by the motion planner to manage acceleration
  float nominal_speed,                      // The nominal speed for this block in mm/sec
        entry_speed,                        // Entry speed at previous-current junction in mm/sec
        max_entry_speed,                    // Maximum allowable junction entry speed in mm/sec
        millimeters,                        // The total travel of this block in mm
        acceleration;                       // acceleration mm/sec^2

  // Settings for the trapezoid generator
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

  uint16_t block_len;

} block_t;

#define BLOCK_MOD(n) ((n)&(BLOCK_BUFFER_SIZE-1))

class Planner {

  public: /** Constructor */

    Planner();

  public: /** Public Parameters */

    /**
     * The current position of the tool in absolute steps
     * Recalculated if any axis_steps_per_mm are changed by gcode
     */
    static int32_t position[NUM_AXIS];

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
    static block_t block_buffer[BLOCK_BUFFER_SIZE];
    static volatile uint8_t block_buffer_head,  // Index of the next block to be pushed
                            block_buffer_tail;  // Index of the busy block, if any

    /**
     * Limit where 64bit math is necessary for acceleration calculation
     */
    static uint32_t cutoff_long;

    #if ENABLED(LIN_ADVANCE)
      static float  extruder_advance_K,
                    position_float[XYZE];
    #endif

  private: /** Private Parameters */

    /**
     * Speed of previous path line segment
     */
    static float previous_speed[NUM_AXIS];

    /**
     * Nominal speed of previous path line segment
     */
    static float previous_nominal_speed;

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
      static unsigned char old_direction_bits;
      // Segment times (in µs). Used for speed calculations
      static uint32_t axis_segment_time_us[2][3];
    #endif

    #if ENABLED(ULTRA_LCD)
      volatile static uint32_t block_buffer_runtime_us; // Theoretical block buffer runtime in µs
    #endif

  public: /** Public Function */

    void init();

    /**
     * Static (class) Methods
     */

    // Manage fans, paste pressure, etc.
    static void check_axes_activity();

    /**
     * Number of moves currently in the planner
     */
    FORCE_INLINE static uint8_t movesplanned() { return BLOCK_MOD(block_buffer_head - block_buffer_tail + BLOCK_BUFFER_SIZE); }

    FORCE_INLINE static bool is_full() { return block_buffer_tail == next_block_index(block_buffer_head); }

    /**
     * Planner::buffer_steps
     *
     * Add a new linear movement to the buffer (in terms of steps).
     *
     *  target      - target position in steps units
     *  fr_mm_s     - (target) speed of the move
     *  extruder    - target extruder
     *  millimeters - the length of the movement, if known
     */
    #if ENABLED(LIN_ADVANCE)
      static void buffer_steps(const int32_t (&target)[XYZE], const float (&target_float)[XYZE], float fr_mm_s, const uint8_t extruder, const float &millimeters=0.0);
    #else
      static void buffer_steps(const int32_t (&target)[XYZE], float fr_mm_s, const uint8_t extruder, const float &millimeters=0.0);
    #endif

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
    static void buffer_segment(const float &a, const float &b, const float &c, const float &e, const float &fr_mm_s, const uint8_t extruder, const float &millimeters=0.0);

    /**
     * Add a new linear movement to the buffer.
     * The target is NOT translated to delta/scara
     *
     * Leveling will be applied to input on cartesians.
     * Kinematic machines should call buffer_line_kinematic (for leveled moves).
     * (Cartesians may also call buffer_line_kinematic.)
     *
     *  rx,ry,rz,e  - target position in mm or degrees
     *  fr_mm_s     - (target) speed of the move (mm/s)
     *  extruder    - target extruder
     *  millimeters - the length of the movement, if known
     */
    static void buffer_line(ARG_X, ARG_Y, ARG_Z, const float &e, const float &fr_mm_s, const uint8_t extruder, const float millimeters=0.0);

    /**
     * Add a new linear movement to the buffer.
     * The target is cartesian, it's translated to delta/scara if
     * needed.
     *
     *  cart        - x,y,z,e CARTESIAN target in mm
     *  fr_mm_s     - (target) speed of the move (mm/s)
     *  extruder    - target extruder
     *  millimeters - the length of the movement, if known
     */
    static void buffer_line_kinematic(const float cart[XYZE], const float &fr_mm_s, const uint8_t extruder, const float millimeters=0.0);

    FORCE_INLINE static void zero_previous_nominal_speed() { previous_nominal_speed = 0.0; } // Resets planner junction speeds. Assumes start from rest.
    FORCE_INLINE static void zero_previous_speed(const AxisEnum axis) { previous_speed[axis] = 0.0; }
    FORCE_INLINE static void zero_previous_speed() { ZERO(previous_speed); }

    /**
     * Sync from the stepper positions. (e.g., after an interrupted move)
     */
    static void sync_from_steppers();

    /**
     * Abort Printing
     */
    void abort();

    /**
     * Does the buffer have any blocks queued?
     */
    FORCE_INLINE static bool has_blocks_queued() { return (block_buffer_head != block_buffer_tail); }

    /**
     * "Discard" the block and "release" the memory.
     * Called when the current block is no longer needed.
     */
    FORCE_INLINE static void discard_current_block() {
      if (has_blocks_queued())
        block_buffer_tail = BLOCK_MOD(block_buffer_tail + 1);
    }

    /**
     * "Discard" the next block if it's continued.
     * Called after an interrupted move to throw away the rest of the move.
     */
    FORCE_INLINE static bool discard_continued_block() {
      const bool discard = has_blocks_queued() && TEST(block_buffer[block_buffer_tail].flag, BLOCK_BIT_CONTINUED);
      if (discard) discard_current_block();
      return discard;
    }

    /**
     * length of commands in planner
     */
    FORCE_INLINE static uint16_t command_in_planner_len() {
      uint8_t _block_buffer_head = block_buffer_head;
      uint8_t _block_buffer_tail = block_buffer_tail;
      uint16_t block_len = 0;

      while (_block_buffer_head != _block_buffer_tail) {
        block_len += block_buffer[_block_buffer_tail].block_len;
        _block_buffer_tail = (_block_buffer_tail + 1) & (BLOCK_BUFFER_SIZE - 1);
      }
      return block_len;
    }

    /**
     * Number of block in planner
     */
    FORCE_INLINE static uint8_t number_of_blocks() {
      return (block_buffer_head + BLOCK_BUFFER_SIZE - block_buffer_tail) & (BLOCK_BUFFER_SIZE - 1);
    }

    FORCE_INLINE void add_block_length(uint16_t block_len) {
      if (block_buffer_head != block_buffer_tail)
        block_buffer[prev_block_index(block_buffer_head)].block_len += block_len;
    }

    /**
     * The current block. NULL if the buffer is empty.
     * This also marks the block as busy.
     * WARNING: Called from Stepper ISR context!
     */
    static block_t* get_current_block() {
      if (has_blocks_queued()) {
        block_t * const block = &block_buffer[block_buffer_tail];

        // If the block has no trapezoid calculated, it's unsafe to execute.
        if (movesplanned() > 1) {
          const block_t * const next = &block_buffer[next_block_index(block_buffer_tail)];
          if (TEST(block->flag, BLOCK_BIT_RECALCULATE) || TEST(next->flag, BLOCK_BIT_RECALCULATE))
            return NULL;
        }
        else if (TEST(block->flag, BLOCK_BIT_RECALCULATE))
          return NULL;

        #if ENABLED(ULTRA_LCD)
          block_buffer_runtime_us -= block->segment_time_us; // We can't be sure how long an active block will take, so don't count it.
        #endif
        SBI(block->flag, BLOCK_BIT_BUSY);
        return block;
      }
      else {
        #if ENABLED(ULTRA_LCD)
          clear_block_buffer_runtime(); // paranoia. Buffer is empty now - so reset accumulated time to zero.
        #endif
        return NULL;
      }
    }

    #if ENABLED(ULTRA_LCD)

      static uint16_t block_buffer_runtime() {
        CRITICAL_SECTION_START
          millis_t bbru = block_buffer_runtime_us;
        CRITICAL_SECTION_END
        // To translate µs to ms a division by 1000 would be required.
        // We introduce 2.4% error here by dividing by 1024.
        // Doesn't matter because block_buffer_runtime_us is already too small an estimation.
        bbru >>= 10;
        // limit to about a minute.
        NOMORE(bbru, 0xFFFFul);
        return bbru;
      }

      static void clear_block_buffer_runtime() {
        CRITICAL_SECTION_START
          block_buffer_runtime_us = 0;
        CRITICAL_SECTION_END
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
    static constexpr int8_t next_block_index(const int8_t block_index) { return BLOCK_MOD(block_index + 1); }
    static constexpr int8_t prev_block_index(const int8_t block_index) { return BLOCK_MOD(block_index - 1); }

    /**
     * Calculate the distance (not time) it takes to accelerate
     * from initial_rate to target_rate using the given acceleration:
     */
    static float estimate_acceleration_distance(const float &initial_rate, const float &target_rate, const float &accel) {
      if (accel == 0) return 0; // accel was 0, set acceleration distance to 0
      return (sq(target_rate) - sq(initial_rate)) / (accel * 2.0);
    }

    /**
     * Return the point at which you must start braking (at the rate of -'acceleration') if
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
     * to reach 'target_velocity' using 'acceleration' within a given
     * 'distance'.
     */
    static float max_allowable_speed(const float &accel, const float &target_velocity, const float &distance) {
      return SQRT(sq(target_velocity) - 2 * accel * distance);
    }

    static void calculate_trapezoid_for_block(block_t* const block, const float &entry_factor, const float &exit_factor);

    static void reverse_pass_kernel(block_t* const current, const block_t * const next);
    static void forward_pass_kernel(const block_t * const previous, block_t* const current);

    static void reverse_pass();
    static void forward_pass();

    static void recalculate_trapezoids();

    static void recalculate();

};

#define PLANNER_XY_FEEDRATE() (min(mechanics.max_feedrate_mm_s[X_AXIS], mechanics.max_feedrate_mm_s[Y_AXIS]))

extern Planner planner;

#endif // PLANNER_H
