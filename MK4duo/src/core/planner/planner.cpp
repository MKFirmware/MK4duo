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
 * planner.cpp
 *
 * Buffer movement commands and manage the acceleration profile plan
 *
 * Derived from Grbl
 * Copyright (c) 2009-2011 Simen Svale Skogsrud
 *
 * The ring buffer implementation gleaned from the wiring_serial library by David A. Mellis.
 *
 *
 * Reasoning behind the mathematics in this module (in the key of 'Mathematica'):
 *
 * s == speed, a == acceleration, t == time, d == distance
 *
 * Basic definitions:
 *   Speed[s_, a_, t_] := s + (a*t)
 *   Travel[s_, a_, t_] := Integrate[Speed[s, a, t], t]
 *
 * Distance to reach a specific speed with a constant acceleration:
 *   Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, d, t]
 *   d -> (m^2 - s^2)/(2 a) --> estimate_acceleration_distance()
 *
 * Speed after a given distance of travel with constant acceleration:
 *   Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, m, t]
 *   m -> Sqrt[2 a d + s^2]
 *
 * DestinationSpeed[s_, a_, d_] := Sqrt[2 a d + s^2]
 *
 * When to start braking (di) to reach a specified destination speed (s2) after accelerating
 * from initial speed s1 without ever stopping at a plateau:
 *   Solve[{DestinationSpeed[s1, a, di] == DestinationSpeed[s2, a, d - di]}, di]
 *   di -> (2 a d - s1^2 + s2^2)/(4 a) --> intersection_distance()
 *
 * IntersectionDistance[s1_, s2_, a_, d_] := (2 a d - s1^2 + s2^2)/(4 a)
 *
 */

#include "../../../MK4duo.h"

Planner planner;

/**
 * A ring buffer of moves described in steps
 */
block_t Planner::block_buffer[BLOCK_BUFFER_SIZE];
volatile uint8_t  Planner::block_buffer_head = 0, // Index of the next block to be pushed
                  Planner::block_buffer_tail = 0;

#if HAS_TEMP_HOTEND && ENABLED(AUTOTEMP)
  float Planner::autotemp_max = 250,
        Planner::autotemp_min = 210,
        Planner::autotemp_factor = 0.1;
  bool Planner::autotemp_enabled = false;
#endif

int32_t Planner::position[NUM_AXIS] = { 0 };

uint32_t Planner::cutoff_long;

float Planner::previous_speed[NUM_AXIS],
      Planner::previous_nominal_speed;

#if ENABLED(DISABLE_INACTIVE_EXTRUDER)
  uint8_t Planner::g_uc_extruder_last_move[EXTRUDERS] = { 0 };
#endif

#if ENABLED(XY_FREQUENCY_LIMIT)
  // Old direction bits. Used for speed calculations
  unsigned char Planner::old_direction_bits = 0;
  // Segment times (in Âµs). Used for speed calculations
  uint32_t Planner::axis_segment_time_us[2][3] = { { MAX_FREQ_TIME_US + 1, 0, 0 }, { MAX_FREQ_TIME_US + 1, 0, 0 } };
#endif

#if ENABLED(LIN_ADVANCE)
  float Planner::extruder_advance_K   = LIN_ADVANCE_K,
        Planner::position_float[XYZE] = { 0.0 };
#endif

#if ENABLED(ULTRA_LCD)
  volatile uint32_t Planner::block_buffer_runtime_us = 0;
#endif

/**
 * Class and Instance Methods
 */
Planner::Planner() { init(); }

void Planner::init() {
  block_buffer_head = block_buffer_tail = 0;
  ZERO(position);
  #if ENABLED(LIN_ADVANCE)
    ZERO(position_float);
  #endif
  ZERO(previous_speed);
  previous_nominal_speed = 0.0;
  #if ABL_PLANAR
    bedlevel.matrix.set_to_identity();
  #endif
}

#define MINIMAL_STEP_RATE 120

/**
 * Calculate trapezoid parameters, multiplying the entry- and exit-speeds
 * by the provided factors.
 */
void Planner::calculate_trapezoid_for_block(block_t* const block, const float &entry_factor, const float &exit_factor) {
  uint32_t initial_rate = CEIL(entry_factor * block->nominal_rate),
           final_rate   = CEIL(exit_factor  * block->nominal_rate); // (steps per second)

  // Limit minimal step rate (Otherwise the timer will overflow.)
  NOLESS(initial_rate, MINIMAL_STEP_RATE);
  NOLESS(final_rate, MINIMAL_STEP_RATE);

  const int32_t accel = block->acceleration_steps_per_s2;

          // Steps required for acceleration, deceleration to/from nominal rate
  int32_t accelerate_steps = CEIL(estimate_acceleration_distance(initial_rate, block->nominal_rate, accel)),
          decelerate_steps = FLOOR(estimate_acceleration_distance(block->nominal_rate, final_rate, -accel)),
          // Steps between acceleration and deceleration, if any
          plateau_steps = block->step_event_count - accelerate_steps - decelerate_steps;

  // Does accelerate_steps + decelerate_steps exceed step_event_count?
  // Then we can't possibly reach the nominal rate, there will be no cruising.
  // Use intersection_distance() to calculate accel / braking time in order to
  // reach the final_rate exactly at the end of this block.
  if (plateau_steps < 0) {
    accelerate_steps = CEIL(intersection_distance(initial_rate, final_rate, accel, block->step_event_count));
    NOLESS(accelerate_steps, 0); // Check limits due to numerical round-off
    accelerate_steps = min((uint32_t)accelerate_steps, block->step_event_count);//(We can cast here to unsigned, because the above line ensures that we are above zero)
    plateau_steps = 0;
  }

  // block->accelerate_until = accelerate_steps;
  // block->decelerate_after = accelerate_steps+plateau_steps;

  CRITICAL_SECTION_START
    if (!TEST(block->flag, BLOCK_BIT_BUSY)) { // Don't update variables if block is busy.
      block->accelerate_until = accelerate_steps;
      block->decelerate_after = accelerate_steps + plateau_steps;
      block->initial_rate = initial_rate;
      block->final_rate = final_rate;
    }
  CRITICAL_SECTION_END
}

// The kernel called by recalculate() when scanning the plan from last to first entry.
void Planner::reverse_pass_kernel(block_t* const current, const block_t * const next) {
  if (!current || !next) return;
  // If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
  // If not, block in state of acceleration or deceleration. Reset entry speed to maximum and
  // check for maximum allowable speed reductions to ensure maximum possible planned speed.
  float max_entry_speed = current->max_entry_speed;
  if (current->entry_speed != max_entry_speed) {
    // If nominal length true, max junction speed is guaranteed to be reached. Only compute
    // for max allowable speed if block is decelerating and nominal length is false.
    current->entry_speed = (TEST(current->flag, BLOCK_BIT_NOMINAL_LENGTH) || max_entry_speed <= next->entry_speed)
      ? max_entry_speed
      : min(max_entry_speed, max_allowable_speed(-current->acceleration, next->entry_speed, current->millimeters));
    SBI(current->flag, BLOCK_BIT_RECALCULATE);
  }
}

/**
 * recalculate() needs to go over the current plan twice.
 * Once in reverse and once forward. This implements the reverse pass.
 */
void Planner::reverse_pass() {
  if (movesplanned() > 3) {
    const uint8_t endnr = BLOCK_MOD(block_buffer_tail + 2); // tail is running. tail+1 shouldn't be altered because it's connected to the running block.
                                                            // tail+2 because the index is not yet advanced when checked
    uint8_t blocknr     = prev_block_index(block_buffer_head);
    block_t* current    = &block_buffer[blocknr];

    do {
      const block_t * const next = current;
      blocknr = prev_block_index(blocknr);
      current = &block_buffer[blocknr];
      if (TEST(current->flag, BLOCK_BIT_START_FROM_FULL_HALT)) // Up to this every block is already optimized.
        break;
      reverse_pass_kernel(current, next);
    } while (blocknr != endnr);
  }
}

// The kernel called by recalculate() when scanning the plan from first to last entry.
void Planner::forward_pass_kernel(const block_t * const previous, block_t* const current) {
  if (!previous) return;

  // If the previous block is an acceleration block, but it is not long enough to complete the
  // full speed change within the block, we need to adjust the entry speed accordingly. Entry
  // speeds have already been reset, maximized, and reverse planned by reverse planner.
  // If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.
  if (!TEST(previous->flag, BLOCK_BIT_NOMINAL_LENGTH)) {
    if (previous->entry_speed < current->entry_speed) {
      float entry_speed = min(current->entry_speed,
                               max_allowable_speed(-previous->acceleration, previous->entry_speed, previous->millimeters));
      // Check for junction speed change
      if (current->entry_speed != entry_speed) {
        current->entry_speed = entry_speed;
        SBI(current->flag, BLOCK_BIT_RECALCULATE);
      }
    }
  }
}

/**
 * recalculate() needs to go over the current plan twice.
 * Once in reverse and once forward. This implements the forward pass.
 */
void Planner::forward_pass() {
  block_t* block[3] = { NULL, NULL, NULL };

  for (uint8_t b = block_buffer_tail; b != block_buffer_head; b = next_block_index(b)) {
    block[0] = block[1];
    block[1] = block[2];
    block[2] = &block_buffer[b];
    forward_pass_kernel(block[0], block[1]);
  }
  forward_pass_kernel(block[1], block[2]);
}

/**
 * Recalculate the trapezoid speed profiles for all blocks in the plan
 * according to the entry_factor for each junction. Must be called by
 * recalculate() after updating the blocks.
 */
void Planner::recalculate_trapezoids() {
  int8_t block_index = block_buffer_tail;
  block_t *current, *next = NULL;

  while (block_index != block_buffer_head) {
    current = next;
    next = &block_buffer[block_index];
    if (current) {
      // Recalculate if current block entry or exit junction speed has changed.
      if (TEST(current->flag, BLOCK_BIT_RECALCULATE) || TEST(next->flag, BLOCK_BIT_RECALCULATE)) {
        // NOTE: Entry and exit factors always > 0 by all previous logic operations.
        const float nomr = 1.0 / current->nominal_speed;
        calculate_trapezoid_for_block(current, current->entry_speed * nomr, next->entry_speed * nomr);
        #if ENABLED(LIN_ADVANCE)
          if (current->use_advance_lead) {
            const float comp = current->e_D_ratio * extruder_advance_K * mechanics.axis_steps_per_mm[E_AXIS];
            current->max_adv_steps = current->nominal_speed * comp;
            current->final_adv_steps = next->entry_speed * comp;
          }
        #endif
        CBI(current->flag, BLOCK_BIT_RECALCULATE); // Reset current only to ensure next trapezoid is computed
      }
    }
    block_index = next_block_index(block_index);
  }
  // Last/newest block in buffer. Exit speed is set with MINIMUM_PLANNER_SPEED. Always recalculated.
  if (next) {
    const float nomr = 1.0 / next->nominal_speed;
    calculate_trapezoid_for_block(next, next->entry_speed * nomr, (MINIMUM_PLANNER_SPEED) * nomr);
    #if ENABLED(LIN_ADVANCE)
      if (next->use_advance_lead) {
        const float comp = next->e_D_ratio * extruder_advance_K * mechanics.axis_steps_per_mm[E_AXIS];
        next->max_adv_steps = next->nominal_speed * comp;
        next->final_adv_steps = (MINIMUM_PLANNER_SPEED) * comp;
      }
    #endif
    CBI(next->flag, BLOCK_BIT_RECALCULATE);
  }
}

/**
 * Recalculate the motion plan according to the following algorithm:
 *
 *   1. Go over every block in reverse order...
 *
 *      Calculate a junction speed reduction (block_t.entry_factor) so:
 *
 *      a. The junction jerk is within the set limit, and
 *
 *      b. No speed reduction within one block requires faster
 *         deceleration than the one, true constant acceleration.
 *
 *   2. Go over every block in chronological order...
 *
 *      Dial down junction speed reduction values if:
 *      a. The speed increase within one block would require faster
 *         acceleration than the one, true constant acceleration.
 *
 * After that, all blocks will have an entry_factor allowing all speed changes to
 * be performed using only the one, true constant acceleration, and where no junction
 * jerk is jerkier than the set limit, Jerky. Finally it will:
 *
 *   3. Recalculate "trapezoids" for all blocks.
 */
void Planner::recalculate() {
  reverse_pass();
  forward_pass();
  recalculate_trapezoids();
}


#if HAS_TEMP_HOTEND && ENABLED(AUTOTEMP)

  void Planner::getHighESpeed() {
    static float oldt = 0;

    if (!autotemp_enabled) return;
    if (heaters[0].target_temperature + 2 < autotemp_min) return; // probably temperature set to zero.

    float high = 0.0;
    for (uint8_t b = block_buffer_tail; b != block_buffer_head; b = next_block_index(b)) {
      block_t* block = &block_buffer[b];
      if (block->steps[X_AXIS] || block->steps[Y_AXIS] || block->steps[Z_AXIS]) {
        float se = (float)block->steps[E_AXIS] / block->step_event_count * block->nominal_speed; // mm/sec;
        NOLESS(high, se);
      }
    }

    float t = autotemp_min + high * autotemp_factor;
    t = constrain(t, autotemp_min, autotemp_max);
    if (t < oldt) t = t * (1 - (AUTOTEMP_OLDWEIGHT)) + oldt * (AUTOTEMP_OLDWEIGHT);
    oldt = t;
    heaters[0].setTarget(t);
  }

#endif // HAS_TEMP_HOTEND && ENABLED(AUTOTEMP)

/**
 * Maintain fans, paste extruder pressure,
 */
void Planner::check_axes_activity() {
  unsigned char axis_active[NUM_AXIS] = { 0 };

  #if ENABLED(BARICUDA)
    #if HAS_HEATER_1
      uint8_t tail_valve_pressure;
    #endif
    #if HAS_HEATER_2
      uint8_t tail_e_to_p_pressure;
    #endif
  #endif

  if (has_blocks_queued()) {

    block_t* block;

    #if ENABLED(BARICUDA)
      block = &block_buffer[block_buffer_tail];
      #if HAS_HEATER_1
        tail_valve_pressure = block->valve_pressure;
      #endif
      #if HAS_HEATER_2
        tail_e_to_p_pressure = block->e_to_p_pressure;
      #endif
    #endif

    for (uint8_t b = block_buffer_tail; b != block_buffer_head; b = next_block_index(b)) {
      block = &block_buffer[b];
      LOOP_XYZE(i) if (block->steps[i]) axis_active[i]++;
    }
  }
  else {
    #if ENABLED(BARICUDA)
      #if HAS_HEATER_1
        tail_valve_pressure = printer.baricuda_valve_pressure;;
      #endif
      #if HAS_HEATER_2
        tail_e_to_p_pressure = printer.baricuda_e_to_p_pressure;
      #endif
    #endif
  }

  #if DISABLE_X
    if (!axis_active[X_AXIS]) disable_X();
  #endif
  #if DISABLE_Y
    if (!axis_active[Y_AXIS]) disable_Y();
  #endif
  #if DISABLE_Z
    if (!axis_active[Z_AXIS]) disable_Z();
  #endif
  #if DISABLE_E
    if (!axis_active[E_AXIS]) stepper.disable_e_steppers();
  #endif

  #if HAS_TEMP_HOTEND && ENABLED(AUTOTEMP)
    getHighESpeed();
  #endif

  #if ENABLED(BARICUDA)
    #if HAS_HEATER_1
      analogWrite(HEATER_1_PIN, tail_valve_pressure);
    #endif
    #if HAS_HEATER_2
      analogWrite(HEATER_2_PIN, tail_e_to_p_pressure);
    #endif
  #endif
}

/**
 * Planner::buffer_steps
 *
 * Add a new linear movement to the buffer (in terms of steps).
 *
 *  target      - target position in steps units
 *  fr_mm_s     - (target) speed of the move
 *  extruder    - target extruder
 */
#if ENABLED(LIN_ADVANCE)
  void Planner::buffer_steps(const int32_t (&target)[XYZE], const float (&target_float)[XYZE], float fr_mm_s, const uint8_t extruder, const float &millimeters/*=0.0*/)
#else
  void Planner::buffer_steps(const int32_t (&target)[XYZE], float fr_mm_s, const uint8_t extruder, const float &millimeters/*=0.0*/)
#endif
{

  const int32_t dx = target[X_AXIS] - position[X_AXIS],
                dy = target[Y_AXIS] - position[Y_AXIS],
                dz = target[Z_AXIS] - position[Z_AXIS];

  int32_t de = target[E_AXIS] - position[E_AXIS];

  /* <-- add a slash to enable
    SERIAL_MV("  buffer_steps FR:", fr_mm_s);
    SERIAL_MV(" A:", target[A_AXIS]);
    SERIAL_MV(" (", da);
    SERIAL_MV(" steps) B:", target[B_AXIS]);
    SERIAL_MV(" (", db);
    SERIAL_MV(" steps) C:", target[C_AXIS]);
    SERIAL_MV(" (", dc);
    SERIAL_MV(" steps) E:", target[E_AXIS]);
    SERIAL_MV(" (", de);
    SERIAL_EM(" steps)");
  //*/

  #if ENABLED(PREVENT_COLD_EXTRUSION) || ENABLED(PREVENT_LENGTHY_EXTRUDE)
    if (de
      #if HAS_MULTI_MODE
        && printer.mode == PRINTER_MODE_FFF
      #endif
    ) {
      #if ENABLED(PREVENT_COLD_EXTRUSION)
        if (thermalManager.tooColdToExtrude(extruder)) {
          position[E_AXIS] = target[E_AXIS]; // Behave as if the move really took place, but ignore E part
          #if ENABLED(LIN_ADVANCE)
            position_float[E_AXIS] = target_float[E_AXIS];
          #endif
          de = 0; // no difference
          SERIAL_LM(ER, MSG_ERR_COLD_EXTRUDE_STOP);
        }
      #endif
      #if ENABLED(PREVENT_LENGTHY_EXTRUDE)
        if (labs(de * tools.e_factor[extruder]) > (int32_t)mechanics.axis_steps_per_mm[E_AXIS_N] * (EXTRUDE_MAXLENGTH)) {
          position[E_AXIS] = target[E_AXIS]; // Behave as if the move really took place, but ignore E part
          #if ENABLED(LIN_ADVANCE)
            position_float[E_AXIS] = target_float[E_AXIS];
          #endif
          de = 0; // no difference
          SERIAL_LM(ER, MSG_ERR_LONG_EXTRUDE_STOP);
        }
      #endif // PREVENT_LENGTHY_EXTRUDE
    }
  #endif // PREVENT_COLD_EXTRUSION || PREVENT_LENGTHY_EXTRUDE

  #if CORE_IS_XY
    long da = dx + CORE_FACTOR * dy;
    long db = dx - CORE_FACTOR * dy;
  #elif CORE_IS_XZ
    long da = dx + CORE_FACTOR * dz;
    long dc = dx - CORE_FACTOR * dz;
  #elif CORE_IS_YZ
    long db = dy + CORE_FACTOR * dz;
    long dc = dy - CORE_FACTOR * dz;
  #endif

  // Compute direction bit for this block
  uint8_t dirb = 0;
  #if CORE_IS_XY
    if (dx < 0) SBI(dirb, X_HEAD);            // Save the real Extruder (head) direction in X Axis
    if (dy < 0) SBI(dirb, Y_HEAD);            // ...and Y
    if (dz < 0) SBI(dirb, Z_AXIS);
    if (da < 0) SBI(dirb, A_AXIS);            // Motor A direction
    if (CORESIGN(db) < 0) SBI(dirb, B_AXIS);  // Motor B direction
  #elif CORE_IS_XZ
    if (dx < 0) SBI(dirb, X_HEAD);            // Save the real Extruder (head) direction in X Axis
    if (dy < 0) SBI(dirb, Y_AXIS);
    if (dz < 0) SBI(dirb, Z_HEAD);            // ...and Z
    if (da < 0) SBI(dirb, A_AXIS);            // Motor A direction
    if (CORESIGN(dc) < 0) SBI(dirb, C_AXIS);  // Motor C direction
  #elif CORE_IS_YZ
    if (dx < 0) SBI(dirb, X_AXIS);
    if (dy < 0) SBI(dirb, Y_HEAD);            // Save the real Extruder (head) direction in Y Axis
    if (dz < 0) SBI(dirb, Z_HEAD);            // ...and Z
    if (db < 0) SBI(dirb, B_AXIS);            // Motor B direction
    if (CORESIGN(dc) < 0) SBI(dirb, C_AXIS);  // Motor C direction
  #else
    if (dx < 0) SBI(dirb, X_AXIS);
    if (dy < 0) SBI(dirb, Y_AXIS);
    if (dz < 0) SBI(dirb, Z_AXIS);
  #endif
  if (de < 0) SBI(dirb, E_AXIS);

  const float esteps_float = de * tools.e_factor[extruder];
  const int32_t esteps = abs(esteps_float) + 0.5;

  // Calculate the buffer head after we push this byte
  const uint8_t next_buffer_head = next_block_index(block_buffer_head);

  // If the buffer is full: good! That means we are well ahead of the robot.
  // Rest here until there is room in the buffer.
  while (block_buffer_tail == next_buffer_head) printer.idle();

  // Prepare to set up new block
  block_t* block = &block_buffer[block_buffer_head];

  // Clear all flags, including the "busy" bit
  block->flag = 0x00;

  // Set direction bits
  block->direction_bits = dirb;

  // Number of steps for each axis
  // See http://www.corexy.com/theory.html
  #if CORE_IS_XY
    // corexy planning
    block->steps[A_AXIS] = labs(da);
    block->steps[B_AXIS] = labs(db);
    block->steps[Z_AXIS] = labs(dz);
  #elif CORE_IS_XZ
    // corexz planning
    block->steps[A_AXIS] = labs(da);
    block->steps[Y_AXIS] = labs(dy);
    block->steps[C_AXIS] = labs(dc);
  #elif CORE_IS_YZ
    // coreyz planning
    block->steps[X_AXIS] = labs(dx);
    block->steps[B_AXIS] = labs(db);
    block->steps[C_AXIS] = labs(dc);
  #else
    // default non-h-bot planning
    block->steps[X_AXIS] = labs(dx);
    block->steps[Y_AXIS] = labs(dy);
    block->steps[Z_AXIS] = labs(dz);
  #endif

  block->steps[E_AXIS] = esteps;
  block->step_event_count = MAX4(block->steps[X_AXIS], block->steps[Y_AXIS], block->steps[Z_AXIS], esteps);

  #if HAS_MULTI_MODE
    if (printer.mode != PRINTER_MODE_LASER)
  #endif
    // Bail if this is a zero-length block
    if (block->step_event_count < MIN_STEPS_PER_SEGMENT) return;

  // For a mixing extruder, get a magnified step_event_count for each
  #if ENABLED(COLOR_MIXING_EXTRUDER)
    for (uint8_t i = 0; i < MIXING_STEPPERS; i++)
      block->mix_event_count[i] = mixing_factor[i] * block->step_event_count;
  #endif

  #if ENABLED(BARICUDA)
    block->valve_pressure   = printer.baricuda_valve_pressure;
    block->e_to_p_pressure  = printer.baricuda_e_to_p_pressure;
  #endif

  block->active_extruder = extruder;

  #if HAS_POWER_SWITCH
    if ((block->steps[X_AXIS] || block->steps[Y_AXIS] || block->steps[Z_AXIS]) && (!powerManager.lastPowerOn))
      powerManager.power_on();
  #endif

  #if HAS_MKMULTI_TOOLS
    block->active_driver = tools.active_driver;
  #else
    block->active_driver = extruder;
  #endif

  // Enable active axes
  #if CORE_IS_XY
    if (block->steps[A_AXIS] || block->steps[B_AXIS]) {
      enable_X();
      enable_Y();
    }
    #if DISABLED(Z_LATE_ENABLE)
      if (block->steps[Z_AXIS]) enable_Z();
    #endif
  #elif CORE_IS_XZ
    if (block->steps[A_AXIS] || block->steps[C_AXIS]) {
      enable_X();
      enable_Z();
    }
    if (block->steps[Y_AXIS]) enable_Y();
  #elif CORE_IS_YZ
    if (block->steps[B_AXIS] || block->steps[C_AXIS]) {
      enable_Y();
      enable_Z();
    }
    if (block->steps[X_AXIS]) enable_X();
  #else
    if (block->steps[X_AXIS]) enable_X();
    if (block->steps[Y_AXIS]) enable_Y();
    #if DISABLED(Z_LATE_ENABLE)
      if (block->steps[Z_AXIS]) enable_Z();
    #endif
  #endif

  // Enable extruder(s)
  if (esteps) {

    #if HAS_POWER_SWITCH
      if (!powerManager.lastPowerOn) powerManager.power_on();
    #endif

    #if !HAS_MKMULTI_TOOLS

      #if EXTRUDERS > 0 && ENABLED(DISABLE_INACTIVE_EXTRUDER) // Enable only the selected extruder

        for (uint8_t i = 0; i < EXTRUDERS; i++)
          if (g_uc_extruder_last_move[i] > 0) g_uc_extruder_last_move[i]--;

        switch(extruder) {
          case 0:
            enable_E0();
            g_uc_extruder_last_move[0] = (BLOCK_BUFFER_SIZE) * 2;
            #if ENABLED(DUAL_X_CARRIAGE)
              if (extruder_duplication_enabled) {
                enable_E1();
                g_uc_extruder_last_move[1] = (BLOCK_BUFFER_SIZE) * 2;
              }
            #endif
            #if EXTRUDERS > 1
              if (g_uc_extruder_last_move[1] == 0) disable_E1();
              #if EXTRUDERS > 2
                if (g_uc_extruder_last_move[2] == 0) disable_E2();
                #if EXTRUDERS > 3
                  if (g_uc_extruder_last_move[3] == 0) disable_E3();
                  #if EXTRUDERS > 4
                    if (g_uc_extruder_last_move[4] == 0) disable_E4();
                    #if EXTRUDERS > 5
                      if (g_uc_extruder_last_move[5] == 0) disable_E5();
                    #endif
                  #endif
                #endif
              #endif
            #endif
          break;
          #if EXTRUDERS > 1
            case 1:
              enable_E1();
              g_uc_extruder_last_move[1] = (BLOCK_BUFFER_SIZE) * 2;
              if (g_uc_extruder_last_move[0] == 0) disable_E0();
              #if EXTRUDERS > 2
                if (g_uc_extruder_last_move[2] == 0) disable_E2();
                #if EXTRUDERS > 3
                  if (g_uc_extruder_last_move[3] == 0) disable_E3();
                  #if EXTRUDERS > 4
                    if (g_uc_extruder_last_move[4] == 0) disable_E4();
                    #if EXTRUDERS > 5
                      if (g_uc_extruder_last_move[5] == 0) disable_E5();
                    #endif
                  #endif
                #endif
              #endif
            break;
            #if EXTRUDERS > 2
              case 2:
                enable_E2();
                g_uc_extruder_last_move[2] = (BLOCK_BUFFER_SIZE) * 2;
                if (g_uc_extruder_last_move[0] == 0) disable_E0();
                if (g_uc_extruder_last_move[1] == 0) disable_E1();
                #if EXTRUDERS > 3
                  if (g_uc_extruder_last_move[3] == 0) disable_E3();
                  #if EXTRUDERS > 4
                    if (g_uc_extruder_last_move[4] == 0) disable_E4();
                    #if EXTRUDERS > 5
                      if (g_uc_extruder_last_move[5] == 0) disable_E5();
                    #endif
                  #endif
                #endif
              break;
              #if EXTRUDERS > 3
                case 3:
                  enable_E3();
                  g_uc_extruder_last_move[3] = (BLOCK_BUFFER_SIZE) * 2;
                  if (g_uc_extruder_last_move[0] == 0) disable_E0();
                  if (g_uc_extruder_last_move[1] == 0) disable_E1();
                  if (g_uc_extruder_last_move[2] == 0) disable_E2();
                  #if EXTRUDERS > 4
                    if (g_uc_extruder_last_move[4] == 0) disable_E4();
                    #if EXTRUDERS > 5
                      if (g_uc_extruder_last_move[5] == 0) disable_E5();
                    #endif
                  #endif
                break;
                #if EXTRUDERS > 4
                  case 4:
                    enable_E4();
                    g_uc_extruder_last_move[4] = (BLOCK_BUFFER_SIZE) * 2;
                    if (g_uc_extruder_last_move[0] == 0) disable_E0();
                    if (g_uc_extruder_last_move[1] == 0) disable_E1();
                    if (g_uc_extruder_last_move[2] == 0) disable_E2();
                    if (g_uc_extruder_last_move[3] == 0) disable_E3();
                    #if EXTRUDERS > 5
                      if (g_uc_extruder_last_move[5] == 0) disable_E5();
                    #endif
                  break;
                  #if EXTRUDERS > 5
                    case 4:
                      enable_E5();
                      g_uc_extruder_last_move[5] = (BLOCK_BUFFER_SIZE) * 2;
                      if (g_uc_extruder_last_move[0] == 0) disable_E0();
                      if (g_uc_extruder_last_move[1] == 0) disable_E1();
                      if (g_uc_extruder_last_move[2] == 0) disable_E2();
                      if (g_uc_extruder_last_move[3] == 0) disable_E3();
                      if (g_uc_extruder_last_move[4] == 0) disable_E4();
                    break;
                  #endif // EXTRUDERS > 5
                #endif // EXTRUDERS > 4
              #endif // EXTRUDERS > 3
            #endif // EXTRUDERS > 2
          #endif // EXTRUDERS > 1
        }
      #else // enable all
        enable_E0();
        enable_E1();
        enable_E2();
        enable_E3();
        enable_E4();
        enable_E5();
      #endif
    #elif ENABLED(MKR6)
      switch(extruder) {
        case 0:
        case 1:
        case 2:
          enable_E0();
          break;
        case 3:
        case 4:
        case 5:
          enable_E1();
          break;
      }
    #elif ENABLED(MKR12)
      switch(extruder) {
        case 0:
        case 1:
        case 2:
          enable_E0();
          break;
        case 3:
        case 4:
        case 5:
          enable_E1();
          break;
        case 6:
        case 7:
        case 8:
          enable_E2();
          break;
        case 9:
        case 10:
        case 11:
          enable_E3();
          break;
      }
    #elif ENABLED(MKR4) && (EXTRUDERS == 2) && (DRIVER_EXTRUDERS == 1)
      enable_E0();
    #elif ENABLED(MKR4)
      switch(extruder) {
        case 0:
          enable_E0();
        break;
        case 1:
          enable_E1();
        break;
        case 2:
          enable_E0();
        break;
        case 3:
          enable_E1();
        break;
      }
    #elif ENABLED(DONDOLO_SINGLE_MOTOR)
      enable_E0();
    #endif
  }

  if (esteps)
    NOLESS(fr_mm_s, mechanics.min_feedrate_mm_s);
  else
    NOLESS(fr_mm_s, mechanics.min_travel_feedrate_mm_s);

  /**
   * This part of the code calculates the total length of the movement.
   * For cartesian bots, the X_AXIS is the real X movement and same for Y_AXIS.
   * But for corexy bots, that is not true. The "X_AXIS" and "Y_AXIS" motors (that should be named to A_AXIS
   * and B_AXIS) cannot be used for X and Y length, because A=X+Y and B=X-Y.
   * So we need to create other 2 "AXIS", named X_HEAD and Y_HEAD, meaning the real displacement of the Head.
   * Having the real displacement of the head, we can calculate the total movement length and apply the desired speed.
   */
  #if IS_CORE
    float delta_mm[Z_HEAD + 1];
    #if CORE_IS_XY
      delta_mm[X_HEAD] = dx * mechanics.steps_to_mm[A_AXIS];
      delta_mm[Y_HEAD] = dy * mechanics.steps_to_mm[B_AXIS];
      delta_mm[Z_AXIS] = dz * mechanics.steps_to_mm[Z_AXIS];
      delta_mm[A_AXIS] = da * mechanics.steps_to_mm[A_AXIS];
      delta_mm[B_AXIS] = CORESIGN(db) * mechanics.steps_to_mm[B_AXIS];
    #elif CORE_IS_XZ
      delta_mm[X_HEAD] = dx * mechanics.steps_to_mm[A_AXIS];
      delta_mm[Y_AXIS] = dy * mechanics.steps_to_mm[Y_AXIS];
      delta_mm[Z_HEAD] = dz * mechanics.steps_to_mm[C_AXIS];
      delta_mm[A_AXIS] = da * mechanics.steps_to_mm[A_AXIS];
      delta_mm[C_AXIS] = CORESIGN(dc) * mechanics.steps_to_mm[C_AXIS];
    #elif CORE_IS_YZ
      delta_mm[X_AXIS] = dx * mechanics.steps_to_mm[X_AXIS];
      delta_mm[Y_HEAD] = dy * mechanics.steps_to_mm[B_AXIS];
      delta_mm[Z_HEAD] = dz * mechanics.steps_to_mm[C_AXIS];
      delta_mm[B_AXIS] = db * mechanics.steps_to_mm[B_AXIS];
      delta_mm[C_AXIS] = CORESIGN(dc) * mechanics.steps_to_mm[C_AXIS];
    #endif
  #else
    float delta_mm[XYZE];
    delta_mm[X_AXIS] = dx * mechanics.steps_to_mm[X_AXIS];
    delta_mm[Y_AXIS] = dy * mechanics.steps_to_mm[Y_AXIS];
    delta_mm[Z_AXIS] = dz * mechanics.steps_to_mm[Z_AXIS];
  #endif
  delta_mm[E_AXIS] = esteps_float * mechanics.steps_to_mm[E_AXIS_N];

  if (block->steps[X_AXIS] < MIN_STEPS_PER_SEGMENT && block->steps[Y_AXIS] < MIN_STEPS_PER_SEGMENT && block->steps[Z_AXIS] < MIN_STEPS_PER_SEGMENT) {
    block->millimeters = FABS(delta_mm[E_AXIS]);
  }
  else if (!millimeters) {
    block->millimeters = SQRT(
      #if CORE_IS_XY
        sq(delta_mm[X_HEAD]) + sq(delta_mm[Y_HEAD]) + sq(delta_mm[Z_AXIS])
      #elif CORE_IS_XZ
        sq(delta_mm[X_HEAD]) + sq(delta_mm[Y_AXIS]) + sq(delta_mm[Z_HEAD])
      #elif CORE_IS_YZ
        sq(delta_mm[X_AXIS]) + sq(delta_mm[Y_HEAD]) + sq(delta_mm[Z_HEAD])
      #else
        sq(delta_mm[X_AXIS]) + sq(delta_mm[Y_AXIS]) + sq(delta_mm[Z_AXIS])
      #endif
    );
  }
  else
    block->millimeters = millimeters;

  #if ENABLED(LASER)

    block->laser_intensity = laser.intensity;
    block->laser_duration = laser.duration;
    block->laser_status = laser.status;
    block->laser_mode = laser.mode;

    // When operating in PULSED or RASTER modes, laser pulsing must operate in sync with movement.
    // Calculate steps between laser firings (steps_l) and consider that when determining largest
    // interval between steps for X, Y, Z, E, L to feed to the motion control code.
    if (laser.mode == RASTER || laser.mode == PULSED) {
      block->steps_l = labs(block->millimeters * laser.ppm);
      #if ENABLED(LASER_RASTER)
        for (uint8_t i = 0; i < LASER_MAX_RASTER_LINE; i++) {
          // Scale the image intensity based on the raster power.
          // 100% power on a pixel basis is 255, convert back to 255 = 100.
          #if ENABLED(LASER_REMAP_INTENSITY)
            const int NewRange = (laser.rasterlaserpower * 255.0 / 100.0 - LASER_REMAP_INTENSITY);
            float     NewValue = (float)(((((float)laser.raster_data[i] - 0) * NewRange) / 255.0) + LASER_REMAP_INTENSITY);
          #else
            const int NewRange = (laser.rasterlaserpower * 255.0 / 100.0);
            float     NewValue = (float)(((((float)laser.raster_data[i] - 0) * NewRange) / 255.0));
          #endif

          #if ENABLED(LASER_REMAP_INTENSITY)
            // If less than 7%, turn off the laser tube.
            if (NewValue <= LASER_REMAP_INTENSITY) NewValue = 0;
          #endif

          block->laser_raster_data[i] = NewValue;
        }
      #endif
    }
    else
      block->steps_l = 0;

    block->step_event_count = max(block->step_event_count, block->steps_l);

    if (laser.diagnostics && block->laser_status == LASER_ON)
      SERIAL_LM(ECHO, "Laser firing enabled");

  #endif // LASER

  const float inverse_millimeters = 1.0 / block->millimeters;  // Inverse millimeters to remove multiple divides

  // Calculate inverse time for this move. No divide by zero due to previous checks.
  // Example: At 120mm/s a 60mm move takes 0.5s. So this will give 2.0.
  float inverse_secs = fr_mm_s * inverse_millimeters;

  const uint8_t moves_queued = movesplanned();

  // Slow down when the buffer starts to empty, rather than wait at the corner for a buffer refill
  #if ENABLED(SLOWDOWN) || ENABLED(ULTRA_LCD) || ENABLED(XY_FREQUENCY_LIMIT)
    // Segment time im micro seconds
    uint32_t segment_time_us = LROUND(1000000.0 / inverse_secs);
  #endif

  #if ENABLED(SLOWDOWN)
    if (WITHIN(moves_queued, 2, (BLOCK_BUFFER_SIZE) / 2 - 1)) {
      if (segment_time_us < mechanics.min_segment_time_us) {
        // buffer is draining, add extra time.  The amount of time added increases if the buffer is still emptied more.
        const uint32_t nst = segment_time_us + LROUND(2 * (mechanics.min_segment_time_us - segment_time_us) / moves_queued);
        inverse_secs = 1000000.0 / nst;
        #if ENABLED(XY_FREQUENCY_LIMIT) || ENABLED(ULTRA_LCD)
          segment_time_us = nst;
        #endif
      }
    }
  #endif

  #if ENABLED(ULTRA_LCD)
    CRITICAL_SECTION_START
      block_buffer_runtime_us += segment_time_us;
    CRITICAL_SECTION_END
  #endif

  block->nominal_speed = block->millimeters * inverse_secs;           //   (mm/sec) Always > 0
  block->nominal_rate = CEIL(block->step_event_count * inverse_secs); // (step/sec) Always > 0

  #if ENABLED(FILAMENT_SENSOR)
    static float filwidth_e_count = 0, filwidth_delay_dist = 0;

    // FMM update ring buffer used for delay with filament measurements
    if (extruder == FILAMENT_SENSOR_EXTRUDER_NUM && filwidth_delay_index[1] >= 0) {  // only for extruder with filament sensor and if ring buffer is initialized

      constexpr int MMD_CM = MAX_MEASUREMENT_DELAY + 1, MMD_MM = MMD_CM * 10;

      // increment counters with next move in e axis
      filwidth_e_count += delta_mm[E_AXIS];
      filwidth_delay_dist += delta_mm[E_AXIS];

      // Only get new measurements on forward E movement
      if (!UNEAR_ZERO(filwidth_e_count)) {

        // Loop the delay distance counter (modulus by the mm length)
        while (filwidth_delay_dist >= MMD_MM) filwidth_delay_dist -= MMD_MM;

        // Convert into an index into the measurement array
        filwidth_delay_index[0] = int8_t(filwidth_delay_dist * 0.1);

        // If the index has changed (must have gone forward)...
        if (filwidth_delay_index[0] != filwidth_delay_index[1]) {
          filwidth_e_count = 0; // Reset the E movement counter
          const int8_t meas_sample = thermalManager.widthFil_to_size_ratio();
          do {
            filwidth_delay_index[1] = (filwidth_delay_index[1] + 1) % MMD_CM; // The next unused slot
            measurement_delay[filwidth_delay_index[1]] = meas_sample;         // Store the measurement
          } while (filwidth_delay_index[0] != filwidth_delay_index[1]);       // More slots to fill?
        }
      }
    }
  #endif

  // Calculate and limit speed in mm/sec for each axis
  float current_speed[NUM_AXIS], speed_factor = 1.0;  // factor <1 decreases speed
  LOOP_XYZE(i) {
    const float cs = FABS((current_speed[i] = delta_mm[i] * inverse_secs));
    if (i == E_AXIS) i += extruder;
    if (cs > mechanics.max_feedrate_mm_s[i]) NOMORE(speed_factor, mechanics.max_feedrate_mm_s[i] / cs);
  }

  // Max segment time in Âµs.
  #if ENABLED(XY_FREQUENCY_LIMIT)

    // Check and limit the xy direction change frequency
    const unsigned char direction_change = block->direction_bits ^ old_direction_bits;
    old_direction_bits = block->direction_bits;
    segment_time_us = LROUND((float)segment_time_us / speed_factor);

    uint32_t  xs0 = axis_segment_time_us[X_AXIS][0],
              xs1 = axis_segment_time_us[X_AXIS][1],
              xs2 = axis_segment_time_us[X_AXIS][2],
              ys0 = axis_segment_time_us[Y_AXIS][0],
              ys1 = axis_segment_time_us[Y_AXIS][1],
              ys2 = axis_segment_time_us[Y_AXIS][2];

    if (TEST(direction_change, X_AXIS)) {
      xs2 = axis_segment_time_us[X_AXIS][2] = xs1;
      xs1 = axis_segment_time_us[X_AXIS][1] = xs0;
      xs0 = 0;
    }
    xs0 = axis_segment_time_us[X_AXIS][0] = xs0 + segment_time_us;

    if (TEST(direction_change, Y_AXIS)) {
      ys2 = axis_segment_time_us[Y_AXIS][2] = axis_segment_time_us[Y_AXIS][1];
      ys1 = axis_segment_time_us[Y_AXIS][1] = axis_segment_time_us[Y_AXIS][0];
      ys0 = 0;
    }
    ys0 = axis_segment_time_us[Y_AXIS][0] = ys0 + segment_time_us;

    const uint32_t  max_x_segment_time = MAX3(xs0, xs1, xs2),
                    max_y_segment_time = MAX3(ys0, ys1, ys2),
                    min_xy_segment_time = min(max_x_segment_time, max_y_segment_time);
    if (min_xy_segment_time < MAX_FREQ_TIME_US) {
      const float low_sf = speed_factor * min_xy_segment_time / (MAX_FREQ_TIME_US);
      NOMORE(speed_factor, low_sf);
    }
  #endif // XY_FREQUENCY_LIMIT

  // Correct the speed
  if (speed_factor < 1.0) {
    LOOP_XYZE(i) current_speed[i] *= speed_factor;
    block->nominal_speed *= speed_factor;
    block->nominal_rate *= speed_factor;
  }

  // Compute and limit the acceleration rate for the trapezoid generator.
  const float steps_per_mm = block->step_event_count * inverse_millimeters;
  uint32_t accel;
  if (!block->steps[X_AXIS] && !block->steps[Y_AXIS] && !block->steps[Z_AXIS]) {
    // convert to: acceleration steps/sec^2
    accel = CEIL(mechanics.retract_acceleration[extruder] * steps_per_mm);
    #if ENABLED(LIN_ADVANCE)
      block->use_advance_lead = false;
    #endif
  }
  else {
    #define LIMIT_ACCEL_LONG(AXIS,INDX) do{ \
      if (block->steps[AXIS] && mechanics.max_acceleration_steps_per_s2[AXIS+INDX] < accel) { \
        const uint32_t comp = mechanics.max_acceleration_steps_per_s2[AXIS+INDX] * block->step_event_count; \
        if (accel * block->steps[AXIS] > comp) accel = comp / block->steps[AXIS]; \
      } \
    }while(0)

    #define LIMIT_ACCEL_FLOAT(AXIS,INDX) do{ \
      if (block->steps[AXIS] && mechanics.max_acceleration_steps_per_s2[AXIS+INDX] < accel) { \
        const float comp = (float)mechanics.max_acceleration_steps_per_s2[AXIS+INDX] * (float)block->step_event_count; \
        if ((float)accel * (float)block->steps[AXIS] > comp) accel = comp / (float)block->steps[AXIS]; \
      } \
    }while(0)

    // Start with print or travel acceleration
    accel = CEIL((esteps ? mechanics.acceleration : mechanics.travel_acceleration) * steps_per_mm);

    #if ENABLED(LIN_ADVANCE)
      /**
       *
       * Use LIN_ADVANCE for blocks if all these are true:
       *
       * esteps             : This is a print move, because we checked for A, B, C steps before.
       *
       * extruder_advance_K : There is an advance factor set.
       *
       * de > 0             : Extruder is running forward (e.g., for "Wipe while retracting" (Slic3r) or "Combing" (Cura) moves)
       */
      block->use_advance_lead =  esteps
                              && extruder_advance_K
                              && de > 0;

      if (block->use_advance_lead) {
        block->e_D_ratio = (target_float[E_AXIS] - position_float[E_AXIS]) /
          #if IS_KINEMATIC
            block->millimeters
          #else
            SQRT(sq(target_float[X_AXIS] - position_float[X_AXIS])
               + sq(target_float[Y_AXIS] - position_float[Y_AXIS])
               + sq(target_float[Z_AXIS] - position_float[Z_AXIS]))
          #endif
        ;

        // Check for unusual high e_D ratio to detect if a retract move was combined with the last print move due to min. steps per segment. Never execute this with advance!
        // This assumes no one will use a retract length of 0mm < retr_length < ~0.2mm and no one will print 100mm wide lines using 3mm filament or 35mm wide lines using 1.75mm filament.
        if (block->e_D_ratio > 3.0)
          block->use_advance_lead = false;
        else {
          const uint32_t max_accel_steps_per_s2 = mechanics.max_jerk[E_AXIS] / (extruder_advance_K * block->e_D_ratio) * steps_per_mm;
          #if ENABLED(LA_DEBUG)
            if (accel > max_accel_steps_per_s2)
              SERIAL_EM("Acceleration limited.");
          #endif
          NOMORE(accel, max_accel_steps_per_s2);
        }
      }
    #endif

    // Limit acceleration per axis
    if (block->step_event_count <= cutoff_long) {
      LIMIT_ACCEL_LONG(X_AXIS, 0);
      LIMIT_ACCEL_LONG(Y_AXIS, 0);
      LIMIT_ACCEL_LONG(Z_AXIS, 0);
      LIMIT_ACCEL_LONG(E_AXIS, extruder);
    }
    else {
      LIMIT_ACCEL_FLOAT(X_AXIS, 0);
      LIMIT_ACCEL_FLOAT(Y_AXIS, 0);
      LIMIT_ACCEL_FLOAT(Z_AXIS, 0);
      LIMIT_ACCEL_FLOAT(E_AXIS, extruder);
    }
  }
  block->acceleration_steps_per_s2 = accel;
  block->acceleration = accel / steps_per_mm;
  block->acceleration_rate = (long)(accel * (HAL_ACCELERATION_RATE));
  #if ENABLED(LIN_ADVANCE)
    if (block->use_advance_lead) {
      block->advance_speed = (HAL_TIMER_RATE) / (extruder_advance_K * block->e_D_ratio * block->acceleration * mechanics.axis_steps_per_mm[E_AXIS_N]);
      #if ENABLED(LA_DEBUG)
        if (extruder_advance_K * block->e_D_ratio * block->acceleration * 2 < block->nominal_speed * block->e_D_ratio)
          SERIAL_EM("More than 2 steps per eISR loop executed.");
        if (block->advance_speed < 200)
          SERIAL_EM("eISR running at > 10kHz.");
      #endif
    }
  #endif

  // Initial limit on the segment entry velocity
  float vmax_junction;

  /**
   * Start with a safe speed (from which the machine may halt to stop immediately).
   */

  // Exit speed limited by a jerk to full halt of a previous last segment
  static float previous_safe_speed;

  float safe_speed = block->nominal_speed;
  uint8_t limited = 0;
  LOOP_XYZE(i) {
    const float jerk = FABS(current_speed[i]),
                maxj = (i == E_AXIS) ? mechanics.max_jerk[i + extruder] : mechanics.max_jerk[i];

    if (jerk > maxj) {
      if (limited) {
        const float mjerk = maxj * block->nominal_speed;
        if (jerk * safe_speed > mjerk) safe_speed = mjerk / jerk;
      }
      else {
        ++limited;
        safe_speed = maxj;
      }
    }
  }

  if (moves_queued && !UNEAR_ZERO(previous_nominal_speed)) {
    // Estimate a maximum velocity allowed at a joint of two successive segments.
    // If this maximum velocity allowed is lower than the minimum of the entry / exit safe velocities,
    // then the machine is not coasting anymore and the safe entry / exit velocities shall be used.

    // The junction velocity will be shared between successive segments. Limit the junction velocity to their minimum.
    // Pick the smaller of the nominal speeds. Higher speed shall not be achieved at the junction during coasting.
    vmax_junction = min(block->nominal_speed, previous_nominal_speed);

    // Factor to multiply the previous / current nominal velocities to get componentwise limited velocities.
    float v_factor = 1;
    limited = 0;

    // Now limit the jerk in all axes.
    const float smaller_speed_factor = vmax_junction / previous_nominal_speed;
    LOOP_XYZE(axis) {
      // Limit an axis. We have to differentiate: coasting, reversal of an axis, full stop.
      float v_exit = previous_speed[axis] * smaller_speed_factor,
            v_entry = current_speed[axis];
      if (limited) {
        v_exit *= v_factor;
        v_entry *= v_factor;
      }

      // Calculate jerk depending on whether the axis is coasting in the same direction or reversing.
      const float jerk = (v_exit > v_entry)
          ? //                                  coasting             axis reversal
            ( (v_entry > 0 || v_exit < 0) ? (v_exit - v_entry) : max(v_exit, -v_entry) )
          : // v_exit <= v_entry                coasting             axis reversal
            ( (v_entry < 0 || v_exit > 0) ? (v_entry - v_exit) : max(-v_exit, v_entry) );

      const float maxj = (axis == E_AXIS) ? mechanics.max_jerk[axis + extruder] : mechanics.max_jerk[axis];
      if (jerk > maxj) {
        v_factor *= maxj / jerk;
        ++limited;
      }
    }
    if (limited) vmax_junction *= v_factor;
    // Now the transition velocity is known, which maximizes the shared exit / entry velocity while
    // respecting the jerk factors, it may be possible, that applying separate safe exit / entry velocities will achieve faster prints.
    const float vmax_junction_threshold = vmax_junction * 0.99f;
    if (previous_safe_speed > vmax_junction_threshold && safe_speed > vmax_junction_threshold) {
      // Not coasting. The machine will stop and start the movements anyway,
      // better to start the segment from start.
      SBI(block->flag, BLOCK_BIT_START_FROM_FULL_HALT);
      vmax_junction = safe_speed;
    }
  }
  else {
    SBI(block->flag, BLOCK_BIT_START_FROM_FULL_HALT);
    vmax_junction = safe_speed;
  }

  // Max entry speed of this block equals the max exit speed of the previous block.
  block->max_entry_speed = vmax_junction;

  // Initialize block entry speed. Compute based on deceleration to user-defined MINIMUM_PLANNER_SPEED.
  const float v_allowable = max_allowable_speed(-block->acceleration, MINIMUM_PLANNER_SPEED, block->millimeters);
  // If stepper ISR is disabled, this indicates buffer_segment wants to add a split block.
  // In this case start with the max. allowed speed to avoid an interrupted first move.
  block->entry_speed = STEPPER_ISR_ENABLED() ? MINIMUM_PLANNER_SPEED : min(vmax_junction, v_allowable);

  // Initialize planner efficiency flags
  // Set flag if block will always reach maximum junction speed regardless of entry/exit speeds.
  // If a block can de/ac-celerate from nominal speed to zero within the length of the block, then
  // the current block and next block junction speeds are guaranteed to always be at their maximum
  // junction speeds in deceleration and acceleration, respectively. This is due to how the current
  // block nominal speed limits both the current and next maximum junction speeds. Hence, in both
  // the reverse and forward planners, the corresponding block junction speed will always be at the
  // the maximum junction speed and may always be ignored for any speed reduction checks.
  block->flag |= block->nominal_speed <= v_allowable ? BLOCK_FLAG_RECALCULATE | BLOCK_FLAG_NOMINAL_LENGTH : BLOCK_FLAG_RECALCULATE;

  // Update previous path unit_vector and nominal speed
  COPY_ARRAY(previous_speed, current_speed);
  previous_nominal_speed = block->nominal_speed;
  previous_safe_speed = safe_speed;

  // Move buffer head
  block_buffer_head = next_buffer_head;

  // Update the position (only when a move was queued)
  static_assert(COUNT(target) > 1, "Parameter to buffer_steps must be (&target)[XYZE]!");
  COPY_ARRAY(position, target);
  #if ENABLED(LIN_ADVANCE)
    COPY_ARRAY(position_float, target_float);
  #endif

  recalculate();

} // buffer_steps()

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
void Planner::buffer_segment(const float &a, const float &b, const float &c, const float &e, const float &fr_mm_s, const uint8_t extruder, const float &millimeters/*=0.0*/) {

  // The target position of the tool in absolute steps
  // Calculate target position in absolute steps
  const int32_t target[XYZE] = {
    LROUND(a * mechanics.axis_steps_per_mm[X_AXIS]),
    LROUND(b * mechanics.axis_steps_per_mm[Y_AXIS]),
    LROUND(c * mechanics.axis_steps_per_mm[Z_AXIS]),
    LROUND(e * mechanics.axis_steps_per_mm[E_AXIS_N])
  };

  #if ENABLED(LIN_ADVANCE)
    const float target_float[XYZE] = { a, b, c, e };
  #endif

  // DRYRUN or Simulation prevents E moves from taking place
  if (printer.debugDryrun() || printer.debugSimulation()) {
    position[E_AXIS] = target[E_AXIS];
    #if ENABLED(LIN_ADVANCE)
      position_float[E_AXIS] = e;
    #endif
  }

  /* <-- add a slash to enable
    SERIAL_MV("  buffer_segment FR:", fr_mm_s);
    #if IS_KINEMATIC
      SERIAL_MV(" A:", a);
      SERIAL_MV(" (", position[A_AXIS]);
      SERIAL_MV("->", target[A_AXIS]);
      SERIAL_MV(") B:", b);
    #else
      SERIAL_MV(" X:", a);
      SERIAL_MV(" (", position[X_AXIS]);
      SERIAL_MV("->", target[X_AXIS]);
      SERIAL_MV(") Y:", b);
    #endif
    SERIAL_MV(" (", position[Y_AXIS]);
    SERIAL_MV("->", target[Y_AXIS]);
    #if MECH(DELTA)
      SERIAL_MV(") C:", c);
    #else
      SERIAL_MV(") Z:", c);
    #endif
    SERIAL_MV(" (", position[Z_AXIS]);
    SERIAL_MV("->", target[Z_AXIS]);
    SERIAL_MV(") E:", e);
    SERIAL_MV(" (", position[E_AXIS]);
    SERIAL_MV("->", target[E_AXIS]);
    SERIAL_EM(")");
  //*/

  // Simulation Mode no movement
  if (printer.debugSimulation()) {
    LOOP_XYZ(axis)
      position[axis] = target[axis];
  }

  #if ENABLED(LIN_ADVANCE)
    buffer_steps(target, target_float, fr_mm_s, extruder, millimeters);
  #else
    buffer_steps(target, fr_mm_s, extruder, millimeters);
  #endif

  stepper.wake_up();

} // buffer_segment()

/**
 * Add a new linear movement to the buffer.
 * The target is NOT translated to delta/scara
 *
 * Leveling will be applied to input on cartesians.
 * Kinematic machines should call buffer_line_kinematic (for leveled moves).
 * (Cartesians may also call buffer_line_kinematic.)
 *
 *  rx,ry,rz,e   - target position in mm or degrees
 *  fr_mm_s      - (target) speed of the move (mm/s)
 *  extruder     - target extruder
 *  millimeters  - the length of the movement, if known
 */
void Planner::buffer_line(ARG_X, ARG_Y, ARG_Z, const float &e, const float &fr_mm_s, const uint8_t extruder, const float millimeters/*=0.0*/) {
  #if PLANNER_LEVELING && (IS_CARTESIAN || IS_CORE)
    bedlevel.apply_leveling(rx, ry, rz);
  #endif
  #if ENABLED(ZWOBBLE)
    // Calculate ZWobble
    mechanics.insert_zwobble_correction(rz);
  #endif
  #if ENABLED(HYSTERESIS)
    // Calculate Hysteresis
    mechanics.insert_hysteresis_correction(rx, ry, rz, e);
  #endif
  buffer_segment(rx, ry, rz, e, fr_mm_s, extruder, millimeters);
}

/**
 * Add a new linear movement to the buffer.
 * The target is cartesian, it's translated to delta/scara if
 * needed.
 *
 *  cart      - x,y,z,e CARTESIAN target in mm
 *  fr_mm_s   - (target) speed of the move (mm/s)
 *  extruder  - target extruder
 */
void Planner::buffer_line_kinematic(const float cart[XYZE], const float &fr_mm_s, const uint8_t extruder, const float millimeters/*= 0.0*/) {
  #if PLANNER_LEVELING || ENABLED(ZWOBBLE) || ENABLED(HYSTERESIS)
    float raw[XYZ]={ cart[X_AXIS], cart[Y_AXIS], cart[Z_AXIS] };
    #if PLANNER_LEVELING
      bedlevel.apply_leveling(raw);
    #endif
    #if ENABLED(ZWOBBLE)
      // Calculate ZWobble
      mechanics.insert_zwobble_correction(raw[Z_AXIS]);
    #endif
    #if ENABLED(HYSTERESIS)
      // Calculate Hysteresis
      mechanics.insert_hysteresis_correction(raw[X_AXIS], raw[Y_AXIS], raw[Z_AXIS], cart[E_AXIS]);
    #endif
  #else
    const float * const raw = cart;
  #endif

  #if IS_KINEMATIC
    mechanics.Transform(raw);
    buffer_segment(mechanics.delta[A_AXIS], mechanics.delta[B_AXIS], mechanics.delta[C_AXIS], cart[E_AXIS], fr_mm_s, extruder, millimeters);
  #else
    buffer_segment(raw[X_AXIS], raw[Y_AXIS], raw[Z_AXIS], cart[E_AXIS], fr_mm_s, extruder, millimeters);
  #endif
}

/**
 * Sync from the stepper positions. (e.g., after an interrupted move)
 */
void Planner::sync_from_steppers() {
  LOOP_XYZE(i) {
    position[i] = stepper.position((AxisEnum)i);
    #if ENABLED(LIN_ADVANCE)
      position_float[i] = position[i] * mechanics.steps_to_mm[i + (i == E_AXIS ? tools.active_extruder : 0)];
    #endif
  }
}

/**
 * Abort Printing
 */
void Planner::abort() {

  // Abort the stepper routine
  DISABLE_STEPPER_INTERRUPT();

  // First update the planner's current position in the physical motor steps.
  sync_from_steppers();

  mechanics.set_current_from_steppers_for_axis(ALL_AXES);
  mechanics.set_destination_to_current();

  stepper.quick_stop();

  // Resets planner junction speeds. Assumes start from rest.
  previous_nominal_speed = 0.0;
  LOOP_XYZE(i)
    previous_speed[i] = 0.0;

  block_buffer_head = block_buffer_tail = 0;
  ZERO(block_buffer);
}

#if HAS_TEMP_HOTEND && ENABLED(AUTOTEMP)

  void Planner::autotemp_M104_M109() {
    if ((autotemp_enabled = parser.seen('F'))) autotemp_factor = parser.value_float();
    if (parser.seen('S')) autotemp_min = parser.value_celsius();
    if (parser.seen('B')) autotemp_max = parser.value_celsius();
  }

#endif
