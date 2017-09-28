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
 * Buffer movement commands and manage the mechanics.acceleration profile plan
 *
 * Derived from Grbl
 * Copyright (c) 2009-2011 Simen Svale Skogsrud
 *
 * The ring buffer implementation gleaned from the wiring_serial library by David A. Mellis.
 *
 *
 * Reasoning behind the mathematics in this module (in the key of 'Mathematica'):
 *
 * s == speed, a == mechanics.acceleration, t == time, d == distance
 *
 * Basic definitions:
 *   Speed[s_, a_, t_] := s + (a*t)
 *   Travel[s_, a_, t_] := Integrate[Speed[s, a, t], t]
 *
 * Distance to reach a specific speed with a constant mechanics.acceleration:
 *   Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, d, t]
 *   d -> (m^2 - s^2)/(2 a) --> estimate_acceleration_distance()
 *
 * Speed after a given distance of travel with constant mechanics.acceleration:
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

#include "../../base.h"

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

long Planner::position[NUM_AXIS] = { 0 };

uint32_t Planner::cutoff_long;

float Planner::previous_speed[NUM_AXIS],
      Planner::previous_nominal_speed;

#if ENABLED(DISABLE_INACTIVE_EXTRUDER)
  uint8_t Planner::g_uc_extruder_last_move[EXTRUDERS] = { 0 };
#endif // DISABLE_INACTIVE_EXTRUDER

#if ENABLED(XY_FREQUENCY_LIMIT)
  // Old direction bits. Used for speed calculations
  unsigned char Planner::old_direction_bits = 0;
  // Segment times (in µs). Used for speed calculations
  long Planner::axis_segment_time[2][3] = { {MAX_FREQ_TIME + 1, 0, 0}, {MAX_FREQ_TIME + 1, 0, 0} };
#endif

#if ENABLED(LIN_ADVANCE)
  float Planner::extruder_advance_k = LIN_ADVANCE_K,
        Planner::advance_ed_ratio = LIN_ADVANCE_E_D_RATIO,
        Planner::position_float[NUM_AXIS] = { 0 };
#endif

#if ENABLED(ULTRA_LCD)
  volatile uint32_t Planner::block_buffer_runtime_us = 0;
#endif

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
  uint32_t initial_rate = CEIL(block->nominal_rate * entry_factor),
           final_rate = CEIL(block->nominal_rate * exit_factor); // (steps per second)

  // Limit minimal step rate (Otherwise the timer will overflow.)
  NOLESS(initial_rate, MINIMAL_STEP_RATE);
  NOLESS(final_rate, MINIMAL_STEP_RATE);

  int32_t accel = block->acceleration_steps_per_s2,
          accelerate_steps = CEIL(estimate_acceleration_distance(initial_rate, block->nominal_rate, accel)),
          decelerate_steps = FLOOR(estimate_acceleration_distance(block->nominal_rate, final_rate, -accel)),
          plateau_steps = block->step_event_count - accelerate_steps - decelerate_steps;

  // Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
  // have to use intersection_distance() to calculate when to abort accel and start braking
  // in order to reach the final_rate exactly at the end of this block.
  if (plateau_steps < 0) {
    accelerate_steps = CEIL(intersection_distance(initial_rate, final_rate, accel, block->step_event_count));
    NOLESS(accelerate_steps, 0); // Check limits due to numerical round-off
    accelerate_steps = min((uint32_t)accelerate_steps, block->step_event_count);//(We can cast here to unsigned, because the above line ensures that we are above zero)
    plateau_steps = 0;
  }

  // block->accelerate_until = accelerate_steps;
  // block->decelerate_after = accelerate_steps+plateau_steps;

  CRITICAL_SECTION_START;  // Fill variables used by the stepper in a critical section
  if (!TEST(block->flag, BLOCK_BIT_BUSY)) { // Don't update variables if block is busy.
    block->accelerate_until = accelerate_steps;
    block->decelerate_after = accelerate_steps + plateau_steps;
    block->initial_rate = initial_rate;
    block->final_rate = final_rate;
    #if ENABLED(ADVANCE)
      block->initial_advance = block->advance * sq(entry_factor);
      block->final_advance = block->advance * sq(exit_factor);
    #endif
  }
  CRITICAL_SECTION_END;
}

// The kernel called by recalculate() when scanning the plan from last to first entry.
void Planner::reverse_pass_kernel(block_t* const current, const block_t *next) {
  if (!current || !next) return;
  // If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
  // If not, block in state of mechanics.acceleration or deceleration. Reset entry speed to maximum and
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

    block_t* block[3] = { NULL, NULL, NULL };

    // Make a local copy of block_buffer_tail, because the interrupt can alter it
    //CRITICAL_SECTION_START;
    uint8_t tail = block_buffer_tail;
    //CRITICAL_SECTION_END

    uint8_t b = BLOCK_MOD(block_buffer_head - 3);
    while (b != tail) {
      if (block[0] && TEST(block[0]->flag, BLOCK_BIT_START_FROM_FULL_HALT)) break;
      b = prev_block_index(b);
      block[2] = block[1];
      block[1] = block[0];
      block[0] = &block_buffer[b];
      reverse_pass_kernel(block[1], block[2]);
    }
  }
}

// The kernel called by recalculate() when scanning the plan from first to last entry.
void Planner::forward_pass_kernel(const block_t* previous, block_t* const current) {
  if (!previous) return;

  // If the previous block is an mechanics.acceleration block, but it is not long enough to complete the
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
        float nom = current->nominal_speed;
        calculate_trapezoid_for_block(current, current->entry_speed / nom, next->entry_speed / nom);
        CBI(current->flag, BLOCK_BIT_RECALCULATE); // Reset current only to ensure next trapezoid is computed
      }
    }
    block_index = next_block_index(block_index);
  }
  // Last/newest block in buffer. Exit speed is set with MINIMUM_PLANNER_SPEED. Always recalculated.
  if (next) {
    float nom = next->nominal_speed;
    calculate_trapezoid_for_block(next, next->entry_speed / nom, (MINIMUM_PLANNER_SPEED) / nom);
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
 *         deceleration than the one, true constant mechanics.acceleration.
 *
 *   2. Go over every block in chronological order...
 *
 *      Dial down junction speed reduction values if:
 *      a. The speed increase within one block would require faster
 *         mechanics.acceleration than the one, true constant mechanics.acceleration.
 *
 * After that, all blocks will have an entry_factor allowing all speed changes to
 * be performed using only the one, true constant mechanics.acceleration, and where no junction
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

#endif //AUTOTEMP

/**
 * Maintain fans, paste extruder pressure,
 */
void Planner::check_axes_activity() {
  unsigned char axis_active[NUM_AXIS] = { 0 };

  #if ENABLED(BARICUDA)
    #if HAS_HEATER_1
      unsigned char tail_valve_pressure = ValvePressure;
    #endif
    #if HAS_HEATER_2
      unsigned char tail_e_to_p_pressure = EtoPPressure;
    #endif
  #endif

  if (blocks_queued()) {

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
    if (!axis_active[E_AXIS]) {
      disable_E0();
      disable_E1();
      disable_E2();
      disable_E3();
      disable_E4();
      disable_E5();
    }
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
 * Planner::_buffer_line
 *
 * Add a new linear movement to the buffer.
 *
 * Leveling and kinematics should be applied ahead of calling this.
 *
 *  a,b,c,e     - target positions in mm or degrees
 *  fr_mm_s     - (target) speed of the move
 *  extruder    - target extruder
 */
void Planner::_buffer_line(const float &a, const float &b, const float &c, const float &e, float fr_mm_s, const uint8_t extruder) {

  // The target position of the tool in absolute steps
  // Calculate target position in absolute steps
  // this should be done after the wait, because otherwise a M92 code within the gcode disrupts this calculation somehow
  const long target[XYZE] = {
    LROUND(a * mechanics.axis_steps_per_mm[X_AXIS]),
    LROUND(b * mechanics.axis_steps_per_mm[Y_AXIS]),
    LROUND(c * mechanics.axis_steps_per_mm[Z_AXIS]),
    LROUND(e * mechanics.axis_steps_per_mm[E_AXIS_N])
  };

  #if ENABLED(LIN_ADVANCE)
    const float mm_D_float = SQRT(sq(a - position_float[X_AXIS]) + sq(b - position_float[Y_AXIS]));
  #endif

  const long  dx = target[X_AXIS] - position[X_AXIS],
              dy = target[Y_AXIS] - position[Y_AXIS],
              dz = target[Z_AXIS] - position[Z_AXIS];

  // DRYRUN ignores all temperature constraints and assures that the extruder is instantly satisfied
  if (DEBUGGING(DRYRUN)) {
    position[E_AXIS] = target[E_AXIS];
    #if ENABLED(LIN_ADVANCE)
      position_float[E_AXIS] = e;
    #endif
  }

  long de = target[E_AXIS] - position[E_AXIS];

  #if ENABLED(LIN_ADVANCE)
    float de_float = e - position_float[E_AXIS];
  #endif

  #if ENABLED(PREVENT_COLD_EXTRUSION)
    if (de
      #if HAS_MULTI_MODE
        && printer.mode == PRINTER_MODE_FFF
      #endif
    ) {
      #if ENABLED(NPR2)
        if (extruder != 1)
      #endif
        {
          if (thermalManager.tooColdToExtrude(extruder)) {
            position[E_AXIS] = target[E_AXIS]; // Behave as if the move really took place, but ignore E part
            de = 0; // no difference
            #if ENABLED(LIN_ADVANCE)
              position_float[E_AXIS] = e;
              de_float = 0;
            #endif
            SERIAL_LM(ER, MSG_ERR_COLD_EXTRUDE_STOP);
          }
        }

      #if ENABLED(PREVENT_LENGTHY_EXTRUDE)
        if (labs(de) > (int32_t)mechanics.axis_steps_per_mm[E_AXIS_N] * (EXTRUDE_MAXLENGTH)) {
          #if ENABLED(EASY_LOAD)
            if (!printer.allow_lengthy_extrude_once) {
          #endif
          position[E_AXIS] = target[E_AXIS]; // Behave as if the move really took place, but ignore E part
          de = 0; // no difference
          #if ENABLED(LIN_ADVANCE)
            position_float[E_AXIS] = e;
            de_float = 0;
          #endif
          SERIAL_LM(ER, MSG_ERR_LONG_EXTRUDE_STOP);
          #if ENABLED(EASY_LOAD)
            }
            printer.allow_lengthy_extrude_once = false;
          #endif
        }
      #endif // PREVENT_LENGTHY_EXTRUDE
    }
  #endif // PREVENT_COLD_EXTRUSION

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

  const float esteps_float = de * tools.volumetric_multiplier[extruder] * tools.flow_percentage[extruder] * 0.01;
  const int32_t esteps = abs(esteps_float) + 0.5;

  // Calculate the buffer head after we push this byte
  const int8_t next_buffer_head = next_block_index(block_buffer_head);

  // If the buffer is full: good! That means we are well ahead of the robot.
  // Rest here until there is room in the buffer.
  while (block_buffer_tail == next_buffer_head) printer.idle();

  // Prepare to set up new block
  block_t* block = &block_buffer[block_buffer_head];

  // Clear the block flags
  block->flag = 0;

  // Set direction bits
  block->direction_bits = dirb;

  // Number of steps for each axis
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

  // For a mixing extruder, get steps for each
  #if ENABLED(COLOR_MIXING_EXTRUDER)
    for (uint8_t i = 0; i < MIXING_STEPPERS; i++)
      block->mix_event_count[i] = tools.mixing_factor[i] * block->step_event_count;
  #endif

  #if ENABLED(BARICUDA)
    block->valve_pressure   = printer.baricuda_valve_pressure;
    block->e_to_p_pressure  = printer.baricuda_e_to_p_pressure;
  #endif

  block->active_extruder = extruder;

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
    #elif ENABLED(NPR2)
      enable_E0();
      enable_E1();
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
    float delta_mm[E_AXIS + 1];
    delta_mm[X_AXIS] = dx * mechanics.steps_to_mm[X_AXIS];
    delta_mm[Y_AXIS] = dy * mechanics.steps_to_mm[Y_AXIS];
    delta_mm[Z_AXIS] = dz * mechanics.steps_to_mm[Z_AXIS];
  #endif
  delta_mm[E_AXIS] = esteps_float * mechanics.steps_to_mm[E_AXIS_N];

  if (block->steps[X_AXIS] < MIN_STEPS_PER_SEGMENT && block->steps[Y_AXIS] < MIN_STEPS_PER_SEGMENT && block->steps[Z_AXIS] < MIN_STEPS_PER_SEGMENT) {
    block->millimeters = FABS(delta_mm[E_AXIS]);
  }
  else {
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
    }
    else
      block->steps_l = 0;

    block->step_event_count = max(block->step_event_count, block->steps_l);

    if (laser.diagnostics && block->laser_status == LASER_ON)
      SERIAL_LM(ECHO, "Laser firing enabled");

  #endif // LASER

  float inverse_millimeters = 1.0 / block->millimeters;  // Inverse millimeters to remove multiple divides

  // Calculate moves/second for this move. No divide by zero due to previous checks.
  float inverse_mm_s = fr_mm_s * inverse_millimeters;

  const uint8_t moves_queued = movesplanned();

  // Slow down when the buffer starts to empty, rather than wait at the corner for a buffer refill
  #if ENABLED(SLOWDOWN) || ENABLED(ULTRA_LCD) || defined(XY_FREQUENCY_LIMIT)
    // Segment time im micro seconds
    unsigned long segment_time = LROUND(1000000.0 / inverse_mm_s);
  #endif

  #if ENABLED(SLOWDOWN)
    if (moves_queued > 1 && moves_queued < (BLOCK_BUFFER_SIZE) / 2) {
      if (segment_time < mechanics.min_segment_time) {
        // buffer is draining, add extra time.  The amount of time added increases if the buffer is still emptied more.
        inverse_mm_s = 1000000.0 / (segment_time + LROUND(2 * (mechanics.min_segment_time - segment_time) / moves_queued));
        #if ENABLED(XY_FREQUENCY_LIMIT)
          segment_time = LROUND(1000000.0 / inverse_mm_s);
        #endif
      }
    }
  #endif

  #if ENABLED(ULTRA_LCD)
    CRITICAL_SECTION_START
      block_buffer_runtime_us += segment_time;
    CRITICAL_SECTION_END
  #endif

  block->nominal_speed = block->millimeters * inverse_mm_s; // (mm/sec) Always > 0
  block->nominal_rate = CEIL(block->step_event_count * inverse_mm_s); // (step/sec) Always > 0

  #if ENABLED(FILAMENT_SENSOR)
    static float filwidth_e_count = 0, filwidth_delay_dist = 0;

    // FMM update ring buffer used for delay with filament measurements
    if (extruder == FILAMENT_SENSOR_EXTRUDER_NUM && filwidth_delay_index[1] >= 0) {  // only for extruder with filament sensor and if ring buffer is initialized

      const int MMD_CM = MAX_MEASUREMENT_DELAY + 1, MMD_MM = MMD_CM * 10;

      // increment counters with next move in e axis
      filwidth_e_count += delta_mm[E_AXIS];
      filwidth_delay_dist += delta_mm[E_AXIS];

      // Only get new measurements on forward E movement
      if (filwidth_e_count > 0.0001) {

        // Loop the delay distance counter (modulus by the mm length)
        while (filwidth_delay_dist >= MMD_MM) filwidth_delay_dist -= MMD_MM;

        // Convert into an index into the measurement array
        filwidth_delay_index[0] = int8_t(filwidth_delay_dist * 0.1);

        // If the index has changed (must have gone forward)...
        if (filwidth_delay_index[0] != filwidth_delay_index[1]) {
          filwidth_e_count = 0; // Reset the E movement counter
          const uint8_t meas_sample = thermalManager.widthFil_to_size_ratio() - 100; // Subtract 100 to reduce magnitude - to store in a signed char
          do {
            filwidth_delay_index[1] = (filwidth_delay_index[1] + 1) % MMD_CM; // The next unused slot
            measurement_delay[filwidth_delay_index[1]] = meas_sample;         // Store the measurement
          } while (filwidth_delay_index[0] != filwidth_delay_index[1]);       // More slots to fill?
        }
      }
    }
  #endif

  // Calculate and limit speed in mm/sec for each axis
  float current_speed[NUM_AXIS], speed_factor = 1.0; // factor <1 decreases speed
  LOOP_XYZE(i) {
    const float cs = FABS(current_speed[i] = delta_mm[i] * inverse_mm_s);
    if (i == E_AXIS) i += extruder;
    if (cs > mechanics.max_feedrate_mm_s[i]) NOMORE(speed_factor, mechanics.max_feedrate_mm_s[i] / cs);
  }

  // Max segment time in µs.
  #if ENABLED(XY_FREQUENCY_LIMIT)

    // Check and limit the xy direction change frequency
    const unsigned char direction_change = block->direction_bits ^ old_direction_bits;
    old_direction_bits = block->direction_bits;
    segment_time = LROUND((float)segment_time / speed_factor);

    long xs0 = axis_segment_time[X_AXIS][0],
         xs1 = axis_segment_time[X_AXIS][1],
         xs2 = axis_segment_time[X_AXIS][2],
         ys0 = axis_segment_time[Y_AXIS][0],
         ys1 = axis_segment_time[Y_AXIS][1],
         ys2 = axis_segment_time[Y_AXIS][2];

    if (TEST(direction_change, X_AXIS)) {
      xs2 = axis_segment_time[X_AXIS][2] = xs1;
      xs1 = axis_segment_time[X_AXIS][1] = xs0;
      xs0 = 0;
    }
    xs0 = axis_segment_time[X_AXIS][0] = xs0 + segment_time;

    if (TEST(direction_change, Y_AXIS)) {
      ys2 = axis_segment_time[Y_AXIS][2] = axis_segment_time[Y_AXIS][1];
      ys1 = axis_segment_time[Y_AXIS][1] = axis_segment_time[Y_AXIS][0];
      ys0 = 0;
    }
    ys0 = axis_segment_time[Y_AXIS][0] = ys0 + segment_time;

    const long  max_x_segment_time = MAX3(xs0, xs1, xs2),
                max_y_segment_time = MAX3(ys0, ys1, ys2),
                min_xy_segment_time = min(max_x_segment_time, max_y_segment_time);
    if (min_xy_segment_time < MAX_FREQ_TIME) {
      const float low_sf = speed_factor * min_xy_segment_time / (MAX_FREQ_TIME);
      NOMORE(speed_factor, low_sf);
    }
  #endif // XY_FREQUENCY_LIMIT

  // Correct the speed
  if (speed_factor < 1.0) {
    LOOP_XYZE(i) current_speed[i] *= speed_factor;
    block->nominal_speed *= speed_factor;
    block->nominal_rate *= speed_factor;
  }

  // Compute and limit the mechanics.acceleration rate for the trapezoid generator.
  const float steps_per_mm = block->step_event_count * inverse_millimeters;
  uint32_t accel;
  if (!block->steps[X_AXIS] && !block->steps[Y_AXIS] && !block->steps[Z_AXIS]) {
    // convert to: mechanics.acceleration steps/sec^2
    accel = CEIL(mechanics.retract_acceleration[extruder] * steps_per_mm);
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

    // Start with print or travel mechanics.acceleration
    accel = CEIL((esteps ? mechanics.acceleration : mechanics.travel_acceleration) * steps_per_mm);

    // Limit mechanics.acceleration per axis
    if (block->step_event_count <= cutoff_long){
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

  #if ENABLED(ARDUINO_ARCH_SAM)
    block->acceleration_rate = (long)(accel * (4294967296.0 / (HAL_STEPPER_TIMER_RATE)));
  #else
    block->acceleration_rate = (long)(accel * 16777216.0 / (HAL_STEPPER_TIMER_RATE));
  #endif

  // Initial limit on the segment entry velocity
  float vmax_junction;

  #if 0  // Use old jerk for now

    float junction_deviation = 0.1;

    // Compute path unit vector
    double unit_vec[XYZ] = {
      delta_mm[X_AXIS] * inverse_millimeters,
      delta_mm[Y_AXIS] * inverse_millimeters,
      delta_mm[Z_AXIS] * inverse_millimeters
    };

    /*
       Compute maximum allowable entry speed at junction by centripetal mechanics.acceleration approximation.

       Let a circle be tangent to both previous and current path line segments, where the junction
       deviation is defined as the distance from the junction to the closest edge of the circle,
       collinear with the circle center.

       The circular segment joining the two paths represents the path of centripetal mechanics.acceleration.
       Solve for max velocity based on max mechanics.acceleration about the radius of the circle, defined
       indirectly by junction deviation.

       This may be also viewed as path width or max_jerk in the previous grbl version. This approach
       does not actually deviate from path, but used as a robust way to compute cornering speeds, as
       it takes into account the nonlinearities of both the junction angle and junction velocity.
     */

    vmax_junction = MINIMUM_PLANNER_SPEED; // Set default max junction speed

    // Skip first block or when previous_nominal_speed is used as a flag for homing and offset cycles.
    if (block_buffer_head != block_buffer_tail && previous_nominal_speed > 0.0) {
      // Compute cosine of angle between previous and current path. (prev_unit_vec is negative)
      // NOTE: Max junction velocity is computed without sin() or acos() by trig half angle identity.
      float cos_theta = - previous_unit_vec[X_AXIS] * unit_vec[X_AXIS]
                        - previous_unit_vec[Y_AXIS] * unit_vec[Y_AXIS]
                        - previous_unit_vec[Z_AXIS] * unit_vec[Z_AXIS];
      // Skip and use default max junction speed for 0 degree acute junction.
      if (cos_theta < 0.95) {
        vmax_junction = min(previous_nominal_speed, block->nominal_speed);
        // Skip and avoid divide by zero for straight junctions at 180 degrees. Limit to min() of nominal speeds.
        if (cos_theta > -0.95) {
          // Compute maximum junction velocity based on maximum mechanics.acceleration and junction deviation
          float sin_theta_d2 = SQRT(0.5 * (1.0 - cos_theta)); // Trig half angle identity. Always positive.
          NOMORE(vmax_junction, SQRT(block->acceleration * junction_deviation * sin_theta_d2 / (1.0 - sin_theta_d2)));
        }
      }
    }
  #endif

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

  if (moves_queued > 1 && previous_nominal_speed > 0.0001) {
    // Estimate a maximum velocity allowed at a joint of two successive segments.
    // If this maximum velocity allowed is lower than the minimum of the entry / exit safe velocities,
    // then the machine is not coasting anymore and the safe entry / exit velocities shall be used.

    // The junction velocity will be shared between successive segments. Limit the junction velocity to their minimum.
    bool prev_speed_larger = previous_nominal_speed > block->nominal_speed;
    float smaller_speed_factor = prev_speed_larger ? (block->nominal_speed / previous_nominal_speed) : (previous_nominal_speed / block->nominal_speed);
    // Pick the smaller of the nominal speeds. Higher speed shall not be achieved at the junction during coasting.
    vmax_junction = prev_speed_larger ? block->nominal_speed : previous_nominal_speed;
    // Factor to multiply the previous / current nominal velocities to get componentwise limited velocities.
    float v_factor = 1.f;
    limited = 0;
    // Now limit the jerk in all axes.
    LOOP_XYZE(axis) {
      // Limit an axis. We have to differentiate: coasting, reversal of an axis, full stop.
      float v_exit = previous_speed[axis], v_entry = current_speed[axis];
      const float maxj = (axis == E_AXIS) ? mechanics.max_jerk[axis + extruder] : mechanics.max_jerk[axis];

      if (prev_speed_larger) v_exit *= smaller_speed_factor;
      if (limited) {
        v_exit *= v_factor;
        v_entry *= v_factor;
      }
      // Calculate jerk depending on whether the axis is coasting in the same direction or reversing.
      const float jerk = (v_exit > v_entry)
          ? //                                  coasting             axis reversal
            ( (v_entry > 0.f || v_exit < 0.f) ? (v_exit - v_entry) : max(v_exit, -v_entry) )
          : // v_exit <= v_entry                coasting             axis reversal
            ( (v_entry < 0.f || v_exit > 0.f) ? (v_entry - v_exit) : max(-v_exit, v_entry) );

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
  block->entry_speed = min(vmax_junction, v_allowable);

  // Initialize planner efficiency flags
  // Set flag if block will always reach maximum junction speed regardless of entry/exit speeds.
  // If a block can de/ac-celerate from nominal speed to zero within the length of the block, then
  // the current block and next block junction speeds are guaranteed to always be at their maximum
  // junction speeds in deceleration and mechanics.acceleration, respectively. This is due to how the current
  // block nominal speed limits both the current and next maximum junction speeds. Hence, in both
  // the reverse and forward planners, the corresponding block junction speed will always be at the
  // the maximum junction speed and may always be ignored for any speed reduction checks.
  block->flag |= BLOCK_FLAG_RECALCULATE | (block->nominal_speed <= v_allowable ? BLOCK_FLAG_NOMINAL_LENGTH : 0);

  // Update previous path unit_vector and nominal speed
  COPY_ARRAY(previous_speed, current_speed);
  previous_nominal_speed = block->nominal_speed;
  previous_safe_speed = safe_speed;

  #if ENABLED(LIN_ADVANCE)

    //
    // Use LIN_ADVANCE for blocks if all these are true:
    //
    // esteps                                          : We have E steps todo (a printing move)
    // 
    // block->steps[X_AXIS] || block->steps[Y_AXIS]    : We have a movement in XY direction (i.e., not retract / prime).
    // 
    // extruder_advance_k                              : There is an advance factor set.
    // 
    // block->steps[E_AXIS] != block->step_event_count : A problem occurs if the move before a retract is too small.
    //                                                   In that case, the retract and move will be executed together.
    //                                                   This leads to too many advance steps due to a huge e_acceleration.
    //                                                   The math is good, but we must avoid retract moves with advance!
    // de_float > 0.0                                  : Extruder is running forward (e.g., for "Wipe while retracting" (Slic3r) or "Combing" (Cura) moves)
    //
    block->use_advance_lead =  esteps
                            && (block->steps[X_AXIS] || block->steps[Y_AXIS])
                            && extruder_advance_k
                            && (uint32_t)esteps != block->step_event_count
                            && de_float > 0.0;
    if (block->use_advance_lead)
      block->abs_adv_steps_multiplier8 = LROUND(
        extruder_advance_k
        * (UNEAR_ZERO(advance_ed_ratio) ? de_float / mm_D_float : advance_ed_ratio) // Use the fixed ratio, if set
        * (block->nominal_speed / (float)block->nominal_rate)
        * mechanics.axis_steps_per_mm[E_AXIS_N] * 256.0
      );

  #elif ENABLED(ADVANCE)

    // Calculate advance rate
    if (esteps && (block->steps[X_AXIS] || block->steps[Y_AXIS] || block->steps[Z_AXIS])) {
      const long acc_dist = estimate_acceleration_distance(0, block->nominal_rate, block->acceleration_steps_per_s2);
      const float advance = ((STEPS_PER_CUBIC_MM_E) * (EXTRUDER_ADVANCE_K)) * HYPOT(current_speed[E_AXIS], EXTRUSION_AREA) * 256;
      block->advance = advance;
      block->advance_rate = acc_dist ? advance / (float)acc_dist : 0;
    }
    else
      block->advance_rate = block->advance = 0;

    /**
    SERIAL_SMV(ECHO, "advance :", block->advance/256);
    SERIAL_EMV("advance rate :", block->advance_rate/256);
    */

  #endif // ADVANCE or LIN_ADVANCE

  calculate_trapezoid_for_block(block, block->entry_speed / block->nominal_speed, safe_speed / block->nominal_speed);

  // Move buffer head
  block_buffer_head = next_buffer_head;

  // Update the position (only when a move was queued)
  COPY_ARRAY(position, target);
  #if ENABLED(LIN_ADVANCE)
    position_float[X_AXIS] = a;
    position_float[Y_AXIS] = b;
    position_float[Z_AXIS] = c;
    position_float[E_AXIS] = e;
  #endif

  recalculate();

  stepper.wake_up();

} // _buffer_line()

/**
 * Add a new linear movement to the buffer.
 * The target is NOT translated to delta/scara
 *
 * Leveling will be applied to input on cartesians.
 * Kinematic machines should call buffer_line_kinematic (for leveled moves).
 * (Cartesians may also call buffer_line_kinematic.)
 *
 *  lx,ly,lz,e  - target position in mm or degrees
 *  fr_mm_s     - (target) speed of the move (mm/s)
 *  extruder    - target extruder
 */
void Planner::buffer_line(ARG_X, ARG_Y, ARG_Z, const float &e, const float &fr_mm_s, const uint8_t extruder) {
  #if PLANNER_LEVELING && (IS_CARTESIAN || IS_CORE)
    bedlevel.apply_leveling(lx, ly, lz);
  #endif
  #if ENABLED(ZWOBBLE)
    // Calculate ZWobble
    mechanics.insert_zwobble_correction(lz);
  #endif
  #if ENABLED(HYSTERESIS)
    // Calculate Hysteresis
    mechanics.insert_hysteresis_correction(lx, ly, lz, e);
  #endif
  _buffer_line(lx, ly, lz, e, fr_mm_s, extruder);
}

/**
 * Add a new linear movement to the buffer.
 * The target is cartesian, it's translated to delta/scara if
 * needed.
 *
 *  ltarget   - x,y,z,e CARTESIAN target in mm
 *  fr_mm_s   - (target) speed of the move (mm/s)
 *  extruder  - target extruder
 */
void Planner::buffer_line_kinematic(const float ltarget[XYZE], const float &fr_mm_s, const uint8_t extruder) {
  #if PLANNER_LEVELING || ENABLED(ZWOBBLE) || ENABLED(HYSTERESIS)
    float lpos[XYZ]={ ltarget[X_AXIS], ltarget[Y_AXIS], ltarget[Z_AXIS] };
    #if PLANNER_LEVELING
      bedlevel.apply_leveling(lpos);
    #endif
    #if ENABLED(ZWOBBLE)
      // Calculate ZWobble
      mechanics.insert_zwobble_correction(lpos[Z_AXIS]);
    #endif
    #if ENABLED(HYSTERESIS)
      // Calculate Hysteresis
      mechanics.insert_hysteresis_correction(lpos[X_AXIS], lpos[Y_AXIS], lpos[Z_AXIS], ltarget[E_AXIS]);
    #endif
  #else
    const float * const lpos = ltarget;
  #endif

  #if IS_KINEMATIC
    mechanics.Transform(lpos);
    _buffer_line(mechanics.delta[A_AXIS], mechanics.delta[B_AXIS], mechanics.delta[C_AXIS], ltarget[E_AXIS], fr_mm_s, extruder);
  #else
    _buffer_line(lpos[X_AXIS], lpos[Y_AXIS], lpos[Z_AXIS], ltarget[E_AXIS], fr_mm_s, extruder);
  #endif
}

/**
 * Sync from the stepper positions. (e.g., after an interrupted move)
 */
void Planner::sync_from_steppers() {
  LOOP_XYZE(i) position[i] = stepper.position((AxisEnum)i);
  #if ENABLED(LIN_ADVANCE)
    LOOP_XYZE(i) position_float[i] = stepper.position((AxisEnum)i) * (i == E_AXIS ? mechanics.steps_to_mm[E_INDEX] : mechanics.steps_to_mm[i]);
  #endif
}

#if HAS_TEMP_HOTEND && ENABLED(AUTOTEMP)

  void Planner::autotemp_M104_M109() {
    autotemp_enabled = parser.seen('F');
    if (autotemp_enabled) autotemp_factor = parser.value_celsius_diff();
    if (parser.seen('S')) autotemp_min = parser.value_celsius();
    if (parser.seen('B')) autotemp_max = parser.value_celsius();
  }

#endif
