/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2016 Alberto Cotronei @MagoKimbra
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

#ifndef __SAM3X8E__
  #include "speed_lookuptable.h"
#endif

#if HAS(DIGIPOTSS)
  #include <SPI.h>
#endif

Stepper stepper; // Singleton

// public:

block_t* Stepper::current_block = NULL;  // A pointer to the block currently being traced

#if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
  #if ENABLED(ABORT_ON_ENDSTOP_HIT_INIT)
    bool Stepper:abort_on_endstop_hit = ABORT_ON_ENDSTOP_HIT_INIT;
  #else
    bool Stepper:abort_on_endstop_hit = false;
  #endif
#endif

#if ENABLED(Z_DUAL_ENDSTOPS)
  bool Stepper::performing_homing = false;
#endif

// private:

unsigned char Stepper::last_direction_bits = 0;        // The next stepping-bits to be output
unsigned int Stepper::cleaning_buffer_counter = 0;

#if ENABLED(Z_DUAL_ENDSTOPS)
  bool Stepper::locked_z_motor = false;
  bool Stepper::locked_z2_motor = false;
#endif

long  Stepper::counter_X = 0,
      Stepper::counter_Y = 0,
      Stepper::counter_Z = 0,
      Stepper::counter_E = 0;

volatile uint32_t Stepper::step_events_completed = 0; // The number of step events executed in the current block

#if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)
  unsigned char Stepper::old_OCR0A;
  volatile unsigned char Stepper::eISR_Rate = 200; // Keep the ISR at a low rate until needed
  #if ENABLED(LIN_ADVANCE)
    volatile int Stepper::e_steps[EXTRUDERS];
    int Stepper::extruder_advance_k = LIN_ADVANCE_K,
        Stepper::final_estep_rate,
        Stepper::current_estep_rate[EXTRUDERS];
        Stepper::current_adv_steps[EXTRUDERS];
  #else
    long  Stepper::e_steps[EXTRUDERS],
          Stepper::final_advance = 0,
          Stepper::old_advance = 0,
          Stepper::advance_rate,
          Stepper::advance;
  #endif
#endif // ADVANCE or LIN_ADVANCE

long Stepper::acceleration_time, Stepper::deceleration_time;

volatile long Stepper::count_position[NUM_AXIS] = { 0 };
volatile signed char Stepper::count_direction[NUM_AXIS] = { 1, 1, 1, 1 };

#if ENABLED(COLOR_MIXING_EXTRUDER)
  long Stepper::counter_m[MIXING_STEPPERS];
#endif

#ifdef __SAM3X8E__
  uint32_t Stepper::acc_step_rate; // needed for deceleration start point
  uint8_t Stepper::step_loops, Stepper::step_loops_nominal;
  uint32_t Stepper::OCR1A_nominal;
#else
  uint16_t Stepper::acc_step_rate; // needed for deceleration start point
  uint8_t Stepper::step_loops, Stepper::step_loops_nominal;
  uint16_t Stepper::OCR1A_nominal;
#endif

volatile long Stepper::endstops_trigsteps[XYZ];

#if ENABLED(X_DUAL_STEPPER_DRIVERS)
  #define X_APPLY_DIR(v,Q) do{ X_DIR_WRITE(v); X2_DIR_WRITE((v) != INVERT_X2_VS_X_DIR); }while(0)
  #define X_APPLY_STEP(v,Q) do{ X_STEP_WRITE(v); X2_STEP_WRITE(v); }while(0)
#elif ENABLED(DUAL_X_CARRIAGE)
  #define X_APPLY_DIR(v,ALWAYS) \
    if (hotend_duplication_enabled || ALWAYS) { \
      X_DIR_WRITE(v); \
      X2_DIR_WRITE(v); \
    } \
    else { \
      if (TOOL_E_INDEX != 0) X2_DIR_WRITE(v); else X_DIR_WRITE(v); \
    }
  #define X_APPLY_STEP(v,ALWAYS) \
    if (hotend_duplication_enabled || ALWAYS) { \
      X_STEP_WRITE(v); \
      X2_STEP_WRITE(v); \
    } \
    else { \
      if (TOOL_E_INDEX != 0) X2_STEP_WRITE(v); else X_STEP_WRITE(v); \
    }
#else
  #define X_APPLY_DIR(v,Q) X_DIR_WRITE(v)
  #define X_APPLY_STEP(v,Q) X_STEP_WRITE(v)
#endif

#if ENABLED(Y_DUAL_STEPPER_DRIVERS)
  #define Y_APPLY_DIR(v,Q) { Y_DIR_WRITE(v); Y2_DIR_WRITE((v) != INVERT_Y2_VS_Y_DIR); }
  #define Y_APPLY_STEP(v,Q) { Y_STEP_WRITE(v); Y2_STEP_WRITE(v); }
#else
  #define Y_APPLY_DIR(v,Q) Y_DIR_WRITE(v)
  #define Y_APPLY_STEP(v,Q) Y_STEP_WRITE(v)
#endif

#if ENABLED(Z_DUAL_STEPPER_DRIVERS)
  #define Z_APPLY_DIR(v,Q) { Z_DIR_WRITE(v); Z2_DIR_WRITE(v); }
  #if ENABLED(Z_DUAL_ENDSTOPS)
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
  //  TCNT1 = 0;
  ENABLE_STEPPER_DRIVER_INTERRUPT();
  #ifdef __SAM3X8E__
    #if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)
      ENABLE_ADVANCE_EXTRUDER_INTERRUPT();
    #endif
  #endif
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

  #if HAS(X_DIR)
    SET_STEP_DIR(X); // A
  #endif
  #if HAS(Y_DIR)
    SET_STEP_DIR(Y); // B
  #endif
  #if HAS(Z_DIR)
    SET_STEP_DIR(Z); // C
  #endif

  #if DISABLED(ADVANCE)
    if (motor_direction(E_AXIS)) {
      REV_E_DIR();
      count_direction[E_AXIS] = -1;
    }
    else {
      NORM_E_DIR();
      count_direction[E_AXIS] = 1;
    }
  #endif // !ADVANCE
}

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse.
// It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately.
#ifdef __SAM3X8E__
  HAL_STEP_TIMER_ISR { Stepper::isr(); }
#else
  ISR(TIMER1_COMPA_vect) { Stepper::isr(); }
#endif

void Stepper::isr() {

  #ifdef __SAM3X8E__
    stepperChannel->TC_SR;
  #endif

  if (cleaning_buffer_counter) {
    current_block = NULL;
    planner.discard_current_block();
    #if ENABLED(SD_FINISHED_RELEASECOMMAND)
      if ((cleaning_buffer_counter == 1) && (SD_FINISHED_STEPPERRELEASE)) enqueue_and_echo_commands_P(PSTR(SD_FINISHED_RELEASECOMMAND));
    #endif
    cleaning_buffer_counter--;
    #ifdef __SAM3X8E__
      HAL_timer_stepper_count(HAL_TIMER_RATE / 200); // 5ms wait
    #else
      OCR1A = 200;
    #endif
    return;
  }

  #ifdef __SAM3X8E__
    #if ENABLED(LASERBEAM)
      if (laser.firing == LASER_ON && laser.dur != 0 && (laser.last_firing + laser.dur < micros())) {
        if (laser.diagnostics)
          SERIAL_EM("Laser firing duration elapsed, in interrupt handler");
        laser_extinguish();
      }
    #endif
  #else
    #if ENABLED(LASERBEAM) && (!ENABLED(LASER_PULSE_METHOD))
      if (laser.dur != 0 && (laser.last_firing + laser.dur < micros())) {
        if (laser.diagnostics)
          SERIAL_EM("Laser firing duration elapsed, in interrupt handler");
        laser_extinguish();
      }
    #endif
  #endif

  // If there is no current block, attempt to pop one from the buffer
  if (!current_block) {
    // Anything in the buffer?
    current_block = planner.get_current_block();
    if (current_block) {
      current_block->busy = true;
      trapezoid_generator_reset();

      // Initialize Bresenham counters to 1/2 the ceiling
      counter_X = counter_Y = counter_Z = counter_E = -(current_block->step_event_count >> 1);

      #if ENABLED(LASERBEAM)
        #ifdef __SAM3X8E__
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

      #if ENABLED(Z_LATE_ENABLE)
        if (current_block->steps[Z_AXIS] > 0) {
          enable_z();
          #ifdef __SAM3X8E__
            HAL_timer_stepper_count(HAL_TIMER_RATE / 1000); // 1ms wait
          #else
            OCR1A = 2000; // 1ms wait
          #endif
          return;
        }
      #endif

      #if ENABLED(LASERBEAM) && ENABLED(LASER_RASTER)
         if (current_block->laser_mode == RASTER) counter_raster = 0;
      #endif

      // #if ENABLED(ADVANCE)
      //   e_steps[TOOL_E_INDEX] = 0;
      // #endif
    }
    else {
      #ifdef __SAM3X8E__
        HAL_timer_stepper_count(HAL_TIMER_RATE / 1000); // 1kHz.
      #else
        OCR1A = 2000; // 1kHz.
      #endif
    }
  }

  if (current_block) {

    // Continuous firing of the laser during a move happens here, PPM and raster happen further down
    #if ENABLED(LASERBEAM)
      if (current_block->laser_mode == CONTINUOUS && current_block->laser_status == LASER_ON)
        laser_fire(current_block->laser_intensity);

      #if DISABLED(LASER_PULSE_METHOD)
        if (current_block->laser_status == LASER_OFF) {
          if (laser.diagnostics)
            SERIAL_EM("Laser status set to off, in interrupt handler");
          laser_extinguish();
        }
      #endif
    #endif

    // Update endstops state, if enabled
    if (endstops.enabled
      #if HAS(BED_PROBE)
        || endstops.z_probe_enabled
      #endif
    ) endstops.update();

    #define _COUNTER(AXIS) counter_## AXIS
    #define _APPLY_STEP(AXIS) AXIS ##_APPLY_STEP
    #define _INVERT_STEP_PIN(AXIS) INVERT_## AXIS ##_STEP_PIN

    #ifdef __SAM3X8E__
      #define PULSE_START(AXIS) \
        _COUNTER(AXIS) += current_block->steps[_AXIS(AXIS)]; \
        if (_COUNTER(AXIS) > 0) { \
          _APPLY_STEP(AXIS)(!_INVERT_STEP_PIN(AXIS),0); \
          _COUNTER(AXIS) -= current_block->step_event_count; \
          count_position[_AXIS(AXIS)] += count_direction[_AXIS(AXIS)]; \
        }
    #else
      #define PULSE_START(AXIS) \
        _COUNTER(AXIS) += current_block->steps[_AXIS(AXIS)]; \
        if (_COUNTER(AXIS) > 0) _APPLY_STEP(AXIS)(!_INVERT_STEP_PIN(AXIS),0);
    #endif

    #ifdef __SAM3X8E__
      #define PULSE_STOP(AXIS) _APPLY_STEP(AXIS)(_INVERT_STEP_PIN(AXIS),0)
    #else
      #define PULSE_STOP(AXIS) \
        if (_COUNTER(AXIS) > 0) { \
          _COUNTER(AXIS) -= current_block->step_event_count; \
          count_position[_AXIS(AXIS)] += count_direction[_AXIS(AXIS)]; \
          _APPLY_STEP(AXIS)(_INVERT_STEP_PIN(AXIS),0); \
        }
    #endif

    #ifndef __SAM3X8E__ || ENABLED(ENABLE_HIGH_SPEED_STEPPING)
      // Take multiple steps per interrupt (For high speed moves)
      bool all_steps_done = false;
      for (int8_t i = 0; i < step_loops; i++) {
        /*
        #ifndef __SAM3X8E__
          #ifndef USBCON
            MKSERIAL.checkRx(); // Check for serial chars.
          #endif
        #endif
        */

        #if ENABLED(LIN_ADVANCE) // LIN_ADVANCE

          counter_E += current_block->steps[E_AXIS];
          if (counter_E > 0) {
            counter_E -= current_block->step_event_count;
            #if DISABLED(COLOR_MIXING_EXTRUDER)
              // Don't step E here for mixing extruder
              count_position[E_AXIS] += count_direction[E_AXIS];
              motor_direction(E_AXIS) ? --e_steps[TOOL_E_INDEX] : ++e_steps[TOOL_E_INDEX];
            #endif
          }

          #if ENABLED(COLOR_MIXING_EXTRUDER)
            // Step mixing steppers proportionally
            bool dir = motor_direction(E_AXIS);
            MIXING_STEPPERS_LOOP(j) {
              counter_m[j] += current_block->steps[E_AXIS];
              if (counter_m[j] > 0) {
                counter_m[j] -= current_block->mix_event_count[j];
                dir ? --e_steps[j] : ++e_steps[j];
              }
            }
          #endif

          if (current_block->use_advance_lead) {
            int delta_adv_steps = (((long)extruder_advance_k * current_estep_rate[TOOL_E_INDEX]) >> 9) - current_adv_steps[TOOL_E_INDEX];
            #if ENABLED(COLOR_MIXING_EXTRUDER)
              // Mixing extruders apply advance lead proportionally
              MIXING_STEPPERS_LOOP(j) {
                int steps = delta_adv_steps * current_block->step_event_count / current_block->mix_event_count[j];
                e_steps[j] += steps;
                current_adv_steps[j] += steps;
              }
            #else
              // For most extruders, advance the single E stepper
              e_steps[TOOL_E_INDEX] += delta_adv_steps;
              current_adv_steps[TOOL_E_INDEX] += delta_adv_steps;
            #endif
          }

        #elif ENABLED(ADVANCE)

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
            bool dir = motor_direction(E_AXIS);
            MIXING_STEPPERS_LOOP(j) {
              counter_m[j] += current_block->steps[E_AXIS];
              if (counter_m[j] > 0) {
                counter_m[j] -= current_block->mix_event_count[j];
                dir ? --e_steps[j] : ++e_steps[j];
              }
            }
          #endif

        #endif // ADVANCE or LIN_ADVANCE

        #if ENABLED(STEPPER_HIGH_LOW) && STEPPER_HIGH_LOW_DELAY > 0
          static uint32_t pulse_start;
          pulse_start = TCNT0;
        #endif

        #if HAS(X_STEP)
          PULSE_START(X);
        #endif
        #if HAS(Y_STEP)
          PULSE_START(Y);
        #endif
        #if HAS(Z_STEP)
          PULSE_START(Z);
        #endif

        // For non-advance use linear interpolation for E also
        #if DISABLED(ADVANCE) && DISABLED(LIN_ADVANCE)
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
        #endif // !ADVANCE && !LIN_ADVANCE

        #if ENABLED(STEPPER_HIGH_LOW) && STEPPER_HIGH_LOW_DELAY > 0
          #define CYCLES_EATEN_BY_CODE 10
          while ((uint32_t)(TCNT0 - pulse_start) < (STEPPER_HIGH_LOW_DELAY * (F_CPU / 1000000UL)) - CYCLES_EATEN_BY_CODE) { /* nada */ }
        #endif

        #if HAS(X_STEP)
          PULSE_STOP(X);
        #endif
        #if HAS(Y_STEP)
          PULSE_STOP(Y);
        #endif
        #if HAS(Z_STEP)
          PULSE_STOP(Z);
        #endif

        #if DISABLED(ADVANCE) && DISABLED(LIN_ADVANCE)
          #if ENABLED(COLOR_MIXING_EXTRUDER)
            // Always step the single E axis
            if (counter_E > 0) {
              counter_E -= current_block->step_event_count;
              count_position[E_AXIS] += count_direction[E_AXIS];
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

        #if ENABLED(LASERBEAM)
          counter_L += current_block->steps_l;
          if (counter_L > 0) {
            if (current_block->laser_mode == PULSED && current_block->laser_status == LASER_ON) { // Pulsed Firing Mode
              #if ENABLED(LASER_PULSE_METHOD)
                uint32_t ulValue = current_block->laser_raster_intensity_factor * 255;
                laser_pulse(ulValue, current_block->laser_duration);
                laser.time += current_block->laser_duration / 1000; 
              #else
                laser_fire(current_block->laser_intensity);
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
                  laser.time += current_block->laser_duration/1000; 
                #else
                  // For some reason, when comparing raster power to ppm line burns the rasters were around 2% more powerful
                  // going from darkened paper to burning through paper.
                  laser_fire(current_block->laser_raster_data[counter_raster]); 
                #endif
                if (laser.diagnostics) SERIAL_MV("Pixel: ", (float)current_block->laser_raster_data[counter_raster]);
                counter_raster++;
              }
            #endif // LASER_RASTER
            
            #ifdef __SAM3X8E__
              counter_L -= 1000 * current_block->step_event_count;
            #else
              counter_L -= current_block->step_event_count;
            #endif
          }
          #if DISABLED(LASER_PULSE_METHOD)
            if (current_block->laser_duration != 0 && (laser.last_firing + current_block->laser_duration < micros())) {
              if (laser.diagnostics)
                SERIAL_EM("Laser firing duration elapsed, in interrupt fast loop");
              laser_extinguish();
            }
          #endif // DISABLED(LASER_PULSE_METHOD)
        #endif // LASERBEAM

        if (++step_events_completed >= current_block->step_event_count) {
          all_steps_done = true;
          break;
        }
      }

    #else // __SAM3X8E__ && DISABLED(ENABLE_HIGH_SPEED_STEPPING)

      bool all_steps_done = false;

      #if ENABLED(LIN_ADVANCE) // LIN_ADVANCE

        counter_E += current_block->steps[E_AXIS];
        if (counter_E > 0) {
          counter_E -= current_block->step_event_count;
          #if DISABLED(COLOR_MIXING_EXTRUDER)
            // Don't step E here for mixing extruder
            count_position[E_AXIS] += count_direction[E_AXIS];
            motor_direction(E_AXIS) ? --e_steps[TOOL_E_INDEX] : ++e_steps[TOOL_E_INDEX];
          #endif
        }

        #if ENABLED(COLOR_MIXING_EXTRUDER)
          // Step mixing steppers proportionally
          bool dir = motor_direction(E_AXIS);
          MIXING_STEPPERS_LOOP(j) {
            counter_m[j] += current_block->steps[E_AXIS];
            if (counter_m[j] > 0) {
              counter_m[j] -= current_block->mix_event_count[j];
              dir ? --e_steps[j] : ++e_steps[j];
            }
          }
        #endif

        if (current_block->use_advance_lead) {
          int delta_adv_steps = (((long)extruder_advance_k * current_estep_rate[TOOL_E_INDEX]) >> 9) - current_adv_steps[TOOL_E_INDEX];
          #if ENABLED(COLOR_MIXING_EXTRUDER)
            // Mixing extruders apply advance lead proportionally
            MIXING_STEPPERS_LOOP(j) {
              int steps = delta_adv_steps * current_block->step_event_count / current_block->mix_event_count[j];
              e_steps[j] += steps;
              current_adv_steps[j] += steps;
            }
          #else
            // For most extruders, advance the single E stepper
            e_steps[TOOL_E_INDEX] += delta_adv_steps;
            current_adv_steps[TOOL_E_INDEX] += delta_adv_steps;
          #endif
        }

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
          bool dir = motor_direction(E_AXIS) ? -1 : 1;
          MIXING_STEPPERS_LOOP(j) {
            counter_m[j] += current_block->steps[E_AXIS];
            if (counter_m[j] > 0) {
              counter_m[j] -= current_block->mix_event_count[j];
              dir ? --e_steps[j] : ++e_steps[j];
            }
          }
        #endif

      #endif // ADVANCE or LIN_ADVANCE

      #if HAS(X_STEP)
        PULSE_START(X);
      #endif
      #if HAS(Y_STEP)
        PULSE_START(Y);
      #endif
      #if HAS(Z_STEP)
        PULSE_START(Z);
      #endif

      #if DISABLED(ADVANCE) && DISABLED(LIN_ADVANCE)
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
      #endif // !ADVANCE && !LIN_ADVANCE

      #if ENABLED(LASERBEAM)
        counter_L += current_block->steps_l;
        if (counter_L > 0) {
          if (current_block->laser_mode == PULSED && current_block->laser_status == LASER_ON) { // Pulsed Firing Mode
            #if ENABLED(LASER_PULSE_METHOD)
              uint32_t ulValue = current_block->laser_raster_intensity_factor * 255;
              laser_pulse(ulValue, current_block->laser_duration);
              laser.time += current_block->laser_duration / 1000; 
            #else
              laser_fire(current_block->laser_intensity);
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
                laser.time += current_block->laser_duration/1000; 
              #else
                // For some reason, when comparing raster power to ppm line burns the rasters were around 2% more powerful
                // going from darkened paper to burning through paper.
                laser_fire(current_block->laser_raster_data[counter_raster]); 
              #endif
              if (laser.diagnostics) SERIAL_MV("Pixel: ", (float)current_block->laser_raster_data[counter_raster]);
              counter_raster++;
            }
          #endif // LASER_RASTER
          
          #ifdef __SAM3X8E__
            counter_L -= 1000 * current_block->step_event_count;
          #else
            counter_L -= current_block->step_event_count;
          #endif
        }
        #if DISABLED(LASER_PULSE_METHOD)
          if (current_block->laser_duration != 0 && (laser.last_firing + current_block->laser_duration < micros())) {
            if (laser.diagnostics)
              SERIAL_EM("Laser firing duration elapsed, in interrupt fast loop");
            laser_extinguish();
          }
        #endif // DISABLED(LASER_PULSE_METHOD)
      #endif // LASERBEAM

      if (++step_events_completed >= current_block->step_event_count) {
        all_steps_done = true;
      }

    #endif // __SAM3X8E__ && DISABLED(ENABLE_HIGH_SPEED_STEPPING)

    #ifndef __SAM3X8E__
      #if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)
        // If we have esteps to execute, fire the next ISR "now"
        if (e_steps[TOOL_E_INDEX]) OCR0A = TCNT0 + 2;
      #endif
    #endif

    // Calculate new timer value
    #ifdef __SAM3X8E__
      uint32_t timer, step_rate;
    #else
      uint16_t timer, step_rate;
    #endif

    if (step_events_completed <= (uint32_t)current_block->accelerate_until) {

      #ifdef __SAM3X8E__
        MultiU32X32toH32(acc_step_rate, acceleration_time, current_block->acceleration_rate);
      #else
        MultiU24X32toH16(acc_step_rate, acceleration_time, current_block->acceleration_rate);
      #endif
      acc_step_rate += current_block->initial_rate;

      // upper limit
      NOMORE(acc_step_rate, current_block->nominal_rate);

      // step_rate to timer interval
      timer = calc_timer(acc_step_rate);
      #ifndef __SAM3X8E__
        OCR1A = timer;
      #endif
      acceleration_time += timer;

      #if ENABLED(LIN_ADVANCE)

        if (current_block->use_advance_lead) {
          #if ENABLED(COLOR_MIXING_EXTRUDER)
            MIXING_STEPPERS_LOOP(j)
              current_estep_rate[j] = ((uint32_t)acc_step_rate * current_block->e_speed_multiplier8 * current_block->step_event_count / current_block->mix_event_count[j]) >> 8;
          #else
            current_estep_rate[TOOL_E_INDEX] = ((uint32_t)acc_step_rate * current_block->e_speed_multiplier8) >> 8;
          #endif
        }

      #elif ENABLED(ADVANCE)

        advance += advance_rate * step_loops;
        //NOLESS(advance, current_block->advance);

        long advance_whole = advance >> 8,
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
        eISR_Rate = (timer >> 2) * step_loops / abs(e_steps[TOOL_E_INDEX]);
      #endif
    }
    else if (step_events_completed > (uint32_t)current_block->decelerate_after) {
      #ifdef __SAM3X8E__
        MultiU32X32toH32(step_rate, deceleration_time, current_block->acceleration_rate);
      #else
        MultiU24X32toH16(step_rate, deceleration_time, current_block->acceleration_rate);
      #endif

      if (step_rate < acc_step_rate) {
        step_rate = acc_step_rate - step_rate; // Decelerate from acceleration end point.
        NOLESS(step_rate, current_block->final_rate);
      }
      else {
        step_rate = current_block->final_rate;
      }

      // step_rate to timer interval
      timer = calc_timer(step_rate);
      #ifndef __SAM3X8E__
        OCR1A = timer;
      #endif
      deceleration_time += timer;

      #if ENABLED(LIN_ADVANCE)

        if (current_block->use_advance_lead) {
          #if ENABLED(MIXING_EXTRUDER_FEATURE)
            MIXING_STEPPERS_LOOP(j)
              current_estep_rate[j] 0= ((uint32_t)step_rate * current_block->e_speed_multiplier8 * current_block->step_event_count / current_block->mix_event_count[j]) >> 8;
          #else
            current_estep_rate[TOOL_E_INDEX] = ((uint32_t)step_rate * current_block->e_speed_multiplier8) >> 8;
          #endif
        }

      #elif ENABLED(ADVANCE)

        advance -= advance_rate * step_loops;
        NOLESS(advance, final_advance);

        // Do E steps + advance steps
        long advance_whole = advance >> 8,
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
        eISR_Rate = (timer >> 2) * step_loops / abs(e_steps[TOOL_E_INDEX]);
      #endif
    }
    else {

      #if ENABLED(LIN_ADVANCE)

        if (current_block->use_advance_lead)
          current_estep_rate[TOOL_E_INDEX] = final_estep_rate;
        
        eISR_Rate = (OCR1A_nominal >> 2) * step_loops_nominal / abs(e_steps[TOOL_E_INDEX]);

      #endif

      #ifdef __SAM3X8E__
        timer = OCR1A_nominal;
      #else
        OCR1A = OCR1A_nominal;
      #endif
      // ensure we're running at the correct step rate, even if we just came off an acceleration
      step_loops = step_loops_nominal;
    }

    #ifdef __SAM3X8E__ && DISABLED(ENABLE_HIGH_SPEED_STEPPING)

      #if HAS(X_STEP)
        PULSE_STOP(X);
      #endif
      #if HAS(Y_STEP)
        PULSE_STOP(Y);
      #endif
      #if HAS(Z_STEP)
        PULSE_STOP(Z);
      #endif

      #if DISABLED(ADVANCE) && DISABLED(LIN_ADVANCE)
        #if ENABLED(COLOR_MIXING_EXTRUDER)
          // Always step the single E axis
          if (counter_E > 0) {
            counter_E -= current_block->step_event_count;
            count_position[E_AXIS] += count_direction[E_AXIS];
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

    #endif // __SAM3X8E__ && DISABLED(ENABLE_HIGH_SPEED_STEPPING)

    #ifdef __SAM3X8E__
      HAL_timer_stepper_count(timer);
    #else
      NOLESS(OCR1A, TCNT1 + 16);
    #endif

    // If current block is finished, reset pointer
    if (all_steps_done) {
      current_block = NULL;
      planner.discard_current_block();

      #ifdef __SAM3X8E__
        #if ENABLED(LASERBEAM)
          laser_extinguish();
        #endif
      #else
        #if ENABLED(LASERBEAM) && ENABLED(LASER_PULSE_METHOD)
          if (current_block->laser_mode == CONTINUOUS && current_block->laser_status == LASER_ON)
            laser_extinguish();
        #endif
      #endif
    }
  }
}

#if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)

  #ifdef __SAM3X8E__
    HAL_ADVANCE_EXTRUDER_TIMER_ISR { Stepper::advance_isr(); }
  #else
    // Timer interrupt for E. e_steps is set in the main routine;
    // Timer 0 is shared with millies
    ISR(TIMER0_COMPA_vect) { Stepper::advance_isr(); }
  #endif

  void Stepper::advance_isr() {

    #ifdef __SAM3X8E__
      extruderChannel->TC_SR;
    #else
      old_OCR0A += eISR_Rate;
      OCR0A = old_OCR0A;
    #endif

    #define SET_E_STEP_DIR(INDEX) \
      E## INDEX ##_DIR_WRITE(e_steps[INDEX] <= 0 ? INVERT_E## INDEX ##_DIR : !INVERT_E## INDEX ##_DIR)

    #define START_E_PULSE(INDEX) \
      if (e_steps[INDEX]) E## INDEX ##_STEP_WRITE(INVERT_E_STEP_PIN)

    #define STOP_E_PULSE(INDEX) \
      if (e_steps[INDEX]) { \
        e_steps[INDEX] <= 0 ? ++e_steps[INDEX] : --e_steps[INDEX]; \
        E## INDEX ##_STEP_WRITE(!INVERT_E_STEP_PIN); \
      }

    SET_E_STEP_DIR(0);
    #if E_STEPPERS > 1
      SET_E_STEP_DIR(1);
      #if E_STEPPERS > 2
        SET_E_STEP_DIR(2);
        #if E_STEPPERS > 3
          SET_E_STEP_DIR(3);
          #if E_STEPPERS > 4
            SET_E_STEP_DIR(4);
            #if E_STEPPERS > 5
              SET_E_STEP_DIR(5);
            #endif
          #endif
        #endif
      #endif
    #endif

    // Step all E steppers that have steps
    for (uint8_t i = 0; i < step_loops; i++) {

      #if ENABLED(STEPPER_HIGH_LOW) && STEPPER_HIGH_LOW_DELAY > 0
        static uint32_t pulse_start;
        pulse_start = TCNT0;
      #endif

      START_E_PULSE(0);
      #if E_STEPPERS > 1
        START_E_PULSE(1);
        #if E_STEPPERS > 2
          START_E_PULSE(2);
          #if E_STEPPERS > 3
            START_E_PULSE(3);
            #if E_STEPPERS > 4
              START_E_PULSE(4);
              #if E_STEPPERS > 5
                START_E_PULSE(5);
              #endif
            #endif
          #endif
        #endif
      #endif
      
      // For a minimum pulse time wait before stopping pulses
      #if MINIMUM_STEPPER_PULSE > 0
        #define CYCLES_EATEN_BY_E 10
        while ((uint32_t)(TCNT0 - pulse_start) < (MINIMUM_STEPPER_PULSE * (F_CPU / 1000000UL)) - CYCLES_EATEN_BY_E) { /* nada */ }
      #elif defined __SAM3X8E__
        HAL::delayMicroseconds(2U);
      #endif
      
      STOP_E_PULSE(0);
      #if E_STEPPERS > 1
        STOP_E_PULSE(1);
        #if E_STEPPERS > 2
          STOP_E_PULSE(2);
          #if E_STEPPERS > 3
            STOP_E_PULSE(3);
            #if E_STEPPERS > 4
              STOP_E_PULSE(4);
              #if E_STEPPERS > 5
                STOP_E_PULSE(5);
              #endif
            #endif
          #endif
        #endif
      #endif
    }
  }
#endif // ADVANCE or LIN_ADVANCE

void Stepper::init() {
  digipot_init();   // Initialize Digipot Motor Current
  microstep_init(); // Initialize Microstepping Pins

  // initialise TMC Steppers
  #if ENABLED(HAVE_TMCDRIVER)
    tmc_init();
  #endif
    // initialise L6470 Steppers
  #if ENABLED(HAVE_L6470DRIVER)
    L6470_init();
  #endif

  // Initialize Dir Pins
  #if HAS(X_DIR)
    X_DIR_INIT;
  #endif
  #if HAS(X2_DIR)
    X2_DIR_INIT;
  #endif
  #if HAS(Y_DIR)
    Y_DIR_INIT;
    #if ENABLED(Y_DUAL_STEPPER_DRIVERS) && HAS(Y2_DIR)
      Y2_DIR_INIT;
    #endif
  #endif
  #if HAS(Z_DIR)
    Z_DIR_INIT;
    #if ENABLED(Z_DUAL_STEPPER_DRIVERS) && HAS(Z2_DIR)
      Z2_DIR_INIT;
    #endif
  #endif
  #if HAS(E0_DIR)
    E0_DIR_INIT;
  #endif
  #if HAS(E1_DIR)
    E1_DIR_INIT;
  #endif
  #if HAS(E2_DIR)
    E2_DIR_INIT;
  #endif
  #if HAS(E3_DIR)
    E3_DIR_INIT;
  #endif
  #if HAS(E4_DIR)
    E4_DIR_INIT;
  #endif
  #if HAS(E5_DIR)
    E5_DIR_INIT;
  #endif

  //Initialize Enable Pins - steppers default to disabled.

  #if HAS(X_ENABLE)
    X_ENABLE_INIT;
    if (!X_ENABLE_ON) X_ENABLE_WRITE(HIGH);
    #if (ENABLED(DUAL_X_CARRIAGE) || ENABLED(X_DUAL_STEPPER_DRIVERS)) && HAS(X2_ENABLE)
      X2_ENABLE_INIT;
      if (!X_ENABLE_ON) X2_ENABLE_WRITE(HIGH);
    #endif
  #endif

  #if HAS(Y_ENABLE)
    Y_ENABLE_INIT;
    if (!Y_ENABLE_ON) Y_ENABLE_WRITE(HIGH);

    #if ENABLED(Y_DUAL_STEPPER_DRIVERS) && HAS(Y2_ENABLE)
      Y2_ENABLE_INIT;
      if (!Y_ENABLE_ON) Y2_ENABLE_WRITE(HIGH);
    #endif
  #endif

  #if HAS(Z_ENABLE)
    Z_ENABLE_INIT;
    if (!Z_ENABLE_ON) Z_ENABLE_WRITE(HIGH);
    #if ENABLED(Z_DUAL_STEPPER_DRIVERS) && HAS(Z2_ENABLE)
      Z2_ENABLE_INIT;
      if (!Z_ENABLE_ON) Z2_ENABLE_WRITE(HIGH);
    #endif
  #endif

  #if HAS(E0_ENABLE)
    E0_ENABLE_INIT;
    if (!E_ENABLE_ON) E0_ENABLE_WRITE(HIGH);
  #endif
  #if HAS(E1_ENABLE)
    E1_ENABLE_INIT;
    if (!E_ENABLE_ON) E1_ENABLE_WRITE(HIGH);
  #endif
  #if HAS(E2_ENABLE)
    E2_ENABLE_INIT;
    if (!E_ENABLE_ON) E2_ENABLE_WRITE(HIGH);
  #endif
  #if HAS(E3_ENABLE)
    E3_ENABLE_INIT;
    if (!E_ENABLE_ON) E3_ENABLE_WRITE(HIGH);
  #endif
  #if HAS(E4_ENABLE)
    E4_ENABLE_INIT;
    if (!E_ENABLE_ON) E4_ENABLE_WRITE(HIGH);
  #endif
  #if HAS(E5_ENABLE)
    E5_ENABLE_INIT;
    if (!E_ENABLE_ON) E5_ENABLE_WRITE(HIGH);
  #endif

  #if ENABLED(MKR4) // MKR4 System
    #if HAS(E0E1)
      OUT_WRITE_RELE(E0E1_CHOICE_PIN, LOW);
    #endif
    #if HAS(E0E2)
      OUT_WRITE_RELE(E0E2_CHOICE_PIN, LOW);
    #endif
    #if HAS(E1E3)
      OUT_WRITE_RELE(E1E3_CHOICE_PIN, LOW);
    #endif
  #elif ENABLED(MKR6) // MKR6 System
    #if HAS(EX1)
      OUT_WRITE_RELE(EX1_CHOICE_PIN, LOW);
    #endif
    #if HAS(EX2)
      OUT_WRITE_RELE(EX2_CHOICE_PIN, LOW);
    #endif
  #endif

  //
  // Init endstops and pullups here
  //
  endstops.init();

  #define _STEP_INIT(AXIS) AXIS ##_STEP_INIT
  #define _WRITE_STEP(AXIS, HIGHLOW) AXIS ##_STEP_WRITE(HIGHLOW)
  #define _DISABLE(axis) disable_## axis()

  #define AXIS_INIT(axis, AXIS, PIN) \
    _STEP_INIT(AXIS); \
    _WRITE_STEP(AXIS, _INVERT_STEP_PIN(PIN)); \
    _DISABLE(axis)

  #define E_AXIS_INIT(NUM) AXIS_INIT(e## NUM, E## NUM, E)

  // Initialize Step Pins
  #if HAS(X_STEP)
    #if ENABLED(X_DUAL_STEPPER_DRIVERS) || ENABLED(DUAL_X_CARRIAGE)
      X2_STEP_INIT;
      X2_STEP_WRITE(INVERT_X_STEP_PIN);
    #endif
    AXIS_INIT(x, X, X);
  #endif

  #if HAS(Y_STEP)
    #if ENABLED(Y_DUAL_STEPPER_DRIVERS) && HAS(Y2_STEP)
      Y2_STEP_INIT;
      Y2_STEP_WRITE(INVERT_Y_STEP_PIN);
    #endif
    AXIS_INIT(y, Y, Y);
  #endif

  #if HAS(Z_STEP)
    #if ENABLED(Z_DUAL_STEPPER_DRIVERS) && HAS(Z2_STEP)
      Z2_STEP_INIT;
      Z2_STEP_WRITE(INVERT_Z_STEP_PIN);
    #endif
    AXIS_INIT(z, Z, Z);
  #endif

  #if HAS(E0_STEP)
    E_AXIS_INIT(0);
  #endif
  #if HAS(E1_STEP)
    E_AXIS_INIT(1);
  #endif
  #if HAS(E2_STEP)
    E_AXIS_INIT(2);
  #endif
  #if HAS(E3_STEP)
    E_AXIS_INIT(3);
  #endif
  #if HAS(E4_STEP)
    E_AXIS_INIT(4);
  #endif
  #if HAS(E5_STEP)
    E_AXIS_INIT(5);
  #endif

  #ifdef __SAM3X8E__
    HAL_step_timer_start();
  #else
    // waveform generation = 0100 = CTC
    CBI(TCCR1B, WGM13);
    SBI(TCCR1B, WGM12);
    CBI(TCCR1A, WGM11);
    CBI(TCCR1A, WGM10);

    // output mode = 00 (disconnected)
    TCCR1A &= ~(3 << COM1A0);
    TCCR1A &= ~(3 << COM1B0);
    // Set the timer pre-scaler
    // Generally we use a divider of 8, resulting in a 2MHz timer
    // frequency on a 16MHz MCU. If you are going to change this, be
    // sure to regenerate speed_lookuptable.h with
    // create_speed_lookuptable.py
    TCCR1B = (TCCR1B & ~(0x07 << CS10)) | (2 << CS10);

    OCR1A = 0x4000;
    TCNT1 = 0;
  #endif

  ENABLE_STEPPER_DRIVER_INTERRUPT();

  #if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)
    for (uint8_t i = 0; i < E_STEPPERS; i++) {
      e_steps[i] = 0;
      #if ENABLED(LIN_ADVANCE)
        current_adv_steps[i] = 0;
      #endif
    }

    #ifdef __SAM3X8E__
      HAL_advance_extruder_timer_start();
      ENABLE_ADVANCE_EXTRUDER_INTERRUPT();
    #else
      #if defined(TCCR0A) && defined(WGM01)
        CBI(TCCR0A, WGM01);
        CBI(TCCR0A, WGM00);
      #endif
      SBI(TIMSK0, OCIE0A);
    #endif

  #endif // ADVANCE or LIN_ADVANCE

  endstops.enable(true); // Start with endstops active. After homing they can be disabled
  sei();

  set_directions(); // Init directions to last_direction_bits = 0
}


/**
 * Block until all buffered steps are executed
 */
void Stepper::synchronize() { while (planner.blocks_queued()) idle(); }

/**
 * Set the stepper positions directly in steps
 *
 * The input is based on the typical per-axis XYZ steps.
 * For CORE machines XYZ needs to be translated to ABC.
 *
 * This allows get_axis_position_mm to correctly
 * derive the current XYZ position later on.
 */
void Stepper::set_position(const long& x, const long& y, const long& z, const long& e) {
  CRITICAL_SECTION_START;

  #if MECH(COREXY)
    // corexy positioning
    count_position[A_AXIS] = x + COREX_YZ_FACTOR * y;
    count_position[B_AXIS] = x - COREX_YZ_FACTOR * y;
    count_position[Z_AXIS] = z;
  #elif MECH(COREYX)
    // coreyx positioning
    count_position[A_AXIS] = y + COREX_YZ_FACTOR * x;
    count_position[B_AXIS] = y - COREX_YZ_FACTOR * x;
    count_position[Z_AXIS] = z;
  #elif MECH(COREXZ)
    // corexz planning
    count_position[A_AXIS] = x + COREX_YZ_FACTOR * z;
    count_position[Y_AXIS] = y;
    count_position[C_AXIS] = x - COREX_YZ_FACTOR * z;
  #elif MECH(COREZX)
    // corezx planning
    count_position[A_AXIS] = z + COREX_YZ_FACTOR * x;
    count_position[Y_AXIS] = y;
    count_position[C_AXIS] = z - COREX_YZ_FACTOR * x;
  #else
    // default non-h-bot planning
    count_position[X_AXIS] = x;
    count_position[Y_AXIS] = y;
    count_position[Z_AXIS] = z;
  #endif

  count_position[E_AXIS] = e;
  CRITICAL_SECTION_END;
}

void Stepper::set_e_position(const long& e) {
  CRITICAL_SECTION_START;
  count_position[E_AXIS] = e;
  CRITICAL_SECTION_END;
}

/**
 * Get a stepper's position in steps.
 */
long Stepper::position(AxisEnum axis) {
  CRITICAL_SECTION_START;
  long count_pos = count_position[axis];
  CRITICAL_SECTION_END;
  return count_pos;
}

/**
 * Get an axis position according to stepper position(s)
 * For CORE machines apply translation from ABC to XYZ.
 */
float Stepper::get_axis_position_mm(AxisEnum axis) {
  float axis_pos;
  #if MECH(COREXY) || MECH(COREYX) || MECH(COREXZ) || MECH(COREZX)
    if (axis == CORE_AXIS_1 || axis == CORE_AXIS_2) {
      CRITICAL_SECTION_START;
      long  pos1 = count_position[CORE_AXIS_1],
            pos2 = count_position[CORE_AXIS_2];
      CRITICAL_SECTION_END;
      // ((a1+a2)+(a1-a2))/2 -> (a1+a2+a1-a2)/2 -> (a1+a1)/2 -> a1
      // ((a1+a2)-(a1-a2))/2 -> (a1+a2-a1+a2)/2 -> (a2+a2)/2 -> a2
      axis_pos = (pos1 + ((axis == CORE_AXIS_1) ? pos2 : -pos2)) * 0.5f;
    }
    else
      axis_pos = position(axis);
  #else
    axis_pos = position(axis);
  #endif

  return axis_pos * planner.steps_to_mm[axis];
}

void Stepper::enable_all_steppers() {
  enable_x();
  enable_y();
  enable_z();
  enable_e0();
  enable_e1();
  enable_e2();
  enable_e3();
  enable_e4();
  enable_e5();
}

void Stepper::disable_all_steppers() {
  disable_x();
  disable_y();
  disable_z();
  disable_e0();
  disable_e1();
  disable_e2();
  disable_e3();
  disable_e4();
  disable_e5();
}

void Stepper::finish_and_disable() {
  synchronize();
  disable_all_steppers();
}

void Stepper::quick_stop() {
  cleaning_buffer_counter = 5000;
  DISABLE_STEPPER_DRIVER_INTERRUPT();
  while (planner.blocks_queued()) planner.discard_current_block();
  current_block = NULL;
  ENABLE_STEPPER_DRIVER_INTERRUPT();
  #ifdef __SAM3X8E__
    #if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)
      ENABLE_ADVANCE_EXTRUDER_INTERRUPT();
    #endif
  #endif
}

void Stepper::endstop_triggered(AxisEnum axis) {

  #if MECH(COREXY) || MECH(COREYX) || MECH(COREXZ) || MECH(COREZX)

    float axis_pos = count_position[axis];
    if (axis == CORE_AXIS_1)
      axis_pos = (axis_pos + count_position[CORE_AXIS_2]) * 0.5;
    else if (axis == CORE_AXIS_2)
      axis_pos = (count_position[CORE_AXIS_1] - axis_pos) * 0.5;
    endstops_trigsteps[axis] = axis_pos;

  #else // ! COREXY || COREYX || COREXZ || COREZX

    endstops_trigsteps[axis] = count_position[axis];

  #endif // ! COREXY || COREYX || COREXZ || COREZX

  kill_current_block();
}

void Stepper::report_positions() {
  CRITICAL_SECTION_START;
  long xpos = count_position[X_AXIS],
       ypos = count_position[Y_AXIS],
       zpos = count_position[Z_AXIS];
  CRITICAL_SECTION_END;

  #if MECH(COREXY) || MECH(COREYX) || MECH(COREXZ) || MECH(COREZX)
    SERIAL_M(MSG_COUNT_A);
  #elif MECH(DELTA)
    SERIAL_M(MSG_COUNT_ALPHA);
  #else
    SERIAL_M(MSG_COUNT_X);
  #endif
  SERIAL_V(xpos);

  #if MECH(COREXY) || MECH(COREYX)
    SERIAL_M(" B:");
  #elif MECH(DELTA)
    SERIAL_M(" Beta:");
  #else
    SERIAL_M(" Y:");
  #endif
  SERIAL_V(ypos);

  #if MECH(COREXZ) || MECH(COREZX)
    SERIAL_M(" C:");
  #elif MECH(DELTA)
    SERIAL_M(" Teta:");
  #else
    SERIAL_M(" Z:");
  #endif
  SERIAL_V(zpos);

  SERIAL_E;
}

#if ENABLED(NPR2)
  void Stepper::colorstep(long csteps,const bool direction) {
    enable_e1();
    //setup new step
    WRITE(E1_DIR_PIN,(INVERT_E1_DIR)^direction);
    //perform step
    for (long i = 0; i <= csteps; i++){
      WRITE(E1_STEP_PIN, !INVERT_E_STEP_PIN);
      HAL::delayMicroseconds(COLOR_SLOWRATE);
      WRITE(E1_STEP_PIN, INVERT_E_STEP_PIN);
      HAL::delayMicroseconds(COLOR_SLOWRATE);
    }
  }  
#endif //NPR2

#if ENABLED(BABYSTEPPING)

  // MUST ONLY BE CALLED BY AN ISR,
  // No other ISR should ever interrupt this!
  void Stepper::babystep(const uint8_t axis, const bool direction) {

    #define _ENABLE(axis) enable_## axis()
    #define _READ_DIR(AXIS) AXIS ##_DIR_READ
    #define _INVERT_DIR(AXIS) INVERT_## AXIS ##_DIR
    #define _APPLY_DIR(AXIS, INVERT) AXIS ##_APPLY_DIR(INVERT, true)

    #define BABYSTEP_AXIS(axis, AXIS, INVERT) { \
        _ENABLE(axis); \
        uint8_t old_pin = _READ_DIR(AXIS); \
        _APPLY_DIR(AXIS, _INVERT_DIR(AXIS)^direction^INVERT); \
        _APPLY_STEP(AXIS)(!_INVERT_STEP_PIN(AXIS), true); \
        HAL::delayMicroseconds(2U); \
        _APPLY_STEP(AXIS)(_INVERT_STEP_PIN(AXIS), true); \
        _APPLY_DIR(AXIS, old_pin); \
      }

    switch (axis) {

      case X_AXIS:
        BABYSTEP_AXIS(x, X, false);
        break;

      case Y_AXIS:
        BABYSTEP_AXIS(y, Y, false);
        break;

      case Z_AXIS: {

        #if !MECH(DELTA)

          BABYSTEP_AXIS(z, Z, BABYSTEP_INVERT_Z);

        #else // DELTA

          bool z_direction = direction ^ BABYSTEP_INVERT_Z;

          enable_x();
          enable_y();
          enable_z();
          uint8_t old_x_dir_pin = X_DIR_READ,
                  old_y_dir_pin = Y_DIR_READ,
                  old_z_dir_pin = Z_DIR_READ;
          // setup new step
          X_DIR_WRITE(INVERT_X_DIR ^ z_direction);
          Y_DIR_WRITE(INVERT_Y_DIR ^ z_direction);
          Z_DIR_WRITE(INVERT_Z_DIR ^ z_direction);
          // perform step
          X_STEP_WRITE(!INVERT_X_STEP_PIN);
          Y_STEP_WRITE(!INVERT_Y_STEP_PIN);
          Z_STEP_WRITE(!INVERT_Z_STEP_PIN);
          HAL::delayMicroseconds(1U);
          X_STEP_WRITE(INVERT_X_STEP_PIN);
          Y_STEP_WRITE(INVERT_Y_STEP_PIN);
          Z_STEP_WRITE(INVERT_Z_STEP_PIN);
          // get old pin state back.
          X_DIR_WRITE(old_x_dir_pin);
          Y_DIR_WRITE(old_y_dir_pin);
          Z_DIR_WRITE(old_z_dir_pin);

        #endif

      } break;

      default: break;
    }
  }

#endif //BABYSTEPPING

/**
 * Software-controlled Stepper Motor Current
 */
#if HAS(DIGIPOTSS)
  void Stepper::digitalPotWrite(int address, int value) {
    digitalWrite(DIGIPOTSS_PIN, LOW); // take the SS pin low to select the chip
    SPI.transfer(address); //  send in the address and value via SPI:
    SPI.transfer(value);
    digitalWrite(DIGIPOTSS_PIN, HIGH); // take the SS pin high to de-select the chip:
    //HAL::delayMilliseconds(10);
  }
#endif

void Stepper::digipot_init() {
  #if HAS(DIGIPOTSS)
    const uint8_t digipot_motor_current[] = DIGIPOT_MOTOR_CURRENT;

    SPI.begin();
    pinMode(DIGIPOTSS_PIN, OUTPUT);
    for (uint8_t i = 0; i < COUNT(digipot_motor_current); i++) {
      //digitalPotWrite(digipot_ch[i], digipot_motor_current[i]);
      digipot_current(i, digipot_motor_current[i]);
    }
  #endif
  #if HAS(MOTOR_CURRENT_PWM_XY)
    #if PIN_EXISTS(MOTOR_CURRENT_PWM_XY)
      pinMode(MOTOR_CURRENT_PWM_XY_PIN, OUTPUT);
      digipot_current(0, motor_current_setting[0]);
    #endif
    #if PIN_EXISTS(MOTOR_CURRENT_PWM_Z)
      pinMode(MOTOR_CURRENT_PWM_Z_PIN, OUTPUT);
      digipot_current(1, motor_current_setting[1]);
    #endif
    #if PIN_EXISTS(MOTOR_CURRENT_PWM_E)
      pinMode(MOTOR_CURRENT_PWM_E_PIN, OUTPUT);
      digipot_current(2, motor_current_setting[2]);
    #endif
    //Set timer5 to 31khz so the PWM of the motor power is as constant as possible. (removes a buzzing noise)
    TCCR5B = (TCCR5B & ~(_BV(CS50) | _BV(CS51) | _BV(CS52))) | _BV(CS50);
  #endif

  #if MB(ALLIGATOR)
    set_driver_current();
  #endif // MB(ALLIGATOR)
}

#if MB(ALLIGATOR)
  void Stepper::set_driver_current() {
    uint8_t digipot_motor = 0;
    for (uint8_t i = 0; i < 3 + E_STEPPERS; i++) {
      digipot_motor = 255 * motor_current[i] / 3.3;
      ExternalDac::setValue(i, digipot_motor);
    }
  }
#endif

void Stepper::digipot_current(uint8_t driver, int current) {
  #if HAS(DIGIPOTSS)
    const uint8_t digipot_ch[] = DIGIPOT_CHANNELS;
    digitalPotWrite(digipot_ch[driver], current);
  #elif HAS(MOTOR_CURRENT_PWM_XY)
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
  #else
    UNUSED(driver);
    UNUSED(current);
  #endif
}

void Stepper::microstep_init() {
  #if HAS(MICROSTEPS_E1)
    pinMode(E1_MS1_PIN, OUTPUT);
    pinMode(E1_MS2_PIN, OUTPUT);
  #endif

  #if HAS(MICROSTEPS)
    pinMode(X_MS1_PIN, OUTPUT);
    pinMode(X_MS2_PIN, OUTPUT);
    pinMode(Y_MS1_PIN, OUTPUT);
    pinMode(Y_MS2_PIN, OUTPUT);
    pinMode(Z_MS1_PIN, OUTPUT);
    pinMode(Z_MS2_PIN, OUTPUT);
    pinMode(E0_MS1_PIN, OUTPUT);
    pinMode(E0_MS2_PIN, OUTPUT);
    const uint8_t microstep_modes[] = MICROSTEP_MODES;
    for (uint16_t i = 0; i < COUNT(microstep_modes); i++)
      microstep_mode(i, microstep_modes[i]);
  #endif
}

/**
 * Software-controlled Microstepping
 */
void Stepper::microstep_ms(uint8_t driver, int8_t ms1, int8_t ms2) {
  if (ms1 >= 0) switch (driver) {
    case 0: digitalWrite(X_MS1_PIN, ms1); break;
    case 1: digitalWrite(Y_MS1_PIN, ms1); break;
    case 2: digitalWrite(Z_MS1_PIN, ms1); break;
    case 3: digitalWrite(E0_MS1_PIN, ms1); break;
    #if HAS(MICROSTEPS_E1)
      case 4: digitalWrite(E1_MS1_PIN, ms1); break;
    #endif
  }
  if (ms2 >= 0) switch (driver) {
    case 0: digitalWrite(X_MS2_PIN, ms2); break;
    case 1: digitalWrite(Y_MS2_PIN, ms2); break;
    case 2: digitalWrite(Z_MS2_PIN, ms2); break;
    case 3: digitalWrite(E0_MS2_PIN, ms2); break;
    #if PIN_EXISTS(E1_MS2)
      case 4: digitalWrite(E1_MS2_PIN, ms2); break;
    #endif
  }
}

void Stepper::microstep_mode(uint8_t driver, uint8_t stepping_mode) {
  switch (stepping_mode) {
    case 1: microstep_ms(driver,  MICROSTEP1); break;
    case 2: microstep_ms(driver,  MICROSTEP2); break;
    case 4: microstep_ms(driver,  MICROSTEP4); break;
    case 8: microstep_ms(driver,  MICROSTEP8); break;
    case 16: microstep_ms(driver, MICROSTEP16); break;
    #if MB(ALLIGATOR)
      case 32: microstep_ms(driver, MICROSTEP32); break;
    #endif
  }
}

void Stepper::microstep_readings() {
  SERIAL_M(MSG_MICROSTEP_MS1_MS2);
  SERIAL_M(MSG_MICROSTEP_X);
  SERIAL_V(digitalRead(X_MS1_PIN));
  SERIAL_EV(digitalRead(X_MS2_PIN));
  SERIAL_M(MSG_MICROSTEP_Y);
  SERIAL_V(digitalRead(Y_MS1_PIN));
  SERIAL_EV(digitalRead(Y_MS2_PIN));
  SERIAL_M(MSG_MICROSTEP_Z);
  SERIAL_V(digitalRead(Z_MS1_PIN));
  SERIAL_EV(digitalRead(Z_MS2_PIN));
  SERIAL_M(MSG_MICROSTEP_E0);
  SERIAL_V(digitalRead(E0_MS1_PIN));
  SERIAL_EV(digitalRead(E0_MS2_PIN));
  #if HAS(MICROSTEPS_E1)
    SERIAL_M(MSG_MICROSTEP_E1);
    SERIAL_V(digitalRead(E1_MS1_PIN));
    SERIAL_EV(digitalRead(E1_MS2_PIN));
  #endif
}
