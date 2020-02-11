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

// ------------------------
// Defines
// ------------------------
#define FORCE_INLINE __attribute__((always_inline)) inline

#define HAL_TIMER_TYPE_MAX          0xFFFFFFFF
#define HAL_TIMER_RATE              ((F_CPU)/2)
#define NUM_HARDWARE_TIMERS         1                                           // Only Stepper use Hardware Timer
#define NvicPriorityStepper         2
#define NvicPrioritySystick         15

// Stepper Timer
#define STEPPER_TIMER_NUM           0                                           // Index of timer to use for stepper
#define STEPPER_TIMER_PRESCALE      2                                           // Stepper prescaler 2
#define STEPPER_TIMER_RATE          ((HAL_TIMER_RATE)/STEPPER_TIMER_PRESCALE)   // Frequency of stepper timer 42Mhz
#define STEPPER_TIMER_TICKS_PER_US  ((STEPPER_TIMER_RATE)/1000000UL)            // 42 Stepper timer ticks per µs
#define STEPPER_TIMER_PULSE_TICK_NS (1000000000UL / STEPPER_TIMER_RATE)
#define STEPPER_TIMER_MIN_INTERVAL  1                                                         // minimum time in µs between stepper interrupts
#define STEPPER_TIMER_MAX_INTERVAL  (STEPPER_TIMER_TICKS_PER_US * STEPPER_TIMER_MIN_INTERVAL) // maximum time in µs between stepper interrupts

#define START_STEPPER_INTERRUPT()   HAL_timer_start()
#define ENABLE_STEPPER_INTERRUPT()  HAL_timer_enable_interrupt()
#define DISABLE_STEPPER_INTERRUPT() HAL_timer_disable_interrupt()
#define STEPPER_ISR_ENABLED()       HAL_timer_interrupt_is_enabled()

// Estimate the amount of time the ISR will take to execute
#define TIMER_CYCLES                34UL

// The base ISR takes 792 cycles
#define ISR_BASE_CYCLES            792UL

// Linear advance base time is 64 cycles
#if ENABLED(LIN_ADVANCE)
  #define ISR_LA_BASE_CYCLES        64UL
#else
  #define ISR_LA_BASE_CYCLES         0UL
#endif

// Bezier interpolation adds 40 cycles
#if ENABLED(BEZIER_JERK_CONTROL)
  #define ISR_BEZIER_CYCLES         40UL
#else
  #define ISR_BEZIER_CYCLES          0UL
#endif

// Stepper Loop base cycles
#define ISR_LOOP_BASE_CYCLES         4UL

// And each stepper (start + stop pulse) takes in worst case
#define ISR_STEPPER_CYCLES          16UL

// For each stepper, we add its time
#if HAS_X_STEP
  #define ISR_X_STEPPER_CYCLES        ISR_STEPPER_CYCLES
#else
  #define ISR_X_STEPPER_CYCLES        0UL
#endif
#if HAS_Y_STEP
  #define ISR_Y_STEPPER_CYCLES        ISR_STEPPER_CYCLES
#else
  #define ISR_Y_STEPPER_CYCLES        0UL
#endif
#if HAS_Z_STEP
  #define ISR_Z_STEPPER_CYCLES        ISR_STEPPER_CYCLES
#else
  #define ISR_Z_STEPPER_CYCLES        0UL
#endif

// E is always interpolated
#define ISR_E_STEPPER_CYCLES          ISR_STEPPER_CYCLES

// If linear advance is disabled, then the loop also handles them
#if DISABLED(LIN_ADVANCE) && ENABLED(COLOR_MIXING_EXTRUDER)
  #define ISR_MIXING_STEPPER_CYCLES   ((MIXING_STEPPERS) * 16UL)
#else
  #define ISR_MIXING_STEPPER_CYCLES   0UL
#endif

// And the total minimum loop time is, without including the base
#define MIN_ISR_LOOP_CYCLES           (ISR_X_STEPPER_CYCLES + ISR_Y_STEPPER_CYCLES + ISR_Z_STEPPER_CYCLES + ISR_E_STEPPER_CYCLES + ISR_MIXING_STEPPER_CYCLES)

// But the user could be enforcing a minimum time, so the loop time is
#define ISR_LOOP_CYCLES               (ISR_LOOP_BASE_CYCLES + MAX(HAL_min_pulse_cycle, MIN_ISR_LOOP_CYCLES))

#define TIMER_SETUP_NS                (1000UL * TIMER_CYCLES / ((F_CPU) / 1000000UL))

// If linear advance is enabled, then it is handled separately
#if ENABLED(LIN_ADVANCE)

  // Estimate the minimum LA loop time
  #if ENABLED(COLOR_MIXING_EXTRUDER)
    #define MIN_ISR_LA_LOOP_CYCLES  ((MIXING_STEPPERS) * 16UL)
  #else
    #define MIN_ISR_LA_LOOP_CYCLES  16UL
  #endif

  // And the real loop time
  #define ISR_LA_LOOP_CYCLES  MAX(HAL_min_pulse_cycle, MIN_ISR_LA_LOOP_CYCLES)

#else
  #define ISR_LA_LOOP_CYCLES  0UL
#endif

// ------------------------
// Public Variables
// ------------------------
extern uint32_t HAL_min_pulse_cycle,
                HAL_pulse_high_tick,
                HAL_pulse_low_tick,
                HAL_frequency_limit[8];

// ------------------------
// Hardware Timer
// ------------------------
extern HardwareTimer *MK_step_timer;

// ------------------------
// Public functions for timer
// ------------------------
extern void Step_Handler(HardwareTimer*);

// ------------------------
// Public functions
// ------------------------
void HAL_calc_pulse_cycle();
void HAL_timer_start();
void HAL_timer_enable_interrupt();
void HAL_timer_disable_interrupt();
bool HAL_timer_interrupt_is_enabled();
uint32_t HAL_timer_get_Clk_Freq();

FORCE_INLINE bool HAL_timer_initialized() {
  return MK_step_timer != nullptr;
}

FORCE_INLINE uint32_t HAL_timer_get_current_count(const uint8_t) {
  return HAL_timer_initialized() ? MK_step_timer->getCount() : 0;
}

FORCE_INLINE void HAL_timer_set_count(const uint8_t, const uint32_t count) {
  if (HAL_timer_initialized()) {
    MK_step_timer->setOverflow(count + 1, TICK_FORMAT);
    if (count < MK_step_timer->getCount()) STEP_TIMER->EGR |= TIM_EGR_UG; // Generate an immediate update interrupt
  }
}
