/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2019 Alberto Cotronei @MagoKimbra
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

#ifdef STM32F0xx

  #define HAL_TIMER_RATE (F_CPU) // frequency of timer peripherals

  #ifndef STEP_TIMER
    #define STEP_TIMER TIM16
  #endif

  #ifndef TEMP_TIMER
    #define TEMP_TIMER TIM17
  #endif

#elif defined(STM32F1xx)

  #define HAL_TIMER_RATE (F_CPU) // frequency of timer peripherals

  #ifndef STEP_TIMER
    #define STEP_TIMER TIM4
  #endif

  #ifndef TEMP_TIMER
    #define TEMP_TIMER TIM2
  #endif

#elif defined(STM32F4xx) || defined(STM32F7xx)

  #define HAL_TIMER_RATE (F_CPU / 2)

  #ifndef STEP_TIMER
    #define STEP_TIMER TIM5
  #endif

  #ifndef TEMP_TIMER
    #define TEMP_TIMER TIM4
  #endif

#endif

// Stepper Timer
#define STEPPER_TIMER_NUM           0                                           // index of timer to use for stepper
#define STEPPER_TIMER_RATE          2000000                                     // frequency of stepper timer
#define STEPPER_TIMER_TICKS_PER_US  ((STEPPER_TIMER_RATE) / 1000000)            // stepper timer ticks per µs
#define STEPPER_TIMER_PRESCALE      ((HAL_TIMER_RATE)/(STEPPER_TIMER_RATE))
#define STEPPER_TIMER_MIN_INTERVAL  1                                                         // minimum time in µs between stepper interrupts
#define STEPPER_TIMER_MAX_INTERVAL  (STEPPER_TIMER_TICKS_PER_US * STEPPER_TIMER_MIN_INTERVAL) // maximum time in µs between stepper interrupts
#define STEPPER_CLOCK_RATE          ((F_CPU) / 128)                                           // frequency of the clock used for stepper pulse timing
#define PULSE_TIMER_PRESCALE        STEPPER_TIMER_RATE

// Temperature Timer
#define TEMP_TIMER_NUM              1     // index of timer to use for temperature
#define TEMP_TIMER_FREQUENCY        1000

#define ENABLE_STEPPER_INTERRUPT()  HAL_timer_enable_interrupt(STEPPER_TIMER_NUM)
#define DISABLE_STEPPER_INTERRUPT() HAL_timer_disable_interrupt(STEPPER_TIMER_NUM)
#define STEPPER_ISR_ENABLED()       HAL_timer_interrupt_is_enabled(STEPPER_TIMER_NUM)

#define ENABLE_TEMP_INTERRUPT()     HAL_timer_enable_interrupt(TEMP_TIMER_NUM)
#define DISABLE_TEMP_INTERRUPT()    HAL_timer_disable_interrupt(TEMP_TIMER_NUM)
#define TEMP_ISR_ENABLED()          HAL_timer_interrupt_is_enabled(TEMP_TIMER_NUM)

// Estimate the amount of time the ISR will take to execute
// The base ISR takes 752 cycles
#define ISR_BASE_CYCLES               752UL

// Linear advance base time is 64 cycles
#if ENABLED(LIN_ADVANCE)
  #define ISR_LA_BASE_CYCLES          64UL
#else
  #define ISR_LA_BASE_CYCLES          0UL
#endif

// Bezier interpolation adds 40 cycles
#if ENABLED(BEZIER_JERK_CONTROL)
  #define ISR_BEZIER_CYCLES           40UL
#else
  #define ISR_BEZIER_CYCLES           0UL
#endif

// Stepper Loop base cycles
#define ISR_LOOP_BASE_CYCLES          4UL

// To start the step pulse, in the worst case takes
#define ISR_START_STEPPER_CYCLES      13UL

// And each stepper (start + stop pulse) takes in worst case
#define ISR_STEPPER_CYCLES            16UL

// For each stepper, we add its time
#if HAS_X_STEP
  #define ISR_START_X_STEPPER_CYCLES  ISR_START_STEPPER_CYCLES
  #define ISR_X_STEPPER_CYCLES        ISR_STEPPER_CYCLES
#else
  #define ISR_START_X_STEPPER_CYCLES  0UL
  #define ISR_X_STEPPER_CYCLES        0UL
#endif
#if HAS_Y_STEP
  #define ISR_START_Y_STEPPER_CYCLES  ISR_START_STEPPER_CYCLES
  #define ISR_Y_STEPPER_CYCLES        ISR_STEPPER_CYCLES
#else
  #define ISR_START_Y_STEPPER_CYCLES  0UL
  #define ISR_Y_STEPPER_CYCLES        0UL
#endif
#if HAS_Z_STEP
  #define ISR_START_Z_STEPPER_CYCLES  ISR_START_STEPPER_CYCLES
  #define ISR_Z_STEPPER_CYCLES        ISR_STEPPER_CYCLES
#else
  #define ISR_START_Z_STEPPER_CYCLES  0UL
  #define ISR_Z_STEPPER_CYCLES        0UL
#endif

// E is always interpolated
#define ISR_START_E_STEPPER_CYCLES    ISR_START_STEPPER_CYCLES
#define ISR_E_STEPPER_CYCLES          ISR_STEPPER_CYCLES

// If linear advance is disabled, then the loop also handles them
#if DISABLED(LIN_ADVANCE) && ENABLED(COLOR_MIXING_EXTRUDER)
  #define ISR_START_MIXING_STEPPER_CYCLES ((MIXING_STEPPERS) * 13UL)
  #define ISR_MIXING_STEPPER_CYCLES       ((MIXING_STEPPERS) * 16UL)
#else
  #define ISR_START_MIXING_STEPPER_CYCLES 0UL
  #define ISR_MIXING_STEPPER_CYCLES       0UL
#endif

// Calculate the minimum time to start all stepper pulses in the ISR loop
#define MIN_ISR_START_LOOP_CYCLES     (ISR_START_X_STEPPER_CYCLES + ISR_START_Y_STEPPER_CYCLES + ISR_START_Z_STEPPER_CYCLES + ISR_START_E_STEPPER_CYCLES + ISR_START_MIXING_STEPPER_CYCLES)

// And the total minimum loop time is, without including the base
#define MIN_ISR_LOOP_CYCLES           (ISR_X_STEPPER_CYCLES + ISR_Y_STEPPER_CYCLES + ISR_Z_STEPPER_CYCLES + ISR_E_STEPPER_CYCLES + ISR_MIXING_STEPPER_CYCLES)

// But the user could be enforcing a minimum time, so the loop time is
#define ISR_LOOP_CYCLES               (ISR_LOOP_BASE_CYCLES + MAX(HAL_min_pulse_cycle, MIN_ISR_LOOP_CYCLES))

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
                HAL_min_pulse_tick,
                HAL_add_pulse_ticks,
                HAL_frequency_limit[8];
extern bool     HAL_timer_is_active[2];

// ------------------------
// Hardware Timer
// ------------------------
extern HardwareTimer *MK_timer[2];

// ------------------------
// Public functions
// ------------------------
void HAL_calc_pulse_cycle();

// ------------------------
// Public functions for timer
// ------------------------
extern void Step_Handler(HardwareTimer*);
extern void Temp_Handler(HardwareTimer*);

FORCE_INLINE static bool HAL_timer_initialized(const uint8_t timer_num) {
  return MK_timer[timer_num] != nullptr;
}

FORCE_INLINE static void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency) {

  if (!HAL_timer_initialized(timer_num)) {

    switch (timer_num) {
      case STEPPER_TIMER_NUM:
        MK_timer[STEPPER_TIMER_NUM] = new HardwareTimer(STEP_TIMER);
        MK_timer[STEPPER_TIMER_NUM]->setPrescaleFactor(STEPPER_TIMER_PRESCALE);
        MK_timer[STEPPER_TIMER_NUM]->attachInterrupt(Step_Handler);
        MK_timer[STEPPER_TIMER_NUM]->resume();
        HAL_timer_is_active[STEPPER_TIMER_NUM] = true;
        break;

      case TEMP_TIMER_NUM:
        MK_timer[TEMP_TIMER_NUM] = new HardwareTimer(TEMP_TIMER);
        MK_timer[TEMP_TIMER_NUM]->setOverflow(frequency, HERTZ_FORMAT);
        MK_timer[TEMP_TIMER_NUM]->attachInterrupt(Temp_Handler);
        MK_timer[TEMP_TIMER_NUM]->resume();
        HAL_timer_is_active[TEMP_TIMER_NUM] = true;
        break;
    }

  }
}

FORCE_INLINE static void HAL_timer_enable_interrupt(const uint8_t timer_num) {
  if (HAL_timer_initialized(timer_num) && !HAL_timer_is_active[timer_num]) {
    MK_timer[timer_num]->resume();
    HAL_timer_is_active[timer_num] = true;
  }
}

FORCE_INLINE static void HAL_timer_disable_interrupt(const uint8_t timer_num) {
  if (HAL_timer_initialized(timer_num) && HAL_timer_is_active[timer_num]) {
    MK_timer[timer_num]->pause();
    HAL_timer_is_active[timer_num] = false;
  }
}

FORCE_INLINE static bool HAL_timer_interrupt_is_enabled(const uint8_t timer_num) {
  return HAL_timer_is_active[timer_num];
}

FORCE_INLINE static uint32_t HAL_timer_get_count(const uint8_t timer_num) {
  return MK_timer[timer_num]->getCount();
}

FORCE_INLINE static void HAL_timer_set_count(const uint8_t timer_num, const uint32_t count) {
  if (HAL_timer_initialized(timer_num)) {
    MK_timer[timer_num]->setOverflow(count);
    if (MK_timer[timer_num]->getCount() >= count)
      MK_timer[timer_num]->refresh(); // Generate an immediate update interrupt
  }
}

FORCE_INLINE static uint32_t HAL_timer_get_current_count(const uint8_t timer_num) {
  return HAL_timer_initialized(timer_num) ? MK_timer[timer_num]->getOverflow() : 0;
}

FORCE_INLINE static uint32_t HAL_timer_get_Clk_Freq(const uint8_t timer_num) {
  return HAL_timer_initialized(timer_num) ? MK_timer[timer_num]->getTimerClkFreq() : 0;
}

#define HAL_timer_isr_prologue(TIMER_NUM)
#define HAL_timer_isr_epilogue(TIMER_NUM)
