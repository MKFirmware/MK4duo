/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (c) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 * Copyright (c) 2017 Victor Perez
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#pragma once

#include <stdint.h>

// ------------------------
// Defines
// ------------------------
#define NUM_HARDWARE_TIMERS 9

#define FORCE_INLINE __attribute__((always_inline)) inline

// Tone for due
#define TONE_TIMER_NUM
#define HAL_TONE_TIMER_ISR()

#define HAL_TIMER_RATE              (HAL_RCC_GetSysClockFreq() / 2)  // frequency of timer peripherals

// Stepper Timer
#define STEP_TIMER_NUM              0                                           // index of timer to use for stepper
#define STEPPER_TIMER_RATE          (HAL_TIMER_RATE / STEPPER_TIMER_PRESCALE)   // frequency of stepper timer
#define STEPPER_TIMER_TICKS_PER_US  ((STEPPER_TIMER_RATE) / 1000000)            // stepper timer ticks per µs
#define STEPPER_TIMER_PRESCALE      54                                          // was 40,prescaler for setting stepper timer, 2Mhz
#define STEPPER_TIMER_MIN_INTERVAL  1                                                         // minimum time in µs between stepper interrupts
#define STEPPER_TIMER_MAX_INTERVAL  (STEPPER_TIMER_TICKS_PER_US * STEPPER_TIMER_MIN_INTERVAL) // maximum time in µs between stepper interrupts
#define STEPPER_CLOCK_RATE          ((F_CPU) / 128)                                           // frequency of the clock used for stepper pulse timing
#define PULSE_TIMER_PRESCALE        STEPPER_TIMER_PRESCALE

// Temperature Timer
#define TEMP_TIMER_NUM              1  // index of timer to use for temperature
#define TEMP_TIMER_PRESCALE         1000 // prescaler for setting Temp timer, 72Khz
#define TEMP_TIMER_FREQUENCY        1000 // temperature interrupt frequency

#define ENABLE_STEPPER_INTERRUPT()  HAL_timer_enable_interrupt(STEP_TIMER_NUM)
#define DISABLE_STEPPER_INTERRUPT() HAL_timer_disable_interrupt(STEP_TIMER_NUM)
#define STEPPER_ISR_ENABLED()       HAL_timer_interrupt_is_enabled(STEP_TIMER_NUM)

#define ENABLE_TEMP_INTERRUPT()     HAL_timer_enable_interrupt(TEMP_TIMER_NUM)
#define DISABLE_TEMP_INTERRUPT()    HAL_timer_disable_interrupt(TEMP_TIMER_NUM)
#define TEMP_ISR_ENABLED()          HAL_timer_interrupt_is_enabled(TEMP_TIMER_NUM)

// TODO change this

#ifdef STM32GENERIC
  extern void TC5_Handler();
  extern void TC7_Handler();
  #define HAL_STEPPER_TIMER_ISR() void TC5_Handler()
  #define HAL_TEMP_TIMER_ISR()    void TC7_Handler()
#else
  extern void TC5_Handler(stimer_t *htim);
  extern void TC7_Handler(stimer_t *htim);
  #define HAL_STEPPER_TIMER_ISR() void TC5_Handler(stimer_t *htim)
  #define HAL_TEMP_TIMER_ISR()    void TC7_Handler(stimer_t *htim)
#endif

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
// Types
// ------------------------

#ifdef STM32GENERIC
  typedef struct {
    TIM_HandleTypeDef handle;
    uint32_t callback;
  } tTimerConfig;
  typedef tTimerConfig stm32_timer_t;
#else
  typedef stimer_t stm32_timer_t;
#endif

// ------------------------
// Public Variables
// ------------------------

extern stm32_timer_t TimerHandle[];

extern uint32_t HAL_min_pulse_cycle,
                HAL_min_pulse_tick,
                HAL_add_pulse_ticks,
                HAL_frequency_limit[8];

// ------------------------
// Public functions
// ------------------------

void HAL_calc_pulse_cycle();
void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency);
void HAL_timer_enable_interrupt(const uint8_t timer_num);
void HAL_timer_disable_interrupt(const uint8_t timer_num);
bool HAL_timer_interrupt_is_enabled(const uint8_t timer_num);

FORCE_INLINE static uint32_t HAL_timer_get_count(const uint8_t timer_num) {
  return __HAL_TIM_GET_COUNTER(&TimerHandle[timer_num].handle);
}

FORCE_INLINE static void HAL_timer_set_count(const uint8_t timer_num, const uint32_t count) {
  __HAL_TIM_SET_AUTORELOAD(&TimerHandle[timer_num].handle, count);
  if (HAL_timer_get_count(timer_num) >= count)
    TimerHandle[timer_num].handle.Instance->EGR |= TIM_EGR_UG; // Generate an immediate update interrupt
}

FORCE_INLINE static uint32_t HAL_timer_get_current_count(const uint8_t timer_num) {
  return __HAL_TIM_GET_AUTORELOAD(&TimerHandle[timer_num].handle);
}

#ifdef STM32GENERIC
  FORCE_INLINE static void HAL_timer_isr_prologue(const uint8_t timer_num) {
    if (__HAL_TIM_GET_FLAG(&TimerHandle[timer_num].handle, TIM_FLAG_UPDATE) == SET)
      __HAL_TIM_CLEAR_FLAG(&TimerHandle[timer_num].handle, TIM_FLAG_UPDATE);
  }
#else
  #define HAL_timer_isr_prologue(TIMER_NUM)
#endif
