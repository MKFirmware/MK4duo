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
 * This is the main Hardware Abstraction Layer (HAL).
 * To make the firmware work with different processors and toolchains,
 * all hardware related code should be packed into the hal files.
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
 *
 * Description: HAL for Arduino Due and compatible (SAM3X8E)
 *
 * Contributors:
 * Copyright (c) 2014 Bob Cousins bobcousins42@googlemail.com
 *                    Nico Tonnhofer wurstnase.reprap@gmail.com
 *
 * Copyright (c) 2015 - 2016 Alberto Cotronei @MagoKimbra
 *
 * ARDUINO_ARCH_SAM
 */

#ifndef _HAL_TIMERS_H_
#define _HAL_TIMERS_H_

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include <stdint.h>

// --------------------------------------------------------------------------
// Defines
// --------------------------------------------------------------------------

#define NUM_HARDWARE_TIMERS 9

#define NvicPriorityUart    1
#define NvicPrioritySystick 2

#define HAL_TIMER_RATE              ((F_CPU) / 2) // 42 MHz
#define HAL_ACCELERATION_RATE       (4096.0 * 4096.0 * 256.0 / (HAL_TIMER_RATE))

#define STEPPER_TC                  (TC1)
#define STEPPER_TIMER               4
#define STEPPER_TIMER_ISR           void TC4_Handler()
#define STEPPER_TIMER_RATE          HAL_TIMER_RATE
#define STEPPER_TIMER_TICKS_PER_US  ((STEPPER_TIMER_RATE) / 1000000)                          // 42 - stepper timer ticks per µs
#define STEPPER_TIMER_PRESCALE      ((F_CPU / 1000000L) / STEPPER_TIMER_TICKS_PER_US)         // 2
#define STEPPER_TIMER_MIN_INTERVAL  1                                                         // minimum time in µs between stepper interrupts
#define STEPPER_TIMER_MAX_INTERVAL  (STEPPER_TIMER_TICKS_PER_US * STEPPER_TIMER_MIN_INTERVAL) // maximum time in µs between stepper interrupts
#define STEPPER_CLOCK_RATE          ((F_CPU) / 128)                                           // frequency of the clock used for stepper pulse timing
#define PULSE_TIMER_PRESCALE        STEPPER_TIMER_PRESCALE

#define AD_PRESCALE_FACTOR          84  // 500 kHz ADC clock 
#define AD_TRACKING_CYCLES          4   // 0 - 15     + 1 adc clock cycles
#define AD_TRANSFER_CYCLES          1   // 0 - 3      * 2 + 3 adc clock cycles

#define ADC_ISR_EOC(channel)        (0x1u << channel)

#define ENABLE_STEPPER_INTERRUPT()  HAL_timer_enable_interrupt(STEPPER_TIMER)
#define DISABLE_STEPPER_INTERRUPT() HAL_timer_disable_interrupt(STEPPER_TIMER)
#define STEPPER_ISR_ENABLED()       HAL_timer_interrupt_is_enabled(STEPPER_TIMER)

// Estimate the amount of time the ISR will take to execute
// The base ISR takes 792 cycles
#define ISR_BASE_CYCLES               792UL

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

// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------

typedef struct {
  Tc          *pTimerRegs;
  uint16_t    channel;
  IRQn_Type   IRQ_Id;
  uint8_t     priority;
} tTimerConfig;

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------

extern const tTimerConfig TimerConfig[];

extern uint32_t HAL_min_pulse_cycle,
                HAL_min_pulse_tick,
                HAL_add_pulse_ticks,
                HAL_min_isr_frequency,
                HAL_frequency_limit[8];

// --------------------------------------------------------------------------
// Private Variables
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency);

void HAL_calc_pulse_cycle();

uint32_t HAL_calc_timer_interval(uint32_t step_rate, uint8_t* loops, const uint8_t scale);

FORCE_INLINE static void HAL_timer_enable_interrupt(const uint8_t timer_num) {
  IRQn_Type IRQn = TimerConfig[timer_num].IRQ_Id;
  NVIC_EnableIRQ(IRQn);
}

FORCE_INLINE static void HAL_timer_disable_interrupt(const uint8_t timer_num) {
  IRQn_Type IRQn = TimerConfig[timer_num].IRQ_Id;
  NVIC_DisableIRQ(IRQn);

  // We NEED memory barriers to ensure Interrupts are actually disabled!
  // ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )
  __DSB();
  __ISB();
}

FORCE_INLINE static bool HAL_timer_interrupt_is_enabled(const uint8_t timer_num) {
  static const IRQn_Type IRQn = TimerConfig[timer_num].IRQ_Id;
  return (NVIC->ISER[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)));
}

FORCE_INLINE static uint32_t HAL_timer_get_count(const uint8_t timer_num) {
  static const tTimerConfig * const pConfig = &TimerConfig[timer_num];
  return pConfig->pTimerRegs->TC_CHANNEL[pConfig->channel].TC_RC;
}

FORCE_INLINE static void HAL_timer_set_count(const uint8_t timer_num, const uint32_t count) {
  static const tTimerConfig * const pConfig = &TimerConfig[timer_num];
  pConfig->pTimerRegs->TC_CHANNEL[pConfig->channel].TC_RC = count;
}

FORCE_INLINE static uint32_t HAL_timer_get_current_count(const uint8_t timer_num) {
  static const tTimerConfig * const pConfig = &TimerConfig[timer_num];
  return pConfig->pTimerRegs->TC_CHANNEL[pConfig->channel].TC_CV;
}

FORCE_INLINE static void HAL_timer_restricts(const uint8_t timer_num, const uint16_t interval_ticks) {
  static const uint32_t mincmp = HAL_timer_get_current_count(timer_num) + interval_ticks;
  if (HAL_timer_get_count(timer_num) < mincmp) HAL_timer_set_count(timer_num, mincmp);
}

FORCE_INLINE static void HAL_timer_isr_prologue(const uint8_t timer_num) {
  static const tTimerConfig * const pConfig = &TimerConfig[timer_num];
  // Reading the status register clears the interrupt flag
  pConfig->pTimerRegs->TC_CHANNEL[pConfig->channel].TC_SR;
}

#endif /* _HAL_TIMERS_DUE_H_ */
