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

#ifndef _HAL_TIMERS_DUE_H_
#define _HAL_TIMERS_DUE_H_

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

constexpr uint32_t  HAL_TIMER_RATE        = ((VARIANT_MCK) / 2); // 42 MHz
constexpr float     HAL_ACCELERATION_RATE = (4096.0 * 4096.0 * 256.0 / (HAL_TIMER_RATE));

#define STEPPER_TIMER               4
#define STEPPER_TIMER_PRESCALE      2.0
#define STEPPER_TIMER_TICKS_PER_US  ((HAL_TIMER_RATE) / 1000000)                              // 42 - stepper timer ticks per µs
#define STEPPER_TIMER_MIN_INTERVAL  4                                                         // minimum time in µs between stepper interrupts
#define STEPPER_TIMER_MAX_INTERVAL  (STEPPER_TIMER_TICKS_PER_US * STEPPER_TIMER_MIN_INTERVAL) // maximum time in µs between stepper interrupts
#define STEPPER_PULSE_CYCLES        ((MINIMUM_STEPPER_PULSE) * CYCLES_PER_US)                 // Stepper pulse duration, in cycles
#define STEPPER_TIMER_ISR           void TC4_Handler()

#define PULSE_TIMER_PRESCALE        STEPPER_TIMER_PRESCALE

#define AD_PRESCALE_FACTOR          84  // 500 kHz ADC clock 
#define AD_TRACKING_CYCLES          4   // 0 - 15     + 1 adc clock cycles
#define AD_TRANSFER_CYCLES          1   // 0 - 3      * 2 + 3 adc clock cycles

#define ADC_ISR_EOC(channel)        (0x1u << channel)

#define HAL_STEPPER_TIMER_START()   HAL_timer_start(STEPPER_TIMER, 122)
#define ENABLE_STEPPER_INTERRUPT()  HAL_timer_enable_interrupt(STEPPER_TIMER)
#define DISABLE_STEPPER_INTERRUPT() HAL_timer_disable_interrupt(STEPPER_TIMER)
#define STEPPER_ISR_ENABLED()       HAL_timer_interrupt_is_enabled(STEPPER_TIMER)

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

// --------------------------------------------------------------------------
// Private Variables
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency);

FORCE_INLINE static void HAL_timer_enable_interrupt(const uint8_t timer_num) {
  IRQn_Type IRQn = TimerConfig[timer_num].IRQ_Id;
  NVIC_EnableIRQ(IRQn);
}

FORCE_INLINE static void HAL_timer_disable_interrupt(const uint8_t timer_num) {
  IRQn_Type IRQn = TimerConfig[timer_num].IRQ_Id;
  NVIC_DisableIRQ(IRQn);
}

FORCE_INLINE static bool HAL_timer_interrupt_is_enabled(const uint8_t timer_num) {
  IRQn_Type IRQn = TimerConfig[timer_num].IRQ_Id;
  return (NVIC->ISER[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)));
}

FORCE_INLINE static hal_timer_t HAL_timer_get_count(const uint8_t timer_num) {
  const tTimerConfig * const pConfig = &TimerConfig[timer_num];
  return pConfig->pTimerRegs->TC_CHANNEL[pConfig->channel].TC_RC;
}

FORCE_INLINE static void HAL_timer_set_count(const uint8_t timer_num, const hal_timer_t count) {
  const tTimerConfig * const pConfig = &TimerConfig[timer_num];
  pConfig->pTimerRegs->TC_CHANNEL[pConfig->channel].TC_RC = count;
}

FORCE_INLINE static hal_timer_t HAL_timer_get_current_count(const uint8_t timer_num) {
  const tTimerConfig * const pConfig = &TimerConfig[timer_num];
  return pConfig->pTimerRegs->TC_CHANNEL[pConfig->channel].TC_CV;
}

FORCE_INLINE static void HAL_timer_restricts(const uint8_t timer_num, const uint16_t interval_ticks) {
  const hal_timer_t mincmp = HAL_timer_get_current_count(timer_num) + interval_ticks;
  if (HAL_timer_get_count(timer_num) < mincmp) HAL_timer_set_count(timer_num, mincmp);
}

FORCE_INLINE static void HAL_timer_isr_prologue(const uint8_t timer_num) {
  const tTimerConfig * const pConfig = &TimerConfig[timer_num];
  // Reading the status register clears the interrupt flag
  pConfig->pTimerRegs->TC_CHANNEL[pConfig->channel].TC_SR;
}

#endif /* _HAL_TIMERS_DUE_H_ */
