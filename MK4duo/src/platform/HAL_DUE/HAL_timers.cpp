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
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 *
 * ARDUINO_ARCH_SAM
 */

#ifdef ARDUINO_ARCH_SAM

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------
#include "../../../MK4duo.h"
#include "HAL_timers.h"

// --------------------------------------------------------------------------
// Externals
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Local defines
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------

const tTimerConfig TimerConfig [NUM_HARDWARE_TIMERS] = {
  { TC0, 0, TC0_IRQn, 0 },  // 0 - Pin TC 2 - 13
  { TC0, 1, TC1_IRQn, 0 },  // 1 - Pin TC 60 - 61
  { TC0, 2, TC2_IRQn, 0 },  // 2 - Pin TC 58 - 92
  { TC1, 0, TC3_IRQn, 14},  // 3 - [NEOPIXEL] and Tone
  { TC1, 1, TC4_IRQn, 2 },  // 4 - Stepper
  { TC1, 2, TC5_IRQn, 3 },  // 5 - [servo timer5]
  { TC2, 0, TC6_IRQn, 0 },  // 6 - Pin TC 4 - 5
  { TC2, 1, TC7_IRQn, 0 },  // 7 - Pin TC 3 - 10
  { TC2, 2, TC8_IRQn, 0 },  // 8 - Pin TC 11 - 12
};

uint32_t  HAL_min_pulse_cycle     = 0,
          HAL_pulse_high_tick     = 0,
          HAL_pulse_low_tick      = 0,
          HAL_frequency_limit[8]  = { 0 };

// --------------------------------------------------------------------------
// Private Variables
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Function prototypes
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Private functions
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

/*
  Timer_clock1: Prescaler 2 -> 42MHz
  Timer_clock2: Prescaler 8 -> 10.5MHz
  Timer_clock3: Prescaler 32 -> 2.625MHz
  Timer_clock4: Prescaler 128 -> 656.25kHz
*/

void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency/*=100*/) {

  Tc *tc = TimerConfig[timer_num].pTimerRegs;
  IRQn_Type IRQn = TimerConfig[timer_num].IRQ_Id;
  uint32_t channel = TimerConfig[timer_num].channel;

  // Disable interrupt, just in case it was already enabled
  NVIC_DisableIRQ(IRQn);

  // We NEED memory barriers to ensure Interrupts are actually disabled!
  // ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )
  __DSB();
  __ISB();

  // Disable timer interrupt
  tc->TC_CHANNEL[channel].TC_IDR = TC_IDR_CPCS;

  // Stop timer, just in case, to be able to reconfigure it
  TC_Stop(tc, channel);

  pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t)IRQn);
  NVIC_SetPriority(IRQn, TimerConfig[timer_num].priority);

  // wave mode, reset counter on match with RC,
  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1);

  // Set compare value
  TC_SetRC(tc, channel, VARIANT_MCK / 2 / frequency);

  // And start timer
  TC_Start(tc, channel);

  // enable interrupt on RC compare
  tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS;
  tc->TC_CHANNEL[channel].TC_IER = ~TC_IER_CPCS;

  // Finally, enable IRQ
  NVIC_EnableIRQ(IRQn);

}

uint32_t HAL_isr_execuiton_cycle(const uint32_t rate) {
  return (ISR_BASE_CYCLES + ISR_BEZIER_CYCLES + (ISR_LOOP_CYCLES) * rate + ISR_LA_BASE_CYCLES + ISR_LA_LOOP_CYCLES) / rate;
}

uint32_t HAL_ns_to_pulse_tick(const uint32_t ns) {
 return (ns + STEPPER_TIMER_PULSE_TICK_NS / 2) / STEPPER_TIMER_PULSE_TICK_NS;
}

void HAL_calc_pulse_cycle() {

  const uint32_t  HAL_min_step_period_ns = 1000000000UL / stepper.data.maximum_rate;
  uint32_t        HAL_min_pulse_high_ns,
                  HAL_min_pulse_low_ns;

  HAL_min_pulse_cycle = MAX((uint32_t)((F_CPU) / stepper.data.maximum_rate), ((F_CPU) / 500000UL) * MAX((uint32_t)stepper.data.minimum_pulse, 1UL));

  if (stepper.data.minimum_pulse) {
    HAL_min_pulse_high_ns = uint32_t(stepper.data.minimum_pulse) * 1000UL;
    HAL_min_pulse_low_ns  = MAX((HAL_min_step_period_ns - MIN(HAL_min_step_period_ns, HAL_min_pulse_high_ns)), HAL_min_pulse_high_ns);
  }
  else {
    HAL_min_pulse_high_ns = 500000000UL / stepper.data.maximum_rate;
    HAL_min_pulse_low_ns  = HAL_min_pulse_high_ns;
  }

  HAL_pulse_high_tick = uint32_t(HAL_ns_to_pulse_tick(HAL_min_pulse_high_ns - MIN(HAL_min_pulse_high_ns, (TIMER_SETUP_NS))));
  HAL_pulse_low_tick  = uint32_t(HAL_ns_to_pulse_tick(HAL_min_pulse_low_ns - MIN(HAL_min_pulse_low_ns, (TIMER_SETUP_NS))));

  // The stepping frequency limits for each multistepping rate
  HAL_frequency_limit[0] = ((F_CPU) / HAL_isr_execuiton_cycle(1))       ;
  HAL_frequency_limit[1] = ((F_CPU) / HAL_isr_execuiton_cycle(2))   >> 1;
  HAL_frequency_limit[2] = ((F_CPU) / HAL_isr_execuiton_cycle(4))   >> 2;
  HAL_frequency_limit[3] = ((F_CPU) / HAL_isr_execuiton_cycle(8))   >> 3;
  HAL_frequency_limit[4] = ((F_CPU) / HAL_isr_execuiton_cycle(16))  >> 4;
  HAL_frequency_limit[5] = ((F_CPU) / HAL_isr_execuiton_cycle(32))  >> 5;
  HAL_frequency_limit[6] = ((F_CPU) / HAL_isr_execuiton_cycle(64))  >> 6;
  HAL_frequency_limit[7] = ((F_CPU) / HAL_isr_execuiton_cycle(128)) >> 7;

}

#endif // ARDUINO_ARCH_SAM
