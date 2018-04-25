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

#include "../../../MK4duo.h"

#if ENABLED(ARDUINO_ARCH_SAM)

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include "HAL_timers_Due.h"

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
  { TC1, 0, TC3_IRQn, 0 },  // 3 - [NEOPIXEL]
  { TC1, 1, TC4_IRQn, 2 },  // 4 - Stepper
  { TC1, 2, TC5_IRQn, 0 },  // 5 - [servo timer5]
  { TC2, 0, TC6_IRQn, 0 },  // 6 - Pin TC 4 - 5
  { TC2, 1, TC7_IRQn, 0 },  // 7 - Pin TC 3 - 10
  { TC2, 2, TC8_IRQn, 0 },  // 8 - Pin TC 11 - 12
};

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

void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency) {

	Tc *tc = TimerConfig [timer_num].pTimerRegs;
	IRQn_Type irq = TimerConfig [timer_num].IRQ_Id;
	uint32_t channel = TimerConfig [timer_num].channel;

  // Disable interrupt, just in case it was already enabled
  NVIC_DisableIRQ(irq);

  // Disable timer interrupt
  tc->TC_CHANNEL[channel].TC_IDR = TC_IDR_CPCS;

  // Stop timer, just in case, to be able to reconfigure it
  TC_Stop(tc, channel);

	pmc_set_writeprotect(false);
	pmc_enable_periph_clk((uint32_t)irq);
  NVIC_SetPriority (irq, TimerConfig [timer_num].priority);

  // wave mode, reset counter on match with RC,
  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_EEVT_XC0);

  // Set compare value
  TC_SetRC(tc, channel, VARIANT_MCK / 2 / frequency);

  // And start timer
  TC_Start(tc, channel);

  // enable interrupt on RC compare
  tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS;

  // Finally, enable IRQ
	NVIC_EnableIRQ(irq);
}

void HAL_timer_enable_interrupt(const uint8_t timer_num) {
  const tTimerConfig * const pConfig = &TimerConfig[timer_num];
  pConfig->pTimerRegs->TC_CHANNEL [pConfig->channel].TC_IER = TC_IER_CPCS;
}

void HAL_timer_disable_interrupt (const uint8_t timer_num) {
	const tTimerConfig * const pConfig = &TimerConfig [timer_num];
	pConfig->pTimerRegs->TC_CHANNEL [pConfig->channel].TC_IDR = TC_IDR_CPCS;
}

bool HAL_timer_interrupt_is_enabled(const uint8_t timer_num) {
  const tTimerConfig * const pConfig = &TimerConfig[timer_num];
  return (pConfig->pTimerRegs->TC_CHANNEL[pConfig->channel].TC_IMR & TC_IMR_CPCS);
}

#endif // ARDUINO_ARCH_SAM
