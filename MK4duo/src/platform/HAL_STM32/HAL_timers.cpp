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

#if defined(ARDUINO_ARCH_STM32) && !defined(STM32GENERIC)

#include "../../../MK4duo.h"
#include "HAL_timers.h"

// ------------------------
// Public Variables
// ------------------------
uint32_t  HAL_min_pulse_cycle     = 0,
          HAL_pulse_high_tick     = 0,
          HAL_pulse_low_tick      = 0,
          HAL_frequency_limit[8]  = { 0 };

// ------------------------
// Private Variables
// ------------------------
bool timer_enabled = { false };

// ------------------------
// Hardware Timer
// ------------------------
HardwareTimer *MK_step_timer = nullptr;

// ------------------------
// Public functions
// ------------------------
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

void HAL_timer_start() {
  if (!HAL_timer_initialized()) {
    MK_step_timer = new HardwareTimer(STEP_TIMER);
    MK_step_timer->setMode(1, TIMER_OUTPUT_COMPARE);
    MK_step_timer->setInterruptPriority(NvicPriorityStepper, 0);
    MK_step_timer->setPrescaleFactor(STEPPER_TIMER_PRESCALE);
    MK_step_timer->setOverflow(200, TICK_FORMAT);
    MK_step_timer->attachInterrupt(Step_Handler);
    MK_step_timer->resume();
  }
}

void HAL_timer_enable_interrupt() {
  //if (HAL_timer_initialized() && !MK_step_timer->hasInterrupt()) {
  if (HAL_timer_initialized() && !timer_enabled) {
    MK_step_timer->attachInterrupt(Step_Handler);
    timer_enabled = true;
  }
}

void HAL_timer_disable_interrupt() {
  //if (HAL_timer_initialized() && MK_step_timer->hasInterrupt()) {
  if (HAL_timer_initialized() && timer_enabled) {
    MK_step_timer->detachInterrupt();
    timer_enabled = false;
  }
}

bool HAL_timer_interrupt_is_enabled() {
  //return HAL_timer_initialized() && MK_step_timer->hasInterrupt();
  return HAL_timer_initialized() && timer_enabled;
}

uint32_t HAL_timer_get_Clk_Freq() {
  return HAL_timer_initialized() ? MK_step_timer->getTimerClkFreq() : 0;
}

#endif // ARDUINO_ARCH_STM32
