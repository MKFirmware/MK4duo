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

#if defined(ARDUINO_ARCH_STM32) && !defined(STM32GENERIC)

#include "../../../MK4duo.h"
#include "HAL_timers.h"

// ------------------------
// Public Variables
// ------------------------
hal_timer_t HAL_min_pulse_cycle     = 0,
            HAL_min_pulse_tick      = 0,
            HAL_add_pulse_ticks     = 0,
            HAL_frequency_limit[8]  = { 0 };
bool        HAL_timer_is_active     = false;

// ------------------------
// Hardware Timer
// ------------------------
HardwareTimer *MK_timer = nullptr;

// ------------------------
// Public functions
// ------------------------
hal_timer_t HAL_isr_execuiton_cycle(const hal_timer_t rate) {
  return (ISR_BASE_CYCLES + ISR_BEZIER_CYCLES + (ISR_LOOP_CYCLES) * rate + ISR_LA_BASE_CYCLES + ISR_LA_LOOP_CYCLES) / rate;
}

void HAL_calc_pulse_cycle() {
  HAL_min_pulse_cycle = MAX((hal_timer_t)((STEPPER_TIMER_RATE) / stepper.data.maximum_rate), ((STEPPER_TIMER_RATE) / 500000UL) * MAX((hal_timer_t)stepper.data.minimum_pulse, 1UL));

  if (stepper.data.minimum_pulse)
    HAL_min_pulse_tick = (STEPPER_TIMER_TICKS_PER_US) * hal_timer_t(stepper.data.minimum_pulse) + ((MIN_ISR_START_LOOP_CYCLES) / hal_timer_t(STEPPER_TIMER_PRESCALE));
  else
    HAL_min_pulse_tick = ((((STEPPER_TIMER_TICKS_PER_US) + 1) / 2) + ((MIN_ISR_START_LOOP_CYCLES) / hal_timer_t(STEPPER_TIMER_PRESCALE)));

  HAL_add_pulse_ticks = (HAL_min_pulse_cycle / hal_timer_t(STEPPER_TIMER_PRESCALE)) - HAL_min_pulse_tick;

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
    MK_timer = new HardwareTimer(STEP_TIMER);
    MK_timer->setPrescaleFactor(STEPPER_TIMER_PRESCALE);
    MK_timer->setOverflow(HAL_TIMER_TYPE_MAX, TICK_FORMAT);
    MK_timer->attachInterrupt(Step_Handler);
  }
}

void HAL_timer_enable_interrupt() {
  if (HAL_timer_initialized() && !HAL_timer_is_active) {
    MK_timer->resume();
    HAL_timer_is_active = true;
  }
}

void HAL_timer_disable_interrupt() {
  if (HAL_timer_initialized() && HAL_timer_is_active) {
    MK_timer->pause();
    HAL_timer_is_active = false;
  }
}

uint32_t HAL_timer_get_Clk_Freq() {
  return HAL_timer_initialized() ? MK_timer->getTimerClkFreq() : 0;
}

#endif // ARDUINO_ARCH_STM32
