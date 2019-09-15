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

#ifdef ARDUINO_ARCH_STM32

#include "../../../MK4duo.h"
#include "HAL_timers.h"

// ------------------------
// Local defines
// ------------------------

#define NUM_HARDWARE_TIMERS 2
#define STEP_TIMER_IRQ_ID TIM5_IRQn
#define TEMP_TIMER_IRQ_ID TIM7_IRQn

// ------------------------
// Private Variables
// ------------------------

stm32_timer_t TimerHandle[NUM_HARDWARE_TIMERS];

uint32_t  HAL_min_pulse_cycle     = 0,
          HAL_min_pulse_tick      = 0,
          HAL_add_pulse_ticks     = 0,
          HAL_frequency_limit[8]  = { 0 };

bool timers_initialized[NUM_HARDWARE_TIMERS] = {false};

// ------------------------
// Public functions
// ------------------------
uint32_t HAL_isr_execuiton_cycle(const uint32_t rate) {
  return (ISR_BASE_CYCLES + ISR_BEZIER_CYCLES + (ISR_LOOP_CYCLES) * rate + ISR_LA_BASE_CYCLES + ISR_LA_LOOP_CYCLES) / rate;
}

void HAL_calc_pulse_cycle() {
  HAL_min_pulse_cycle = MAX((uint32_t)((F_CPU) / stepper.data.maximum_rate), ((F_CPU) / 500000UL) * MAX((uint32_t)stepper.data.minimum_pulse, 1UL));
  HAL_min_pulse_tick  = uint32_t(stepper.data.minimum_pulse) * (STEPPER_TIMER_TICKS_PER_US);
  HAL_add_pulse_ticks = (HAL_min_pulse_cycle / (PULSE_TIMER_PRESCALE)) - HAL_min_pulse_tick;

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

void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency) {

  if (!timers_initialized[timer_num]) {
    constexpr uint32_t step_prescaler = STEPPER_TIMER_PRESCALE - 1,
                       temp_prescaler = TEMP_TIMER_PRESCALE - 1;
    switch (timer_num) {
      case STEP_TIMER_NUM:
        // STEPPER TIMER TIM5 - use a 32bit timer
        __HAL_RCC_TIM5_CLK_ENABLE();
        TimerHandle[timer_num].handle.Instance            = TIM5;
        TimerHandle[timer_num].handle.Init.Prescaler      = step_prescaler;
        TimerHandle[timer_num].handle.Init.CounterMode    = TIM_COUNTERMODE_UP;
        TimerHandle[timer_num].handle.Init.ClockDivision  = TIM_CLOCKDIVISION_DIV1;
        TimerHandle[timer_num].callback = (uint32_t)TC5_Handler;
        HAL_NVIC_SetPriority(STEP_TIMER_IRQ_ID, 1, 0);
        break;

      case TEMP_TIMER_NUM:
        // TEMP TIMER TIM7 - any available 16bit Timer (1 already used for PWM)
        __HAL_RCC_TIM7_CLK_ENABLE();
        TimerHandle[timer_num].handle.Instance            = TIM7;
        TimerHandle[timer_num].handle.Init.Prescaler      = temp_prescaler;
        TimerHandle[timer_num].handle.Init.CounterMode    = TIM_COUNTERMODE_UP;
        TimerHandle[timer_num].handle.Init.ClockDivision  = TIM_CLOCKDIVISION_DIV1;
        TimerHandle[timer_num].callback = (uint32_t)TC7_Handler;
        HAL_NVIC_SetPriority(TEMP_TIMER_IRQ_ID, 2, 0);
        break;
    }
    timers_initialized[timer_num] = true;
  }

  TimerHandle[timer_num].handle.Init.Period = (((HAL_TIMER_RATE) / TimerHandle[timer_num].handle.Init.Prescaler) / frequency) - 1;
  if (HAL_TIM_Base_Init(&TimerHandle[timer_num].handle) == HAL_OK)
    HAL_TIM_Base_Start_IT(&TimerHandle[timer_num].handle);
}

extern "C" void TIM5_IRQHandler() {
  ((void(*)(void))TimerHandle[0].callback)();
}
extern "C" void TIM7_IRQHandler() {
  ((void(*)(void))TimerHandle[1].callback)();
}

void HAL_timer_enable_interrupt(const uint8_t timer_num) {
  switch (timer_num) {
    case STEP_TIMER_NUM: HAL_NVIC_EnableIRQ(STEP_TIMER_IRQ_ID); break;
    case TEMP_TIMER_NUM: HAL_NVIC_EnableIRQ(TEMP_TIMER_IRQ_ID); break;
  }
}

void HAL_timer_disable_interrupt(const uint8_t timer_num) {
  switch (timer_num) {
    case STEP_TIMER_NUM: HAL_NVIC_DisableIRQ(STEP_TIMER_IRQ_ID); break;
    case TEMP_TIMER_NUM: HAL_NVIC_DisableIRQ(TEMP_TIMER_IRQ_ID); break;
  }
  // We NEED memory barriers to ensure Interrupts are actually disabled!
  // ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )
  __DSB();
  __ISB();
}

bool HAL_timer_interrupt_is_enabled(const uint8_t timer_num) {
  switch (timer_num) {
    case STEP_TIMER_NUM: return NVIC->ISER[(uint32_t)((int32_t)STEP_TIMER_IRQ_ID) >> 5] & (uint32_t)(1 << ((uint32_t)((int32_t)STEP_TIMER_IRQ_ID) & (uint32_t)0x1F));
    case TEMP_TIMER_NUM: return NVIC->ISER[(uint32_t)((int32_t)TEMP_TIMER_IRQ_ID) >> 5] & (uint32_t)(1 << ((uint32_t)((int32_t)TEMP_TIMER_IRQ_ID) & (uint32_t)0x1F));
  }
  return false;
}

#endif // ARDUINO_ARCH_STM32
