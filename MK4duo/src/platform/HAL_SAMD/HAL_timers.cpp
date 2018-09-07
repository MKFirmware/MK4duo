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

#if ENABLED(ARDUINO_ARCH_SAMD)

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include "HAL_timers.h"

uint32_t  HAL_min_pulse_cycle     = 0,
          HAL_min_pulse_tick      = 0,
          HAL_add_pulse_ticks     = 0,
          HAL_min_isr_frequency   = 0,
          HAL_frequency_limit[8]  = { 0 };


/*
  Timer_clock1: Prescaler 2 -> 42MHz
  Timer_clock2: Prescaler 8 -> 10.5MHz
  Timer_clock3: Prescaler 32 -> 2.625MHz
  Timer_clock4: Prescaler 128 -> 656.25kHz
*/

/**
  \brief   Get Interrupt Enable status
  \details Returns a device specific interrupt enable status from the NVIC interrupt controller.
  \param [in]      IRQn  Device specific interrupt number.
  \return             0  Interrupt is not enabled.
  \return             1  Interrupt is enabled.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE uint32_t __NVIC_GetEnableIRQ(IRQn_Type IRQn)
{
    if ((int32_t)(IRQn) >= 0)
    {
        return((uint32_t)(((NVIC->ISER[(((uint32_t)(int32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
    }
    else
    {
        return(0U);
    }
}


bool tcIsSyncing()
{
  return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency) { /* moet nog iets met freq */
    SerialUSB.println("Init stepper");
    GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_TC4_TC5 )) ;
    while (GCLK->STATUS.bit.SYNCBUSY);
    
    TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
    while (tcIsSyncing());
    while (TC5->COUNT16.CTRLA.bit.SWRST);

    TimerConfig[timer_num].pTimerRegs->COUNT16.CTRLA.reg &= ~(TC_CTRLA_ENABLE);       //disable TC module
    TimerConfig[timer_num].pTimerRegs->COUNT16.CTRLA.reg |=TC_CTRLA_MODE_COUNT16;
    TimerConfig[timer_num].pTimerRegs->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
    TimerConfig[timer_num].pTimerRegs->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV2;
    TimerConfig[timer_num].pTimerRegs->COUNT16.CC[0].reg = VARIANT_MCK / 2/ frequency;
    TimerConfig[timer_num].pTimerRegs->COUNT16.INTENSET.reg = TC_INTFLAG_MC0;
    TimerConfig[timer_num].pTimerRegs->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
    TimerConfig[timer_num].pTimerRegs->COUNT16.INTFLAG.reg = 0xFF;
    while (tcIsSyncing());
    
    NVIC_DisableIRQ(TimerConfig[timer_num].IRQ_Id);
    NVIC_ClearPendingIRQ(TimerConfig[timer_num].IRQ_Id);
    NVIC_SetPriority(TimerConfig[timer_num].IRQ_Id, 0);
    NVIC_EnableIRQ(TimerConfig[timer_num].IRQ_Id);
    
}

void HAL_timer_enable_interrupt(const uint8_t timer_num) {
    NVIC_EnableIRQ(TimerConfig[timer_num].IRQ_Id);
    
  /*const tTimerConfig *pConfig = &TimerConfig[timer_num];
  TcCount16* TC = (TcCount16*) TC3;
  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync */
}

void HAL_timer_disable_interrupt (const uint8_t timer_num) {
    NVIC_DisableIRQ(TimerConfig[timer_num].IRQ_Id);
}

bool HAL_timer_interrupt_is_enabled(const uint8_t timer_num) {
    return __NVIC_GetEnableIRQ(TimerConfig[timer_num].IRQ_Id)==1;

}

uint32_t HAL_isr_execution_cycle(const uint32_t rate) {
  return (ISR_BASE_CYCLES + ISR_BEZIER_CYCLES + (ISR_LOOP_CYCLES * rate) + ISR_LA_BASE_CYCLES + ISR_LA_LOOP_CYCLES) / rate;
}

void HAL_calc_pulse_cycle() {
  HAL_min_pulse_cycle   = MAX((uint32_t)((F_CPU) / stepper.maximum_rate), ((F_CPU) / 500000UL) * (uint32_t)stepper.minimum_pulse);
  HAL_min_pulse_tick    = ((uint32_t)stepper.minimum_pulse * (STEPPER_TIMER_TICKS_PER_US)) + ((MIN_ISR_START_LOOP_CYCLES) / (uint32_t)(PULSE_TIMER_PRESCALE));
  HAL_add_pulse_ticks   = (HAL_min_pulse_cycle / (PULSE_TIMER_PRESCALE)) - HAL_min_pulse_tick;
  HAL_min_isr_frequency = (F_CPU) / HAL_isr_execution_cycle(1);

  // The stepping frequency limits for each multistepping rate
  HAL_frequency_limit[0] = ((F_CPU) / HAL_isr_execution_cycle(1))   >> 0;
  HAL_frequency_limit[1] = ((F_CPU) / HAL_isr_execution_cycle(2))   >> 1;
  HAL_frequency_limit[2] = ((F_CPU) / HAL_isr_execution_cycle(4))   >> 2;
  HAL_frequency_limit[3] = ((F_CPU) / HAL_isr_execution_cycle(8))   >> 3;
  HAL_frequency_limit[4] = ((F_CPU) / HAL_isr_execution_cycle(16))  >> 4;
  HAL_frequency_limit[5] = ((F_CPU) / HAL_isr_execution_cycle(32))  >> 5;
  HAL_frequency_limit[6] = ((F_CPU) / HAL_isr_execution_cycle(64))  >> 6;
  HAL_frequency_limit[7] = ((F_CPU) / HAL_isr_execution_cycle(128)) >> 7;
}


uint32_t HAL_calc_timer_interval(uint32_t step_rate, uint8_t* loops, const uint8_t scale) {

  uint8_t multistep = 1;

  // Scale the frequency, as requested by the caller
  step_rate <<= scale;

  #if DISABLED(DISABLE_DOUBLE_QUAD_STEPPING)
  // Select the proper multistepping
    uint8_t idx = 0;
    while (idx < 7 && step_rate > HAL_frequency_limit[idx]) {
      step_rate >>= 1;
      multistep <<= 1;
      ++idx;
    };
  #else
    NOMORE(step_rate, HAL_min_isr_frequency);
  #endif


  *loops = multistep;

  // In case of high-performance processor, it is able to calculate in real-time
  return uint32_t(HAL_TIMER_RATE) / step_rate;

}

STEPPER_TIMER_ISR {

  HAL_timer_isr_prologue(STEPPER_TIMER);

  // Call the Step
  stepper.Step();

}


#endif // ARDUINO_ARCH_SAMD
