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

#ifndef _HAL_TIMERS_DUE_H
#define _HAL_TIMERS_DUE_H

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include <stdint.h>

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
// Defines
// --------------------------------------------------------------------------

#define NUM_HARDWARE_TIMERS 9

#define DELAY_TIMER             TC1
#define DELAY_TIMER_CHANNEL     1
#define DELAY_TIMER_IRQ         ID_TC4  // IRQ not really used, needed for pmc id
#define DELAY_TIMER_CLOCK       TC_CMR_TCCLKS_TIMER_CLOCK2
#define DELAY_TIMER_PRESCALE    8

#define STEPPER_TIMER 2
#define STEPPER_TIMER_PRESCALE  2.0
#define HAL_STEPPER_TIMER_RATE      ((F_CPU) / STEPPER_TIMER_PRESCALE)  // 42 MHz
#define STEPPER_TIMER_TICKS_PER_US  (HAL_STEPPER_TIMER_RATE / 1000000)  // 42

#define TEMP_TIMER 3
#define TEMP_TIMER_FREQUENCY 3906

#define BEEPER_TIMER 4
#define BEEPER_TIMER_COUNTER TC1
#define BEEPER_TIMER_CHANNEL 1

#define AD_PRESCALE_FACTOR      84  // 500 kHz ADC clock 
#define AD_TRACKING_CYCLES      4   // 0 - 15     + 1 adc clock cycles
#define AD_TRANSFER_CYCLES      1   // 0 - 3      * 2 + 3 adc clock cycles

#define ADC_ISR_EOC(channel)    (0x1u << channel)

#define HAL_STEPPER_TIMER_START()           HAL_timer_start(STEPPER_TIMER, 122)
#define HAL_TEMP_TIMER_START()              HAL_timer_start(TEMP_TIMER, TEMP_TIMER_FREQUENCY)

#define ENABLE_STEPPER_INTERRUPT()          HAL_timer_enable_interrupt (STEPPER_TIMER)
#define DISABLE_STEPPER_INTERRUPT()         HAL_timer_disable_interrupt (STEPPER_TIMER)

#define ENABLE_TEMP_INTERRUPT()             HAL_timer_enable_interrupt (TEMP_TIMER)
#define DISABLE_TEMP_INTERRUPT()            HAL_timer_disable_interrupt (TEMP_TIMER)

#define HAL_TIMER_SET_STEPPER_COUNT(count)  HAL_timer_set_count(STEPPER_TIMER, count);
#define HAL_TIMER_SET_TEMP_COUNT(count)     HAL_timer_set_count(TEMP_TIMER, count);

#define HAL_STEP_TIMER_ISR    void TC2_Handler()
#define HAL_TEMP_TIMER_ISR    void TC3_Handler()
#define HAL_BEEPER_TIMER_ISR  void TC4_Handler()

#define _ENABLE_ISRs() \
        do { \
          ENABLE_TEMP_INTERRUPT(); \
          ENABLE_STEPPER_INTERRUPT(); \
        } while(0)

#define _DISABLE_ISRs() \
        do { \
          DISABLE_TEMP_INTERRUPT(); \
          DISABLE_STEPPER_INTERRUPT(); \
          sei(); \
        } while(0)

// Clock speed factor
#define CYCLES_PER_US ((F_CPU) / 1000000) // 84
// Stepper pulse duration, in cycles
#define STEP_PULSE_CYCLES ((MINIMUM_STEPPER_PULSE) * CYCLES_PER_US)

// Highly granular delays for step pulses, etc.
#define DELAY_0_NOP   NOOP
#define DELAY_1_NOP   __asm__("nop\n\t")
#define DELAY_2_NOP   DELAY_1_NOP;  DELAY_1_NOP
#define DELAY_3_NOP   DELAY_2_NOP;  DELAY_1_NOP
#define DELAY_4_NOP   DELAY_3_NOP;  DELAY_1_NOP
#define DELAY_5_NOP   DELAY_4_NOP;  DELAY_1_NOP
#define DELAY_10_NOP  DELAY_5_NOP;  DELAY_5_NOP
#define DELAY_20_NOP  DELAY_10_NOP; DELAY_10_NOP
#define DELAY_40_NOP  DELAY_20_NOP; DELAY_20_NOP
#define DELAY_80_NOP  DELAY_40_NOP; DELAY_40_NOP

#define DELAY_NOPS(X) \
  switch (X) { \
    case 20: DELAY_1_NOP; case 19: DELAY_1_NOP; \
    case 18: DELAY_1_NOP; case 17: DELAY_1_NOP; \
    case 16: DELAY_1_NOP; case 15: DELAY_1_NOP; \
    case 14: DELAY_1_NOP; case 13: DELAY_1_NOP; \
    case 12: DELAY_1_NOP; case 11: DELAY_1_NOP; \
    case 10: DELAY_1_NOP; case 9:  DELAY_1_NOP; \
    case 8:  DELAY_1_NOP; case 7:  DELAY_1_NOP; \
    case 6:  DELAY_1_NOP; case 5:  DELAY_1_NOP; \
    case 4:  DELAY_1_NOP; case 3:  DELAY_1_NOP; \
    case 2:  DELAY_1_NOP; case 1:  DELAY_1_NOP; \
  }

#define DELAY_1US   DELAY_80_NOP; DELAY_4_NOP
#define DELAY_2US   DELAY_1US;    DELAY_1US
#define DELAY_3US   DELAY_1US;    DELAY_2US
#define DELAY_4US   DELAY_1US;    DELAY_3US
#define DELAY_5US   DELAY_1US;    DELAY_4US
#define DELAY_6US   DELAY_1US;    DELAY_5US
#define DELAY_7US   DELAY_1US;    DELAY_6US
#define DELAY_8US   DELAY_1US;    DELAY_7US
#define DELAY_9US   DELAY_1US;    DELAY_8US
#define DELAY_10US  DELAY_1US;    DELAY_9US

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Private Variables
// --------------------------------------------------------------------------

static constexpr tTimerConfig TimerConfig [NUM_HARDWARE_TIMERS] = {
  { TC0, 0, TC0_IRQn, 0 },  // 0 - [servo timer5]
  { TC0, 1, TC1_IRQn, 0 },  // 1
  { TC0, 2, TC2_IRQn, 1 },  // 2 - stepper
  { TC1, 0, TC3_IRQn, 15},  // 3 - temperature
  { TC1, 1, TC4_IRQn, 0 },  // 4 - beeper
  { TC1, 2, TC5_IRQn, 0 },  // 5 - [servo timer3]
  { TC2, 0, TC6_IRQn, 0 },  // 6 - Adafruit Neopixel
  { TC2, 1, TC7_IRQn, 0 },  // 7
  { TC2, 2, TC8_IRQn, 0 },  // 8
};

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency);

static FORCE_INLINE void HAL_timer_set_count (uint8_t timer_num, uint32_t count) {
  const tTimerConfig *pConfig = &TimerConfig[timer_num];

  pConfig->pTimerRegs->TC_CHANNEL[pConfig->channel].TC_RC = count;
}

static FORCE_INLINE HAL_TIMER_TYPE HAL_timer_get_count (uint8_t timer_num) {
  const tTimerConfig *pConfig = &TimerConfig[timer_num];

  return pConfig->pTimerRegs->TC_CHANNEL[pConfig->channel].TC_RC;
}

static FORCE_INLINE uint32_t HAL_timer_get_current_count(uint8_t timer_num) {
  const tTimerConfig *pConfig = &TimerConfig[timer_num];

  return pConfig->pTimerRegs->TC_CHANNEL[pConfig->channel].TC_CV;
}

void HAL_timer_enable_interrupt(const uint8_t timer_num);
void HAL_timer_disable_interrupt(const uint8_t timer_num);

static FORCE_INLINE void HAL_timer_isr_prologue(uint8_t timer_num) {
  const tTimerConfig *pConfig = &TimerConfig[timer_num];

  // Reading the status register clears the interrupt flag
  pConfig->pTimerRegs->TC_CHANNEL[pConfig->channel].TC_SR;
}

// Tone
inline void HAL_timer_isr_status(Tc* tc, uint32_t channel) {
  tc->TC_CHANNEL[channel].TC_SR; // clear status register
}

void tone(uint8_t pin, int frequency, unsigned long duration);
void noTone(uint8_t pin);

#endif // _HAL_TIMERS_DUE_H
