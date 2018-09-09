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

#define NUM_HARDWARE_TIMERS 5

#define NvicPriorityUart    1
#define NvicPrioritySystick 2




#define HAL_TIMER_RATE              ((F_CPU)/2) // 24 MHz
#define HAL_ACCELERATION_RATE       (4096.0 * 4096.0 * 128.0 / (HAL_TIMER_RATE))

#define STEPPER_TIMER               2
#define STEPPER_TIMER_PRESCALE      2.0
#define STEPPER_TIMER_RATE          HAL_TIMER_RATE
#define STEPPER_TIMER_TICKS_PER_US  (HAL_TIMER_RATE / 1000000)  // 24 stepper timer ticks per us
#define STEPPER_TIMER_MIN_INTERVAL  5                                                         // minimum time in µs between stepper interrupts
#define STEPPER_TIMER_MAX_INTERVAL  (STEPPER_TIMER_TICKS_PER_US * STEPPER_TIMER_MIN_INTERVAL) // maximum time in µs between stepper interrupts
#define STEPPER_TIMER_ISR          void TC5_Handler()

#define PULSE_TIMER_NUM             STEPPER_TIMER
#define PULSE_TIMER_PRESCALE        STEPPER_TIMER_PRESCALE
#define STEPPER_ISR_ENABLED()       HAL_timer_interrupt_is_enabled(STEPPER_TIMER)
#define AD_PRESCALE_FACTOR          84  // 500 kHz ADC clock 
#define AD_TRACKING_CYCLES          4   // 0 - 15     + 1 adc clock cycles
#define AD_TRANSFER_CYCLES          1   // 0 - 3      * 2 + 3 adc clock cycles

#define ADC_ISR_EOC(channel)        (0x1u << channel)

#define HAL_STEPPER_TIMER_START()   HAL_timer_start(STEPPER_TIMER)
#define ENABLE_STEPPER_INTERRUPT()  HAL_timer_enable_interrupt(STEPPER_TIMER)
#define DISABLE_STEPPER_INTERRUPT() HAL_timer_disable_interrupt(STEPPER_TIMER)

#define HAL_ENABLE_ISRs()           ENABLE_STEPPER_INTERRUPT()
#define HAL_DISABLE_ISRs()          DISABLE_STEPPER_INTERRUPT()


#define ISRS_ENABLED()          (!__get_PRIMASK())
#define ENABLE_ISRS()           __enable_irq()
#define DISABLE_ISRS()          __disable_irq()


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

// Clock speed factor
#define CYCLES_PER_US               ((F_CPU) / 1000000L) // 84
// Stepper pulse duration, in cycles
#define STEP_PULSE_CYCLES           ((MINIMUM_STEPPER_PULSE) * CYCLES_PER_US)
#define ISR_BASE_CYCLES               752UL
#define HAL_TIMER_TYPE_MAX  0xFFFFFFFF

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
// Calculate the minimum time to start all stepper pulses in the ISR loop
#define MIN_ISR_START_LOOP_CYCLES     (ISR_START_X_STEPPER_CYCLES + ISR_START_Y_STEPPER_CYCLES + ISR_START_Z_STEPPER_CYCLES + ISR_START_E_STEPPER_CYCLES + ISR_START_MIXING_STEPPER_CYCLES)
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

  { TC3, 0, TC3_IRQn, 2 },  // 3 - Stepper
  { TC4, 1, TC4_IRQn, 0 },  // 4 -
  { TC5, 2, TC5_IRQn, 0 },  // 5 - [servo timer5]
  { TC6, 0, TC6_IRQn, 0 },  // 6 - Pin TC 4 - 5
  { TC7, 1, TC7_IRQn, 0 },  // 7 - Pin TC 3 - 10
};


extern uint32_t HAL_min_pulse_cycle,
                HAL_min_pulse_tick,
                HAL_add_pulse_ticks,
                HAL_min_isr_frequency,
                HAL_frequency_limit[8];

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency);
void HAL_timer_enable_interrupt(const uint8_t timer_num);
void HAL_timer_disable_interrupt(const uint8_t timer_num);


bool HAL_timer_interrupt_is_enabled(const uint8_t timer_num);

FORCE_INLINE static void HAL_timer_set_count(const uint8_t timer_num, hal_timer_t count) {
    const tTimerConfig * const pConfig = &TimerConfig[timer_num];
    pConfig->pTimerRegs->COUNT16.CC[0].reg = count;
}

FORCE_INLINE static hal_timer_t HAL_timer_get_count(const uint8_t timer_num) {
    const tTimerConfig * const pConfig = &TimerConfig[timer_num];
    return pConfig->pTimerRegs->COUNT16.CC[0].reg;
}

FORCE_INLINE static void HAL_timer_set_current_count(const uint8_t timer_num, const hal_timer_t count) {
    
    const tTimerConfig * const pConfig = &TimerConfig[timer_num];
    pConfig->pTimerRegs->COUNT16.COUNT.reg = count;
}

FORCE_INLINE static hal_timer_t HAL_timer_get_current_count(const uint8_t timer_num) {
    const tTimerConfig * const pConfig = &TimerConfig[timer_num];
    return pConfig->pTimerRegs->COUNT16.COUNT.reg;
    
}
FORCE_INLINE static void HAL_timer_isr_prologue(uint8_t timer_num) {
    const tTimerConfig * const pConfig = &TimerConfig[timer_num];
    pConfig->pTimerRegs->COUNT16.INTFLAG.bit.MC0 = 1;
}

uint32_t HAL_calc_timer_interval(uint32_t step_rate, uint8_t* loops, const uint8_t scale);
void HAL_calc_pulse_cycle();


#endif /* _HAL_TIMERS_DUE_H_ */
