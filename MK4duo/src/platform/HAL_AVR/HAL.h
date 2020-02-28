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
 * Description:          *** HAL for Arduino ***
 *
 * Contributors:
 * Copyright (c) 2014 Bob Cousins bobcousins42@googlemail.com
 *                    Nico Tonnhofer wurstnase.reprap@gmail.com
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 *
 * ARDUINO_ARCH_AVR
 */

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------
#include <math.h>
#include <stdint.h>
#include <Arduino.h>
#include <Wire.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>

// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------
typedef uint16_t  hal_timer_t;
typedef uint16_t  ptr_int_t;

// --------------------------------------------------------------------------
// Defines
// --------------------------------------------------------------------------
#ifndef WIRE_PORT
  #define WIRE_PORT 1
#endif

#if (WIRE_PORT == 2)
  #define WIRE  Wire1
#else
  #define WIRE  Wire
#endif

// A SW memory barrier, to ensure GCC does not overoptimize loops
#define sw_barrier() asm volatile("": : :"memory")

// AVR compatibility
#define strtof strtod

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------
#define HardwareSerial_h // Hack to prevent HardwareSerial.h header inclusion
#include "hardwareserial/HardwareSerial.h"
#include "watchdog/watchdog.h"
#include "fastio.h"
#include "math.h"
#include "delay.h"
#include "speed_lookuptable.h"

// Serial ports
#if !WITHIN(SERIAL_PORT_1, -1, 3)
  #error "SERIAL_PORT_1 must be from -1 to 3"
#endif
#define MKSERIAL1 MKSerial1

#if ENABLED(SERIAL_PORT_2) && SERIAL_PORT_2 >= -1
  #if !WITHIN(SERIAL_PORT_2, -1, 3)
    #error "SERIAL_PORT_2 must be from -1 to 3"
  #elif SERIAL_PORT_2 == SERIAL_PORT_1
    #error "SERIAL_PORT_2 must be different than SERIAL_PORT_1"
  #endif
  #define MKSERIAL2 MKSerial2
#endif

// --------------------------------------------------------------------------
// Defines
// --------------------------------------------------------------------------
#ifndef CRITICAL_SECTION_START()
  #define CRITICAL_SECTION_START()  unsigned char _sreg = SREG; cli()
  #define CRITICAL_SECTION_END()    SREG = _sreg
#endif

#define ISRS_ENABLED()  TEST(SREG, SREG_I)
#define ENABLE_ISRS()   sei()
#define DISABLE_ISRS()  cli()

#ifdef analogInputToDigitalPin
  #undef analogInputToDigitalPin
#endif

#define analogInputToDigitalPin(p) ((p) + 0xA0)

#define PACK

#if ENABLED(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#undef LOW
#define LOW         0
#undef HIGH
#define HIGH        1

// Voltage for Pin
#define HAL_VOLTAGE_PIN 5.0

#define HAL_TIMER_TYPE_MAX 0xFFFF

// Macros for stepper.cpp
#define HAL_MULTI_ACC(A,B)    MultiU24X32toH16(A,B)

// TEMPERATURE
#define ANALOG_REF_AREF       0
#define ANALOG_REF_AVCC       _BV(REFS0)
#define ANALOG_REF            ANALOG_REF_AVCC
#define ANALOG_PRESCALER      _BV(ADPS0)|_BV(ADPS1)|_BV(ADPS2)
#define NUM_ADC_SAMPLES       16
#define ABS_ZERO              -273.15
#define AD_RANGE              _BV(10)
#define AD595_MAX             500.0f
#define AD8495_MAX            1000.0f

#define GET_PIN_MAP_PIN(index) index
#define GET_PIN_MAP_INDEX(pin) pin
#define PARSED_PIN_INDEX(code, dval) parser.intval(code, dval)

// --------------------------------------------------------------------------
// Timer
// --------------------------------------------------------------------------

#define HAL_TIMER_RATE              ((F_CPU) / 8)

#define STEPPER_TIMER_RATE          HAL_TIMER_RATE
#define STEPPER_TIMER_PRESCALE      8
#define STEPPER_TIMER_TICKS_PER_US  ((STEPPER_TIMER_RATE) / 1000000)
#define STEPPER_TIMER_PULSE_TICK_NS (1000000000UL / STEPPER_TIMER_RATE)
#define STEPPER_TIMER_MIN_INTERVAL  8                                                         // minimum time in µs between stepper interrupts
#define STEPPER_TIMER_MAX_INTERVAL  (STEPPER_TIMER_TICKS_PER_US * STEPPER_TIMER_MIN_INTERVAL) // maximum time in µs between stepper interrupts

#define TEMP_TIMER_FREQUENCY        ((F_CPU) / 64.0 / 256.0) // 976 Hz

#define STEPPER_TIMER_NUM           1
#define STEPPER_TCCR                TCCR1A
#define STEPPER_TIMSK               TIMSK1
#define STEPPER_OCIE                OCIE1A

#define TEMP_TIMER_NUM              0
#define TEMP_OCR                    OCR0B
#define TEMP_TCCR                   TCCR0A
#define TEMP_TIMSK                  TIMSK0
#define TEMP_OCIE                   OCIE0B

#define TIMER_OCR_0                 OCR0A
#define TIMER_OCR_1                 OCR1A
#define TIMER_COUNTER_0             TCNT0
#define TIMER_COUNTER_1             TCNT1

#define START_STEPPER_INTERRUPT()   HAL_timer_start(STEPPER_TIMER_NUM)
#define ENABLE_STEPPER_INTERRUPT()  SBI(STEPPER_TIMSK, STEPPER_OCIE)
#define DISABLE_STEPPER_INTERRUPT() CBI(STEPPER_TIMSK, STEPPER_OCIE)
#define STEPPER_ISR_ENABLED()       TEST(STEPPER_TIMSK, STEPPER_OCIE)

#define ENABLE_TEMP_INTERRUPT()     SBI(TEMP_TIMSK, TEMP_OCIE)
#define DISABLE_TEMP_INTERRUPT()    CBI(TEMP_TIMSK, TEMP_OCIE)
#define TEMP_ISR_ENABLED()          TEST(TEMP_TIMSK, TEMP_OCIE)

#define HAL_timer_set_count(timer, count)   (_CAT(TIMER_OCR_, timer) = count)
#define HAL_timer_get_current_count(timer)  _CAT(TIMER_COUNTER_, timer)

// Estimate the amount of time the ISR will take to execute
#define TIMER_CYCLES                13UL

// The base ISR takes 752 cycles
#define ISR_BASE_CYCLES            752UL

// Linear advance base time is 32 cycles
#if ENABLED(LIN_ADVANCE)
  #define ISR_LA_BASE_CYCLES        32UL
#else
  #define ISR_LA_BASE_CYCLES         0UL
#endif

// Bezier interpolation adds 160 cycles
#if ENABLED(BEZIER_JERK_CONTROL)
  #define ISR_BEZIER_CYCLES        160UL
#else
  #define ISR_BEZIER_CYCLES          0UL
#endif

// Stepper Loop base cycles
#define ISR_LOOP_BASE_CYCLES        32UL

// And each stepper (start + stop pulse) takes in worst case
#define ISR_STEPPER_CYCLES          88UL

// For each stepper, we add its time
#if HAS_X_STEP
  #define ISR_X_STEPPER_CYCLES        ISR_STEPPER_CYCLES
#else
  #define ISR_X_STEPPER_CYCLES        0UL
#endif
#if HAS_Y_STEP
  #define ISR_Y_STEPPER_CYCLES        ISR_STEPPER_CYCLES
#else
  #define ISR_Y_STEPPER_CYCLES        0UL
#endif
#if HAS_Z_STEP
  #define ISR_Z_STEPPER_CYCLES        ISR_STEPPER_CYCLES
#else
  #define ISR_Z_STEPPER_CYCLES        0UL
#endif

// E is always interpolated
#define ISR_E_STEPPER_CYCLES          ISR_STEPPER_CYCLES

// If linear advance is disabled, then the loop also handles them
#if DISABLED(LIN_ADVANCE) && ENABLED(COLOR_MIXING_EXTRUDER)
  #define ISR_MIXING_STEPPER_CYCLES   ((MIXING_STEPPERS) * (ISR_STEPPER_CYCLES))
#else
  #define ISR_MIXING_STEPPER_CYCLES   0UL
#endif

// And the total minimum loop time is, without including the base
#define MIN_ISR_LOOP_CYCLES           (ISR_X_STEPPER_CYCLES + ISR_Y_STEPPER_CYCLES + ISR_Z_STEPPER_CYCLES + ISR_E_STEPPER_CYCLES + ISR_MIXING_STEPPER_CYCLES)

// But the user could be enforcing a minimum time, so the loop time is
#define ISR_LOOP_CYCLES               (ISR_LOOP_BASE_CYCLES + MAX(HAL_min_pulse_cycle, MIN_ISR_LOOP_CYCLES))

#define TIMER_SETUP_NS                (1000UL * (TIMER_CYCLES) / ((F_CPU) / 1000000))

// If linear advance is enabled, then it is handled separately
#if ENABLED(LIN_ADVANCE)

  // Estimate the minimum LA loop time
  #if ENABLED(COLOR_MIXING_EXTRUDER)
    #define MIN_ISR_LA_LOOP_CYCLES    ((MIXING_STEPPERS) * (ISR_STEPPER_CYCLES))
  #else
    #define MIN_ISR_LA_LOOP_CYCLES    ISR_STEPPER_CYCLES
  #endif

  // And the real loop time
  #define ISR_LA_LOOP_CYCLES          MAX(HAL_min_pulse_cycle, MIN_ISR_LA_LOOP_CYCLES)

#else
  #define ISR_LA_LOOP_CYCLES          0UL
#endif

/* 18 cycles maximum latency */
#define HAL_STEPPER_TIMER_ISR() \
extern "C" void TIMER1_COMPA_vect (void) __attribute__ ((signal, naked, used, externally_visible)); \
extern "C" void TIMER1_COMPA_vect_bottom (void) asm ("TIMER1_COMPA_vect_bottom") __attribute__ ((used, externally_visible, noinline)); \
void TIMER1_COMPA_vect () { \
  __asm__ __volatile__ ( \
    A("push r16")                      /* 2 Save R16 */ \
    A("in r16, __SREG__")              /* 1 Get SREG */ \
    A("push r16")                      /* 2 Save SREG into stack */ \
    A("lds r16, %[timsk0]")            /* 2 Load into R0 the Temperature timer Interrupt mask register */ \
    A("push r16")                      /* 2 Save TIMSK0 into the stack */ \
    A("andi r16,~%[msk0]")             /* 1 Disable the temperature ISR */ \
    A("sts %[timsk0], r16")            /* 2 And set the new value */ \
    A("lds r16, %[timsk1]")            /* 2 Load into R0 the stepper timer Interrupt mask register [TIMSK1] */ \
    A("andi r16,~%[msk1]")             /* 1 Disable the stepper ISR */ \
    A("sts %[timsk1], r16")            /* 2 And set the new value */ \
    A("push r16")                      /* 2 Save TIMSK1 into stack */ \
    A("in r16, 0x3B")                  /* 1 Get RAMPZ register */ \
    A("push r16")                      /* 2 Save RAMPZ into stack */ \
    A("in r16, 0x3C")                  /* 1 Get EIND register */ \
    A("push r0")                       /* C runtime can modify all the following registers without restoring them */ \
    A("push r1")                       \
    A("push r18")                      \
    A("push r19")                      \
    A("push r20")                      \
    A("push r21")                      \
    A("push r22")                      \
    A("push r23")                      \
    A("push r24")                      \
    A("push r25")                      \
    A("push r26")                      \
    A("push r27")                      \
    A("push r30")                      \
    A("push r31")                      \
    A("clr r1")                        /* C runtime expects this register to be 0 */ \
    A("call TIMER1_COMPA_vect_bottom") /* Call the bottom handler - No inlining allowed, otherwise registers used are not saved */   \
    A("pop r31")                       \
    A("pop r30")                       \
    A("pop r27")                       \
    A("pop r26")                       \
    A("pop r25")                       \
    A("pop r24")                       \
    A("pop r23")                       \
    A("pop r22")                       \
    A("pop r21")                       \
    A("pop r20")                       \
    A("pop r19")                       \
    A("pop r18")                       \
    A("pop r1")                        \
    A("pop r0")                        \
    A("out 0x3C, r16")                 /* 1 Restore EIND register */ \
    A("pop r16")                       /* 2 Get the original RAMPZ register value */ \
    A("out 0x3B, r16")                 /* 1 Restore RAMPZ register to its original value */ \
    A("pop r16")                       /* 2 Get the original TIMSK1 value but with stepper ISR disabled */ \
    A("ori r16,%[msk1]")               /* 1 Reenable the stepper ISR */ \
    A("cli")                           /* 1 Disable global interrupts - Reenabling Stepper ISR can reenter amd temperature can reenter, and we want that, if it happens, after this ISR has ended */ \
    A("sts %[timsk1], r16")            /* 2 And restore the old value - This reenables the stepper ISR */ \
    A("pop r16")                       /* 2 Get the temperature timer Interrupt mask register [TIMSK0] */ \
    A("sts %[timsk0], r16")            /* 2 And restore the old value - This reenables the temperature ISR */ \
    A("pop r16")                       /* 2 Get the old SREG value */ \
    A("out __SREG__, r16")             /* 1 And restore the SREG value */ \
    A("pop r16")                       /* 2 Restore R16 value */ \
    A("reti")                          /* 4 Return from interrupt */ \
    :                                   \
    : [timsk0] "i" ((uint16_t)&TIMSK0), \
      [timsk1] "i" ((uint16_t)&TIMSK1), \
      [msk0] "M" ((uint8_t)(1<<OCIE0B)),\
      [msk1] "M" ((uint8_t)(1<<OCIE1A)) \
    : \
  ); \
} \
void TIMER1_COMPA_vect_bottom(void)

/* 14 cycles maximum latency */
#define HAL_TEMP_TIMER_ISR \
extern "C" void TIMER0_COMPB_vect (void) __attribute__ ((signal, naked, used, externally_visible)); \
extern "C" void TIMER0_COMPB_vect_bottom(void)  asm ("TIMER0_COMPB_vect_bottom") __attribute__ ((used, externally_visible, noinline)); \
void TIMER0_COMPB_vect () { \
  __asm__ __volatile__ ( \
    A("push r16")                       /* 2 Save R16 */ \
    A("in r16, __SREG__")               /* 1 Get SREG */ \
    A("push r16")                       /* 2 Save SREG into stack */ \
    A("lds r16, %[timsk0]")             /* 2 Load into R0 the Temperature timer Interrupt mask register */ \
    A("andi r16,~%[msk0]")              /* 1 Disable the temperature ISR */ \
    A("sts %[timsk0], r16")             /* 2 And set the new value */ \
    A("sei")                            /* 1 Enable global interrupts - It is safe, as the temperature ISR is disabled, so we cannot reenter it */    \
    A("push r16")                       /* 2 Save TIMSK0 into stack */ \
    A("in r16, 0x3B")                   /* 1 Get RAMPZ register */ \
    A("push r16")                       /* 2 Save RAMPZ into stack */ \
    A("in r16, 0x3C")                   /* 1 Get EIND register */ \
    A("push r0")                        /* C runtime can modify all the following registers without restoring them */ \
    A("push r1")                        \
    A("push r18")                       \
    A("push r19")                       \
    A("push r20")                       \
    A("push r21")                       \
    A("push r22")                       \
    A("push r23")                       \
    A("push r24")                       \
    A("push r25")                       \
    A("push r26")                       \
    A("push r27")                       \
    A("push r30")                       \
    A("push r31")                       \
    A("clr r1")                         /* C runtime expects this register to be 0 */ \
    A("call TIMER0_COMPB_vect_bottom")  /* Call the bottom handler - No inlining allowed, otherwise registers used are not saved */   \
    A("pop r31")                        \
    A("pop r30")                        \
    A("pop r27")                        \
    A("pop r26")                        \
    A("pop r25")                        \
    A("pop r24")                        \
    A("pop r23")                        \
    A("pop r22")                        \
    A("pop r21")                        \
    A("pop r20")                        \
    A("pop r19")                        \
    A("pop r18")                        \
    A("pop r1")                         \
    A("pop r0")                         \
    A("out 0x3C, r16")                  /* 1 Restore EIND register */ \
    A("pop r16")                        /* 2 Get the original RAMPZ register value */ \
    A("out 0x3B, r16")                  /* 1 Restore RAMPZ register to its original value */ \
    A("pop r16")                        /* 2 Get the original TIMSK0 value but with temperature ISR disabled */ \
    A("ori r16,%[msk0]")                /* 1 Enable temperature ISR */ \
    A("cli")                            /* 1 Disable global interrupts - We must do this, as we will reenable the temperature ISR, and we don't want to reenter this handler until the current one is done */ \
    A("sts %[timsk0], r16")             /* 2 And restore the old value */ \
    A("pop r16")                        /* 2 Get the old SREG */ \
    A("out __SREG__, r16")              /* 1 And restore the SREG value */ \
    A("pop r16")                        /* 2 Restore R16 */ \
    A("reti")                           /* 4 Return from interrupt */ \
    :                                   \
    : [timsk0] "i"((uint16_t)&TIMSK0),  \
      [msk0] "M" ((uint8_t)(1<<OCIE0B)) \
    : \
  ); \
} \
void TIMER0_COMPB_vect_bottom(void)

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------

extern uint16_t HAL_pulse_high_tick,
                HAL_pulse_low_tick;
extern uint32_t HAL_min_pulse_cycle,
                HAL_frequency_limit[8];

// --------------------------------------------------------------------------
// Private Variables
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------
void HAL_timer_start(const uint8_t timer_num);

void HAL_calc_pulse_cycle();

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
extern "C" {
  int freeMemory(void);
}
#pragma GCC diagnostic pop

class HAL {

  public: /** Constructor */

    HAL();

    virtual ~HAL();

  public: /** Public Parameters */

    #if ANALOG_INPUTS > 0
      static int16_t AnalogInputValues[NUM_ANALOG_INPUTS];
    #endif

  public: /** Public Function */

    #if ANALOG_INPUTS > 0
      static void analogStart();
      static void AdcChangePin(const pin_t, const pin_t);
    #endif

    static void hwSetup();

    static void showStartReason();

    static void resetHardware();

    static void setPwmFrequency(const pin_t pin, uint8_t val);

    static void analogWrite(const pin_t pin, const uint8_t uValue, const uint16_t freq=1000U);

    static void Tick();

    static pin_t digital_value_pin();
    static pin_t analog_value_pin();

    static inline void digitalWrite(const pin_t pin, const uint8_t value) {
      uint8_t bit = digitalPinToBitMask(pin);
      uint8_t port = digitalPinToPort(pin);
      volatile uint8_t *out = portOutputRegister(port);
      if (value)
        *out |= bit;
      else
        *out &= ~bit;
    }
    static inline uint8_t digitalRead(const pin_t pin) {
      uint8_t bit = digitalPinToBitMask(pin);
      uint8_t port = digitalPinToPort(pin);
      if (*portInputRegister(port) & bit) return HIGH;
      return LOW;
    }
    static inline void pinMode(const pin_t pin, const uint8_t mode) {
      uint8_t bit = digitalPinToBitMask(pin);
      uint8_t port = digitalPinToPort(pin);
      volatile uint8_t  *reg  = portModeRegister(port),
                        *out  = portOutputRegister(port);

      uint8_t oldSREG = SREG;
      switch (mode) {
        case INPUT:
          cli();
          *reg &= ~bit;
          *out &= ~bit;
          SREG = oldSREG;
          break;
        case INPUT_PULLUP:
          cli();
          *reg &= ~bit;
          *out |= bit;
          SREG = oldSREG;
          break;
        case OUTPUT:
          cli();
          *reg |= bit;
          SREG = oldSREG;
          break;
        case OUTPUT_LOW:
          cli();
          *reg |= bit;
          *out &= ~bit;
          SREG = oldSREG;
          break;
        case OUTPUT_HIGH:
          cli();
          *reg |= bit;
          *out |= bit;
          SREG = oldSREG;
          break;
        default: break;
      }
    }
    static inline void setInputPullup(const pin_t pin, const bool onoff) {
      onoff ? pinMode(pin, INPUT_PULLUP) : pinMode(pin, INPUT);
    }

    FORCE_INLINE static void delayNanoseconds(const uint32_t delayNs) {
      HAL_delay_cycles(delayNs * (CYCLES_PER_US) / 1000UL);
    }
    FORCE_INLINE static void delayMicroseconds(const uint32_t delayUs) {
      HAL_delay_cycles(delayUs * (CYCLES_PER_US));
    }
    static inline void delayMilliseconds(uint16_t delayMs) {
      uint16_t del;
      while (delayMs > 0) {
        del = delayMs > 100 ? 100 : delayMs;
        delay(del);
        delayMs -= del;
      }
    }
    static inline uint32_t timeInMilliseconds() {
      return millis();
    }

    //
    // SPI related functions
    //

    // Initialize SPI bus
    static void spiBegin();

    // Configure SPI for specified SPI speed
    static void spiInit(uint8_t spiRate=6);

    // Write single byte to SPI
    static void spiSend(uint8_t nbyte);

    // Write buffer to  SPI
    static void spiSend(const uint8_t* buf, size_t nbyte);

    // Write single byte to specified SPI channel
    static void spiSend(uint32_t chan, uint8_t nbyte);

    // Write buffer to specified SPI channel
    static void spiSend(uint32_t chan ,const uint8_t* buf, size_t nbyte);

    // Read single byte from SPI
    static uint8_t spiReceive(void);

    // Read single byte from specified SPI channel
    static uint8_t spiReceive(uint32_t chan);

    // Read from SPI into buffer
    static void spiReadBlock(uint8_t* buf, uint16_t nbyte);

    // Write token and then write from 512 byte buffer to SPI (for SD card)
    static void spiSendBlock(uint8_t token, const uint8_t* buf);

  private: /** Private Function */

    /**
     * Called from the Temperature ISR
     */
    static void set_current_temp_raw();

};
