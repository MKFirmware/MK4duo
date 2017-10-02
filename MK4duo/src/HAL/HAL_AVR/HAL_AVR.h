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
 * Description:          *** HAL for Arduino ***
 *
 * Contributors:
 * Copyright (c) 2014 Bob Cousins bobcousins42@googlemail.com
 *                    Nico Tonnhofer wurstnase.reprap@gmail.com
 *
 * Copyright (c) 2015 - 2016 Alberto Cotronei @MagoKimbra
 *
 * ARDUINO_ARCH_ARM
 */

#ifndef HAL_AVR_H
#define HAL_AVR_H

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------
#include <math.h>
#include <stdint.h>
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
typedef uint16_t  HAL_TIMER_TYPE;
typedef uint16_t  ptr_int_t;
typedef uint32_t  millis_t;
typedef int8_t    Pin;


// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------
#include "fastio.h"
#include "watchdog_AVR.h"

// BLUETOOTH
#if ENABLED(BLUETOOTH) && BLUETOOTH_PORT > 0
  #undef SERIAL_PORT
  #undef BAUDRATE
  #define SERIAL_PORT BLUETOOTH_PORT
  #define BAUDRATE    BLUETOOTH_BAUD
#endif

//#define EXTERNALSERIAL  // Force using arduino serial
#ifndef EXTERNALSERIAL
  #include "HardwareSerial.h"
  #define MKSERIAL MKSerial
#else
  #define MKSERIAL Serial
#endif

// --------------------------------------------------------------------------
// Defines
// --------------------------------------------------------------------------

#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif

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

// EEPROM START
#define EEPROM_OFFSET 100

// Voltage for Pin
#define HAL_VOLTAGE_PIN 5.0

#define ADV_NEVER 0xFFFF

/**
 * Optimized math functions for AVR
 */

// intRes = intIn1 * intIn2 >> 16
// uses:
// r26 to store 0
// r27 to store the byte 1 of the 24 bit result
#define MultiU16X8toH16(intRes, charIn1, intIn2) \
  asm volatile ( \
                 "clr r26 \n\t" \
                 "mul %A1, %B2 \n\t" \
                 "movw %A0, r0 \n\t" \
                 "mul %A1, %A2 \n\t" \
                 "add %A0, r1 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "lsr r0 \n\t" \
                 "adc %A0, r26 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "clr r1 \n\t" \
                 : \
                 "=&r" (intRes) \
                 : \
                 "d" (charIn1), \
                 "d" (intIn2) \
                 : \
                 "r26" \
               )

// intRes = longIn1 * longIn2 >> 24
// uses:
// r26 to store 0
// r27 to store bits 16-23 of the 48bit result. The top bit is used to round the two byte result.
// note that the lower two bytes and the upper byte of the 48bit result are not calculated.
// this can cause the result to be out by one as the lower bytes may cause carries into the upper ones.
// B0 A0 are bits 24-39 and are the returned value
// C1 B1 A1 is longIn1
// D2 C2 B2 A2 is longIn2
//
#define MultiU24X32toH16(intRes, longIn1, longIn2) \
  asm volatile ( \
                 "clr r26 \n\t" \
                 "mul %A1, %B2 \n\t" \
                 "mov r27, r1 \n\t" \
                 "mul %B1, %C2 \n\t" \
                 "movw %A0, r0 \n\t" \
                 "mul %C1, %C2 \n\t" \
                 "add %B0, r0 \n\t" \
                 "mul %C1, %B2 \n\t" \
                 "add %A0, r0 \n\t" \
                 "adc %B0, r1 \n\t" \
                 "mul %A1, %C2 \n\t" \
                 "add r27, r0 \n\t" \
                 "adc %A0, r1 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "mul %B1, %B2 \n\t" \
                 "add r27, r0 \n\t" \
                 "adc %A0, r1 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "mul %C1, %A2 \n\t" \
                 "add r27, r0 \n\t" \
                 "adc %A0, r1 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "mul %B1, %A2 \n\t" \
                 "add r27, r1 \n\t" \
                 "adc %A0, r26 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "lsr r27 \n\t" \
                 "adc %A0, r26 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "mul %D2, %A1 \n\t" \
                 "add %A0, r0 \n\t" \
                 "adc %B0, r1 \n\t" \
                 "mul %D2, %B1 \n\t" \
                 "add %B0, r0 \n\t" \
                 "clr r1 \n\t" \
                 : \
                 "=&r" (intRes) \
                 : \
                 "d" (longIn1), \
                 "d" (longIn2) \
                 : \
                 "r26" , "r27" \
               )

// Macros for stepper.cpp
#define HAL_MULTI_ACC(intRes, longIn1, longIn2) MultiU24X32toH16(intRes, longIn1, longIn2)

// TEMPERATURE
#define ANALOG_REF_AREF 0
#define ANALOG_REF_AVCC _BV(REFS0)
#define ANALOG_REF ANALOG_REF_AVCC
#define ANALOG_PRESCALER _BV(ADPS0)|_BV(ADPS1)|_BV(ADPS2)
#define OVERSAMPLENR 5

// --------------------------------------------------------------------------
// Timer
// --------------------------------------------------------------------------

#define HAL_STEPPER_TIMER_RATE  ((F_CPU) / 8.0)
#define TEMP_TIMER_FREQUENCY    ((F_CPU) / 64.0 / 64.0) // 3096 Hz

#define STEPPER_TIMER_PRESCALE  64

#define STEPPER_TIMER OCR1A
#define STEPPER_TCCR  TCCR1A
#define STEPPER_TIMSK TIMSK1
#define STEPPER_OCIE  OCIE1A

#define TEMP_TIMER    OCR0B
#define TEMP_TCCR     TCCR0B
#define TEMP_TIMSK    TIMSK0
#define TEMP_OCIE     OCIE0B

#define HAL_STEPPER_TIMER_START()     HAL_stepper_timer_start()
#define HAL_TEMP_TIMER_START()        HAL_temp_timer_start()

#define ENABLE_STEPPER_INTERRUPT()    SBI(STEPPER_TIMSK, STEPPER_OCIE)
#define DISABLE_STEPPER_INTERRUPT()   CBI(STEPPER_TIMSK, STEPPER_OCIE)

#define ENABLE_TEMP_INTERRUPT()       SBI(TEMP_TIMSK, TEMP_OCIE)
#define DISABLE_TEMP_INTERRUPT()      CBI(TEMP_TIMSK, TEMP_OCIE)

#define HAL_timer_start(timer_num, frequency) { }
#define HAL_timer_get_current_count(timer)    TCNT0
#define HAL_timer_set_count(timer, count)     timer = (count)
#define HAL_timer_isr_prologue(timer_num)     { }

#define HAL_TIMER_SET_STEPPER_COUNT(n)  HAL_timer_set_count(STEPPER_TIMER, n)
#define HAL_TIMER_SET_TEMP_COUNT(n)     HAL_timer_set_count(TEMP_TIMER, n)

#define HAL_STEP_TIMER_ISR  ISR(TIMER1_COMPA_vect)
#define HAL_TEMP_TIMER_ISR  ISR(TIMER0_COMPB_vect)

#define HAL_ENABLE_ISRs() \
        do { \
          cli(); \
          ENABLE_TEMP_INTERRUPT(); \
          ENABLE_STEPPER_INTERRUPT(); \
        } while(0)

#define HAL_DISABLE_ISRs() \
        do { \
          DISABLE_TEMP_INTERRUPT(); \
          DISABLE_STEPPER_INTERRUPT(); \
          sei(); \
        } while(0)

// Clock speed factor
#define CYCLES_PER_US ((F_CPU) / 1000000) // 16 or 20
// Stepper pulse duration, in cycles
#define STEP_PULSE_CYCLES ((MINIMUM_STEPPER_PULSE) * CYCLES_PER_US)

// Highly granular delays for step pulses, etc.
#define DELAY_0_NOP   NOOP
#define DELAY_1_NOP   __asm__("nop\n\t")
#define DELAY_2_NOP   DELAY_1_NOP;  DELAY_1_NOP
#define DELAY_3_NOP   DELAY_1_NOP;  DELAY_2_NOP
#define DELAY_4_NOP   DELAY_1_NOP;  DELAY_3_NOP
#define DELAY_5_NOP   DELAY_1_NOP;  DELAY_4_NOP
#define DELAY_10_NOP  DELAY_5_NOP;  DELAY_5_NOP
#define DELAY_20_NOP  DELAY_10_NOP; DELAY_10_NOP

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

#if CYCLES_PER_MICROSECOND == 16
  #define DELAY_1US DELAY_10_NOP; DELAY_5_NOP; DELAY_1_NOP
#else
  #define DELAY_1US DELAY_20_NOP
#endif
#define DELAY_2US  DELAY_1US; DELAY_1US
#define DELAY_3US  DELAY_1US; DELAY_2US
#define DELAY_4US  DELAY_1US; DELAY_3US
#define DELAY_5US  DELAY_1US; DELAY_4US
#define DELAY_6US  DELAY_1US; DELAY_5US
#define DELAY_7US  DELAY_1US; DELAY_6US
#define DELAY_8US  DELAY_1US; DELAY_7US
#define DELAY_9US  DELAY_1US; DELAY_8US
#define DELAY_10US DELAY_1US; DELAY_9US

class InterruptProtectedBlock {
  uint8_t sreg;
  public:
    inline void protect() {
      cli();
    }

    inline void unprotect() {
      SREG = sreg;
    }

    inline InterruptProtectedBlock(bool later = false) {
      sreg = SREG;
      if (!later) cli();
    }

    inline ~InterruptProtectedBlock() {
      SREG = sreg;
    }
};

void HAL_stepper_timer_start();
void HAL_temp_timer_start();

class HAL {

  public: /** Constructor */

    HAL();

    virtual ~HAL();

  public: /** Public Parameters */

    #if ANALOG_INPUTS > 0
      static int16_t AnalogInputValues[NUM_ANALOG_INPUTS];
      static bool Analog_is_ready;
    #endif

    static bool execute_100ms;

  public: /** Public Function */

    // do any hardware-specific initialization here
    static void hwSetup();

    static void showStartReason();

    static int getFreeRam();
    static void resetHardware();

    static void analogStart();

    static void setPwmFrequency(uint8_t pin, uint8_t val);

    static inline void analogWrite(const Pin pin, const uint8_t value) {
      ::analogWrite(pin, value);
    }
    static inline void digitalWrite(const Pin pin, const uint8_t value) {
      ::digitalWrite(pin, value);
    }
    static inline uint8_t digitalRead(const Pin pin) {
      return ::digitalRead(pin);
    }
    static inline void pinMode(const Pin pin, const uint8_t mode) {
      ::pinMode(pin, mode);
    }

    static inline void delayMicroseconds(const uint16_t delayUs) {
      ::delayMicroseconds(delayUs);
    }
    static inline void delayMilliseconds(const uint16_t delayMs) {
      ::delay(delayMs);
    }
    static inline uint32_t timeInMilliseconds() {
      return millis();
    }

    static inline void serialSetBaudrate(const uint16_t baud) {
      MKSERIAL.begin(baud);
    }
    static inline bool serialByteAvailable() {
      return MKSERIAL.available() > 0;
    }
    static inline uint8_t serialReadByte() {
      return MKSERIAL.read();
    }
    static inline void serialWriteByte(const char b) {
      MKSERIAL.write(b);
    }
    static inline void serialFlush() {
      MKSERIAL.flush();
    }

    // SPI related functions
    #if ENABLED(SOFTWARE_SPI)
      static void spiBegin();
      static void spiInit(uint8_t spiRate);
      static uint8_t spiReceive(void);
      static void spiReadBlock(uint8_t* buf, uint16_t nbyte);
      static void spiSend(uint8_t b);
      static void spiSendBlock(uint8_t token, const uint8_t* buf);
    #else
      // Hardware setup
      static void spiBegin();
      static void spiInit(uint8_t spiRate);
      // Write single byte to SPI
      static void spiSend(byte b);
      static void spiSend(const uint8_t* buf, size_t n);
      // Read single byte from SPI
      static uint8_t spiReceive(uint8_t send=0xFF);
      // Read from SPI into buffer
      static void spiReadBlock(uint8_t* buf, uint16_t nbyte);
      // Write from buffer to SPI
      static void spiSendBlock(uint8_t token, const uint8_t* buf);
    #endif

};

/**
 * math function
 */

#undef ATAN2
#undef FABS
#undef POW
#undef SQRT
#undef CEIL
#undef FLOOR
#undef LROUND
#undef FMOD
#undef COS
#undef SIN
#define ATAN2(y, x) atan2(y, x)
#define FABS(x)     fabs(x)
#define POW(x, y)   pow(x, y)
#define SQRT(x)     sqrt(x)
#define CEIL(x)     ceil(x)
#define FLOOR(x)    floor(x)
#define LROUND(x)   lround(x)
#define FMOD(x, y)  fmod(x, y)
#define COS(x)      cos(x)
#define SIN(x)      sin(x)

#endif // HAL_AVR_H
