/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2016 Alberto Cotronei @MagoKimbra
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

#ifndef HAL_H
#define HAL_H

#include "fastio.h"

// Arduino < 1.0.0 does not define this, so we need to do it ourselves
#ifndef analogInputToDigitalPin
  #define analogInputToDigitalPin(p) ((p) + 0xA0)
#endif

#if DISABLED(CRITICAL_SECTION_START)
  #define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif

//#define EXTERNALSERIAL  // Force using arduino serial
#ifndef EXTERNALSERIAL
  #include "HardwareSerial.h"
  #define MKSERIAL MKSerial
#else
  #define MKSERIAL Serial
#endif

#define PACK

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

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

// Some useful constants
#define ENABLE_STEPPER_DRIVER_INTERRUPT()  SBI(TIMSK1, OCIE1A)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() CBI(TIMSK1, OCIE1A)

/**
 * Types
 */
typedef uint32_t millis_t;

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

class HAL {
  public:

    HAL();

    virtual ~HAL();

    static void showStartReason();
    static int getFreeRam();
    static void resetHardware();

    // SPI related functions
    static void spiBegin() {
      #if SDSS >= 0
        SET_INPUT(MISO_PIN);
        SET_OUTPUT(MOSI_PIN);
        SET_OUTPUT(SCK_PIN);
        // SS must be in output mode even it is not chip select
        SET_OUTPUT(SDSS);
        // set SS high - may be chip select for another SPI device
        WRITE(SDSS, HIGH);
      #endif
    }
    static inline void spiInit(uint8_t spiRate) {
      uint8_t r = 0;
      for (uint8_t b = 2; spiRate > b && r < 6; b <<= 1, r++);

      SET_OUTPUT(SDSS);
      WRITE(SDSS, HIGH);
      SET_OUTPUT(SCK_PIN);
      SET_OUTPUT(MOSI_PIN);
      SET_INPUT(MISO_PIN);
      #ifdef	PRR
        PRR &= ~(1<<PRSPI);
      #elif defined PRR0
        PRR0 &= ~(1<<PRSPI);
      #endif
      // See avr processor documentation
      SPCR = (1 << SPE) | (1 << MSTR) | (r >> 1);
      SPSR = (r & 1 || r == 6 ? 0 : 1) << SPI2X;
    }
    static inline uint8_t spiReceive(uint8_t send = 0xFF) {
      SPDR = send;
      while (!(SPSR & (1 << SPIF))) {}
      return SPDR;
    }
    static inline void spiReadBlock(uint8_t* buf, size_t nbyte) {
      if (nbyte-- == 0) return;
      SPDR = 0XFF;
      for (size_t i = 0; i < nbyte; i++) {
        while (!(SPSR & (1 << SPIF))) {}
        buf[i] = SPDR;
        SPDR = 0XFF;
      }
      while (!(SPSR & (1 << SPIF))) {}
      buf[nbyte] = SPDR;
    }
    static inline void spiSend(uint8_t b) {
      SPDR = b;
      while (!(SPSR & (1 << SPIF))) {}
    }
    static inline void spiSend(const uint8_t* buf, size_t n) {
      if (n == 0) return;
      SPDR = buf[0];
      if (n > 1) {
        uint8_t b = buf[1];
        size_t i = 2;
        while (1) {
          while (!(SPSR & (1 << SPIF))) {}
          SPDR = b;
          if (i == n) break;
          b = buf[i++];
        }
      }
      while (!(SPSR & (1 << SPIF))) {}
    }
    static inline __attribute__((always_inline))
    void spiSendBlock(uint8_t token, const uint8_t* buf) {
      SPDR = token;
      for (uint16_t i = 0; i < 512; i += 2) {
        while (!(SPSR & (1 << SPIF))) {}
        SPDR = buf[i];
        while (!(SPSR & (1 << SPIF))) {}
        SPDR = buf[i + 1];
      }
      while (!(SPSR & (1 << SPIF))) {}
    }

    static inline void digitalWrite(uint8_t pin,uint8_t value) {
      ::digitalWrite(pin,value);
    }
    static inline uint8_t digitalRead(uint8_t pin) {
      return ::digitalRead(pin);
    }
    static inline void pinMode(uint8_t pin,uint8_t mode) {
      ::pinMode(pin,mode);
    }

    static inline void delayMicroseconds(unsigned int delayUs) {
      ::delayMicroseconds(delayUs);
    }
    static inline void delayMilliseconds(unsigned int delayMs) {
      ::delay(delayMs);
    }
    static inline unsigned long timeInMilliseconds() {
      return millis();
    }

    static inline void serialSetBaudrate(long baud) {
      MKSERIAL.begin(baud);
    }
    static inline bool serialByteAvailable() {
      return MKSERIAL.available() > 0;
    }
    static inline uint8_t serialReadByte() {
      return MKSERIAL.read();
    }
    static inline void serialWriteByte(char b) {
      MKSERIAL.write(b);
    }
    static inline void serialFlush() {
      MKSERIAL.flush();
    }

  protected:
  private:
};

#endif // HAL_H
