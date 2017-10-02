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

#ifndef _HAL_DUE_H
#define _HAL_DUE_H

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------
#include <stdint.h>
#include <Arduino.h>

// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------
typedef uint32_t  HAL_TIMER_TYPE;
typedef uint32_t  ptr_int_t;
typedef uint32_t  millis_t;
typedef int8_t    Pin;
static const Pin  NoPin = -1;

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------
#include "fastio_Due.h"
#include "watchdog_Due.h"
#include "HAL_timers_Due.h"

// --------------------------------------------------------------------------
// Defines
// --------------------------------------------------------------------------

// do not use program space memory with Due
#define PROGMEM
#ifndef PGM_P
  #define PGM_P const char*
#endif
#undef PSTR
#define PSTR(s) s
#undef pgm_read_byte_near
#define pgm_read_byte_near(x) (*(int8_t*)x)
#undef pgm_read_byte
#define pgm_read_byte(x) (*(int8_t*)x)
#undef pgm_read_float
#define pgm_read_float(addr) (*(const float *)(addr))
#undef pgm_read_word
//#define pgm_read_word(addr) (*(const unsigned int *)(addr))
#define pgm_read_word(addr) (*(addr))
#undef pgm_read_word_near
#define pgm_read_word_near(addr) pgm_read_word(addr)
#undef pgm_read_dword
#define pgm_read_dword(addr) (*(addr))
//#define pgm_read_dword(addr) (*(const unsigned long *)(addr))
#undef pgm_read_dword_near
#define pgm_read_dword_near(addr) pgm_read_dword(addr)
#undef pgm_read_ptr
#define pgm_read_ptr(addr) (*(addr))
#ifndef strncpy_P
  #define strncpy_P(dest, src, num) strncpy((dest), (src), (num))
#endif
#ifndef vsnprintf_P
  #define vsnprintf_P(buf, size, a, b) vsnprintf((buf), (size), (a), (b))
#endif

// SERIAL
#if SERIAL_PORT == -1
  #define MKSERIAL SerialUSB
#elif SERIAL_PORT == 0
  #define MKSERIAL Serial
#elif SERIAL_PORT == 1
  #define MKSERIAL Serial1
#elif SERIAL_PORT == 2
  #define MKSERIAL Serial2
#elif SERIAL_PORT == 3
  #define MKSERIAL Serial3
#endif

#if ENABLED(BLUETOOTH) && BLUETOOTH_PORT > 0
  #undef MKSERIAL
  #if BLUETOOTH_PORT == 1
    #define MKSERIAL Serial1
  #elif BLUETOOTH_PORT == 2
    #define MKSERIAL Serial2
  #elif BLUETOOTH_PORT == 3
    #define MKSERIAL Serial3
  #endif
#endif

// EEPROM START
#define EEPROM_OFFSET 10

// MATH
#define MATH_USE_HAL
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
#define ATAN2(y, x) atan2f(y, x)
#define FABS(x)     fabsf(x)
#define POW(x, y)   powf(x, y)
#define SQRT(x)     sqrtf(x)
#define CEIL(x)     ceilf(x)
#define FLOOR(x)    floorf(x)
#define LROUND(x)   lroundf(x)
#define FMOD(x, y)  fmodf(x, y)
#define COS(x)      cosf(x)
#define SIN(x)      sinf(x)

#define CRITICAL_SECTION_START	uint32_t primask=__get_PRIMASK(); __disable_irq();
#define CRITICAL_SECTION_END    if (primask==0) __enable_irq();

// Voltage
#define HAL_VOLTAGE_PIN 3.3

// reset reason
#define RST_POWER_ON   1
#define RST_EXTERNAL   2
#define RST_BROWN_OUT  4
#define RST_WATCHDOG   8
#define RST_JTAG       16
#define RST_SOFTWARE   32
#define RST_BACKUP     64

#define SPR0    0
#define SPR1    1

#define PACK    __attribute__ ((packed))

#undef LOW
#define LOW         0
#undef HIGH
#define HIGH        1

#define MultiU16X8toH16(intRes, charIn1, intIn2)   intRes = ((charIn1) * (intIn2)) >> 16
#define MultiU32X32toH32(intRes, longIn1, longIn2) intRes = ((uint64_t)longIn1 * longIn2 + 0x80000000) >> 32
// Macros for stepper.cpp
#define HAL_MULTI_ACC(intRes, longIn1, longIn2) MultiU32X32toH32(intRes, longIn1, longIn2)

#define ADV_NEVER 0xFFFFFFFF

// TEMPERATURE
#undef analogInputToDigitalPin
#define analogInputToDigitalPin(p) ((p < 12) ? (p) + 54 : -1)
#undef NUM_ANALOG_INPUTS
#define NUM_ANALOG_INPUTS 16
// Bits of the ADC converter
#define ANALOG_INPUT_BITS 12
#define ANALOG_REDUCE_BITS 0
#define ANALOG_REDUCE_FACTOR 1

#define MAX_ANALOG_PIN_NUMBER 11
#define OVERSAMPLENR 6
#define MEDIAN_COUNT 10 // MEDIAN COUNT for Smoother temperature
#define NUM_ADC_SAMPLES (2 + (1 << OVERSAMPLENR))
#define ADC_TEMPERATURE_SENSOR 15

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------

// reset reason set by bootloader
extern uint8_t MCUSR;
volatile static uint32_t debug_counter;

class HAL {

  public: /** Constructor */

    HAL();

    virtual ~HAL();

  public: /** Public Parameters */

    #if ANALOG_INPUTS > 0
      static volatile int16_t AnalogInputValues[NUM_ANALOG_INPUTS];
      static bool Analog_is_ready;
      static adc_channel_num_t PinToAdcChannel(Pin pin);
    #endif

    static bool execute_100ms;

  public: /** Public Function */

    #if ANALOG_INPUTS > 0
      static void analogStart();
      static void AdcEnableChannel(adc_channel_num_t adc_ch) { adc_enable_channel(ADC, adc_ch); }
    #endif

    static void hwSetup(void);

    static bool analogWrite(const Pin pin, const uint8_t value, const uint16_t freq=50);

    static inline void digitalWrite(const Pin pin, const uint8_t value) {
      WRITE_VAR(pin, value);
    }
    static inline uint8_t digitalRead(const Pin pin) {
      return READ_VAR(pin);
    }
    static inline void pinMode(const Pin pin, const uint8_t mode) {
      if (mode == INPUT) {
        SET_INPUT(pin);
      }
      else SET_OUTPUT(pin);
    }

    static FORCE_INLINE void delayMicroseconds(uint32_t usec) { // usec += 3;
      uint32_t n = usec * (F_CPU / 3000000);
      asm volatile(
        "L2_%=_delayMicroseconds:"       "\n\t"
        "subs   %0, #1"                 "\n\t"
        "bge    L2_%=_delayMicroseconds" "\n"
        : "+r" (n) :
      );
    }
    static inline void delayMilliseconds(uint16_t delayMs) {
      uint16_t del;
      while (delayMs > 0) {
        del = delayMs > 100 ? 100 : delayMs;
        delay(del);
        delayMs -= del;
      }
    }
    static inline unsigned long timeInMilliseconds() {
      return millis();
    }

    // Serial communication
    static inline char readFlashByte(PGM_P ptr) {
      return pgm_read_byte(ptr);
    }
    static inline void serialSetBaudrate(long baud) {
      MKSERIAL.begin(baud);
      HAL::delayMilliseconds(1);
    }
    static inline bool serialByteAvailable() {
      return MKSERIAL.available() > 0;
    }
    static inline uint8_t serialReadByte() {
      return MKSERIAL.read();
    }
    static inline void serialWriteByte(char c) {
      MKSERIAL.write(c);
    }
    static inline void serialFlush() {
      MKSERIAL.flush();
    }

    static void showStartReason();

    static int getFreeRam();
    static void resetHardware();

    // SPI related functions
    #if ENABLED(SOFTWARE_SPI)
      static uint8_t spiTransfer(uint8_t b); // using Mode 0
      static void spiBegin();
      static void spiInit(uint8_t spiRate);
      static uint8_t spiReceive();
      static void spiReadBlock(uint8_t* buf, uint16_t nbyte);
      static void spiSend(uint8_t b);
      static void spiSend(const uint8_t* buf , size_t n) ;
      static void spiSendBlock(uint8_t token, const uint8_t* buf);
    #else
      // Hardware setup
      static void spiBegin();
      static void spiInit(uint8_t spiRate);
      // Write single byte to SPI
      static void spiSend(byte b);
      static void spiSend(const uint8_t* buf, size_t n);
      static void spiSend(uint32_t chan, byte b);
      static void spiSend(uint32_t chan ,const uint8_t* buf, size_t n);
      // Read single byte from SPI
      static uint8_t spiReceive(void);
      static uint8_t spiReceive(uint32_t chan);
      // Read from SPI into buffer
      static void spiReadBlock(uint8_t* buf, uint16_t nbyte);
      // Write from buffer to SPI
      static void spiSendBlock(uint8_t token, const uint8_t* buf);
    #endif

};

/**
 * Public functions
 */

// Disable interrupts
void cli(void);

// Enable interrupts
void sei(void);

int freeMemory(void);

// SPI: Extended functions which take a channel number (hardware SPI only)
/** Write single byte to specified SPI channel */
void spiSend(uint32_t chan, byte b);
/** Write buffer to specified SPI channel */
void spiSend(uint32_t chan, const uint8_t* buf, size_t n);
/** Read single byte from specified SPI channel */
uint8_t spiReceive(uint32_t chan);


// EEPROM
uint8_t eeprom_read_byte(uint8_t* pos);
void eeprom_read_block(void* pos, const void* eeprom_address, size_t n);
void eeprom_write_byte(uint8_t* pos, uint8_t value);
void eeprom_update_block(const void* pos, void* eeprom_address, size_t n);

#endif // HAL_SAM_H
