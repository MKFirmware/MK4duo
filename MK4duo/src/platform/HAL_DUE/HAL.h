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
#pragma once

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------
#include <stdint.h>
#include <Arduino.h>
#include <Wire.h>
#include <Reset.h>

// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------
typedef uint32_t  hal_timer_t;
typedef uint32_t  ptr_int_t;

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------
#include "fastio.h"
#include "math.h"
#include "delay.h"
#include "watchdog.h"
#include "HAL_timers.h"

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

// do not use program space memory with Due
#define PROGMEM
#ifndef PGM_P
  #define PGM_P PGM_P
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

// SERIAL ports
#include "HardwareSerial.h"
#if !WITHIN(SERIAL_PORT_1, -1, 3)
  #error "SERIAL_PORT_1 must be from -1 to 3"
#endif
#if SERIAL_PORT_1 == -1
  #define MKSERIAL1 SerialUSB
#elif SERIAL_PORT_1 == 0
  #define MKSERIAL1 MKSerial
#elif SERIAL_PORT_1 == 1
  #define MKSERIAL1 MKSerial1
#elif SERIAL_PORT_1 == 2
  #define MKSERIAL1 MKSerial2
#elif SERIAL_PORT_1 == 3
  #define MKSERIAL1 MKSerial3
#endif

#if ENABLED(SERIAL_PORT_2) && SERIAL_PORT_2 >= -1
  #if !WITHIN(SERIAL_PORT_2, -1, 3)
    #error "SERIAL_PORT_2 must be from -1 to 3"
  #elif SERIAL_PORT_2 == SERIAL_PORT_1
    #error "SERIAL_PORT_2 must be different than SERIAL_PORT_1"
  #elif SERIAL_PORT_2 == -1
    #define MKSERIAL2 SerialUSB
    #define NUM_SERIAL 2
  #elif SERIAL_PORT_2 == 0
    #define MKSERIAL2 MKSerial
    #define NUM_SERIAL 2
  #elif SERIAL_PORT_2 == 1
    #define MKSERIAL2 MKSerial1
    #define NUM_SERIAL 2
  #elif SERIAL_PORT_2 == 2
    #define MKSERIAL2 MKSerial2
    #define NUM_SERIAL 2
  #elif SERIAL_PORT_2 == 3
    #define MKSERIAL2 MKSerial3
    #define NUM_SERIAL 2
  #endif
#else
  #define NUM_SERIAL 1
#endif

// CRITICAL SECTION
#define CRITICAL_SECTION_START  uint32_t primask = __get_PRIMASK(); __disable_irq();
#define CRITICAL_SECTION_END    if (!primask) __enable_irq();

// ISR function
#define ISRS_ENABLED()          (!__get_PRIMASK())
#define ENABLE_ISRS()           __enable_irq()
#define DISABLE_ISRS()          __disable_irq()

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

// Macros for stepper.cpp
#define HAL_MULTI_ACC(A,B)  MultiU32X32toH32(A,B)

#define HAL_TIMER_TYPE_MAX  0xFFFFFFFF

// TEMPERATURE
#undef analogInputToDigitalPin
#define analogInputToDigitalPin(p) ((p < 12) ? (p) + 54 : -1)
#undef NUM_ANALOG_INPUTS
#define NUM_ANALOG_INPUTS       16
#define ADC_TEMPERATURE_SENSOR  15
// Bits of the ADC converter
#define ANALOG_INPUT_BITS 12
#define OVERSAMPLENR       2
#define AD_RANGE       16384
#define ABS_ZERO        -273.15f
#define NUM_ADC_SAMPLES   32
#define AD595_MAX        330.0f
#define AD8495_MAX       660.0f

#define HARDWARE_PWM true

#define GET_PIN_MAP_PIN(index) index
#define GET_PIN_MAP_INDEX(pin) pin
#define PARSED_PIN_INDEX(code, dval) parser.intval(code, dval)

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------

// reset reason set by bootloader
extern uint8_t MCUSR;
volatile static uint32_t debug_counter;

extern "C" char *sbrk(int i);
extern "C" char *dtostrf (double __val, signed char __width, unsigned char __prec, char *__s);

typedef AveragingFilter<NUM_ADC_SAMPLES> ADCAveragingFilter;

// ISR handler type
using pfnISR_Handler = void(*)(void);

// Install a new interrupt vector handler for the given irq, returning the old one
pfnISR_Handler install_isr(IRQn_Type irq, pfnISR_Handler newHandler);

class HAL {

  public: /** Constructor */

    HAL();

    virtual ~HAL();

  public: /** Public Parameters */

    static int16_t AnalogInputValues[NUM_ANALOG_INPUTS];
    static bool Analog_is_ready;

  private: /** Private Parameters */

    #if HEATER_COUNT > 0
      static ADCAveragingFilter sensorFilters[HEATER_COUNT];
    #endif

    #if HAS_FILAMENT_SENSOR
      static ADCAveragingFilter filamentFilter;
    #endif

    #if HAS_POWER_CONSUMPTION_SENSOR
      static ADCAveragingFilter powerFilter;
    #endif

    #if HAS_MCU_TEMPERATURE
      static ADCAveragingFilter mcuFilter;
    #endif

  public: /** Public Function */

    static void analogStart();
    static void AdcChangePin(const pin_t old_pin, const pin_t new_pin);

    static void hwSetup(void);

    static bool pwm_status(const pin_t pin);
    static bool tc_status(const pin_t pin);

    static void analogWrite(const pin_t pin, uint32_t ulValue, const uint16_t freq=1000);

    static void Tick();

    FORCE_INLINE static void pinMode(const pin_t pin, const uint8_t mode) {
      switch (mode) {
        case INPUT:         SET_INPUT(pin);         break;
        case OUTPUT:        SET_OUTPUT(pin);        break;
        case INPUT_PULLUP:  SET_INPUT_PULLUP(pin);  break;
        case OUTPUT_LOW:    SET_OUTPUT(pin);        break;
        case OUTPUT_HIGH:   SET_OUTPUT_HIGH(pin);   break;
        default:                                    break;
      }
    }
    FORCE_INLINE static void digitalWrite(const pin_t pin, const bool value) {
      WRITE_VAR(pin, value);
    }
    FORCE_INLINE static bool digitalRead(const pin_t pin) {
      return READ_VAR(pin);
    }
    FORCE_INLINE static void setInputPullup(const pin_t pin, const bool onoff) {
      const PinDescription& pinDesc = g_APinDescription[pin];
      if (pinDesc.ulPinType != PIO_NOT_A_PIN) {
        if (onoff)
          pinDesc.pPort->PIO_PUER = pinDesc.ulPin;
        else
          pinDesc.pPort->PIO_PUDR = pinDesc.ulPin;
      }
    }

    FORCE_INLINE static void delayNanoseconds(const uint32_t delayNs) {
      HAL_delay_cycles(delayNs * (CYCLES_PER_US) / 1000UL);
    }
    FORCE_INLINE static void delayMicroseconds(const uint32_t delayUs) {
      HAL_delay_cycles(delayUs * (CYCLES_PER_US));
    }
    FORCE_INLINE static void delayMilliseconds(uint16_t delayMs) {
      uint16_t del;
      while (delayMs > 0) {
        del = delayMs > 100 ? 100 : delayMs;
        delay(del);
        delayMs -= del;
        watchdog.reset();
      }
    }
    FORCE_INLINE static uint32_t timeInMilliseconds() {
      return millis();
    }

    static void showStartReason();

    static int getFreeRam();
    static void resetHardware();

    // SPI related functions
    #if ENABLED(SOFTWARE_SPI)
      static uint8_t spiTransfer(uint8_t nbyte); // using Mode 0
      static void spiBegin();
      static void spiInit(uint8_t spiRate);
      static uint8_t spiReceive();
      static void spiReadBlock(uint8_t* buf, uint16_t nbyte);
      static void spiSend(uint8_t nbyte);
      static void spiSend(const uint8_t* buf , size_t nbyte) ;
      static void spiSendBlock(uint8_t token, const uint8_t* buf);
    #else
      // Hardware setup
      static void spiBegin();
      static void spiInit(uint8_t spiRate=6);
      static uint8_t spiTransfer(uint8_t nbyte);
      // Write single byte to SPI
      static void spiSend(uint8_t nbyte);
      static void spiSend(const uint8_t* buf, size_t nbyte);
      static void spiSend(uint32_t chan, uint8_t nbyte);
      static void spiSend(uint32_t chan ,const uint8_t* buf, size_t nbyte);
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

// SPI: Extended functions which take a channel number (hardware SPI only)
/** Write single byte to specified SPI channel */
void spiSend(uint32_t chan, byte b);
/** Write buffer to specified SPI channel */
void spiSend(uint32_t chan, const uint8_t* buf, size_t n);
/** Read single byte from specified SPI channel */
uint8_t spiReceive(uint32_t chan);

// Tone for due
void tone(const pin_t _pin, const uint16_t frequency, const uint16_t duration=0);
void noTone(const pin_t _pin);

// EEPROM
uint8_t eeprom_read_byte(uint8_t* pos);
void eeprom_read_block(void* pos, const void* eeprom_address, size_t n);
void eeprom_write_byte(uint8_t* pos, uint8_t value);
void eeprom_update_block(const void* pos, void* eeprom_address, size_t n);
