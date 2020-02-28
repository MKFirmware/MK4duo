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
#pragma once

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------
#include <stdint.h>

// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------
typedef uint32_t  hal_timer_t;
typedef uint32_t  ptr_int_t;

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------
#include "hardwareserial/HardwareSerial.h"
#include "watchdog/watchdog.h"
#include "fastio.h"
#include "math.h"
#include "delay.h"
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
#undef pgm_read_dword
#define pgm_read_dword(addr) (*(addr))
//#define pgm_read_dword(addr) (*(const unsigned long *)(addr))
#undef pgm_read_dword_near
#define pgm_read_dword_near(addr) pgm_read_dword(addr)
#undef pgm_read_ptr
#define pgm_read_ptr(addr) (*(addr))
#ifndef strncpy_P
  #define strncpy_P strncpy
#endif
#ifndef strchr_P
  #define strchr_P strchr
#endif
#ifndef vsnprintf_P
  #define vsnprintf_P vsnprintf
#endif
#ifndef snprintf_P
  #define snprintf_P snprintf
#endif

// CRITICAL SECTION
#define CRITICAL_SECTION_START()  uint32_t primask = __get_PRIMASK(); __disable_irq()
#define CRITICAL_SECTION_END()    if (!primask) __enable_irq()

// ISR function
#define ISRS_ENABLED()          (!__get_PRIMASK())
#define ENABLE_ISRS()           __enable_irq()
#define DISABLE_ISRS()          __disable_irq()
#define cli()                   __disable_irq()
#define sei()                   __enable_irq()

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
#define HAL_MULTI_ACC(A,B)  MultiU32X24toH32(A,B)

// Bits for PWM
#undef PWM_RESOLUTION
#define PWM_RESOLUTION       8

// Bits of the ADC converter
#define ANALOG_INPUT_BITS   12
#define AD_RANGE            _BV(ANALOG_INPUT_BITS)
#define ABS_ZERO          -273.15f
#define NUM_ADC_SAMPLES     32
#define AD595_MAX          330.0f
#define AD8495_MAX         660.0f

#define GET_PIN_MAP_PIN(index) index
#define GET_PIN_MAP_INDEX(pin) pin
#define PARSED_PIN_INDEX(code, dval) parser.intval(code, dval)

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------

extern int32_t HAL_VREF;

// reset reason set by bootloader
extern uint8_t MCUSR;
volatile static uint32_t debug_counter;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
extern "C" {
  char* _sbrk(int incr);
  int freeMemory(void);
}
#pragma GCC diagnostic pop

typedef AveragingFilter<NUM_ADC_SAMPLES> ADCAveragingFilter;

class HAL {

  public: /** Constructor */

    HAL() { }

    virtual ~HAL() {}

  private: /** Private Parameters */

    #if HAS_HOTENDS
      static ADCAveragingFilter HOTENDsensorFilters[MAX_HOTEND];
    #endif
    #if HAS_BEDS
      static ADCAveragingFilter BEDsensorFilters[MAX_BED];
    #endif
    #if HAS_CHAMBERS
      static ADCAveragingFilter CHAMBERsensorFilters[MAX_CHAMBER];
    #endif
    #if HAS_COOLERS
      static ADCAveragingFilter COOLERsensorFilters[MAX_COOLER];
    #endif

    #if ENABLED(FILAMENT_WIDTH_SENSOR)
      static ADCAveragingFilter filamentFilter;
    #endif

    #if HAS_POWER_CONSUMPTION_SENSOR
      static ADCAveragingFilter powerFilter;
    #endif

    #if HAS_MCU_TEMPERATURE
      static ADCAveragingFilter mcuFilter;
    #endif

    #if HAS_VREF_MONITOR
      static ADCAveragingFilter vrefFilter;
    #endif

  public: /** Public Function */

    static void analogStart();
    static void AdcChangePin(const pin_t, const pin_t new_pin);

    static void hwSetup(void);

    static void analogWrite(const pin_t pin, uint32_t ulValue, const uint16_t PWM_freq=1000U);

    static void Tick();

    #if HAS_VREF_MONITOR
      static int32_t analog2mv(const int16_t adc_raw);
    #endif
    #if HAS_MCU_TEMPERATURE
      static int32_t analog2tempMCU(const int16_t adc_raw);
    #endif

    static pin_t digital_value_pin();
    static pin_t analog_value_pin();

    FORCE_INLINE static void pinMode(const pin_t pin, const uint8_t mode) {
      switch (mode) {
        case INPUT:         SET_INPUT(pin);         break;
        case OUTPUT:        SET_OUTPUT(pin);        break;
        case INPUT_PULLUP:  SET_INPUT_PULLUP(pin);  break;
        case OUTPUT_LOW:    SET_OUTPUT_LOW(pin);    break;
        case OUTPUT_HIGH:   SET_OUTPUT_HIGH(pin);   break;
        default:                                    break;
      }
    }
    FORCE_INLINE static void digitalWrite(const pin_t pin, const bool value) {
      WRITE(pin, value);
    }
    FORCE_INLINE static bool digitalRead(const pin_t pin) {
      return READ(pin);
    }
    FORCE_INLINE static void setInputPullup(const pin_t pin, const bool onoff) {
      onoff ? pinMode(pin, INPUT_PULLUP) : pinMode(pin, INPUT);
    }

    FORCE_INLINE static void delayNanoseconds(const uint32_t delayNs) {
      HAL_delay_cycles(delayNs * (CYCLES_PER_US) / 1000UL);
    }
    FORCE_INLINE static void delayMicroseconds(const uint32_t delayUs) {
      HAL_delay_cycles(delayUs * (CYCLES_PER_US));
    }
    FORCE_INLINE static void delayMilliseconds(const uint16_t delayMs) {
      delay(delayMs);
    }
    FORCE_INLINE static uint32_t timeInMilliseconds() {
      return millis();
    }

    static void showStartReason();

    static void resetHardware();

    // SPI related functions
    static void spiBegin();
    static void spiInit(uint8_t spiRate);
    // Write single byte to SPI
    static void spiSend(uint8_t nbyte);
    // Read single byte from SPI
    static uint8_t spiReceive(void);
    // Read from SPI into buffer
    static void spiReadBlock(uint8_t* buf, uint16_t nbyte);
    // Write from buffer to SPI
    static void spiSendBlock(uint8_t token, const uint8_t* buf);

};

// EEPROM
uint8_t eeprom_read_byte(uint8_t* pos);
void eeprom_read_block(void* pos, const void* eeprom_address, size_t n);
void eeprom_write_byte(uint8_t* pos, uint8_t value);
void eeprom_update_block(const void* pos, void* eeprom_address, size_t n);
