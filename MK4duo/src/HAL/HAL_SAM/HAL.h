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
 * Description:          *** HAL for Arduino Due ***
 *
 * Contributors:
 * Copyright (c) 2014 Bob Cousins bobcousins42@googlemail.com
 *                    Nico Tonnhofer wurstnase.reprap@gmail.com
 *
 * Copyright (c) 2015 - 2016 Alberto Cotronei @MagoKimbra
 *
 * ARDUINO_ARCH_SAM
 */

#ifndef HAL_SAM_H
#define HAL_SAM_H

#include <stdint.h>
#include "Arduino.h"
#include "Print.h"
#include "fastio.h"

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
#undef pgm_read_ptr
#define pgm_read_ptr(addr) (*(addr))
#define pgm_read_dword_near(addr) pgm_read_dword(addr)
#ifndef strncpy_P
  #define strncpy_P(dest, src, num) strncpy((dest), (src), (num))
#endif

/**
 * Defines & Macros
 */
#define MAX_ANALOG_PIN_NUMBER 11
#define analogInputToDigitalPin(IO) IO

// Bracket code that shouldn't be interrupted
#if DISABLED(CRITICAL_SECTION_START)
  #define CRITICAL_SECTION_START	uint32_t primask=__get_PRIMASK(); __disable_irq();
  #define CRITICAL_SECTION_END    if (primask==0) __enable_irq();
#endif

#define SPR0    0
#define SPR1    1

#define PACK    __attribute__ ((packed))

#define READ_VAR(pin) (g_APinDescription[pin].pPort->PIO_PDSR & g_APinDescription[pin].ulPin ? 1 : 0) // does return 0 or pin value
#define _READ(pin)    (DIO ##  pin ## _PORT->PIO_PDSR & DIO ##  pin ## _PIN ? 1 : 0) // does return 0 or pin value
#define READ(pin)     _READ(pin)

#define	WRITE_VAR(pin, v) do{if(v) {g_APinDescription[pin].pPort->PIO_SODR = g_APinDescription[pin].ulPin;} else {g_APinDescription[pin].pPort->PIO_CODR = g_APinDescription[pin].ulPin; }}while(0)
#define	_WRITE(port, v)   do { if (v) {DIO ##  port ## _PORT -> PIO_SODR = DIO ## port ## _PIN; } else {DIO ##  port ## _PORT->PIO_CODR = DIO ## port ## _PIN; }; } while (0)
#define WRITE(pin,v)      _WRITE(pin,v)

#define	SET_INPUT(pin)  pmc_enable_periph_clk(g_APinDescription[pin].ulPeripheralId); \
                        PIO_Configure(g_APinDescription[pin].pPort, PIO_INPUT, g_APinDescription[pin].ulPin, 0)
#define	SET_OUTPUT(pin) PIO_Configure(g_APinDescription[pin].pPort, PIO_OUTPUT_1, \
                                      g_APinDescription[pin].ulPin, g_APinDescription[pin].ulPinConfiguration)

#define TOGGLE(pin) WRITE(pin,!READ(pin))
#define TOGGLE_VAR(pin) HAL::digitalWrite(pin,!HAL::digitalRead(pin))

// Shorthand
#define OUT_WRITE(IO, v)  { SET_OUTPUT(IO); WRITE(IO, v); }

// Write doesn't work for pullups
#define   PULLUP(IO)      { pinMode(IO, INPUT_PULLUP); }

#undef LOW
#define LOW         0
#undef HIGH
#define HIGH        1

/**
 * Stepper Definition
 */
// intRes = intIn1 * intIn2 >> 16
#define MultiU16X8toH16(intRes, charIn1, intIn2)   intRes = ((charIn1) * (intIn2)) >> 16
// intRes = longIn1 * longIn2 >> 24
#define MultiU32X32toH32(intRes, longIn1, longIn2) intRes = ((uint64_t)longIn1 * longIn2 + 0x80000000) >> 32

/**
 * Public Variables
 */

#ifndef DUE_SOFTWARE_SPI
  extern int spiDueDividors[];
#endif

// reset reason set by bootloader
extern uint8_t MCUSR;
volatile static uint32_t debug_counter;

/**
 * Setting Serial
 */
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

#if defined(BLUETOOTH) && BLUETOOTH_PORT > 0
  #undef MKSERIAL
  #if BLUETOOTH_PORT == 1
    #define MKSERIAL Serial1
  #elif BLUETOOTH_PORT == 2
    #define MKSERIAL Serial2
  #elif BLUETOOTH_PORT == 3
    #define MKSERIAL Serial3
  #endif
#endif

class HAL {
  public:

    HAL();

    virtual ~HAL();

    #ifdef DUE_SOFTWARE_SPI
      static uint8_t spiTransfer(uint8_t b); // using Mode 0
      static void spiBegin();
      static void spiInit(uint8_t spiClock);
      static uint8_t spiReceive();
      static void spiReadBlock(uint8_t*buf, uint16_t nbyte);
      static void spiSend(uint8_t b);
      static void spiSend(const uint8_t* buf , size_t n) ;
      static void spiSendBlock(uint8_t token, const uint8_t* buf);
    #else
      // Hardware setup
      static void spiBegin();
      static void spiInit(uint8_t spiClock);
      // Write single byte to SPI
      static void spiSend(byte b);
      static void spiSend(const uint8_t* buf, size_t n);
      static void spiSend(uint32_t chan, byte b);
      static void spiSend(uint32_t chan , const uint8_t* buf , size_t n);
      // Read single byte from SPI
      static uint8_t spiReceive();
      static uint8_t spiReceive(uint32_t chan);
      // Read from SPI into buffer
      static void spiReadBlock(uint8_t* buf, uint16_t nbyte);
      // Write from buffer to SPI
      static void spiSendBlock(uint8_t token, const uint8_t* buf);
    #endif

    static inline void digitalWrite(uint8_t pin, uint8_t value) {
      WRITE_VAR(pin, value);
    }
    static inline uint8_t digitalRead(uint8_t pin) {
      return READ_VAR(pin);
    }
    static inline void pinMode(uint8_t pin, uint8_t mode) {
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
    static inline void delayMilliseconds(unsigned int delayMs) {
      unsigned int del;
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

  protected:
  private:
};

// Disable interrupts
void cli(void);

// Enable interrupts
void sei(void);

int freeMemory(void);

// EEPROM
uint8_t eeprom_read_byte(uint8_t* pos);
void eeprom_read_block(void* pos, const void* eeprom_address, size_t n);
void eeprom_write_byte(uint8_t* pos, uint8_t value);
void eeprom_update_block(const void* pos, void* eeprom_address, size_t n);

/**
 * Timers
 */
typedef struct {
  Tc          *pTimerRegs;
  uint16_t    channel;
  IRQn_Type   IRQ_Id;
} tTimerConfig;

#define  NUM_HARDWARE_TIMERS 9

static const tTimerConfig TimerConfig [NUM_HARDWARE_TIMERS] =
{
  { TC0, 0, TC0_IRQn},
  { TC0, 1, TC1_IRQn},
  { TC0, 2, TC2_IRQn},
  { TC1, 0, TC3_IRQn},
  { TC1, 1, TC4_IRQn},
  { TC1, 2, TC5_IRQn},
  { TC2, 0, TC6_IRQn},
  { TC2, 1, TC7_IRQn},
  { TC2, 2, TC8_IRQn},
};

/*
	Timer_clock1: Prescaler 2 -> 42MHz
	Timer_clock2: Prescaler 8 -> 10.5MHz
	Timer_clock3: Prescaler 32 -> 2.625MHz
	Timer_clock4: Prescaler 128 -> 656.25kHz
*/

#if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)
  #define EXTRUDER_TIMER 1
  #define EXTRUDER_TIMER_PRIORITY 6
  #define EXTRUDER_TIMER_FREQUENCY REFERENCE_EXTRUDER_TIMER_FREQUENCY
  #define EXTRUDER_TIMER_CLOCK TC_CMR_TCCLKS_TIMER_CLOCK1 // TIMER_CLOCK1 -> 2 divisor
  #define EXTRUDER_TIMER_PRESCALE 2
  #define HAL_EXTRUDER_TIMER_RATE (F_CPU / EXTRUDER_TIMER_PRESCALE) // = 42MHz
  #define EXTRUDER_TIMER_FACTOR (HAL_EXTRUDER_TIMER_RATE / HAL_REFERENCE_EXTRUDER_TIMER_RATE)
  #define EXTRUDER_TIMER_TICKS_PER_MILLISECOND (HAL_EXTRUDER_TIMER_RATE / 1000)
  #define HAL_EXTRUDER_TIMER_ISR  void TC1_Handler()
#endif

#define STEPPER_TIMER 2
#define STEPPER_TIMER_PRIORITY 2
#define STEPPER_TIMER_FREQUENCY REFERENCE_STEPPER_TIMER_FREQUENCY
#define STEPPER_TIMER_CLOCK TC_CMR_TCCLKS_TIMER_CLOCK1 // TIMER_CLOCK1 -> 2 divisor
#define STEPPER_TIMER_PRESCALE 2
#define HAL_STEPPER_TIMER_RATE (F_CPU / STEPPER_TIMER_PRESCALE) // = 42MHz
#define STEPPER_TIMER_FACTOR (HAL_STEPPER_TIMER_RATE / HAL_REFERENCE_STEPPER_TIMER_RATE)
#define STEPPER_TIMER_TICKS_PER_MILLISECOND (HAL_STEPPER_TIMER_RATE / 1000)
#define HAL_STEP_TIMER_ISR  void TC2_Handler()

#define TEMP_TIMER 3
#define TEMP_TIMER_PRIORITY 15
#define TEMP_TIMER_FREQUENCY (REFERENCE_TEMP_TIMER_FREQUENCY * 4)
#define TEMP_TIMER_CLOCK TC_CMR_TCCLKS_TIMER_CLOCK4 // TIMER_CLOCK4 -> 128 divisor
#define TEMP_TIMER_PRESCALE 128
#define HAL_TEMP_TIMER_RATE (F_CPU / TEMP_TIMER_PRESCALE) // = 656.25kHz
#define TEMP_TIMER_FACTOR (HAL_TEMP_TIMER_RATE / HAL_REFERENCE_TEMP_TIMER_RATE)
#define TEMP_TIMER_TICKS_PER_MILLISECOND (HAL_TEMP_TIMER_RATE / 1000)
#define HAL_TEMP_TIMER_ISR  void TC3_Handler()

#define BEEPER_TIMER 4
#define BEEPER_TIMER_COUNTER TC1
#define BEEPER_TIMER_CHANNEL 1
#define HAL_BEEPER_TIMER_ISR  void TC4_Handler()

#define HAL_TIMER_START(n) HAL_timer_start(n, n ## _PRIORITY, n ## _FREQUENCY, n ## _CLOCK, n ## _PRESCALE)

#if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)
  #define ENABLE_EXTRUDER_INTERRUPT()       HAL_timer_enable_interrupt(EXTRUDER_TIMER)
  #define DISABLE_EXTRUDER_INTERRUPT()      HAL_timer_disable_interrupt(EXTRUDER_TIMER)
  #define HAL_TIMER_SET_EXTRUDER_COUNT(n)   HAL_timer_set_count(EXTRUDER_TIMER, n)
#endif

#define ENABLE_STEPPER_DRIVER_INTERRUPT()   HAL_timer_enable_interrupt(STEPPER_TIMER)
#define DISABLE_STEPPER_DRIVER_INTERRUPT()  HAL_timer_disable_interrupt(STEPPER_TIMER)
#define HAL_TIMER_SET_STEPPER_COUNT(n)      HAL_timer_set_count(STEPPER_TIMER, n)

#define ENABLE_TEMPERATURE_INTERRUPT()      HAL_timer_enable_interrupt(TEMP_TIMER)
#define DISABLE_TEMPERATURE_INTERRUPT()     HAL_timer_disable_interrupt(TEMP_TIMER)
#define HAL_TIMER_SET_TEMP_COUNT(n)         HAL_timer_set_count(TEMP_TIMER, n)

// Delays
#define CYCLES_EATEN_BY_CODE 12
#define CYCLES_EATEN_BY_E 12

// Voltage for PIN
#define HAL_VOLTAGE_PIN 3.3

// Types
#define HAL_TIMER_TYPE uint32_t
typedef uint32_t millis_t;

// Timers
void HAL_timer_start(uint8_t timer_num, uint8_t priority, uint32_t frequency, uint32_t clock, uint8_t prescale);
void HAL_timer_enable_interrupt(uint8_t timer_num);
void HAL_timer_disable_interrupt(uint8_t timer_num);

static FORCE_INLINE void HAL_timer_isr_prologue(uint8_t timer_num) {
  const tTimerConfig *pConfig = &TimerConfig[timer_num];
  TC_GetStatus(pConfig->pTimerRegs, pConfig->channel); // clear status register
}

static FORCE_INLINE uint32_t HAL_timer_get_count(uint8_t timer_num) {
  const tTimerConfig *pConfig = &TimerConfig[timer_num];
  return pConfig->pTimerRegs->TC_CHANNEL[pConfig->channel].TC_RC;
}

static FORCE_INLINE uint32_t HAL_timer_get_current_count(uint8_t timer_num) {
  const tTimerConfig *pConfig = &TimerConfig[timer_num];
  return TC_ReadCV(pConfig->pTimerRegs, pConfig->channel);
}

static FORCE_INLINE void HAL_timer_set_count(uint8_t timer_num, uint32_t count) {
  const tTimerConfig *pConfig = &TimerConfig[timer_num];
  TC_SetRC(pConfig->pTimerRegs, pConfig->channel, count);
}

// ADC
uint16_t getAdcReading(adc_channel_num_t chan);
void startAdcConversion(adc_channel_num_t chan);
adc_channel_num_t pinToAdcChannel(int pin);

uint16_t getAdcFreerun(adc_channel_num_t chan, bool wait_for_conversion = false);
uint16_t getAdcSuperSample(adc_channel_num_t chan);
void setAdcFreerun(void);
void stopAdcFreerun(adc_channel_num_t chan);

// Tone
inline void HAL_timer_isr_status(Tc* tc, uint32_t channel) {
  tc->TC_CHANNEL[channel].TC_SR; // clear status register
}

void tone(uint8_t pin, int frequency, unsigned long duration);
void noTone(uint8_t pin);

#if ENABLED(LASERBEAM)
  #define LASER_PWM_MAX_DUTY 255
  void HAL_laser_init_pwm(uint8_t pin, uint16_t freq);
  void HAL_laser_intensity(uint8_t intensity); // Range: 0 - LASER_PWM_MAX_DUTY
#endif

#endif // HAL_SAM_H
