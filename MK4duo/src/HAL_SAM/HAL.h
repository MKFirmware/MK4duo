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

#ifndef HAL_H
#define HAL_H

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
#define pgm_read_dword_near(addr) pgm_read_dword(addr)
#define strncpy_P(dest, src, num) strncpy((dest), (src), (num))

/**
 * Defines & Macros
 */
#define analogInputToDigitalPin(IO) IO

#define CRITICAL_SECTION_START	uint32_t primask=__get_PRIMASK(); __disable_irq();
#define CRITICAL_SECTION_END    if (primask==0) __enable_irq();

#define SPR0    0
#define SPR1    1

#define PACK    __attribute__ ((packed))

#define READ_VAR(pin) (g_APinDescription[pin].pPort->PIO_PDSR & g_APinDescription[pin].ulPin ? 1 : 0) // does return 0 or pin value
#define _READ(pin) (DIO ##  pin ## _PORT->PIO_PDSR & DIO ##  pin ## _PIN ? 1 : 0) // does return 0 or pin value
#define READ(pin) _READ(pin)

#define	WRITE_VAR(pin, v) do{if(v) {g_APinDescription[pin].pPort->PIO_SODR = g_APinDescription[pin].ulPin;} else {g_APinDescription[pin].pPort->PIO_CODR = g_APinDescription[pin].ulPin; }}while(0)
#define		_WRITE(port, v)			do { if (v) {DIO ##  port ## _PORT -> PIO_SODR = DIO ## port ## _PIN; } else {DIO ##  port ## _PORT->PIO_CODR = DIO ## port ## _PIN; }; } while (0)
#define WRITE(pin,v) _WRITE(pin,v)

#define	SET_INPUT(pin)  pmc_enable_periph_clk(g_APinDescription[pin].ulPeripheralId); \
                        PIO_Configure(g_APinDescription[pin].pPort, PIO_INPUT, g_APinDescription[pin].ulPin, 0)
#define	SET_OUTPUT(pin) PIO_Configure(g_APinDescription[pin].pPort, PIO_OUTPUT_1, \
                                      g_APinDescription[pin].ulPin, g_APinDescription[pin].ulPinConfiguration)

#define TOGGLE(pin) WRITE(pin,!READ(pin))
#define TOGGLE_VAR(pin) HAL::digitalWrite(pin,!HAL::digitalRead(pin))

// Shorthand
#define OUT_WRITE(IO, v)  { SET_OUTPUT(IO); WRITE(IO, v); }

// Write doesn't work for pullups
#define   PULLUP(IO, v)   { pinMode(IO, (v!=LOW ? INPUT_PULLUP : INPUT)); }

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
#if MSG_PORT == -1
  #define MKSERIAL SerialUSB
#elif MSG_PORT == 0
  #define MKSERIAL Serial
#elif MSG_PORT == 1
  #define MKSERIAL Serial1
#elif MSG_PORT == 2
  #define MKSERIAL Serial2
#elif MSG_PORT == 3
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
      return MKSERIAL.available();
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
void eeprom_write_byte(unsigned char* pos, unsigned char value);
uint8_t eeprom_read_byte(uint8_t* pos);

// timers
#define ADVANCE_EXTRUDER_TIMER_NUM 1
#define ADVANCE_EXTRUDER_TIMER_COUNTER TC0
#define ADVANCE_EXTRUDER_TIMER_CHANNEL 1
#define ADVANCE_EXTRUDER_FREQUENCY 60000
#define ADVANCE_EXTRUDER_TIMER_IRQN TC1_IRQn
#define HAL_ADVANCE_EXTRUDER_TIMER_ISR 	void TC1_Handler()

#define STEP_TIMER_NUM 2
#define STEP_TIMER_COUNTER TC0
#define STEP_TIMER_CHANNEL 2
#define STEP_FREQUENCY 1000
#define STEP_TIMER_IRQN TC2_IRQn
#define HAL_STEP_TIMER_ISR 	void TC2_Handler()

#define TEMP_TIMER_NUM 3
#define TEMP_TIMER_COUNTER TC1
#define TEMP_TIMER_CHANNEL 0
#define TEMP_FREQUENCY 4000
#define TEMP_TIMER_IRQN TC3_IRQn
#define HAL_TEMP_TIMER_ISR 	void TC3_Handler()

#define BEEPER_TIMER_NUM 4
#define BEEPER_TIMER_COUNTER TC1
#define BEEPER_TIMER_CHANNEL 1
#define BEEPER_TIMER_IRQN TC4_IRQn
#define HAL_BEEPER_TIMER_ISR  void TC4_Handler()

#define HAL_TIMER_RATE 		     (F_CPU/2)
#define TICKS_PER_NANOSECOND   (HAL_TIMER_RATE)/1000

#define ENABLE_STEPPER_DRIVER_INTERRUPT()	HAL_timer_enable_interrupt (STEP_TIMER_NUM)
#define DISABLE_STEPPER_DRIVER_INTERRUPT()	HAL_timer_disable_interrupt (STEP_TIMER_NUM)

#if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)
  #define ENABLE_ADVANCE_EXTRUDER_INTERRUPT()	HAL_timer_enable_interrupt (ADVANCE_EXTRUDER_TIMER_NUM)
  #define DISABLE_ADVANCE_EXTRUDER_INTERRUPT()	HAL_timer_disable_interrupt (ADVANCE_EXTRUDER_TIMER_NUM)
  void HAL_advance_extruder_timer_start(void);
  extern  TcChannel *extruderChannel;
#endif

extern TcChannel* stepperChannel;

void HAL_step_timer_start(void);
void HAL_temp_timer_start (uint8_t timer_num);

void HAL_timer_enable_interrupt (uint8_t timer_num);
void HAL_timer_disable_interrupt (uint8_t timer_num);

inline void HAL_timer_isr_status (Tc* tc, uint32_t channel) {
  tc->TC_CHANNEL[channel].TC_SR; // clear status register
}

int HAL_timer_get_count (uint8_t timer_num);

static FORCE_INLINE void HAL_timer_stepper_count(uint32_t count) {
  uint32_t counter_value = stepperChannel->TC_CV + 42;  // we need time for other stuff!
  //if(count < 105) count = 105;
  stepperChannel->TC_RC = (counter_value <= count) ? count : counter_value;
}

void tone(uint8_t pin, int frequency, unsigned long duration);
void noTone(uint8_t pin);

uint16_t getAdcReading(adc_channel_num_t chan);
void startAdcConversion(adc_channel_num_t chan);
adc_channel_num_t pinToAdcChannel(int pin);

uint16_t getAdcFreerun(adc_channel_num_t chan, bool wait_for_conversion = false);
uint16_t getAdcSuperSample(adc_channel_num_t chan);
void stopAdcFreerun(adc_channel_num_t chan);

#if ENABLED(LASERBEAM)
  #define LASER_PWM_MAX_DUTY 255
  void HAL_laser_init_pwm(uint8_t pin, uint16_t freq);
  void HAL_laser_intensity(uint8_t intensity); // Range: 0 - LASER_PWM_MAX_DUTY
#endif

#endif // HAL_H
