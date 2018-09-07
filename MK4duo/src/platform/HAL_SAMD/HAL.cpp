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

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include "../../../MK4duo.h"

#if ENABLED(ARDUINO_ARCH_SAMD)

#include <malloc.h>
#include <Wire.h>
#include "wiring_private.h"
// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------
extern "C" char *sbrk(int i);
uint8_t MCUSR;

#if ANALOG_INPUTS > 0
  int16_t HAL::AnalogInputValues[NUM_ANALOG_INPUTS] = { 0 };
  bool    HAL::Analog_is_ready = false;
#endif


// Wait for synchronization of registers between the clock domains
static __inline__ void syncTC_16(Tc* TCx) __attribute__((always_inline, unused));
static void syncTC_16(Tc* TCx) {
  while (TCx->COUNT16.STATUS.bit.SYNCBUSY);
}

// Wait for synchronization of registers between the clock domains
static __inline__ void syncTCC(Tcc* TCCx) __attribute__((always_inline, unused));
static void syncTCC(Tcc* TCCx) {
  while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);
}

// disable interrupts
void cli(void) {
  noInterrupts();
}

// enable interrupts
void sei(void) {
  interrupts();
}

// Tone for due
// input parameters: Arduino pin number, frequency in Hz, duration in milliseconds
void tone(const pin_t t_pin, const uint16_t frequency, const uint16_t duration) {

  millis_t endTime = millis() + duration;
  const uint32_t halfPeriod = 1000000L / frequency / 2;

  HAL::pinMode(t_pin, OUTPUT_LOW);

  while (PENDING(millis(),  endTime)) {
    HAL::digitalWrite(t_pin, HIGH);
    HAL::delayMicroseconds(halfPeriod);
    HAL::digitalWrite(t_pin, LOW);
    HAL::delayMicroseconds(halfPeriod);
  }
  HAL::pinMode(t_pin, OUTPUT_LOW);
}

static inline void ConfigurePin(const PinDescription& pinDesc) {
 // PIO_Configure(pinDesc.ulPort, pinDesc.ulPinType, pinDesc.ulPin, 0);
}

// This intercepts the 1ms system tick. It must return 'false', otherwise the Arduino core tick handler will be bypassed.
extern "C" int sysTickHook() {
  HAL::Tick();
  return 0;
}

HAL::HAL() {
  // ctor
}

HAL::~HAL() {
  // dtor
}

bool HAL::execute_100ms = false;

// do any hardware-specific initialization here
void HAL::hwSetup(void) {
  while (!SerialUSB) {
  
  }
  
  


}

// Print apparent cause of start/restart
void HAL::showStartReason() {



}

// Return available memory
int HAL::getFreeRam() {
  struct mallinfo memstruct = mallinfo();
  register char * stack_ptr asm ("sp");

  // avail mem in heap + (bottom of stack addr - end of heap addr)
  return (memstruct.fordblks + (int)stack_ptr -  (int)sbrk(0));
}


// Start converting the enabled channels
void AnalogInStartConversion() {

}

// Enable or disable a channel.
void AnalogInEnablePin(const pin_t r_pin, const bool enable) {
 
}   

// Read the most recent 12-bit result from a pin
uint16_t AnalogInReadPin(const pin_t r_pin) {
SerialUSB.println("Temp read!");
  return 800;
}

// Initialize ADC channels
void HAL::analogStart(void) {

}

void HAL::AdcChangePin(const pin_t old_pin, const pin_t new_pin) {

}

// Reset peripherals and cpu
void HAL::resetHardware() {


}

// --------------------------------------------------------------------------
// Analogic write to a PWM Pin
// --------------------------------------------------------------------------
static bool     PWMEnabled      = false;
static uint16_t PWMChanFreq[8]  = {0},
                PWMChanPeriod[8];

static const uint32_t PwmFastClock =  25000 * 255;        // fast PWM clock for Intel spec PWM fans that need 25kHz PWM
static const uint32_t PwmSlowClock = (25000 * 255) / 256; // slow PWM clock to allow us to get slow speeds

static inline uint32_t ConvertRange(const float f, const uint32_t top) { return LROUND(f * (float)top); }

// AnalogWritePwm to a PWM pin
// Return true if successful, false if we need to call software pwm
static void AnalogWritePwm(const PinDescription& pinDesc, const float ulValue, const uint16_t freq) {

  return;
}


// AnalogWriteTc to a TC pin
// Return true if successful, false if we need to call software pwm
static void AnalogWriteTc(const PinDescription& pinDesc, const float ulValue, const uint16_t freq) {

  
  return;
}

bool HAL::pwm_status(const pin_t pin) {
  const PinDescription& pinDesc = g_APinDescription[pin];
  const uint32_t attr = pinDesc.ulPinAttribute;
  if (attr & PIN_ATTR_PWM) return true;
  else return false;
}
  
bool HAL::tc_status(const pin_t pin) {
  const PinDescription& pinDesc = g_APinDescription[pin];
  const uint32_t attr = pinDesc.ulPinAttribute;
  if (attr & PIN_ATTR_TIMER) return true;
  else return false;
}


static __inline__ void syncDAC() __attribute__((always_inline, unused));
static void syncDAC() {
  while (DAC->STATUS.bit.SYNCBUSY == 1)
    ;
}

static int _writeResolution = 8;

static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to)
{
  if (from == to) {
    return value;
  }
  if (from > to) {
    return value >> (from-to);
  }
  return value << (to-from);
}


void HAL::analogWrite(pin_t pin,  uint32_t value, const uint16_t freq/*=1000*/) {


PinDescription pinDesc = g_APinDescription[pin];
  uint32_t attr = pinDesc.ulPinAttribute;


  if ((attr & PIN_ATTR_ANALOG) == PIN_ATTR_ANALOG)
  {
    // DAC handling code

    if (pin != PIN_A0) { // Only 1 DAC on A0 (PA02)
      return;
    }

    value = mapResolution(value, _writeResolution, 10);

    syncDAC();
    DAC->DATA.reg = value & 0x3FF;  // DAC on 10 bits.
    syncDAC();
    DAC->CTRLA.bit.ENABLE = 0x01;     // Enable DAC
    syncDAC();
    return;
  }

  if ((attr & PIN_ATTR_PWM) == PIN_ATTR_PWM)
  { 
    value = mapResolution(value, _writeResolution, 16);

    uint32_t tcNum = GetTCNumber(pinDesc.ulPWMChannel);
    uint8_t tcChannel = GetTCChannelNumber(pinDesc.ulPWMChannel);
    static bool tcEnabled[TCC_INST_NUM+TC_INST_NUM];

    if (attr & PIN_ATTR_TIMER) {
      #if !(ARDUINO_SAMD_VARIANT_COMPLIANCE >= 10603)
      // Compatibility for cores based on SAMD core <=1.6.2
      if (pinDesc.ulPinType == PIO_TIMER_ALT) {
        pinPeripheral(pin, PIO_TIMER_ALT);
      } else
      #endif
      {
        pinPeripheral(pin, PIO_TIMER);
      }
    } else {
      // We suppose that attr has PIN_ATTR_TIMER_ALT bit set...
      pinPeripheral(pin, PIO_TIMER_ALT);
    }

    if (!tcEnabled[tcNum]) {
      tcEnabled[tcNum] = true;

      uint16_t GCLK_CLKCTRL_IDs[] = {
        GCLK_CLKCTRL_ID(GCM_TCC0_TCC1), // TCC0
        GCLK_CLKCTRL_ID(GCM_TCC0_TCC1), // TCC1
        GCLK_CLKCTRL_ID(GCM_TCC2_TC3),  // TCC2
        GCLK_CLKCTRL_ID(GCM_TCC2_TC3),  // TC3
        GCLK_CLKCTRL_ID(GCM_TC4_TC5),   // TC4
        GCLK_CLKCTRL_ID(GCM_TC4_TC5),   // TC5
        GCLK_CLKCTRL_ID(GCM_TC6_TC7),   // TC6
        GCLK_CLKCTRL_ID(GCM_TC6_TC7),   // TC7
      };
      GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_IDs[tcNum]);
      while (GCLK->STATUS.bit.SYNCBUSY == 1);

      // Set PORT
      if (tcNum >= TCC_INST_NUM) {
        // -- Configure TC
        Tc* TCx = (Tc*) GetTC(pinDesc.ulPWMChannel);
        // Disable TCx
        TCx->COUNT16.CTRLA.bit.ENABLE = 0;
        syncTC_16(TCx);
        // Set Timer counter Mode to 16 bits, normal PWM
        TCx->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16 | TC_CTRLA_WAVEGEN_NPWM;
        syncTC_16(TCx);
        // Set the initial value
        TCx->COUNT16.CC[tcChannel].reg = (uint32_t) value;
        syncTC_16(TCx);
        // Enable TCx
        TCx->COUNT16.CTRLA.bit.ENABLE = 1;
        syncTC_16(TCx);
      } else {
        // -- Configure TCC
        Tcc* TCCx = (Tcc*) GetTC(pinDesc.ulPWMChannel);
        // Disable TCCx
        TCCx->CTRLA.bit.ENABLE = 0;
        syncTCC(TCCx);
        // Set TCCx as normal PWM
        TCCx->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;
        syncTCC(TCCx);
        // Set the initial value
        TCCx->CC[tcChannel].reg = (uint32_t) value;
        syncTCC(TCCx);
        // Set PER to maximum counter value (resolution : 0xFFFF)
        TCCx->PER.reg = 0xFFFF;
        syncTCC(TCCx);
        // Enable TCCx
        TCCx->CTRLA.bit.ENABLE = 1;
        syncTCC(TCCx);
      }
    } else {
      if (tcNum >= TCC_INST_NUM) {
        Tc* TCx = (Tc*) GetTC(pinDesc.ulPWMChannel);
        TCx->COUNT16.CC[tcChannel].reg = (uint32_t) value;
        syncTC_16(TCx);
      } else {
        Tcc* TCCx = (Tcc*) GetTC(pinDesc.ulPWMChannel);
        TCCx->CTRLBSET.bit.LUPD = 1;
        syncTCC(TCCx);
        TCCx->CCB[tcChannel].reg = (uint32_t) value;
        syncTCC(TCCx);
        TCCx->CTRLBCLR.bit.LUPD = 1;
        syncTCC(TCCx);
      }
    }
    return;
  }

  // -- Defaults to digital write
  pinMode(pin, OUTPUT);
  value = mapResolution(value, _writeResolution, 8);
  if (value < 128) {
    digitalWrite(pin, LOW);
  } else {
    digitalWrite(pin, HIGH);
  }
}

/**
 * Tick is is called 1000 timer per second.
 * It is used to update pwm values for heater and some other frequent jobs.
 *
 *  - Manage PWM to all the heaters and fan
 *  - Prepare or Measure one of the raw ADC sensor values
 *  - Step the babysteps value for each axis towards 0
 *  - For PINS_DEBUGGING, monitor and report endstop pins
 *  - For ENDSTOP_INTERRUPTS_FEATURE check endstops if flagged
 */
void HAL::Tick() {
 
  static uint8_t  cycle_100ms = 0;

  if (!printer.isRunning()) return;

  #if HEATER_COUNT > 0
  
    LOOP_HEATER() heaters[h].SetHardwarePwm();
  #endif

  #if FAN_COUNT > 0
    //LOOP_FAN() fans[f].SetHardwarePwm();
  #endif

  // Calculation cycle approximate a 100ms
  cycle_100ms++;
  if (cycle_100ms >= 100) {
    cycle_100ms = 0;
    execute_100ms = true;
  }

  // read analog values
  #if ANALOG_INPUTS > 0
  
    for (uint8_t h = 0; h < HEATER_COUNT; h++) {
    
      AnalogInputValues[heaters[h].sensor.pin] = (analogRead(heaters[h].sensor.pin)*16);
  
    }
    
    Analog_is_ready = true;
    AnalogInStartConversion();

    // Update the raw values if they've been read. Else we could be updating them during reading.
    if (HAL::Analog_is_ready) thermalManager.set_current_temp_raw();

  #endif

  #if ENABLED(BABYSTEPPING)
    LOOP_XYZ(axis) {
      int curTodo = mechanics.babystepsTodo[axis]; //get rid of volatile for performance

      if (curTodo) {
        stepper.babystep((AxisEnum)axis, curTodo > 0);
        if (curTodo > 0) mechanics.babystepsTodo[axis]--;
                    else mechanics.babystepsTodo[axis]++;
      }
    }
  #endif //BABYSTEPPING

  endstops.Tick();

}


char *dtostrf (double val, signed char width, unsigned char prec, char *sout) {
  asm(".global _printf_float");

  char fmt[20];
  sprintf(fmt, "%%%d.%df", width, prec);
  sprintf(sout, fmt, val);
  return sout;
}

#endif // ARDUINO_ARCH_SAMD
