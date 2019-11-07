/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2019 Alberto Cotronei @MagoKimbra
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

/**
 * Fast I/O interfaces for STM32
 * These use GPIO register access for fast port manipulation.
 */

// ------------------------
// Public Variables
// ------------------------

extern GPIO_TypeDef * FastIOPortMap[];

// ------------------------
// Public functions
// ------------------------

void FastIO_init(); // Must be called before using fast io macros

// ------------------------
// Defines
// ------------------------

#define _BV32(b) (1UL << (b))

#define OUTPUT_LOW  0x3
#define OUTPUT_HIGH 0x4

#ifndef PWM
  #define PWM OUTPUT
#endif

// Read a pin
FORCE_INLINE static bool READ(const uint8_t pin) {
  #if ENABLED(PCF8574_EXPANSION_IO)
    if (pin >= PIN_START_FOR_PCF8574) {
      return pcf8574.digitalRead(pin - PIN_START_FOR_PCF8574);
    }
    else
  #endif
  {
    return digitalReadFast(digitalPinToPinName(pin));
  }
}

// Write to a pin
FORCE_INLINE static void WRITE(const uint8_t pin, const bool flag) {
  #if ENABLED(PCF8574_EXPANSION_IO)
    if (pin >= PIN_START_FOR_PCF8574) {
      pcf8574.digitalWrite(pin - PIN_START_FOR_PCF8574, flag);
    }
    else
  #endif
  {
    digitalWriteFast(digitalPinToPinName(pin), flag);
  }
}

// Toogle pin
FORCE_INLINE static void TOGGLE(const uint8_t pin) {
  digitalToggleFast(digitalPinToPinName(pin));
}

// Set pin as input
FORCE_INLINE static void SET_INPUT(const pin_t pin) {
  #if ENABLED(PCF8574_EXPANSION_IO)
    if (pin >= PIN_START_FOR_PCF8574) {
      pcf8574.pinMode(pin - PIN_START_FOR_PCF8574, INPUT);
    }
    else
  #endif
  {
    const PinName p = digitalPinToPinName(pin);
    if (p != NC) {
      pin_function(p, STM_PIN_DATA(STM_MODE_INPUT, GPIO_NOPULL, 0));
    }
  }
}

// Set pin as output
FORCE_INLINE static void SET_OUTPUT(const pin_t pin) {
  #if ENABLED(PCF8574_EXPANSION_IO)
    if (pin >= PIN_START_FOR_PCF8574) {
      pcf8574.pinMode(pin - PIN_START_FOR_PCF8574, OUTPUT);
    }
    else
  #endif
  {
    const PinName p = digitalPinToPinName(pin);
    if (p != NC) {
      pin_function(p, STM_PIN_DATA(STM_MODE_OUTPUT_PP, GPIO_NOPULL, 0));
    }
  }
}
FORCE_INLINE static void SET_OUTPUT_LOW(const pin_t pin) {
  #if ENABLED(PCF8574_EXPANSION_IO)
    if (pin >= PIN_START_FOR_PCF8574) {
      pcf8574.pinMode(pin - PIN_START_FOR_PCF8574, OUTPUT);
      pcf8574.digitalWrite(pin - PIN_START_FOR_PCF8574, LOW);
    }
    else
  #endif
  {
    const PinName p = digitalPinToPinName(pin);
    if (p != NC) {
      pin_function(p, STM_PIN_DATA(STM_MODE_OUTPUT_PP, GPIO_NOPULL, 0));
      digitalWriteFast(p, LOW);
    }
  }
}
FORCE_INLINE static void SET_OUTPUT_HIGH(const pin_t pin) {
  #if ENABLED(PCF8574_EXPANSION_IO)
    if (pin >= PIN_START_FOR_PCF8574) {
      pcf8574.pinMode(pin - PIN_START_FOR_PCF8574, OUTPUT);
      pcf8574.digitalWrite(pin - PIN_START_FOR_PCF8574, HIGH);
    }
    else
  #endif
  {
    const PinName p = digitalPinToPinName(pin);
    if (p != NC) {
      pin_function(p, STM_PIN_DATA(STM_MODE_OUTPUT_PP, GPIO_NOPULL, 0));
      digitalWriteFast(p, HIGH);
    }
  }
}

// Set pin as input with pullup
FORCE_INLINE static void SET_INPUT_PULLUP(const pin_t pin) {
  const PinName p = digitalPinToPinName(pin);
  if (p != NC) {
    pin_function(p, STM_PIN_DATA(STM_MODE_INPUT, GPIO_PULLUP, 0));
  }
}

// Shorthand
FORCE_INLINE static void OUT_WRITE(const pin_t pin, const uint8_t flag) {
  #if ENABLED(PCF8574_EXPANSION_IO)
    if (pin >= PIN_START_FOR_PCF8574) {
      pcf8574.pinMode(pin - PIN_START_FOR_PCF8574, OUTPUT);
      pcf8574.digitalWrite(pin - PIN_START_FOR_PCF8574, flag);
    }
    else
  #endif
  {
    flag ? SET_OUTPUT_HIGH(pin) : SET_OUTPUT_LOW(pin);
  }
}

FORCE_INLINE static bool USEABLE_HARDWARE_PWM(const pin_t pin) {
  return digitalPinHasPWM(pin);
}
