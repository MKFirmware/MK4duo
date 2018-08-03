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
 * Description: Fast IO functions for Arduino Due and compatible (SAM3X8E)
 *
 * Contributors:
 *  Copyright (c) 2014 Bob Cousins bobcousins42@googlemail.com
 *  Copyright (c) 2015-2016 Nico Tonnhofer wurstnase.reprap@gmail.com
 */

// **************************************************************************
//
// Description: Fast IO functions for Arduino Due
//
// ARDUINO_ARCH_SAM
// **************************************************************************

#ifndef _HAL_FASTIO_DUE_H
#define _HAL_FASTIO_DUE_H

#include <Arduino.h>

/**
 * Types
 */

typedef struct {
  Pio* base_address;
  uint32_t shift_count;
} Fastio_Param;

/**
 * ports and functions
 *
 * added as necessary or if I feel like it- not a comprehensive list!
 */

// UART
#define RXD (0u)
#define TXD (1u)

/**
 * pins
 */

static constexpr Fastio_Param Fastio[] = {
  // D0 to D9
  { PIOA,  8 }, { PIOA,  9 }, { PIOB, 25 }, { PIOC, 28 }, { PIOC, 26 }, { PIOC, 25 }, { PIOC, 24 }, { PIOC, 23 }, { PIOC, 22 }, { PIOC, 21 },

  // D10 to D19
  { PIOC, 29 }, { PIOD,  7 }, { PIOD,  8 }, { PIOB, 27 }, { PIOD,  4 }, { PIOD,  5 }, { PIOA, 13 }, { PIOA, 12 }, { PIOA, 11 }, { PIOA, 10 },

  // D20 to D29
  { PIOB, 12 }, { PIOB, 13 }, { PIOB, 26 }, { PIOA, 14 }, { PIOA, 15 }, { PIOD,  0 }, { PIOD,  1 }, { PIOD,  2 }, { PIOD,  3 }, { PIOD,  6 },

  // D30 to D39
  { PIOD,  9 }, { PIOA,  7 }, { PIOD, 10 }, { PIOC,  1 }, { PIOC,  2 }, { PIOC,  3 }, { PIOC,  4 }, { PIOC,  5 }, { PIOC,  6 }, { PIOC,  7 },

  // D40 to D49
  { PIOC,  8 }, { PIOC,  9 }, { PIOA, 19 }, { PIOA, 20 }, { PIOC, 19 }, { PIOC, 18 }, { PIOC, 17 }, { PIOC, 16 }, { PIOC, 15 }, { PIOC, 14 },

  // D50 to D59
  { PIOC, 13 }, { PIOC, 12 }, { PIOB, 21 }, { PIOB, 14 }, { PIOA, 16 }, { PIOA, 24 }, { PIOA, 23 }, { PIOA, 22 }, { PIOA,  6 }, { PIOA,  4 },

  // D60 to D69
  { PIOA,  3 }, { PIOA,  2 }, { PIOB, 17 }, { PIOB, 18 }, { PIOB, 19 }, { PIOB, 20 }, { PIOB, 15 }, { PIOB, 16 }, { PIOA,  1 }, { PIOA,  0 },

  // D70 to D79
  { PIOA, 17 }, { PIOA, 18 }, { PIOC, 30 }, { PIOA, 21 }, { PIOA, 25 }, { PIOA, 26 }, { PIOA, 27 }, { PIOA, 28 }, { PIOB, 23 }, { PIOA, 17 },

  // D80 to D89
  { PIOB, 12 }, { PIOA,  8 }, { PIOA, 11 }, { PIOA, 13 }, { PIOD,  4 }, { PIOB, 11 }, { PIOB, 21 }, { PIOA, 29 }, { PIOB, 15 }, { PIOB, 14 },

  // D90 to D91
  { PIOA,  1 }, { PIOB, 15 },
  
  #if ENABLED(ARDUINO_SAM_ARCHIM)

    // D92 to D99
    { PIOC,  11 }, { PIOB, 2 }, { PIOB, 1 }, { PIOB, 0 }, { PIOC, 10 }, { PIOB, 24 }, { PIOB, 7 }, { PIOB, 6 },

    // D100 to D107
    { PIOB, 8 }, { PIOB, 5 }, { PIOB, 4 }, { PIOB, 3 }, { PIOC, 20 }, { PIOB, 22 }, { PIOC, 27 }, { PIOB, 10 },

    // D108 to D110
    { PIOB, 7 }, { PIOB, 8 }, { PIOB, 11 }

  #elif MB(ALLIGATOR_R2)

    // D92 to D99
    { PIOA,  5 }, { PIOB, 12 }, { PIOB, 22 }, { PIOB, 23 }, { PIOB, 24 }, { PIOC, 20 }, { PIOC, 27 }, { PIOC, 10 },

    // D100
    { PIOC, 11 }

  #elif MB(ALLIGATOR_R3)

    // D92 to D99
    { PIOA,  5 }, { PIOB, 12 }, { PIOB, 22 }, { PIOB, 23 }, { PIOB, 24 }, { PIOC, 20 }, { PIOC, 27 }, { PIOC, 10 },

    // D100 to D107
    { PIOC, 11 }, { PIOB, 4 }, { PIOB, 5 }, { PIOB, 6 }, { PIOB, 7 }, { PIOB, 8 }, { PIOB, 9 }, { PIOB, 10 },

    // D108 to D111
    { PIOB, 0 }, { PIOB, 1 }, { PIOB, 2 }, { PIOB, 3 }

  #endif

};

/**
 * utility functions
 */

#ifndef MASK
  #define MASK(PIN) (1 << PIN)
#endif

#define OUTPUT_LOW  0x3
#define OUTPUT_HIGH 0x4

/**
 * magic I/O routines
 * now you can simply SET_OUTPUT(STEP); WRITE(STEP, 1); WRITE(STEP, 0);
 */

// NOT CHANGE uint8_t in Pin, ALLIGATOR board crashed!!!
// Read a pin
FORCE_INLINE static bool READ(const uint8_t pin) {
  return (bool)(Fastio[pin].base_address->PIO_PDSR & (MASK(Fastio[pin].shift_count)));
}
FORCE_INLINE static bool READ_VAR(const uint8_t pin) {
  const PinDescription& pinDesc = g_APinDescription[pin];
	if (pinDesc.ulPinType != PIO_NOT_A_PIN) {
    if (pinDesc.pPort->PIO_PDSR & pinDesc.ulPin)
      return true;
  }
  return false;
}

// write to a pin
// On some boards pins > 0x100 are used. These are not converted to atomic actions. An critical section is needed.
FORCE_INLINE static void WRITE(const uint8_t pin, const bool flag) {
  if (flag)
    Fastio[pin].base_address->PIO_SODR = MASK(Fastio[pin].shift_count);
  else
    Fastio[pin].base_address->PIO_CODR = MASK(Fastio[pin].shift_count);
}
FORCE_INLINE static void WRITE_VAR(const uint8_t pin, const bool flag) {
  volatile Pio* port = digitalPinToPort(pin);
  uint32_t mask = g_APinDescription[pin].ulPin;
  if (flag)
    port->PIO_SODR = mask;
  else
    port->PIO_CODR = mask;
}

// set pin as input
FORCE_INLINE static void SET_INPUT(const pin_t pin) {
  const PinDescription& pinDesc = g_APinDescription[pin];
  if (pinDesc.ulPinType != PIO_NOT_A_PIN) {
    pmc_enable_periph_clk(pinDesc.ulPeripheralId);
    PIO_Configure(pinDesc.pPort, PIO_INPUT, pinDesc.ulPin, 0);
  }
}

// set pin as output
FORCE_INLINE static void SET_OUTPUT(const pin_t pin) {
  const PinDescription& pinDesc = g_APinDescription[pin];
  if (pinDesc.ulPinType != PIO_NOT_A_PIN)
    PIO_Configure(pinDesc.pPort, PIO_OUTPUT_0, pinDesc.ulPin, pinDesc.ulPinConfiguration);
}
FORCE_INLINE static void SET_OUTPUT_HIGH(const pin_t pin) {
  const PinDescription& pinDesc = g_APinDescription[pin];
  if (pinDesc.ulPinType != PIO_NOT_A_PIN)
    PIO_Configure(pinDesc.pPort, PIO_OUTPUT_1, pinDesc.ulPin, pinDesc.ulPinConfiguration);
}

// set pin as input with pullup
FORCE_INLINE static void SET_INPUT_PULLUP(const pin_t pin) {
  const PinDescription& pinDesc = g_APinDescription[pin];
  if (pinDesc.ulPinType != PIO_NOT_A_PIN) {
    pmc_enable_periph_clk(pinDesc.ulPeripheralId);
    PIO_Configure(pinDesc.pPort, PIO_INPUT, pinDesc.ulPin, PIO_PULLUP);
  }
}

// Shorthand
FORCE_INLINE static void OUT_WRITE(const pin_t pin, const uint8_t flag) {
  SET_OUTPUT(pin);
  WRITE(pin, flag);
}

FORCE_INLINE static bool USEABLE_HARDWARE_PWM(const pin_t pin) {
  const uint32_t attr = g_APinDescription[pin].ulPinAttribute;
  if ((attr & PIN_ATTR_PWM) != 0 || (attr & PIN_ATTR_TIMER) != 0)
    return true;
  else
    return false;
}

#endif  // _HAL_FASTIO_DUE_H
