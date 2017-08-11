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

static constexpr Fastio_Param Fastio[111] = {
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

  // D90 to D99
  { PIOA,  1 }, { PIOB, 15 }, { PIOA,  5 }, { PIOB, 12 }, { PIOB, 22 }, { PIOB, 23 }, { PIOB, 24 }, { PIOC, 20 }, { PIOC, 27 }, { PIOC, 10 },

  // D100 to D109
  { PIOC, 11 }, { PIOB, 0 }, { PIOB, 1 }, { PIOB, 2 }, { PIOB, 3 }, { PIOB, 4 }, { PIOB, 5 }, { PIOB, 6 }, { PIOB, 7 }, { PIOB, 8 },
  
  // D110
  { PIOB, 11 }
};

/**
 * utility functions
 */

#ifndef MASK
  #define MASK(PIN) (1 << PIN)
#endif

/**
 * magic I/O routines
 * now you can simply SET_OUTPUT(STEP); WRITE(STEP, 1); WRITE(STEP, 0);
 */

// Read a pin
static FORCE_INLINE bool READ(const uint8_t pin) {
  return (bool)(Fastio[pin].base_address -> PIO_PDSR & (MASK(Fastio[pin].shift_count)));
}

static FORCE_INLINE bool READ_VAR(const uint8_t pin) {
  return g_APinDescription[pin].pPort->PIO_PDSR & g_APinDescription[pin].ulPin ? true : false;
}

// write to a pin
// On some boards pins > 0x100 are used. These are not converted to atomic actions. An critical section is needed.
static FORCE_INLINE void WRITE(const uint8_t pin, uint8_t flag) {
   flag ? Fastio[pin].base_address -> PIO_SODR = MASK(Fastio[pin].shift_count) : Fastio[pin].base_address -> PIO_CODR = MASK(Fastio[pin].shift_count);
}

static FORCE_INLINE void WRITE_VAR(const uint8_t pin, uint8_t flag) {
  flag ? g_APinDescription[pin].pPort->PIO_SODR = g_APinDescription[pin].ulPin : g_APinDescription[pin].pPort->PIO_CODR = g_APinDescription[pin].ulPin;
}

// toggle a pin
static FORCE_INLINE void TOGGLE(const uint8_t pin) {
  WRITE(pin, !READ(pin));
}

/*
// check if pin is an input
#define _GET_INPUT(IO)

// check if pin is an output
#define _GET_OUTPUT(IO)

// check if pin is an timer
#define _GET_TIMER(IO)
*/

// set pin as input
static FORCE_INLINE void SET_INPUT(const Pin pin) {
  pmc_enable_periph_clk(g_APinDescription[pin].ulPeripheralId);
  PIO_Configure(g_APinDescription[pin].pPort, PIO_INPUT, g_APinDescription[pin].ulPin, 0);
}

// set pin as output
static FORCE_INLINE void _SET_OUTPUT(const Pin pin) {
  PIO_Configure(g_APinDescription[pin].pPort, PIO_OUTPUT_1, g_APinDescription[pin].ulPin, g_APinDescription[pin].ulPinConfiguration);
}

// Write doesn't work for pullups
static FORCE_INLINE void PULLUP(const Pin pin) {
  pinMode(pin, INPUT_PULLUP);
}

/*
// check if pin is an input wrapper
#define GET_INPUT(IO) _GET_INPUT(IO)

// check if pin is an output wrapper
#define GET_OUTPUT(IO) _GET_OUTPUT(IO)

// check if pin is an timer wrapper
#define GET_TIMER(IO) _GET_TIMER(IO)
*/

// set pin as input with pullup wrapper
static FORCE_INLINE void SET_INPUT_PULLUP(const Pin pin) {
  SET_INPUT(pin);
  PULLUP(pin);
}

// Shorthand
static FORCE_INLINE void OUT_WRITE(const Pin pin, const uint8_t flag) {
  _SET_OUTPUT(pin);
  WRITE(pin, flag);
}

// set pin as output wrapper
static FORCE_INLINE void SET_OUTPUT(const Pin pin) {
  OUT_WRITE(pin, LOW);
}

#endif  // _HAL_FASTIO_DUE_H
