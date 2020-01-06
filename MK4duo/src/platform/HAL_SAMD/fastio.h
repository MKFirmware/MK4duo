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

/**
 * Description: Fast IO functions for Arduino Due and compatible (SAM3X8E)
 *
 * Contributors:
 *  Copyright (c) 2014 Bob Cousins bobcousins42@googlemail.com
 *  Copyright (c) 2015-2016 Nico Tonnhofer wurstnase.reprap@gmail.com
 *  Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

// **************************************************************************
//
// Description: Fast IO functions for Arduino Due
//
// ARDUINO_ARCH_SAM
// **************************************************************************

#include <Arduino.h>

/**
 * Types
 */

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


// Read a pin
FORCE_INLINE static bool READ(const uint8_t pin) {
  return !!(PORT->Group[g_APinDescription[pin].ulPort].IN.reg & (1ul << g_APinDescription[pin].ulPin));
}

// write to a pin
// On some boards pins > 0x100 are used. These are not converted to atomic actions. An critical section is needed.
FORCE_INLINE static void WRITE(const uint8_t pin, const bool flag) {
  if (flag)  
    PORT->Group[g_APinDescription[pin].ulPort].OUTSET.reg = (1ul << g_APinDescription[pin].ulPin);
  else    
    PORT->Group[g_APinDescription[pin].ulPort].OUTCLR.reg = (1ul << g_APinDescription[pin].ulPin);
}

// set pin as input
FORCE_INLINE static void SET_INPUT(const pin_t pin) {
  pinMode(pin, INPUT);
}

// set pin as output
FORCE_INLINE static void SET_OUTPUT(const pin_t pin) {
  pinMode(pin, OUTPUT);
}
FORCE_INLINE static void SET_OUTPUT_HIGH(const pin_t pin) {
  pinMode(pin, OUTPUT);
  WRITE(pin, HIGH);
}

// set pin as input with pullup
FORCE_INLINE static void SET_INPUT_PULLUP(const pin_t pin) {
  pinMode(pin, INPUT_PULLUP);
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
