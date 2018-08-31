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
 * Support routines for Due
 */

/**
 * Translation of routines & variables used by pinsDebug.h
 */

#include "Arduino.h"

/**
 * Due/MK4duo quirks
 *
 * a) determining the state of a pin
 *     The Due/Arduino status definitions for the g_pinStatus[pin] array are:
 *       #define PIN_STATUS_DIGITAL_INPUT_PULLUP  (0x01)
 *       #define PIN_STATUS_DIGITAL_INPUT         (0x02)
 *       #define PIN_STATUS_DIGITAL_OUTPUT        (0x03)
 *       #define PIN_STATUS_ANALOG                (0x04)
 *       #define PIN_STATUS_PWM                   (0x05)
 *       #define PIN_STATUS_TIMER                 (0x06)
 *
 *     These are only valid if the following Due/Arduino provided functions are used:
 *       analogRead
 *       analogWrite
 *       digitalWrite
 *       pinMode
 *
 *     The FASTIO routines do not touch the g_pinStatus[pin] array.
 *
 *     The net result is that both the g_pinStatus[pin] array and the PIO_OSR register
 *     needs to be looked at when determining if a pin is an input or an output.
 *
 * b) Due has only pins 6, 7, 8 & 9 enabled for PWMs. FYI - they run at 1KHz
 *
 * c) NUM_DIGITAL_PINS does not include the analog pins
 *
 * d) Pins 0-78 are defined for Due but 78 has a comment of "unconnected!".  78 is
 *    included just in case.
 */

#if MB(ALLIGATOR_R2) || MB(ALLIGATOR_R3)
  #define NUMBER_PINS_TOTAL 100
#else
  #define NUMBER_PINS_TOTAL PINS_COUNT
#endif

#define digitalRead_mod(p)  digitalRead(p)  // AVR digitalRead disabled PWM before it read the pin
#define PRINT_PORT(p)       SERIAL_SP(12);
#define NAME_FORMAT(p)      PSTR("%-##p##s")
#define PRINT_ARRAY_NAME(x) do {sprintf_P(buffer, PSTR("%-" STRINGIFY(MAX_NAME_LENGTH) "s"), pin_array[x].name); SERIAL_PS(buffer);} while (0)
#define PRINT_PIN(p)        do {sprintf_P(buffer, PSTR("%02d  g_pinStatus:  %02d"), p, (g_pinStatus[pin] & 0xF)); SERIAL_PS(buffer);} while (0)
#define GET_ARRAY_PIN(p)    pin_array[p].pin
#define VALID_PIN(pin)      (pin >= 0 && pin < (uint8_t)NUMBER_PINS_TOTAL ? 1 : 0)
#define DIGITAL_PIN_TO_ANALOG_PIN(p) int(p - analogInputToDigitalPin(0))
#define IS_ANALOG(P)        (((P) >= analogInputToDigitalPin(0)) && ((P) <= analogInputToDigitalPin(NUM_ANALOG_INPUTS - 1)))

bool GET_PINMODE(const pin_t pin) {  // 1: output, 0: input
  volatile Pio* port = g_APinDescription[pin].pPort;
  uint32_t mask = g_APinDescription[pin].ulPin;
  uint8_t pin_status = g_pinStatus[pin] & 0xF;
  return (  (pin_status == 0 && (port->PIO_OSR & mask))
          || pin_status == PIN_STATUS_DIGITAL_OUTPUT
          || HAL::pwm_status(pin)
          || HAL::tc_status(pin));
}

bool GET_ARRAY_IS_DIGITAL(const pin_t pin) {
  const uint8_t pin_status = g_pinStatus[pin] & 0xF;
  return  !(pin_status == PIN_STATUS_ANALOG);
}

void pwm_details(int32_t pin) {
  if (HAL::pwm_status(pin)) {
    const uint32_t chan = g_APinDescription[pin].ulPWMChannel;
    SERIAL_MV("PWM = ", PWM_INTERFACE->PWM_CH_NUM[chan].PWM_CDTY);
  }
  else if (HAL::tc_status(pin)) {
    const uint32_t chan = g_APinDescription[pin].ulTCChannel >> 1;
    SERIAL_MV("TC = ", TC_INTERFACE->TC_CHANNEL[chan].TC_RB);
  }
}
