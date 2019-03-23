/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
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
 * softpwm.cpp
 *
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"

SoftPWM softpwm;

/** Private Parameters */
volatile uint8_t soft_pwm_count = 0xff;

uint8_t SoftPWM::used_channel = 0;

softPWMChannel SoftPWM::channels[SOFTPWM_MAXCHANNELS];

/** Public Function */
void SoftPWM::init() {
  for (uint8_t i = 0; i < SOFTPWM_MAXCHANNELS; i++) {
    channels[i].pin       = -1;
    channels[i].pwm_value = 0;
    channels[i].pwm_pos   = 0;
  }
}

void SoftPWM::spin(void) {

  if (soft_pwm_count == 0) {
    for (uint8_t i = 0; i < used_channel; i++) {
      // now set the pin high (if not 0)
      if (channels[i].pin > -1 && ((channels[i].pwm_pos = (channels[i].pwm_value & SOFT_PWM_MASK)) > 0))
        HAL::digitalWrite(channels[i].pin, HIGH);
    }
  }

  for (uint8_t i = 0; i < used_channel; i++) {
    // if it's a valid pin turn off the channel
    if (channels[i].pin > -1 && channels[i].pwm_pos == soft_pwm_count && channels[i].pwm_pos != SOFT_PWM_MASK)
      HAL::digitalWrite(channels[i].pin, LOW);
  }

  soft_pwm_count += SOFT_PWM_STEP;

}

void SoftPWM::set(const pin_t pin, const uint8_t value) {

  int8_t firstfree = -1; // first free index

  // If the pin isn't already set, add it
  for (uint8_t i = 0; i < SOFTPWM_MAXCHANNELS; i++) {
    if (pin > -1 && channels[i].pin == pin) {
      channels[i].pwm_value = value;
      return;
    }

    // get the first free pin if available
    if (firstfree < 0 && channels[i].pin < 0)
      firstfree = i;
  }

  if (pin > -1 && firstfree > -1) {
    // we have a free pin we can use
    channels[firstfree].pin = pin;
    channels[firstfree].pwm_value = value;
    used_channel = firstfree + 1;
    //HAL::pinMode(pin, OUTPUT_LOW);
  }

}
