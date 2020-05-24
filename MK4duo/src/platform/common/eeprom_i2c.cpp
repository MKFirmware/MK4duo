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

/**
 * Description: functions for I2C connected external EEPROM.
 * Not platform dependent.
 */

#include "../../../MK4duo.h"

#if HAS_EEPROM_I2C

#include <Wire.h>

#ifndef EEPROM_WRITE_DELAY
  #define EEPROM_WRITE_DELAY      5
#endif
#ifndef EEPROM_DEVICE_ADDRESS
  #define EEPROM_DEVICE_ADDRESS   0x50
#endif

static constexpr uint8_t eeprom_device_address = EEPROM_DEVICE_ADDRESS;

void eeprom_init() { WIRE.begin(); }

void eeprom_write_byte(uint8_t* pos, uint8_t value) {
  unsigned eeprom_address = (unsigned) pos;

  WIRE.beginTransmission(eeprom_device_address);
  WIRE.write((int)(eeprom_address >> 8));   // MSB
  WIRE.write((int)(eeprom_address & 0xFF)); // LSB
  WIRE.write(value);
  WIRE.endTransmission();

  // wait for write cycle to complete
  // this could be done more efficiently with "acknowledge polling"
  HAL::delayMilliseconds(EEPROM_WRITE_DELAY);
}

uint8_t eeprom_read_byte(uint8_t* pos) {
  const unsigned eeprom_address = (unsigned) pos;

  WIRE.beginTransmission(eeprom_device_address);
  WIRE.write((int)(eeprom_address >> 8));   // MSB
  WIRE.write((int)(eeprom_address & 0xFF)); // LSB
  WIRE.endTransmission();
  WIRE.requestFrom(eeprom_device_address, (byte)1);
  return WIRE.available() ? WIRE.read() : 0xFF;
}

#endif // HAS_EEPROM_I2C
