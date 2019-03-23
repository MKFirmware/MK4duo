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
 * Description: functions for I2C connected external EEPROM.
 * Not platform dependent.
 */

#include "../../../MK4duo.h"

#if HAS_EEPROM_I2C

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include "../platform.h"

// --------------------------------------------------------------------------
// Local defines
// --------------------------------------------------------------------------

#ifndef EEPROM_DELAY
  #define EEPROM_DELAY 5
#endif

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

static uint8_t eeprom_device_address = 0x50;

static void eeprom_init(void) {
  static bool eeprom_initialised = false;
  if (!eeprom_initialised) {
    WIRE.begin();
    eeprom_initialised = true;
  }
}

void eeprom_write_byte(uint8_t* pos, uint8_t value) {
  unsigned eeprom_address = (unsigned) pos;

  eeprom_init();

  WIRE.beginTransmission(eeprom_device_address);
  WIRE.write((int)(eeprom_address >> 8));   // MSB
  WIRE.write((int)(eeprom_address & 0xFF)); // LSB
  WIRE.write(value);
  WIRE.endTransmission();

  // wait for write cycle to complete
  // this could be done more efficiently with "acknowledge polling"
  HAL::delayMilliseconds(EEPROM_DELAY);
}

// WARNING: address is a page address, 6-bit end will wrap around
// also, data can be maximum of about 30 bytes, because the Wire library has a buffer of 32 bytes
void eeprom_update_block(const void* pos, void* eeprom_address, size_t n) {

  eeprom_init();

  WIRE.beginTransmission(eeprom_device_address);
  WIRE.write((int)((unsigned)eeprom_address >> 8));   // MSB
  WIRE.write((int)((unsigned)eeprom_address & 0xFF)); // LSB
  WIRE.endTransmission();

  uint8_t *ptr = (uint8_t*)pos;
  uint8_t flag = 0;
  WIRE.requestFrom(eeprom_device_address, (byte)n);
  for (byte c = 0; c < n && WIRE.available(); c++)
    flag |= WIRE.read() ^ ptr[c];

  if (flag) {
    WIRE.beginTransmission(eeprom_device_address);
    WIRE.write((int)((unsigned)eeprom_address >> 8));   // MSB
    WIRE.write((int)((unsigned)eeprom_address & 0xFF)); // LSB
    WIRE.write((uint8_t*)(pos), n);
    WIRE.endTransmission();

    // wait for write cycle to complete
    // this could be done more efficiently with "acknowledge polling"
    HAL::delayMilliseconds(EEPROM_DELAY);
  }
}

uint8_t eeprom_read_byte(uint8_t* pos) {
  byte data = 0xFF;
  unsigned eeprom_address = (unsigned) pos;

  eeprom_init ();

  WIRE.beginTransmission(eeprom_device_address);
  WIRE.write((int)(eeprom_address >> 8));   // MSB
  WIRE.write((int)(eeprom_address & 0xFF)); // LSB
  WIRE.endTransmission();
  WIRE.requestFrom(eeprom_device_address, (byte)1);
  return WIRE.available() ? WIRE.read() : 0xFF;
}

// maybe let's not read more than 30 or 32 bytes at a time!
void eeprom_read_block(void* pos, const void* eeprom_address, size_t n) {
  eeprom_init();

  WIRE.beginTransmission(eeprom_device_address);
  WIRE.write((int)((unsigned)eeprom_address >> 8));   // MSB
  WIRE.write((int)((unsigned)eeprom_address & 0xFF)); // LSB
  WIRE.endTransmission();
  WIRE.requestFrom(eeprom_device_address, (byte)n);
  for (byte c = 0; c < n; c++ )
    if (WIRE.available()) *((uint8_t*)pos + c) = WIRE.read();
}

#endif // HAS_EEPROM_I2C
