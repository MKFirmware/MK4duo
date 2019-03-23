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

#include "../../../MK4duo.h"

#if ENABLED(PCF8574_EXPANSION_IO)

  uint8_t PCF8574::_address;

  byte  PCF8574::writeMode          = 0,
        PCF8574::readMode           = 0,
        PCF8574::byteBuffered       = 0,
        PCF8574::writeByteBuffered  = 0;

  millis_t PCF8574::lastReadMillis  = 0;

  void PCF8574::begin() {

    WIRE.begin();

    // Check if there are pins to set low
    if (writeMode || readMode) {
      WIRE.beginTransmission(_address);
      byte usedPin = writeMode | readMode;
      WIRE.write(~usedPin);
      WIRE.endTransmission();
    }

    lastReadMillis = millis();
  }

  void PCF8574::pinMode(const uint8_t pin, const uint8_t mode){

    if (mode == OUTPUT) {
      writeMode = writeMode |  bit(pin);
      readMode  = readMode  & ~bit(pin);
    }
    else if (mode == INPUT) {
      writeMode = writeMode &  ~bit(pin);
      readMode  = readMode  |   bit(pin);
    }
    begin();
  };

  void PCF8574::digitalWrite(const uint8_t pin, const uint8_t value) {
    WIRE.beginTransmission(_address);   // Begin the transmission to PCF8574
    if (value == HIGH)
      writeByteBuffered = writeByteBuffered | bit(pin);
    else
      writeByteBuffered = writeByteBuffered & ~bit(pin);

    writeByteBuffered = writeByteBuffered & writeMode;
    WIRE.write(writeByteBuffered);
    WIRE.endTransmission();
  };

  uint8_t PCF8574::digitalRead(const uint8_t pin) {
    uint8_t value = LOW;

    // Check if pin already HIGH than read and prevent reread of i2c
    if ((bit(pin) & byteBuffered))
      value = HIGH;
    else if ((millis() > lastReadMillis + READ_ELAPSED_TIME)) {
      WIRE.requestFrom(_address, (uint8_t)1); // Begin transmission to PCF8574
      lastReadMillis = millis();

      if (WIRE.available()) {                 // If bytes are available to be recieved
        byte iInput = WIRE.read();            // Read a byte

        if ((iInput & readMode)) {
          byteBuffered = byteBuffered | (byte)iInput;
          if ((bit(pin) & byteBuffered))
            value = HIGH;
        }
      }
    }

    // If HIGH set to low to read buffer only one time
    if (value == HIGH)
      byteBuffered = ~bit(pin) & byteBuffered;

    return value;
  };

  PCF8574 pcf8574(PCF8574_ADDRESS);

#endif // PCF8574_EXPANSION_IO
