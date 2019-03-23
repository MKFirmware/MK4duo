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

#pragma once

#if ENABLED(PCF8574_EXPANSION_IO)

  #define READ_ELAPSED_TIME      10
  #define PIN_START_FOR_PCF8574 120

  class PCF8574 {

    public: /** Constructor */

      PCF8574(uint8_t addre) { _address = addre; };

    private: /** Private Parameters */

      static uint8_t  _address;

      static byte writeMode,
                  readMode,
                  byteBuffered,
                  writeByteBuffered;

      static millis_t lastReadMillis;

      bool        _usingInterrupt = false;

    public: /** Public Function */

      static void begin();
      static void pinMode(const uint8_t pin, const uint8_t mode);
      static void     digitalWrite(const uint8_t pin, const uint8_t value);
      static uint8_t  digitalRead(const uint8_t pin);

  };

  extern PCF8574 pcf8574;

#endif // ENABLED(PCF8574)
