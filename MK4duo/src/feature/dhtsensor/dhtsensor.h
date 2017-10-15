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
 * dhtsensor.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#ifndef _DHTSENSOR_H_
#define _DHTSENSOR_H_

#if ENABLED(DHT_SENSOR)

  // Define types of sensors.
  #define DHT11 11
  #define DHT21 21
  #define DHT22 22

  class DhtSensor {

    public: /** Constructor */

      DhtSensor(const Pin _pin, const uint8_t _type);

    public: /** Public Parameters */

      Pin     pin;
      uint8_t type;

    private: /** Private Parameters */

      uint8_t data[5];

      #if ENABLED(__AVR__)
        // Use direct GPIO access on an 8-bit AVR so keep track of the port and bitmask
        // for the digital pin connected to the DHT. Other platforms will use digitalRead.
        uint8_t _bit, _port;
      #endif

      millis_t  lastreadtime,
                maxcycles;

      bool lastresult;

    public: /** Public Function */

      void init(void);
      void change_type(const uint8_t dhtType);
      void print_parameters(void);

      float readTemperature(const bool force=false);
      float readHumidity();

    private: /** Private Funtion */

      bool read(const bool force=false);
      uint32_t expectPulse(const bool level);

  };

  class InterruptLock {
    public:
     InterruptLock()  { noInterrupts(); }
     ~InterruptLock() { interrupts(); }
  };

  extern DhtSensor dhtsensor;

#endif // ENABLED(DHT_SENSOR)

#endif /* _DHTSENSOR_H_ */
