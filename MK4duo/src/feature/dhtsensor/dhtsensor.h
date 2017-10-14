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

  enum class DhtSensorType { Dht11, Dht21, Dht22 };

  class DhtSensor {

    public: /** Constructor */

      DhtSensor() {}

    public: /** Public Parameters */

    private: /** Private Parameters */

      uint8_t data[5];

      DhtSensorType type = DhtSensorType::Dht11;

      uint32_t  lastreadtime,
                maxcycles;

      bool lastresult;

    public: /** Public Function */

      void init();
      void Configure(const uint8_t dhtType);

      float readTemperature(const bool force=false);
      float readHumidity(const bool force=false);

    private: /** Private Funtion */

      bool read(const bool force=false);
      uint32_t expectPulse(bool level);

  };

  class InterruptLock {
    public:
     InterruptLock()  { noInterrupts(); }
     ~InterruptLock() { interrupts(); }
  };

  extern DhtSensor dhtsensor;

#endif // ENABLED(DHT_SENSOR)

#endif /* _DHTSENSOR_H_ */
