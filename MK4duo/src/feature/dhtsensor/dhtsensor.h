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

      static float  current_temperature,
                    current_humidity;

    private: /** Private Parameters */

      static DhtSensorType type;
      static millis_t lastReadTime,
                      lastOperationTime;

      static enum SensorState	{
        Initialising,
        Starting,
        Starting2,
        Reading
      } state;

    public: /** Public Function */

      static void init();
      static void Configure(const uint8_t dhtType);
      static void Spin();

  };

  extern DhtSensor dhtsensor;

#endif // DHT_SENSOR

#endif /* _DHTSENSOR_H_ */
