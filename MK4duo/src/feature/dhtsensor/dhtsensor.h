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
#pragma once

/**
 * dhtsensor.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#if ENABLED(DHT_SENSOR)

  // Define types of sensors.
  #define DHT11 11
  #define DHT21 21
  #define DHT22 22

  class DHTSensor {

    public: /** Constructor */

      DHTSensor() {}

    public: /** Public Parameters */

      static pin_t    pin;
      static uint8_t  type;

      static float    Temperature,
                      Humidity;

    private: /** Private Parameters */

      static enum SensorState {
        Init,
        Wait_250ms,
        Wait_20ms,
        Read
      } state;
  
    public: /** Public Function */

      static void init();
      static void factory_parameters();
      static void change_type(const uint8_t dhtType);
      static void print_M305();
      static void spin();

  };

  extern DHTSensor dhtsensor;

#endif // ENABLED(DHT_SENSOR)
