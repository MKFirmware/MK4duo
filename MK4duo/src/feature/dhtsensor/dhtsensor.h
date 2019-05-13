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

/**
 * dhtsensor.h
 *
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
 */

#if ENABLED(DHT_SENSOR)

// Define types of sensors.
enum DHTEnum : uint8_t { DHT11=11, DHT12=12, DHT21=21, DHT22=22 };

// Struct DHT data
typedef struct {
  pin_t   pin;
  DHTEnum type;
} dht_data_t;

class DHTSensor {

  public: /** Constructor */

    DHTSensor() {}

  public: /** Public Parameters */

    static dht_data_t data;

    static float  Temperature,
                  Humidity;

  private: /** Private Parameters */

    static const millis_l _maxcycles;

    static uint8_t read_data[5];

  public: /** Public Function */

    static void init();
    static void factory_parameters();
    static void change_type(const DHTEnum dhtType);
    static void print_M305();
    static void spin();

    static float dewPoint();
    static float dewPointFast();

  private: /** Private Function */

    static uint32_t expectPulse(bool level);

    static float read_temperature();
    static float read_humidity();

};

extern DHTSensor dhtsensor;

#endif // ENABLED(DHT_SENSOR)
