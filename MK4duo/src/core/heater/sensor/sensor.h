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
 * sensor.h - heater object
 */

#ifndef _SENSOR_H_
#define _SENSOR_H_

class TemperatureSensor {

  public: /** Public Parameters */

    pin_t   pin;
    int16_t type,
            raw,
            adcLowOffset,
            adcHighOffset;
    float   r25,
            beta,
            pullupR,
            shA,
            shB,
            shC;

    #if ENABLED(SUPPORT_AD8495) || ENABLED(SUPPORT_AD595)
      float ad595_offset,
            ad595_gain;
    #endif

  public: /** Public Function */

    void CalcDerivedParameters();
    float getTemperature();

    #if ENABLED(SUPPORT_MAX6675)
      int16_t read_max6675();
    #endif

    #if ENABLED(SUPPORT_MAX31855)
      int16_t read_max31855();
    #endif

};

#endif /* _SENSOR_H_ */
