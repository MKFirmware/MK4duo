/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2017 Alberto Cotronei @MagoKimbra
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
 * laser.h - Laser control library for Arduino using 16 bit timers- Version 1
 * Copyright (c) 2013 Timothy Schmidt.  All right reserved.
 * Copyright (c) 2016 Franco (nextime) Lanza
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef LASER_H
  #define LASER_H

  #include <inttypes.h>

  // split into planned and status
  typedef struct {

    int       fired;        // method used to ask the laser to fire - LASER_FIRE_G1, LASER_FIRE_SPINDLE, LASER_FIRE_E, etc
    float     intensity,    // Laser firing instensity 0.0 - 100.0
              ppm;          // pulses per millimeter, for pulsed firing mode

    uint32_t  duration,     // laser firing duration in microseconds, for pulsed firing mode
              dur;          // instantaneous duration

    bool      status,       // LASER_ON / LASER_OFF - buffered
              firing,       // LASER_ON / LASER_OFF - instantaneous
              diagnostics;  // Verbose debugging output over serial

    uint8_t   mode;         // CONTINUOUS, PULSED, RASTER

    millis_t  last_firing;  // microseconds since last laser firing

    uint16_t  time,         // temporary counter to limit eeprom writes
              lifetime;     // laser lifetime firing counter in minutes

    #if ENABLED(LASER_RASTER)

      unsigned char raster_data[LASER_MAX_RASTER_LINE],
                    rasterlaserpower;

      float         raster_aspect_ratio,
                    raster_mm_per_pulse;

      int           raster_raw_length,
                    raster_num_pixels;

      uint8_t       raster_direction;

    #endif

  } laser_t;

  extern laser_t laser;

  void laser_init();
  void laser_fire(float intensity=0.5);
  void laser_fire_byte(uint8_t intensity);
  void laser_extinguish();
  void laser_update_lifetime();
  void laser_set_mode(int mode);
  void laser_diagnose();

  #if ENABLED(LASER_PERIPHERALS)
    bool laser_peripherals_ok();
    void laser_peripherals_on();
    void laser_peripherals_off();
    void laser_wait_for_peripherals();
  #endif // LASER_PERIPHERALS

#endif // LASER_H
