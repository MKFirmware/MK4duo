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

typedef uint32_t  millis_l;
typedef uint16_t  millis_s;
typedef int8_t    pin_t;

typedef struct {
  float min,
        max;
} axis_limits_t;

#ifdef __AVR__
  typedef uint8_t       mixer_color_t;
  typedef int8_t        mixer_accu_t;
  typedef int8_t        mixer_perc_t;
#else
  typedef uint_fast16_t mixer_color_t;
  typedef uint_fast16_t mixer_accu_t;
  typedef int8_t        mixer_perc_t;
#endif
