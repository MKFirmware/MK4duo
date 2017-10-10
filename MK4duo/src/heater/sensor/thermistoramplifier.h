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

// PT100 with INA826 amp on Ultimaker v2.0 electronics
// The PT100 in the Ultimaker v2.0 electronics has a high sample value for a high temperature.
// This does not match the normal thermistor behaviour so we need to set the following defines

const short temptable_amplifier[][2] PROGMEM = {
  {   0,    0 },
  { 227,    1 },
  { 236,   10 },
  { 245,   20 },
  { 253,   30 },
  { 262,   40 },
  { 270,   50 },
  { 279,   60 },
  { 287,   70 },
  { 295,   80 },
  { 304,   90 },
  { 312,  100 },
  { 320,  110 },
  { 329,  120 },
  { 337,  130 },
  { 345,  140 },
  { 353,  150 },
  { 361,  160 },
  { 369,  170 },
  { 377,  180 },
  { 385,  190 },
  { 393,  200 },
  { 401,  210 },
  { 409,  220 },
  { 417,  230 },
  { 424,  240 },
  { 432,  250 },
  { 440,  260 },
  { 447,  270 },
  { 455,  280 },
  { 463,  290 },
  { 470,  300 },
  { 478,  310 },
  { 485,  320 },
  { 493,  330 },
  { 500,  340 },
  { 507,  350 },
  { 515,  360 },
  { 522,  370 },
  { 529,  380 },
  { 537,  390 },
  { 544,  400 },
  { 614,  500 },
  { 681,  600 },
  { 744,  700 },
  { 805,  800 },
  { 862,  900 },
  { 917, 1000 },
  { 968, 1100 }
};
