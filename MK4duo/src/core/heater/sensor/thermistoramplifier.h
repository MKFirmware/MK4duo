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

// PT100 with INA826 amp on Ultimaker v2.0 electronics
// The PT100 in the Ultimaker v2.0 electronics has a high sample value for a high temperature.
// This does not match the normal thermistor behaviour so we need to set the following defines

#ifdef __AVR__

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

#else

  const short temptable_amplifier[][2] PROGMEM = {
    {    0,    0 },
    {  5511,    1 },
    {  5710,   10 },
    {  5958,   20 },
    {  6156,   30 },
    {  6355,   40 },
    {  6554,   50 },
    {  6752,   60 },
    {  6951,   70 },
    {  7149,   80 },
    {  7348,   90 },
    {  7547,  100 },
    {  7745,  110 },
    {  7993,  120 },
    {  8192,  130 },
    {  8341,  140 },
    {  8540,  150 },
    {  8738,  160 },
    {  8937,  170 },
    {  9135,  180 },
    {  9334,  190 },
    {  9533,  200 },
    {  9731,  210 },
    {  9930,  220 },
    { 10128,  230 },
    { 10277,  240 },
    { 10476,  250 },
    { 10674,  260 },
    { 10823,  270 },
    { 11022,  280 },
    { 11221,  290 },
    { 11370,  300 },
    { 11568,  310 },
    { 11767,  320 },
    { 11965,  330 },
    { 12114,  340 },
    { 12313,  350 },
    { 12462,  360 },
    { 12660,  370 },
    { 12809,  380 },
    { 13008,  390 },
    { 13206,  400 },
    { 14895,  500 },
    { 16533,  600 },
    { 18022,  700 },
    { 19512,  800 },
    { 20902,  900 },
    { 22243, 1000 },
    { 23484, 1100 }
  };

#endif
