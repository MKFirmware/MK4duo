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

// bqh2 stock thermistor
const short temptable_70[][2] PROGMEM = {
  {   22, 300 },
  {   24, 295 },
  {   25, 290 },
  {   27, 285 },
  {   29, 280 },
  {   32, 275 },
  {   34, 270 },
  {   37, 265 },
  {   40, 260 },
  {   43, 255 },
  {   46, 250 },
  {   50, 245 },
  {   54, 240 },
  {   59, 235 },
  {   64, 230 },
  {   70, 225 },
  {   76, 220 },
  {   83, 215 },
  {   90, 210 },
  {   99, 205 },
  {  108, 200 },
  {  118, 195 },
  {  129, 190 },
  {  141, 185 },
  {  154, 180 },
  {  169, 175 },
  {  185, 170 },
  {  203, 165 },
  {  222, 160 },
  {  243, 155 },
  {  266, 150 },
  {  290, 145 },
  {  317, 140 },
  {  346, 135 },
  {  376, 130 },
  {  408, 125 },
  {  442, 120 },
  {  477, 115 },
  {  513, 110 },
  {  551, 105 },
  {  588, 100 },
  {  626,  95 },
  {  663,  90 },
  {  699,  85 },
  {  735,  80 },
  {  768,  75 },
  {  800,  70 },
  {  829,  65 },
  {  856,  60 },
  {  881,  55 },
  {  903,  50 },
  {  922,  45 },
  {  939,  40 },
  {  954,  35 },
  {  966,  30 },
  {  977,  25 },
  {  986,  20 },
  {  994,  15 },
  { 1000,  10 },
  { 1005,   5 },
  { 1009,   0 } // safety
};
