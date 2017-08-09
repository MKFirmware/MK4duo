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

// 100k Epcos thermistor
const short temptable_6[][2] PROGMEM = {
  {    1, 350 },
  {   28, 250 }, // top rating 250C
  {   31, 245 },
  {   35, 240 },
  {   39, 235 },
  {   42, 230 },
  {   44, 225 },
  {   49, 220 },
  {   53, 215 },
  {   62, 210 },
  {   71, 205 }, // fitted graphically
  {   78, 200 }, // fitted graphically
  {   94, 190 },
  {  102, 185 },
  {  116, 170 },
  {  143, 160 },
  {  183, 150 },
  {  223, 140 },
  {  270, 130 },
  {  318, 120 },
  {  383, 110 },
  {  413, 105 },
  {  439, 100 },
  {  484,  95 },
  {  513,  90 },
  {  607,  80 },
  {  664,  70 },
  {  781,  60 },
  {  810,  55 },
  {  849,  50 },
  {  914,  45 },
  {  914,  40 },
  {  935,  35 },
  {  954,  30 },
  {  970,  25 },
  {  978,  22 },
  { 1008,   3 },
  { 1023,   0 } // to allow internal 0 degrees C
};
