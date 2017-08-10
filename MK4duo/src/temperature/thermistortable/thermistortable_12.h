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

// 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup) (calibrated for Makibox hot bed)
const short temptable_12[][2] PROGMEM = {
  {   35, 180 }, // top rating 180C
  {  211, 140 },
  {  233, 135 },
  {  261, 130 },
  {  290, 125 },
  {  328, 120 },
  {  362, 115 },
  {  406, 110 },
  {  446, 105 },
  {  496, 100 },
  {  539,  95 },
  {  585,  90 },
  {  629,  85 },
  {  675,  80 },
  {  718,  75 },
  {  758,  70 },
  {  793,  65 },
  {  822,  60 },
  {  841,  55 },
  {  875,  50 },
  {  899,  45 },
  {  926,  40 },
  {  946,  35 },
  {  962,  30 },
  {  977,  25 },
  {  987,  20 },
  {  995,  15 },
  { 1001,  10 },
  { 1010,   0 },
  { 1023, -40 }
};
