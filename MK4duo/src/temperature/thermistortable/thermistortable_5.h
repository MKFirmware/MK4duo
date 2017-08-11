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

// 100k ParCan thermistor (104GT-2)
// ATC Semitec 104GT-2 (Used in ParCan)
// Verified by linagee. Source: http://shop.arcol.hu/static/datasheets/thermistors.pdf
// Calculated using 4.7kohm pullup, voltage divider math, and manufacturer provided temp/resistance
const short temptable_5[][2] PROGMEM = {
  {    1, 713 },
  {   17, 300 }, // top rating 300C
  {   20, 290 },
  {   23, 280 },
  {   27, 270 },
  {   31, 260 },
  {   37, 250 },
  {   43, 240 },
  {   51, 230 },
  {   61, 220 },
  {   73, 210 },
  {   87, 200 },
  {  106, 190 },
  {  128, 180 },
  {  155, 170 },
  {  189, 160 },
  {  230, 150 },
  {  278, 140 },
  {  336, 130 },
  {  402, 120 },
  {  476, 110 },
  {  554, 100 },
  {  635,  90 },
  {  713,  80 },
  {  784,  70 },
  {  846,  60 },
  {  897,  50 },
  {  937,  40 },
  {  966,  30 },
  {  986,  20 },
  { 1000,  10 },
  { 1010,   0 }
};
