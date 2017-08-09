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

//
// 200k ATC Semitec 204GT-2
// Verified by linagee. Source: http://shop.arcol.hu/static/datasheets/thermistors.pdf
// Calculated using 4.7kohm pullup, voltage divider math, and manufacturer provided temp/resistance
//
const short temptable_2[][2] PROGMEM = {
  {    1, 848 },
  {   30, 300 }, // top rating 300C
  {   34, 290 },
  {   39, 280 },
  {   46, 270 },
  {   53, 260 },
  {   63, 250 },
  {   74, 240 },
  {   87, 230 },
  {  104, 220 },
  {  124, 210 },
  {  148, 200 },
  {  176, 190 },
  {  211, 180 },
  {  252, 170 },
  {  301, 160 },
  {  357, 150 },
  {  420, 140 },
  {  489, 130 },
  {  562, 120 },
  {  636, 110 },
  {  708, 100 },
  {  775,  90 },
  {  835,  80 },
  {  884,  70 },
  {  924,  60 },
  {  955,  50 },
  {  977,  40 },
  {  993,  30 },
  { 1004,  20 },
  { 1012,  10 },
  { 1016,   0 }
};
