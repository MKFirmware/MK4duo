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

// 100k ATC Semitec 104GT-2 (Used on ParCan) (WITH 1kohm RESISTOR FOR PULLUP, R9 ON SANGUINOLOLU! NOT FOR 4.7kohm PULLUP! THIS IS NOT NORMAL!)
// Verified by linagee. Source: http://shop.arcol.hu/static/datasheets/thermistors.pdf
// Calculated using 1kohm pullup, voltage divider math, and manufacturer provided temp/resistance
// Advantage: More resolution and better linearity from 150C to 200C
const short temptable_55[][2] PROGMEM = {
  {    1, 500 },
  {   76, 300 },
  {   87, 290 },
  {  100, 280 },
  {  114, 270 },
  {  131, 260 },
  {  152, 250 },
  {  175, 240 },
  {  202, 230 },
  {  234, 220 },
  {  271, 210 },
  {  312, 200 },
  {  359, 190 },
  {  411, 180 },
  {  467, 170 },
  {  527, 160 },
  {  590, 150 },
  {  652, 140 },
  {  713, 130 },
  {  770, 120 },
  {  822, 110 },
  {  867, 100 },
  {  905,  90 },
  {  936,  80 },
  {  961,  70 },
  {  979,  60 },
  {  993,  50 },
  { 1003,  40 },
  { 1010,  30 },
  { 1015,  20 },
  { 1018,  10 },
  { 1020,   0 }
};
