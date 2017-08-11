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

// 200k ATC Semitec 204GT-2 (WITH 1kohm RESISTOR FOR PULLUP, R9 ON SANGUINOLOLU! NOT FOR 4.7kohm PULLUP! THIS IS NOT NORMAL!)
// Verified by linagee. Source: http://shop.arcol.hu/static/datasheets/thermistors.pdf
// Calculated using 1kohm pullup, voltage divider math, and manufacturer provided temp/resistance
// Advantage: More resolution and better linearity from 150C to 200C
const short temptable_52[][2] PROGMEM = {
  {    1, 500 },
  {  125, 300 }, // top rating 300C
  {  142, 290 },
  {  162, 280 },
  {  185, 270 },
  {  211, 260 },
  {  240, 250 },
  {  274, 240 },
  {  312, 230 },
  {  355, 220 },
  {  401, 210 },
  {  452, 200 },
  {  506, 190 },
  {  563, 180 },
  {  620, 170 },
  {  677, 160 },
  {  732, 150 },
  {  783, 140 },
  {  830, 130 },
  {  871, 120 },
  {  906, 110 },
  {  935, 100 },
  {  958,  90 },
  {  976,  80 },
  {  990,  70 },
  { 1000,  60 },
  { 1008,  50 },
  { 1013,  40 },
  { 1017,  30 },
  { 1019,  20 },
  { 1021,  10 },
  { 1022,   0 }
};
