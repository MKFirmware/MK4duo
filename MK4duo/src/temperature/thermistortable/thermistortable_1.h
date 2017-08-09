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

// 100k bed thermistor
const short temptable_1[][2] PROGMEM = {
  {   23, 300 },
  {   25, 295 },
  {   27, 290 },
  {   28, 285 },
  {   31, 280 },
  {   33, 275 },
  {   35, 270 },
  {   38, 265 },
  {   41, 260 },
  {   44, 255 },
  {   48, 250 },
  {   52, 245 },
  {   56, 240 },
  {   61, 235 },
  {   66, 230 },
  {   71, 225 },
  {   78, 220 },
  {   84, 215 },
  {   92, 210 },
  {  100, 205 },
  {  109, 200 },
  {  120, 195 },
  {  131, 190 },
  {  143, 185 },
  {  156, 180 },
  {  171, 175 },
  {  187, 170 },
  {  205, 165 },
  {  224, 160 },
  {  245, 155 },
  {  268, 150 },
  {  293, 145 },
  {  320, 140 },
  {  348, 135 },
  {  379, 130 },
  {  411, 125 },
  {  445, 120 },
  {  480, 115 },
  {  516, 110 },
  {  553, 105 },
  {  591, 100 },
  {  628,  95 },
  {  665,  90 },
  {  702,  85 },
  {  737,  80 },
  {  770,  75 },
  {  801,  70 },
  {  830,  65 },
  {  857,  60 },
  {  881,  55 },
  {  903,  50 },
  {  922,  45 },
  {  939,  40 },
  {  954,  35 },
  {  966,  30 },
  {  977,  25 },
  {  985,  20 },
  {  993,  15 },
  {  999,  10 },
  { 1004,   5 },
  { 1008,   0 },
  { 1012,  -5 },
  { 1016, -10 },
  { 1020, -15 }
};
