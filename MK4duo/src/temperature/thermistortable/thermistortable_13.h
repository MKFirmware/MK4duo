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

// Hisens thermistor B25/50 =3950 +/-1%
const short temptable_13[][2] PROGMEM = {
  {  20.04, 300 },
  {  23.19, 290 },
  {  26.71, 280 },
  {  31.23, 270 },
  {  36.52, 260 },
  {  42.75, 250 },
  {  50.68, 240 },
  {  60.22, 230 },
  {  72.03, 220 },
  {  86.84, 210 },
  { 102.79, 200 },
  { 124.46, 190 },
  { 151.02, 180 },
  { 182.86, 170 },
  { 220.72, 160 },
  { 316.96, 140 },
  { 447.17, 120 },
  { 590.61, 100 },
  { 737.31,  80 },
  { 857.77,  60 },
  { 939.52,  40 },
  { 986.03,  20 },
  { 1008.7,   0 }
};
