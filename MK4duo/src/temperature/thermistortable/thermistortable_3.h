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

// mendel-parts
const short temptable_3[][2] PROGMEM = {
  {    1, 864 },
  {   21, 300 },
  {   25, 290 },
  {   29, 280 },
  {   33, 270 },
  {   39, 260 },
  {   46, 250 },
  {   54, 240 },
  {   64, 230 },
  {   75, 220 },
  {   90, 210 },
  {  107, 200 },
  {  128, 190 },
  {  154, 180 },
  {  184, 170 },
  {  221, 160 },
  {  265, 150 },
  {  316, 140 },
  {  375, 130 },
  {  441, 120 },
  {  513, 110 },
  {  588, 100 },
  {  734,  80 },
  {  856,  60 },
  {  938,  40 },
  {  986,  20 },
  { 1008,   0 },
  { 1018, -20 }
};
