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

// 10k thermistor
const short temptable_4[][2] PROGMEM = {
  {    1, 430 },
  {   54, 137 },
  {  107, 107 },
  {  160,  91 },
  {  213,  80 },
  {  266,  71 },
  {  319,  64 },
  {  372,  57 },
  {  425,  51 },
  {  478,  46 },
  {  531,  41 },
  {  584,  35 },
  {  637,  30 },
  {  690,  25 },
  {  743,  20 },
  {  796,  14 },
  {  849,   7 },
  {  902,   0 },
  {  955, -11 },
  { 1008, -35 }
};
