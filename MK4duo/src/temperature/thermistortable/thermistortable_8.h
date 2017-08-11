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

// 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup)
const short temptable_8[][2] PROGMEM = {
  {    1, 704 },
  {   54, 216 },
  {  107, 175 },
  {  160, 152 },
  {  213, 137 },
  {  266, 125 },
  {  319, 115 },
  {  372, 106 },
  {  425,  99 },
  {  478,  91 },
  {  531,  85 },
  {  584,  78 },
  {  637,  71 },
  {  690,  65 },
  {  743,  58 },
  {  796,  50 },
  {  849,  42 },
  {  902,  31 },
  {  955,  17 },
  { 1008,   0 }
};
