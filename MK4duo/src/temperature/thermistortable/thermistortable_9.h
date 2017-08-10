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

// 100k GE Sensing AL03006-58.2K-97-G1 (4.7k pullup)
const short temptable_9[][2] PROGMEM = {
  {    1, 936 },
  {   36, 300 },
  {   71, 246 },
  {  106, 218 },
  {  141, 199 },
  {  176, 185 },
  {  211, 173 },
  {  246, 163 },
  {  281, 155 },
  {  316, 147 },
  {  351, 140 },
  {  386, 134 },
  {  421, 128 },
  {  456, 122 },
  {  491, 117 },
  {  526, 112 },
  {  561, 107 },
  {  596, 102 },
  {  631,  97 },
  {  666,  92 },
  {  701,  87 },
  {  736,  81 },
  {  771,  76 },
  {  806,  70 },
  {  841,  63 },
  {  876,  56 },
  {  911,  48 },
  {  946,  38 },
  {  981,  23 },
  { 1005,   5 },
  { 1016,   0 }
};
