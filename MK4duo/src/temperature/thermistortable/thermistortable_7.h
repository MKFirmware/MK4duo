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

// 100k Honeywell 135-104LAG-J01
const short temptable_7[][2] PROGMEM = {
  {    1, 941 },
  {   19, 362 },
  {   37, 299 }, // top rating 300C
  {   55, 266 },
  {   73, 245 },
  {   91, 229 },
  {  109, 216 },
  {  127, 206 },
  {  145, 197 },
  {  163, 190 },
  {  181, 183 },
  {  199, 177 },
  {  217, 171 },
  {  235, 166 },
  {  253, 162 },
  {  271, 157 },
  {  289, 153 },
  {  307, 149 },
  {  325, 146 },
  {  343, 142 },
  {  361, 139 },
  {  379, 135 },
  {  397, 132 },
  {  415, 129 },
  {  433, 126 },
  {  451, 123 },
  {  469, 121 },
  {  487, 118 },
  {  505, 115 },
  {  523, 112 },
  {  541, 110 },
  {  559, 107 },
  {  577, 105 },
  {  595, 102 },
  {  613,  99 },
  {  631,  97 },
  {  649,  94 },
  {  667,  92 },
  {  685,  89 },
  {  703,  86 },
  {  721,  84 },
  {  739,  81 },
  {  757,  78 },
  {  775,  75 },
  {  793,  72 },
  {  811,  69 },
  {  829,  66 },
  {  847,  62 },
  {  865,  59 },
  {  883,  55 },
  {  901,  51 },
  {  919,  46 },
  {  937,  41 },
  {  955,  35 },
  {  973,  27 },
  {  991,  17 },
  { 1009,   1 },
  { 1023,   0 } // to allow internal 0 degrees C
};
