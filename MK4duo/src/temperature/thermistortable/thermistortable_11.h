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

// QU-BD silicone bed QWG-104F-3950 thermistor
const short temptable_11[][2] PROGMEM = {
  {    1, 938 },
  {   31, 314 },
  {   41, 290 },
  {   51, 272 },
  {   61, 258 },
  {   71, 247 },
  {   81, 237 },
  {   91, 229 },
  {  101, 221 },
  {  111, 215 },
  {  121, 209 },
  {  131, 204 },
  {  141, 199 },
  {  151, 195 },
  {  161, 190 },
  {  171, 187 },
  {  181, 183 },
  {  191, 179 },
  {  201, 176 },
  {  221, 170 },
  {  241, 165 },
  {  261, 160 },
  {  281, 155 },
  {  301, 150 },
  {  331, 144 },
  {  361, 139 },
  {  391, 133 },
  {  421, 128 },
  {  451, 123 },
  {  491, 117 },
  {  531, 111 },
  {  571, 105 },
  {  611, 100 },
  {  641,  95 },
  {  681,  90 },
  {  711,  85 },
  {  751,  79 },
  {  791,  72 },
  {  811,  69 },
  {  831,  65 },
  {  871,  57 },
  {  881,  55 },
  {  901,  51 },
  {  921,  45 },
  {  941,  39 },
  {  971,  28 },
  {  981,  23 },
  {  991,  17 },
  { 1001,   9 },
  { 1021, -27 }
};
