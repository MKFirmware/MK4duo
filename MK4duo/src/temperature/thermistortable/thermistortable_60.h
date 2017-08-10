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

// Maker's Tool Works Kapton Bed Thermistor
// ./createTemperatureLookup.py --r0=100000 --t0=25 --r1=0 --r2=4700 --beta=3950
// r0: 100000
// t0: 25
// r1: 0 (parallel with rTherm)
// r2: 4700 (series with rTherm)
// beta: 3950
// min adc: 1 at 0.0048828125 V
// max adc: 1023 at 4.9951171875 V
const short temptable_60[][2] PROGMEM = {
  {   51, 272 },
  {   61, 258 },
  {   71, 247 },
  {   81, 237 },
  {   91, 229 },
  {  101, 221 },
  {  131, 204 },
  {  161, 190 },
  {  191, 179 },
  {  231, 167 },
  {  271, 157 },
  {  311, 148 },
  {  351, 140 },
  {  381, 135 },
  {  411, 130 },
  {  441, 125 },
  {  451, 123 },
  {  461, 122 },
  {  471, 120 },
  {  481, 119 },
  {  491, 117 },
  {  501, 116 },
  {  511, 114 },
  {  521, 113 },
  {  531, 111 },
  {  541, 110 },
  {  551, 108 },
  {  561, 107 },
  {  571, 105 },
  {  581, 104 },
  {  591, 102 },
  {  601, 101 },
  {  611, 100 },
  {  621,  98 },
  {  631,  97 },
  {  641,  95 },
  {  651,  94 },
  {  661,  92 },
  {  671,  91 },
  {  681,  90 },
  {  691,  88 },
  {  701,  87 },
  {  711,  85 },
  {  721,  84 },
  {  731,  82 },
  {  741,  81 },
  {  751,  79 },
  {  761,  77 },
  {  771,  76 },
  {  781,  74 },
  {  791,  72 },
  {  801,  71 },
  {  811,  69 },
  {  821,  67 },
  {  831,  65 },
  {  841,  63 },
  {  851,  62 },
  {  861,  60 },
  {  871,  57 },
  {  881,  55 },
  {  891,  53 },
  {  901,  51 },
  {  911,  48 },
  {  921,  45 },
  {  931,  42 },
  {  941,  39 },
  {  951,  36 },
  {  961,  32 },
  {  981,  23 },
  {  991,  17 },
  { 1001,   9 },
  { 1008,   0 }
};
