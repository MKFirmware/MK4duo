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

// 100k EPCOS (WITH 1kohm RESISTOR FOR PULLUP, R9 ON SANGUINOLOLU! NOT FOR 4.7kohm PULLUP! THIS IS NOT NORMAL!)
// Verified by linagee.
// Calculated using 1kohm pullup, voltage divider math, and manufacturer provided temp/resistance
// Advantage: Twice the resolution and better linearity from 150C to 200C
const short temptable_51[][2] PROGMEM = {
  {    1, 350 },
  {  190, 250 }, // top rating 250C
  {  203, 245 },
  {  217, 240 },
  {  232, 235 },
  {  248, 230 },
  {  265, 225 },
  {  283, 220 },
  {  302, 215 },
  {  322, 210 },
  {  344, 205 },
  {  366, 200 },
  {  390, 195 },
  {  415, 190 },
  {  440, 185 },
  {  467, 180 },
  {  494, 175 },
  {  522, 170 },
  {  551, 165 },
  {  580, 160 },
  {  609, 155 },
  {  638, 150 },
  {  666, 145 },
  {  695, 140 },
  {  722, 135 },
  {  749, 130 },
  {  775, 125 },
  {  800, 120 },
  {  823, 115 },
  {  845, 110 },
  {  865, 105 },
  {  884, 100 },
  {  901,  95 },
  {  917,  90 },
  {  932,  85 },
  {  944,  80 },
  {  956,  75 },
  {  966,  70 },
  {  975,  65 },
  {  982,  60 },
  {  989,  55 },
  {  995,  50 },
  { 1000,  45 },
  { 1004,  40 },
  { 1007,  35 },
  { 1010,  30 },
  { 1013,  25 },
  { 1015,  20 },
  { 1017,  15 },
  { 1018,  10 },
  { 1019,   5 },
  { 1020,   0 },
  { 1021,  -5 }
};
