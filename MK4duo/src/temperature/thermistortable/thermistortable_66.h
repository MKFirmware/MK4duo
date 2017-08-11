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

// DyzeDesign 500°C Thermistor
const short temptable_66[][2] PROGMEM = {
  {   17.5, 850 },
  {   17.9, 500 },
  {   21.7, 480 },
  {   26.6, 460 },
  {   33.1, 440 },
  {   41.0, 420 },
  {   52.3, 400 },
  {   67.7, 380 },
  {   86.5, 360 },
  {  112.0, 340 },
  {  147.2, 320 },
  {  194.0, 300 },
  {  254.3, 280 },
  {  330.2, 260 },
  {  427.9, 240 },
  {  533.4, 220 },
  {  646.5, 200 },
  {  754.4, 180 },
  {  844.3, 160 },
  {  911.7, 140 },
  {  958.6, 120 },
  {  988.8, 100 },
  { 1006.6,  80 },
  { 1015.8,  60 },
  { 1021.3,  30 },
  {   1023 - 1, 25},
  {   1023,  20}
};
