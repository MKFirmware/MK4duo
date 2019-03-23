/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
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
#pragma once

/**
 * table_gcode.h
 *
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
 */

#define G_CODE_TYPE uint8_t

using command_t = void(*)();

typedef struct {
	const G_CODE_TYPE code;
	const command_t command;
} GCode_command_t;

constexpr GCode_command_t GCode_Table[] = {

  #if ENABLED(CODE_G2)
    { 2, gcode_G2 },
  #endif
  #if ENABLED(CODE_G3)
    { 3, gcode_G3 },
  #endif
  #if ENABLED(CODE_G4)
    { 4, gcode_G4 },
  #endif
  #if ENABLED(CODE_G5)
    { 5, gcode_G5 },
  #endif
  #if ENABLED(CODE_G6)
    { 6, gcode_G6 },
  #endif
  #if ENABLED(CODE_G7)
    { 7, gcode_G7 },
  #endif
  #if ENABLED(CODE_G8)
    { 8, gcode_G8 },
  #endif
  #if ENABLED(CODE_G9)
    { 9, gcode_G9 },
  #endif
  #if ENABLED(CODE_G10)
    { 10, gcode_G10 },
  #endif
  #if ENABLED(CODE_G11)
    { 11, gcode_G11 },
  #endif
  #if ENABLED(CODE_G12)
    { 12, gcode_G12 },
  #endif
  #if ENABLED(CODE_G13)
    { 13, gcode_G13 },
  #endif
  #if ENABLED(CODE_G14)
    { 14, gcode_G14 },
  #endif
  #if ENABLED(CODE_G15)
    { 15, gcode_G15 },
  #endif
  #if ENABLED(CODE_G16)
    { 16, gcode_G16 },
  #endif
  #if ENABLED(CODE_G17)
    { 17, gcode_G17 },
  #endif
  #if ENABLED(CODE_G18)
    { 18, gcode_G18 },
  #endif
  #if ENABLED(CODE_G19)
    { 19, gcode_G19 },
  #endif
  #if ENABLED(CODE_G20)
    { 20, gcode_G20 },
  #endif
  #if ENABLED(CODE_G21)
    { 21, gcode_G21 },
  #endif
  #if ENABLED(CODE_G22)
    { 22, gcode_G22 },
  #endif
  #if ENABLED(CODE_G23)
    { 23, gcode_G23 },
  #endif
  #if ENABLED(CODE_G24)
    { 24, gcode_G24 },
  #endif
  #if ENABLED(CODE_G25)
    { 25, gcode_G25 },
  #endif
  #if ENABLED(CODE_G26)
    { 26, gcode_G26 },
  #endif
  #if ENABLED(CODE_G27)
    { 27, gcode_G27 },
  #endif
  #if ENABLED(CODE_G28)
    { 28, gcode_G28 },
  #endif
  #if ENABLED(CODE_G29)
    { 29, gcode_G29 },
  #endif
  #if ENABLED(CODE_G30)
    { 30, gcode_G30 },
  #endif
  #if ENABLED(CODE_G31)
    { 31, gcode_G31 },
  #endif
  #if ENABLED(CODE_G32)
    { 32, gcode_G32 },
  #endif
  #if ENABLED(CODE_G33)
    { 33, gcode_G33 },
  #endif
  #if ENABLED(CODE_G34)
    { 34, gcode_G34 },
  #endif
  #if ENABLED(CODE_G35)
    { 35, gcode_G35 },
  #endif
  #if ENABLED(CODE_G36)
    { 36, gcode_G36 },
  #endif
  #if ENABLED(CODE_G37)
    { 37, gcode_G37 },
  #endif
  #if ENABLED(CODE_G38)
    { 38, gcode_G38 },
  #endif
  #if ENABLED(CODE_G39)
    { 39, gcode_G39 },
  #endif
  #if ENABLED(CODE_G40)
    { 40, gcode_G40 },
  #endif
  #if ENABLED(CODE_G41)
    { 41, gcode_G41 },
  #endif
  #if ENABLED(CODE_G42)
    { 42, gcode_G42 },
  #endif
  #if ENABLED(CODE_G43)
    { 43, gcode_G43 },
  #endif
  #if ENABLED(CODE_G44)
    { 44, gcode_G44 },
  #endif
  #if ENABLED(CODE_G45)
    { 45, gcode_G45 },
  #endif
  #if ENABLED(CODE_G46)
    { 46, gcode_G46 },
  #endif
  #if ENABLED(CODE_G47)
    { 47, gcode_G47 },
  #endif
  #if ENABLED(CODE_G48)
    { 48, gcode_G48 },
  #endif
  #if ENABLED(CODE_G49)
    { 49, gcode_G49 },
  #endif
  #if ENABLED(CODE_G50)
    { 50, gcode_G50 },
  #endif
  #if ENABLED(CODE_G51)
    { 51, gcode_G51 },
  #endif
  #if ENABLED(CODE_G52)
    { 52, gcode_G52 },
  #endif
  #if ENABLED(CODE_G53)
    { 53, gcode_G53 },
  #endif
  #if ENABLED(CODE_G54)
    { 54, gcode_G54 },
  #endif
  #if ENABLED(CODE_G55)
    { 55, gcode_G55 },
  #endif
  #if ENABLED(CODE_G56)
    { 56, gcode_G56 },
  #endif
  #if ENABLED(CODE_G57)
    { 57, gcode_G57 },
  #endif
  #if ENABLED(CODE_G58)
    { 58, gcode_G58 },
  #endif
  #if ENABLED(CODE_G59)
    { 59, gcode_G59 },
  #endif
  #if ENABLED(CODE_G60)
    { 60, gcode_G60 },
  #endif
  #if ENABLED(CODE_G61)
    { 61, gcode_G61 },
  #endif
  #if ENABLED(CODE_G62)
    { 62, gcode_G62 },
  #endif
  #if ENABLED(CODE_G63)
    { 63, gcode_G63 },
  #endif
  #if ENABLED(CODE_G64)
    { 64, gcode_G64 },
  #endif
  #if ENABLED(CODE_G65)
    { 65, gcode_G65 },
  #endif
  #if ENABLED(CODE_G66)
    { 66, gcode_G66 },
  #endif
  #if ENABLED(CODE_G67)
    { 67, gcode_G67 },
  #endif
  #if ENABLED(CODE_G68)
    { 68, gcode_G68 },
  #endif
  #if ENABLED(CODE_G69)
    { 69, gcode_G69 },
  #endif
  #if ENABLED(CODE_G70)
    { 70, gcode_G70 },
  #endif
  #if ENABLED(CODE_G71)
    { 71, gcode_G71 },
  #endif
  #if ENABLED(CODE_G72)
    { 72, gcode_G72 },
  #endif
  #if ENABLED(CODE_G73)
    { 73, gcode_G73 },
  #endif
  #if ENABLED(CODE_G74)
    { 74, gcode_G74 },
  #endif
  #if ENABLED(CODE_G75)
    { 75, gcode_G75 },
  #endif
  #if ENABLED(CODE_G76)
    { 76, gcode_G76 },
  #endif
  #if ENABLED(CODE_G77)
    { 77, gcode_G77 },
  #endif
  #if ENABLED(CODE_G78)
    { 78, gcode_G78 },
  #endif
  #if ENABLED(CODE_G79)
    { 79, gcode_G79 },
  #endif
  #if ENABLED(CODE_G80)
    { 80, gcode_G80 },
  #endif
  #if ENABLED(CODE_G81)
    { 81, gcode_G81 },
  #endif
  #if ENABLED(CODE_G82)
    { 82, gcode_G82 },
  #endif
  #if ENABLED(CODE_G83)
    { 83, gcode_G83 },
  #endif
  #if ENABLED(CODE_G84)
    { 84, gcode_G84 },
  #endif
  #if ENABLED(CODE_G85)
    { 85, gcode_G85 },
  #endif
  #if ENABLED(CODE_G86)
    { 86, gcode_G86 },
  #endif
  #if ENABLED(CODE_G87)
    { 87, gcode_G87 },
  #endif
  #if ENABLED(CODE_G88)
    { 88, gcode_G88 },
  #endif
  #if ENABLED(CODE_G89)
    { 89, gcode_G89 },
  #endif
  #if ENABLED(CODE_G90)
    { 90, gcode_G90 },
  #endif
  #if ENABLED(CODE_G91)
    { 91, gcode_G91 },
  #endif
  #if ENABLED(CODE_G92)
    { 92, gcode_G92 },
  #endif
  #if ENABLED(CODE_G93)
    { 93, gcode_G93 },
  #endif
  #if ENABLED(CODE_G94)
    { 94, gcode_G94 },
  #endif
  #if ENABLED(CODE_G95)
    { 95, gcode_G95 },
  #endif
  #if ENABLED(CODE_G96)
    { 96, gcode_G96 },
  #endif
  #if ENABLED(CODE_G97)
    { 97, gcode_G97 },
  #endif
  #if ENABLED(CODE_G98)
    { 98, gcode_G98 },
  #endif
  #if ENABLED(CODE_G99)
    { 99, gcode_G99 }
  #endif

};
