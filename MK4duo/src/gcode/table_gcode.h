/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2017 Alberto Cotronei @MagoKimbra
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

/**
 * table_gcode.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#define G_CODE_TYPE uint8_t

typedef struct{
	G_CODE_TYPE code;
	void (* command) ();	
} Gcode_t;

constexpr Gcode_t GCode_Table[] = {
  #if ENABLED(G0)
    {0 , gcode_G0},
  #endif
  #if ENABLED(G1)
    {1 , gcode_G1},
  #endif
  #if ENABLED(G2)
    {2 , gcode_G2},
  #endif
  #if ENABLED(G3)
    {3 , gcode_G3},
  #endif
  #if ENABLED(G4)
    {4 , gcode_G4},
  #endif
  #if ENABLED(G5)
    {5 , gcode_G5},
  #endif
  #if ENABLED(G6)
    {6 , gcode_G6},
  #endif
  #if ENABLED(G7)
    {7 , gcode_G7},
  #endif
  #if ENABLED(G8)
    {8 , gcode_G8},
  #endif
  #if ENABLED(G9)
    {9 , gcode_G9},
  #endif
  #if ENABLED(G10)
    {10 , gcode_G10},
  #endif
  #if ENABLED(G11)
    {11 , gcode_G11},
  #endif
  #if ENABLED(G12)
    {12 , gcode_G12},
  #endif
  #if ENABLED(G13)
    {13 , gcode_G13},
  #endif
  #if ENABLED(G14)
    {14 , gcode_G14},
  #endif
  #if ENABLED(G15)
    {15 , gcode_G15},
  #endif
  #if ENABLED(G16)
    {16 , gcode_G16},
  #endif
  #if ENABLED(G17)
    {17 , gcode_G17},
  #endif
  #if ENABLED(G18)
    {18 , gcode_G18},
  #endif
  #if ENABLED(G19)
    {19 , gcode_G19},
  #endif
  #if ENABLED(G20)
    {20 , gcode_G20},
  #endif
  #if ENABLED(G21)
    {21 , gcode_G21},
  #endif
  #if ENABLED(G22)
    {22 , gcode_G22},
  #endif
  #if ENABLED(G23)
    {23 , gcode_G23},
  #endif
  #if ENABLED(G24)
    {24 , gcode_G24},
  #endif
  #if ENABLED(G25)
    {25 , gcode_G25},
  #endif
  #if ENABLED(G26)
    {26 , gcode_G26},
  #endif
  #if ENABLED(G27)
    {27 , gcode_G27},
  #endif
  #if ENABLED(G28)
    {28 , gcode_G28},
  #endif
  #if ENABLED(G29)
    {29 , gcode_G29},
  #endif
  #if ENABLED(G30)
    {30 , gcode_G30},
  #endif
  #if ENABLED(G31)
    {31 , gcode_G31},
  #endif
  #if ENABLED(G32)
    {32 , gcode_G32},
  #endif
  #if ENABLED(G33)
    {33 , gcode_G33},
  #endif
  #if ENABLED(G34)
    {34 , gcode_G34},
  #endif
  #if ENABLED(G35)
    {35 , gcode_G35},
  #endif
  #if ENABLED(G36)
    {36 , gcode_G36},
  #endif
  #if ENABLED(G37)
    {37 , gcode_G37},
  #endif
  #if ENABLED(G38)
    {38 , gcode_G38},
  #endif
  #if ENABLED(G39)
    {39 , gcode_G39},
  #endif
  #if ENABLED(G40)
    {40 , gcode_G40},
  #endif
  #if ENABLED(G41)
    {41 , gcode_G41},
  #endif
  #if ENABLED(G42)
    {42 , gcode_G42},
  #endif
  #if ENABLED(G43)
    {43 , gcode_G43},
  #endif
  #if ENABLED(G44)
    {44 , gcode_G44},
  #endif
  #if ENABLED(G45)
    {45 , gcode_G45},
  #endif
  #if ENABLED(G46)
    {46 , gcode_G46},
  #endif
  #if ENABLED(G47)
    {47 , gcode_G47},
  #endif
  #if ENABLED(G48)
    {48 , gcode_G48},
  #endif
  #if ENABLED(G49)
    {49 , gcode_G49},
  #endif
  #if ENABLED(G50)
    {50 , gcode_G50},
  #endif
  #if ENABLED(G51)
    {51 , gcode_G51},
  #endif
  #if ENABLED(G52)
    {52 , gcode_G52},
  #endif
  #if ENABLED(G53)
    {53 , gcode_G53},
  #endif
  #if ENABLED(G54)
    {54 , gcode_G54},
  #endif
  #if ENABLED(G55)
    {55 , gcode_G55},
  #endif
  #if ENABLED(G56)
    {56 , gcode_G56},
  #endif
  #if ENABLED(G57)
    {57 , gcode_G57},
  #endif
  #if ENABLED(G58)
    {58 , gcode_G58},
  #endif
  #if ENABLED(G59)
    {59 , gcode_G59},
  #endif
  #if ENABLED(G60)
    {60 , gcode_G60},
  #endif
  #if ENABLED(G61)
    {61 , gcode_G61},
  #endif
  #if ENABLED(G62)
    {62 , gcode_G62},
  #endif
  #if ENABLED(G63)
    {63 , gcode_G63},
  #endif
  #if ENABLED(G64)
    {64 , gcode_G64},
  #endif
  #if ENABLED(G65)
    {65 , gcode_G65},
  #endif
  #if ENABLED(G66)
    {66 , gcode_G66},
  #endif
  #if ENABLED(G67)
    {67 , gcode_G67},
  #endif
  #if ENABLED(G68)
    {68 , gcode_G68},
  #endif
  #if ENABLED(G69)
    {69 , gcode_G69},
  #endif
  #if ENABLED(G70)
    {70 , gcode_G70},
  #endif
  #if ENABLED(G71)
    {71 , gcode_G71},
  #endif
  #if ENABLED(G72)
    {72 , gcode_G72},
  #endif
  #if ENABLED(G73)
    {73 , gcode_G73},
  #endif
  #if ENABLED(G74)
    {74 , gcode_G74},
  #endif
  #if ENABLED(G75)
    {75 , gcode_G75},
  #endif
  #if ENABLED(G76)
    {76 , gcode_G76},
  #endif
  #if ENABLED(G77)
    {77 , gcode_G77},
  #endif
  #if ENABLED(G78)
    {78 , gcode_G78},
  #endif
  #if ENABLED(G79)
    {79 , gcode_G79},
  #endif
  #if ENABLED(G80)
    {80 , gcode_G80},
  #endif
  #if ENABLED(G81)
    {81 , gcode_G81},
  #endif
  #if ENABLED(G82)
    {82 , gcode_G82},
  #endif
  #if ENABLED(G83)
    {83 , gcode_G83},
  #endif
  #if ENABLED(G84)
    {84 , gcode_G84},
  #endif
  #if ENABLED(G85)
    {85 , gcode_G85},
  #endif
  #if ENABLED(G86)
    {86 , gcode_G86},
  #endif
  #if ENABLED(G87)
    {87 , gcode_G87},
  #endif
  #if ENABLED(G88)
    {88 , gcode_G88},
  #endif
  #if ENABLED(G89)
    {89 , gcode_G89},
  #endif
  #if ENABLED(G90)
    {90 , gcode_G90},
  #endif
  #if ENABLED(G91)
    {91 , gcode_G91},
  #endif
  #if ENABLED(G92)
    {92 , gcode_G92},
  #endif
  #if ENABLED(G93)
    {93 , gcode_G93},
  #endif
  #if ENABLED(G94)
    {94 , gcode_G94},
  #endif
  #if ENABLED(G95)
    {95 , gcode_G95},
  #endif
  #if ENABLED(G96)
    {96 , gcode_G96},
  #endif
  #if ENABLED(G97)
    {97 , gcode_G97},
  #endif
  #if ENABLED(G98)
    {98 , gcode_G98},
  #endif
  #if ENABLED(G99)
    {99 , gcode_G99}
  #endif
};
