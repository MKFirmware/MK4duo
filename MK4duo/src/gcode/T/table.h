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
 * T/table.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#include "t_code.h"

#define T_CODE_TYPE uint8_t

typedef struct{
	T_CODE_TYPE code;
	void (* command) ();	
}Tcode_t;

static const Tcode_t TCode_Table [] = {
  #if ENABLED(T0)
    {0, gcode_T0},
  #endif
  #if ENABLED(T1)
    {1, gcode_T1},
  #endif
  #if ENABLED(T2)
    {2, gcode_T2},
  #endif
  #if ENABLED(T3)
    {3, gcode_T3},
  #endif
  #if ENABLED(T4)
    {4, gcode_T4},
  #endif
  #if ENABLED(T5)
    {5, gcode_T5},
  #endif
  #if ENABLED(T6)
    {6, gcode_T6},
  #endif
  #if ENABLED(T7)
    {7, gcode_T7},
  #endif
  #if ENABLED(T8)
    {8, gcode_T8},
  #endif
  #if ENABLED(T9)
    {9, gcode_T9},
  #endif
  #if ENABLED(T10)
    {10, gcode_T10},
  #endif
  #if ENABLED(T11)
    {11, gcode_T11},
  #endif
  #if ENABLED(T12)
    {12, gcode_T12},
  #endif
  #if ENABLED(T13)
    {13, gcode_T13},
  #endif
  #if ENABLED(T14)
    {14, gcode_T14},
  #endif
  #if ENABLED(T15)
    {15, gcode_T15},
  #endif
  #if ENABLED(T16)
    {16, gcode_T16},
  #endif
  #if ENABLED(T17)
    {17, gcode_T17},
  #endif
  #if ENABLED(T18)
    {18, gcode_T18},
  #endif
  #if ENABLED(T19)
    {19, gcode_T19},
  #endif
  #if ENABLED(T20)
    {20, gcode_T20},
  #endif
  #if ENABLED(T21)
    {21, gcode_T21},
  #endif
  #if ENABLED(T22)
    {22, gcode_T22},
  #endif
  #if ENABLED(T23)
    {23, gcode_T23},
  #endif
  #if ENABLED(T24)
    {24, gcode_T24},
  #endif
  #if ENABLED(T25)
    {25, gcode_T25},
  #endif
  #if ENABLED(T26)
    {26, gcode_T26},
  #endif
  #if ENABLED(T27)
    {27, gcode_T27},
  #endif
  #if ENABLED(T28)
    {28, gcode_T28},
  #endif
  #if ENABLED(T29)
    {29, gcode_T29},
  #endif
  #if ENABLED(T30)
    {30, gcode_T30},
  #endif
  #if ENABLED(T31)
    {31, gcode_T31},
  #endif
  #if ENABLED(T32)
    {32, gcode_T32},
  #endif
  #if ENABLED(T33)
    {33, gcode_T33},
  #endif
  #if ENABLED(T34)
    {34, gcode_T34},
  #endif
  #if ENABLED(T35)
    {35, gcode_T35},
  #endif
  #if ENABLED(T36)
    {36, gcode_T36},
  #endif
  #if ENABLED(T37)
    {37, gcode_T37},
  #endif
  #if ENABLED(T38)
    {38, gcode_T38},
  #endif
  #if ENABLED(T39)
    {39, gcode_T39},
  #endif
  #if ENABLED(T40)
    {40, gcode_T40},
  #endif
  #if ENABLED(T41)
    {41, gcode_T41},
  #endif
  #if ENABLED(T42)
    {42, gcode_T42},
  #endif
  #if ENABLED(T43)
    {43, gcode_T43},
  #endif
  #if ENABLED(T44)
    {44, gcode_T44},
  #endif
  #if ENABLED(T45)
    {45, gcode_T45},
  #endif
  #if ENABLED(T46)
    {46, gcode_T46},
  #endif
  #if ENABLED(T47)
    {47, gcode_T47},
  #endif
  #if ENABLED(T48)
    {48, gcode_T48},
  #endif
  #if ENABLED(T49)
    {49, gcode_T49},
  #endif
  #if ENABLED(T50)
    {50, gcode_T50},
  #endif
  #if ENABLED(T51)
    {51, gcode_T51},
  #endif
  #if ENABLED(T52)
    {52, gcode_T52},
  #endif
  #if ENABLED(T53)
    {53, gcode_T53},
  #endif
  #if ENABLED(T54)
    {54, gcode_T54},
  #endif
  #if ENABLED(T55)
    {55, gcode_T55},
  #endif
  #if ENABLED(T56)
    {56, gcode_T56},
  #endif
  #if ENABLED(T57)
    {57, gcode_T57},
  #endif
  #if ENABLED(T58)
    {58, gcode_T58},
  #endif
  #if ENABLED(T59)
    {59, gcode_T59},
  #endif
  #if ENABLED(T60)
    {60, gcode_T60},
  #endif
  #if ENABLED(T61)
    {61, gcode_T61},
  #endif
  #if ENABLED(T62)
    {62, gcode_T62},
  #endif
  #if ENABLED(T63)
    {63, gcode_T63},
  #endif
  #if ENABLED(T64)
    {64, gcode_T64},
  #endif
  #if ENABLED(T65)
    {65, gcode_T65},
  #endif
  #if ENABLED(T66)
    {66, gcode_T66},
  #endif
  #if ENABLED(T67)
    {67, gcode_T67},
  #endif
  #if ENABLED(T68)
    {68, gcode_T68},
  #endif
  #if ENABLED(T69)
    {69, gcode_T69},
  #endif
  #if ENABLED(T70)
    {70, gcode_T70},
  #endif
  #if ENABLED(T71)
    {71, gcode_T71},
  #endif
  #if ENABLED(T72)
    {72, gcode_T72},
  #endif
  #if ENABLED(T73)
    {73, gcode_T73},
  #endif
  #if ENABLED(T74)
    {74, gcode_T74},
  #endif
  #if ENABLED(T75)
    {75, gcode_T75},
  #endif
  #if ENABLED(T76)
    {76, gcode_T76},
  #endif
  #if ENABLED(T77)
    {77, gcode_T77},
  #endif
  #if ENABLED(T78)
    {78, gcode_T78},
  #endif
  #if ENABLED(T79)
    {79, gcode_T79},
  #endif
  #if ENABLED(T80)
    {80, gcode_T80},
  #endif
  #if ENABLED(T81)
    {81, gcode_T81},
  #endif
  #if ENABLED(T82)
    {82, gcode_T82},
  #endif
  #if ENABLED(T83)
    {83, gcode_T83},
  #endif
  #if ENABLED(T84)
    {84, gcode_T84},
  #endif
  #if ENABLED(T85)
    {85, gcode_T85},
  #endif
  #if ENABLED(T86)
    {86, gcode_T86},
  #endif
  #if ENABLED(T87)
    {87, gcode_T87},
  #endif
  #if ENABLED(T88)
    {88, gcode_T88},
  #endif
  #if ENABLED(T89)
    {89, gcode_T89},
  #endif
  #if ENABLED(T90)
    {90, gcode_T90},
  #endif
  #if ENABLED(T91)
    {91, gcode_T91},
  #endif
  #if ENABLED(T92)
    {92, gcode_T92},
  #endif
  #if ENABLED(T93)
    {93, gcode_T93},
  #endif
  #if ENABLED(T94)
    {94, gcode_T94},
  #endif
  #if ENABLED(T95)
    {95, gcode_T95},
  #endif
  #if ENABLED(T96)
    {96, gcode_T96},
  #endif
  #if ENABLED(T97)
    {97, gcode_T97},
  #endif
  #if ENABLED(T98)
    {98, gcode_T98},
  #endif
  #if ENABLED(T99)
    {99, gcode_T99},
  #endif
};
