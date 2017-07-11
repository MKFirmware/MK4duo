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
 * gcode.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#include "motion/g1.h"
#include "motion/g2.h"
#include "motion/g4.h"
#include "motion/g5.h"
#include "laser/g7.h"
#include "motion/g10.h"
#include "feature/g12.h"
#include "cnc/g17.h"
#include "units/g20.h"
#include "feature/g27.h"
#include "calibrate/g28.h"
#include "calibrate/g29.h"
#include "probe/g30.h"
#include "probe/g31.h"
#include "delta/g33.h"
#include "probe/g38.h"
#include "calibrate/g42.h"
#include "feature/g60.h"
#include "feature/g61.h"
#include "motion/g90.h"
#include "motion/g91.h"
#include "geometry/g92.h"

#include "m_code.h"
#include "t_code.h"

static const unsigned int GCode_Table [][2] = {
  #if ENABLED(G0)
    {0 , (unsigned int)gcode_G0},
  #endif
  #if ENABLED(G1)
    {1 , (unsigned int)gcode_G1},
  #endif
  #if ENABLED(G2)
    {2 , (unsigned int)gcode_G2},
  #endif
  #if ENABLED(G3)
    {3 , (unsigned int)gcode_G3},
  #endif
  #if ENABLED(G4)
    {4 , (unsigned int)gcode_G4},
  #endif
  #if ENABLED(G5)
    {5 , (unsigned int)gcode_G5},
  #endif
  #if ENABLED(G6)
    {6 , (unsigned int)gcode_G6},
  #endif
  #if ENABLED(G7)
    {7 , (unsigned int)gcode_G7},
  #endif
  #if ENABLED(G8)
    {8 , (unsigned int)gcode_G8},
  #endif
  #if ENABLED(G9)
    {9 , (unsigned int)gcode_G9},
  #endif
  #if ENABLED(G10)
    {10 , (unsigned int)gcode_G10},
  #endif
  #if ENABLED(G11)
    {11 , (unsigned int)gcode_G11},
  #endif
  #if ENABLED(G12)
    {12 , (unsigned int)gcode_G12},
  #endif
  #if ENABLED(G13)
    {13 , (unsigned int)gcode_G13},
  #endif
  #if ENABLED(G14)
    {14 , (unsigned int)gcode_G14},
  #endif
  #if ENABLED(G15)
    {15 , (unsigned int)gcode_G15},
  #endif
  #if ENABLED(G16)
    {16 , (unsigned int)gcode_G16},
  #endif
  #if ENABLED(G17)
    {17 , (unsigned int)gcode_G17},
  #endif
  #if ENABLED(G18)
    {18 , (unsigned int)gcode_G18},
  #endif
  #if ENABLED(G19)
    {19 , (unsigned int)gcode_G19},
  #endif
  #if ENABLED(G20)
    {20 , (unsigned int)gcode_G20},
  #endif
  #if ENABLED(G21)
    {21 , (unsigned int)gcode_G21},
  #endif
  #if ENABLED(G22)
    {22 , (unsigned int)gcode_G22},
  #endif
  #if ENABLED(G23)
    {23 , (unsigned int)gcode_G23},
  #endif
  #if ENABLED(G24)
    {24 , (unsigned int)gcode_G24},
  #endif
  #if ENABLED(G25)
    {25 , (unsigned int)gcode_G25},
  #endif
  #if ENABLED(G26)
    {26 , (unsigned int)gcode_G26},
  #endif
  #if ENABLED(G27)
    {27 , (unsigned int)gcode_G27},
  #endif
  #if ENABLED(G28)
    {28 , (unsigned int)gcode_G28},
  #endif
  #if ENABLED(G29)
    {29 , (unsigned int)gcode_G29},
  #endif
  #if ENABLED(G30)
    {30 , (unsigned int)gcode_G30},
  #endif
  #if ENABLED(G31)
    {31 , (unsigned int)gcode_G31},
  #endif
  #if ENABLED(G32)
    {32 , (unsigned int)gcode_G32},
  #endif
  #if ENABLED(G33)
    {33 , (unsigned int)gcode_G33},
  #endif
  #if ENABLED(G34)
    {34 , (unsigned int)gcode_G34},
  #endif
  #if ENABLED(G35)
    {35 , (unsigned int)gcode_G35},
  #endif
  #if ENABLED(G36)
    {36 , (unsigned int)gcode_G36},
  #endif
  #if ENABLED(G37)
    {37 , (unsigned int)gcode_G37},
  #endif
  #if ENABLED(G38)
    {38 , (unsigned int)gcode_G38},
  #endif
  #if ENABLED(G39)
    {39 , (unsigned int)gcode_G39},
  #endif
  #if ENABLED(G40)
    {40 , (unsigned int)gcode_G40},
  #endif
  #if ENABLED(G41)
    {41 , (unsigned int)gcode_G41},
  #endif
  #if ENABLED(G42)
    {42 , (unsigned int)gcode_G42},
  #endif
  #if ENABLED(G43)
    {43 , (unsigned int)gcode_G43},
  #endif
  #if ENABLED(G44)
    {44 , (unsigned int)gcode_G44},
  #endif
  #if ENABLED(G45)
    {45 , (unsigned int)gcode_G45},
  #endif
  #if ENABLED(G46)
    {46 , (unsigned int)gcode_G46},
  #endif
  #if ENABLED(G47)
    {47 , (unsigned int)gcode_G47},
  #endif
  #if ENABLED(G48)
    {48 , (unsigned int)gcode_G48},
  #endif
  #if ENABLED(G49)
    {49 , (unsigned int)gcode_G49},
  #endif
  #if ENABLED(G50)
    {50 , (unsigned int)gcode_G50},
  #endif
  #if ENABLED(G51)
    {51 , (unsigned int)gcode_G51},
  #endif
  #if ENABLED(G52)
    {52 , (unsigned int)gcode_G52},
  #endif
  #if ENABLED(G53)
    {53 , (unsigned int)gcode_G53},
  #endif
  #if ENABLED(G54)
    {54 , (unsigned int)gcode_G54},
  #endif
  #if ENABLED(G55)
    {55 , (unsigned int)gcode_G55},
  #endif
  #if ENABLED(G56)
    {56 , (unsigned int)gcode_G56},
  #endif
  #if ENABLED(G57)
    {57 , (unsigned int)gcode_G57},
  #endif
  #if ENABLED(G58)
    {58 , (unsigned int)gcode_G58},
  #endif
  #if ENABLED(G59)
    {59 , (unsigned int)gcode_G59},
  #endif
  #if ENABLED(G60)
    {60 , (unsigned int)gcode_G60},
  #endif
  #if ENABLED(G61)
    {61 , (unsigned int)gcode_G61},
  #endif
  #if ENABLED(G62)
    {62 , (unsigned int)gcode_G62},
  #endif
  #if ENABLED(G63)
    {63 , (unsigned int)gcode_G63},
  #endif
  #if ENABLED(G64)
    {64 , (unsigned int)gcode_G64},
  #endif
  #if ENABLED(G65)
    {65 , (unsigned int)gcode_G65},
  #endif
  #if ENABLED(G66)
    {66 , (unsigned int)gcode_G66},
  #endif
  #if ENABLED(G67)
    {67 , (unsigned int)gcode_G67},
  #endif
  #if ENABLED(G68)
    {68 , (unsigned int)gcode_G68},
  #endif
  #if ENABLED(G69)
    {69 , (unsigned int)gcode_G69},
  #endif
  #if ENABLED(G70)
    {70 , (unsigned int)gcode_G70},
  #endif
  #if ENABLED(G71)
    {71 , (unsigned int)gcode_G71},
  #endif
  #if ENABLED(G72)
    {72 , (unsigned int)gcode_G72},
  #endif
  #if ENABLED(G73)
    {73 , (unsigned int)gcode_G73},
  #endif
  #if ENABLED(G74)
    {74 , (unsigned int)gcode_G74},
  #endif
  #if ENABLED(G75)
    {75 , (unsigned int)gcode_G75},
  #endif
  #if ENABLED(G76)
    {76 , (unsigned int)gcode_G76},
  #endif
  #if ENABLED(G77)
    {77 , (unsigned int)gcode_G77},
  #endif
  #if ENABLED(G78)
    {78 , (unsigned int)gcode_G78},
  #endif
  #if ENABLED(G79)
    {79 , (unsigned int)gcode_G79},
  #endif
  #if ENABLED(G80)
    {80 , (unsigned int)gcode_G80},
  #endif
  #if ENABLED(G81)
    {81 , (unsigned int)gcode_G81},
  #endif
  #if ENABLED(G82)
    {82 , (unsigned int)gcode_G82},
  #endif
  #if ENABLED(G83)
    {83 , (unsigned int)gcode_G83},
  #endif
  #if ENABLED(G84)
    {84 , (unsigned int)gcode_G84},
  #endif
  #if ENABLED(G85)
    {85 , (unsigned int)gcode_G85},
  #endif
  #if ENABLED(G86)
    {86 , (unsigned int)gcode_G86},
  #endif
  #if ENABLED(G87)
    {87 , (unsigned int)gcode_G87},
  #endif
  #if ENABLED(G88)
    {88 , (unsigned int)gcode_G88},
  #endif
  #if ENABLED(G89)
    {89 , (unsigned int)gcode_G89},
  #endif
  #if ENABLED(G90)
    {90 , (unsigned int)gcode_G90},
  #endif
  #if ENABLED(G91)
    {91 , (unsigned int)gcode_G91},
  #endif
  #if ENABLED(G92)
    {92 , (unsigned int)gcode_G92},
  #endif
  #if ENABLED(G93)
    {93 , (unsigned int)gcode_G93},
  #endif
  #if ENABLED(G94)
    {94 , (unsigned int)gcode_G94},
  #endif
  #if ENABLED(G95)
    {95 , (unsigned int)gcode_G95},
  #endif
  #if ENABLED(G96)
    {96 , (unsigned int)gcode_G96},
  #endif
  #if ENABLED(G97)
    {97 , (unsigned int)gcode_G97},
  #endif
  #if ENABLED(G98)
    {98 , (unsigned int)gcode_G98},
  #endif
  #if ENABLED(G99)
    {99 , (unsigned int)gcode_G99},
  #endif

  {0xFFFF, 0xFFFF}

};
