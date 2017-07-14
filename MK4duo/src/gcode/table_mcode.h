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
 * table_mcode.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#define M_CODE_TYPE uint16_t

typedef struct{
	M_CODE_TYPE code;
	void (* command) ();	
} Mcode_t;

constexpr Mcode_t MCode_Table [] = {
	#if ENABLED(CODE_M0)
		{0, gcode_M0_M1},
	#endif
	#if ENABLED(CODE_M1)
		{1, gcode_M0_M1},
	#endif
	#if ENABLED(CODE_M2)
		{2, gcode_M2},
	#endif
	#if ENABLED(CODE_M3)
		{3, gcode_M3},
	#endif
	#if ENABLED(CODE_M4)
		{4, gcode_M4},
	#endif
	#if ENABLED(CODE_M5)
		{5, gcode_M5},
	#endif
	#if ENABLED(CODE_M6)
		{6, gcode_M6},
	#endif
	#if ENABLED(CODE_M7)
		{7, gcode_M7},
	#endif
	#if ENABLED(CODE_M8)
		{8, gcode_M8},
	#endif
	#if ENABLED(CODE_M9)
		{9, gcode_M9},
	#endif
	#if ENABLED(CODE_M10)
		{10, gcode_M10},
	#endif
	#if ENABLED(CODE_M11)
		{11, gcode_M11},
	#endif
	#if ENABLED(CODE_M12)
		{12, gcode_M12},
	#endif
	#if ENABLED(CODE_M13)
		{13, gcode_M13},
	#endif
	#if ENABLED(CODE_M14)
		{14, gcode_M14},
	#endif
	#if ENABLED(CODE_M15)
		{15, gcode_M15},
	#endif
	#if ENABLED(CODE_M16)
		{16, gcode_M16},
	#endif
	#if ENABLED(CODE_M17)
		{17, gcode_M17},
	#endif
	#if ENABLED(CODE_M18)
		{18, gcode_M18_M84},
	#endif
	#if ENABLED(CODE_M19)
		{19, gcode_M19},
	#endif
	#if ENABLED(CODE_M20)
		{20, gcode_M20},
	#endif
	#if ENABLED(CODE_M21)
		{21, gcode_M21},
	#endif
	#if ENABLED(CODE_M22)
		{22, gcode_M22},
	#endif
	#if ENABLED(CODE_M23)
		{23, gcode_M23},
	#endif
	#if ENABLED(CODE_M24)
		{24, gcode_M24},
	#endif
	#if ENABLED(CODE_M25)
		{25, gcode_M25},
	#endif
	#if ENABLED(CODE_M26)
		{26, gcode_M26},
	#endif
	#if ENABLED(CODE_M27)
		{27, gcode_M27},
	#endif
	#if ENABLED(CODE_M28)
		{28, gcode_M28},
	#endif
	#if ENABLED(CODE_M29)
		{29, gcode_M29},
	#endif
	#if ENABLED(CODE_M30)
		{30, gcode_M30},
	#endif
	#if ENABLED(CODE_M31)
		{31, gcode_M31},
	#endif
	#if ENABLED(CODE_M32)
		{32, gcode_M32},
	#endif
	#if ENABLED(CODE_M33)
		{33, gcode_M33},
	#endif
	#if ENABLED(CODE_M34)
		{34, gcode_M34},
	#endif
	#if ENABLED(CODE_M35)
		{35, gcode_M35},
	#endif
	#if ENABLED(CODE_M36)
		{36, gcode_M36},
	#endif
	#if ENABLED(CODE_M37)
		{37, gcode_M37},
	#endif
	#if ENABLED(CODE_M38)
		{38, gcode_M38},
	#endif
	#if ENABLED(CODE_M39)
		{39, gcode_M39},
	#endif
	#if ENABLED(CODE_M40)
		{40, gcode_M40},
	#endif
	#if ENABLED(CODE_M41)
		{41, gcode_M41},
	#endif
	#if ENABLED(CODE_M42)
		{42, gcode_M42},
	#endif
	#if ENABLED(CODE_M43)
		{43, gcode_M43},
	#endif
	#if ENABLED(CODE_M44)
		{44, gcode_M44},
	#endif
	#if ENABLED(CODE_M45)
		{45, gcode_M45},
	#endif
	#if ENABLED(CODE_M46)
		{46, gcode_M46},
	#endif
	#if ENABLED(CODE_M47)
		{47, gcode_M47},
	#endif
	#if ENABLED(CODE_M48)
		{48, gcode_M48},
	#endif
	#if ENABLED(CODE_M49)
		{49, gcode_M49},
	#endif
	#if ENABLED(CODE_M50)
		{50, gcode_M50},
	#endif
	#if ENABLED(CODE_M51)
		{51, gcode_M51},
	#endif
	#if ENABLED(CODE_M52)
		{52, gcode_M52},
	#endif
	#if ENABLED(CODE_M53)
		{53, gcode_M53},
	#endif
	#if ENABLED(CODE_M54)
		{54, gcode_M54},
	#endif
	#if ENABLED(CODE_M55)
		{55, gcode_M55},
	#endif
	#if ENABLED(CODE_M56)
		{56, gcode_M56},
	#endif
	#if ENABLED(CODE_M57)
		{57, gcode_M57},
	#endif
	#if ENABLED(CODE_M58)
		{58, gcode_M58},
	#endif
	#if ENABLED(CODE_M59)
		{59, gcode_M59},
	#endif
	#if ENABLED(CODE_M60)
		{60, gcode_M60},
	#endif
	#if ENABLED(CODE_M61)
		{61, gcode_M61},
	#endif
	#if ENABLED(CODE_M62)
		{62, gcode_M62},
	#endif
	#if ENABLED(CODE_M63)
		{63, gcode_M63},
	#endif
	#if ENABLED(CODE_M64)
		{64, gcode_M64},
	#endif
	#if ENABLED(CODE_M65)
		{65, gcode_M65},
	#endif
	#if ENABLED(CODE_M66)
		{66, gcode_M66},
	#endif
	#if ENABLED(CODE_M67)
		{67, gcode_M67},
	#endif
	#if ENABLED(CODE_M68)
		{68, gcode_M68},
	#endif
	#if ENABLED(CODE_M69)
		{69, gcode_M69},
	#endif
	#if ENABLED(CODE_M70)
		{70, gcode_M70},
	#endif
	#if ENABLED(CODE_M71)
		{71, gcode_M71},
	#endif
	#if ENABLED(CODE_M72)
		{72, gcode_M72},
	#endif
	#if ENABLED(CODE_M73)
		{73, gcode_M73},
	#endif
	#if ENABLED(CODE_M74)
		{74, gcode_M74},
	#endif
	#if ENABLED(CODE_M75)
		{75, gcode_M75},
	#endif
	#if ENABLED(CODE_M76)
		{76, gcode_M76},
	#endif
	#if ENABLED(CODE_M77)
		{77, gcode_M77},
	#endif
	#if ENABLED(CODE_M78)
		{78, gcode_M78},
	#endif
	#if ENABLED(CODE_M79)
		{79, gcode_M79},
	#endif
	#if ENABLED(CODE_M80)
		{80, gcode_M80},
	#endif
	#if ENABLED(CODE_M81)
		{81, gcode_M81},
	#endif
	#if ENABLED(CODE_M82)
		{82, gcode_M82},
	#endif
	#if ENABLED(CODE_M83)
		{83, gcode_M83},
	#endif
	#if ENABLED(CODE_M84)
		{84, gcode_M18_M84},
	#endif
	#if ENABLED(CODE_M85)
		{85, gcode_M85},
	#endif
	#if ENABLED(CODE_M86)
		{86, gcode_M86},
	#endif
	#if ENABLED(CODE_M87)
		{87, gcode_M87},
	#endif
	#if ENABLED(CODE_M88)
		{88, gcode_M88},
	#endif
	#if ENABLED(CODE_M89)
		{89, gcode_M89},
	#endif
	#if ENABLED(CODE_M90)
		{90, gcode_M90},
	#endif
	#if ENABLED(CODE_M91)
		{91, gcode_M91},
	#endif
	#if ENABLED(CODE_M92)
		{92, gcode_M92},
	#endif
	#if ENABLED(CODE_M93)
		{93, gcode_M93},
	#endif
	#if ENABLED(CODE_M94)
		{94, gcode_M94},
	#endif
	#if ENABLED(CODE_M95)
		{95, gcode_M95},
	#endif
	#if ENABLED(CODE_M96)
		{96, gcode_M96},
	#endif
	#if ENABLED(CODE_M97)
		{97, gcode_M97},
	#endif
	#if ENABLED(CODE_M98)
		{98, gcode_M98},
	#endif
	#if ENABLED(CODE_M99)
		{99, gcode_M99},
	#endif
	#if ENABLED(CODE_M100)
		{100, gcode_M100},
	#endif
	#if ENABLED(CODE_M101)
		{101, gcode_M101},
	#endif
	#if ENABLED(CODE_M102)
		{102, gcode_M102},
	#endif
	#if ENABLED(CODE_M103)
		{103, gcode_M103},
	#endif
	#if ENABLED(CODE_M104)
		{104, gcode_M104},
	#endif
	#if ENABLED(CODE_M105)
		{105, gcode_M105},
	#endif
	#if ENABLED(CODE_M106)
		{106, gcode_M106},
	#endif
	#if ENABLED(CODE_M107)
		{107, gcode_M107},
	#endif
	#if ENABLED(CODE_M108)
		{108, gcode_M108},
	#endif
	#if ENABLED(CODE_M109)
		{109, gcode_M109},
	#endif
	#if ENABLED(CODE_M110)
		{110, gcode_M110},
	#endif
	#if ENABLED(CODE_M111)
		{111, gcode_M111},
	#endif
	#if ENABLED(CODE_M112)
		{112, gcode_M112},
	#endif
	#if ENABLED(CODE_M113)
		{113, gcode_M113},
	#endif
	#if ENABLED(CODE_M114)
		{114, gcode_M114},
	#endif
	#if ENABLED(CODE_M115)
		{115, gcode_M115},
	#endif
	#if ENABLED(CODE_M116)
		{116, gcode_M116},
	#endif
	#if ENABLED(CODE_M117)
		{117, gcode_M117},
	#endif
	#if ENABLED(CODE_M118)
		{118, gcode_M118},
	#endif
	#if ENABLED(CODE_M119)
		{119, gcode_M119},
	#endif
	#if ENABLED(CODE_M120)
		{120, gcode_M120},
	#endif
	#if ENABLED(CODE_M121)
		{121, gcode_M121},
	#endif
	#if ENABLED(CODE_M122)
		{122, gcode_M122},
	#endif
	#if ENABLED(CODE_M123)
		{123, gcode_M123},
	#endif
	#if ENABLED(CODE_M124)
		{124, gcode_M124},
	#endif
	#if ENABLED(CODE_M125)
		{125, gcode_M125},
	#endif
	#if ENABLED(CODE_M126)
		{126, gcode_M126},
	#endif
	#if ENABLED(CODE_M127)
		{127, gcode_M127},
	#endif
	#if ENABLED(CODE_M128)
		{128, gcode_M128},
	#endif
	#if ENABLED(CODE_M129)
		{129, gcode_M129},
	#endif
	#if ENABLED(CODE_M130)
		{130, gcode_M130},
	#endif
	#if ENABLED(CODE_M131)
		{131, gcode_M131},
	#endif
	#if ENABLED(CODE_M132)
		{132, gcode_M132},
	#endif
	#if ENABLED(CODE_M133)
		{133, gcode_M133},
	#endif
	#if ENABLED(CODE_M134)
		{134, gcode_M134},
	#endif
	#if ENABLED(CODE_M135)
		{135, gcode_M135},
	#endif
	#if ENABLED(CODE_M136)
		{136, gcode_M136},
	#endif
	#if ENABLED(CODE_M137)
		{137, gcode_M137},
	#endif
	#if ENABLED(CODE_M138)
		{138, gcode_M138},
	#endif
	#if ENABLED(CODE_M139)
		{139, gcode_M139},
	#endif
	#if ENABLED(CODE_M140)
		{140, gcode_M140},
	#endif
	#if ENABLED(CODE_M141)
		{141, gcode_M141},
	#endif
	#if ENABLED(CODE_M142)
		{142, gcode_M142},
	#endif
	#if ENABLED(CODE_M143)
		{143, gcode_M143},
	#endif
	#if ENABLED(CODE_M144)
		{144, gcode_M144},
	#endif
	#if ENABLED(CODE_M145)
		{145, gcode_M145},
	#endif
	#if ENABLED(CODE_M146)
		{146, gcode_M146},
	#endif
	#if ENABLED(CODE_M147)
		{147, gcode_M147},
	#endif
	#if ENABLED(CODE_M148)
		{148, gcode_M148},
	#endif
	#if ENABLED(CODE_M149)
		{149, gcode_M149},
	#endif
	#if ENABLED(CODE_M150)
		{150, gcode_M150},
	#endif
	#if ENABLED(CODE_M151)
		{151, gcode_M151},
	#endif
	#if ENABLED(CODE_M152)
		{152, gcode_M152},
	#endif
	#if ENABLED(CODE_M153)
		{153, gcode_M153},
	#endif
	#if ENABLED(CODE_M154)
		{154, gcode_M154},
	#endif
	#if ENABLED(CODE_M155)
		{155, gcode_M155},
	#endif
	#if ENABLED(CODE_M156)
		{156, gcode_M156},
	#endif
	#if ENABLED(CODE_M157)
		{157, gcode_M157},
	#endif
	#if ENABLED(CODE_M158)
		{158, gcode_M158},
	#endif
	#if ENABLED(CODE_M159)
		{159, gcode_M159},
	#endif
	#if ENABLED(CODE_M160)
		{160, gcode_M160},
	#endif
	#if ENABLED(CODE_M161)
		{161, gcode_M161},
	#endif
	#if ENABLED(CODE_M162)
		{162, gcode_M162},
	#endif
	#if ENABLED(CODE_M163)
		{163, gcode_M163},
	#endif
	#if ENABLED(CODE_M164)
		{164, gcode_M164},
	#endif
	#if ENABLED(CODE_M165)
		{165, gcode_M165},
	#endif
	#if ENABLED(CODE_M166)
		{166, gcode_M166},
	#endif
	#if ENABLED(CODE_M167)
		{167, gcode_M167},
	#endif
	#if ENABLED(CODE_M168)
		{168, gcode_M168},
	#endif
	#if ENABLED(CODE_M169)
		{169, gcode_M169},
	#endif
	#if ENABLED(CODE_M170)
		{170, gcode_M170},
	#endif
	#if ENABLED(CODE_M171)
		{171, gcode_M171},
	#endif
	#if ENABLED(CODE_M172)
		{172, gcode_M172},
	#endif
	#if ENABLED(CODE_M173)
		{173, gcode_M173},
	#endif
	#if ENABLED(CODE_M174)
		{174, gcode_M174},
	#endif
	#if ENABLED(CODE_M175)
		{175, gcode_M175},
	#endif
	#if ENABLED(CODE_M176)
		{176, gcode_M176},
	#endif
	#if ENABLED(CODE_M177)
		{177, gcode_M177},
	#endif
	#if ENABLED(CODE_M178)
		{178, gcode_M178},
	#endif
	#if ENABLED(CODE_M179)
		{179, gcode_M179},
	#endif
	#if ENABLED(CODE_M180)
		{180, gcode_M180},
	#endif
	#if ENABLED(CODE_M181)
		{181, gcode_M181},
	#endif
	#if ENABLED(CODE_M182)
		{182, gcode_M182},
	#endif
	#if ENABLED(CODE_M183)
		{183, gcode_M183},
	#endif
	#if ENABLED(CODE_M184)
		{184, gcode_M184},
	#endif
	#if ENABLED(CODE_M185)
		{185, gcode_M185},
	#endif
	#if ENABLED(CODE_M186)
		{186, gcode_M186},
	#endif
	#if ENABLED(CODE_M187)
		{187, gcode_M187},
	#endif
	#if ENABLED(CODE_M188)
		{188, gcode_M188},
	#endif
	#if ENABLED(CODE_M189)
		{189, gcode_M189},
	#endif
	#if ENABLED(CODE_M190)
		{190, gcode_M190},
	#endif
	#if ENABLED(CODE_M191)
		{191, gcode_M191},
	#endif
	#if ENABLED(CODE_M192)
		{192, gcode_M192},
	#endif
	#if ENABLED(CODE_M193)
		{193, gcode_M193},
	#endif
	#if ENABLED(CODE_M194)
		{194, gcode_M194},
	#endif
	#if ENABLED(CODE_M195)
		{195, gcode_M195},
	#endif
	#if ENABLED(CODE_M196)
		{196, gcode_M196},
	#endif
	#if ENABLED(CODE_M197)
		{197, gcode_M197},
	#endif
	#if ENABLED(CODE_M198)
		{198, gcode_M198},
	#endif
	#if ENABLED(CODE_M199)
		{199, gcode_M199},
	#endif
	#if ENABLED(CODE_M200)
		{200, gcode_M200},
	#endif
	#if ENABLED(CODE_M201)
		{201, gcode_M201},
	#endif
	#if ENABLED(CODE_M202)
		{202, gcode_M202},
	#endif
	#if ENABLED(CODE_M203)
		{203, gcode_M203},
	#endif
	#if ENABLED(CODE_M204)
		{204, gcode_M204},
	#endif
	#if ENABLED(CODE_M205)
		{205, gcode_M205},
	#endif
	#if ENABLED(CODE_M206)
		{206, gcode_M206},
	#endif
	#if ENABLED(CODE_M207)
		{207, gcode_M207},
	#endif
	#if ENABLED(CODE_M208)
		{208, gcode_M208},
	#endif
	#if ENABLED(CODE_M209)
		{209, gcode_M209},
	#endif
	#if ENABLED(CODE_M210)
		{210, gcode_M210},
	#endif
	#if ENABLED(CODE_M211)
		{211, gcode_M211},
	#endif
	#if ENABLED(CODE_M212)
		{212, gcode_M212},
	#endif
	#if ENABLED(CODE_M213)
		{213, gcode_M213},
	#endif
	#if ENABLED(CODE_M214)
		{214, gcode_M214},
	#endif
	#if ENABLED(CODE_M215)
		{215, gcode_M215},
	#endif
	#if ENABLED(CODE_M216)
		{216, gcode_M216},
	#endif
	#if ENABLED(CODE_M217)
		{217, gcode_M217},
	#endif
	#if ENABLED(CODE_M218)
		{218, gcode_M218},
	#endif
	#if ENABLED(CODE_M219)
		{219, gcode_M219},
	#endif
	#if ENABLED(CODE_M220)
		{220, gcode_M220},
	#endif
	#if ENABLED(CODE_M221)
		{221, gcode_M221},
	#endif
	#if ENABLED(CODE_M222)
		{222, gcode_M222},
	#endif
	#if ENABLED(CODE_M223)
		{223, gcode_M223},
	#endif
	#if ENABLED(CODE_M224)
		{224, gcode_M224},
	#endif
	#if ENABLED(CODE_M225)
		{225, gcode_M225},
	#endif
	#if ENABLED(CODE_M226)
		{226, gcode_M226},
	#endif
	#if ENABLED(CODE_M227)
		{227, gcode_M227},
	#endif
	#if ENABLED(CODE_M228)
		{228, gcode_M228},
	#endif
	#if ENABLED(CODE_M229)
		{229, gcode_M229},
	#endif
	#if ENABLED(CODE_M230)
		{230, gcode_M230},
	#endif
	#if ENABLED(CODE_M231)
		{231, gcode_M231},
	#endif
	#if ENABLED(CODE_M232)
		{232, gcode_M232},
	#endif
	#if ENABLED(CODE_M233)
		{233, gcode_M233},
	#endif
	#if ENABLED(CODE_M234)
		{234, gcode_M234},
	#endif
	#if ENABLED(CODE_M235)
		{235, gcode_M235},
	#endif
	#if ENABLED(CODE_M236)
		{236, gcode_M236},
	#endif
	#if ENABLED(CODE_M237)
		{237, gcode_M237},
	#endif
	#if ENABLED(CODE_M238)
		{238, gcode_M238},
	#endif
	#if ENABLED(CODE_M239)
		{239, gcode_M239},
	#endif
	#if ENABLED(CODE_M240)
		{240, gcode_M240},
	#endif
	#if ENABLED(CODE_M241)
		{241, gcode_M241},
	#endif
	#if ENABLED(CODE_M242)
		{242, gcode_M242},
	#endif
	#if ENABLED(CODE_M243)
		{243, gcode_M243},
	#endif
	#if ENABLED(CODE_M244)
		{244, gcode_M244},
	#endif
	#if ENABLED(CODE_M245)
		{245, gcode_M245},
	#endif
	#if ENABLED(CODE_M246)
		{246, gcode_M246},
	#endif
	#if ENABLED(CODE_M247)
		{247, gcode_M247},
	#endif
	#if ENABLED(CODE_M248)
		{248, gcode_M248},
	#endif
	#if ENABLED(CODE_M249)
		{249, gcode_M249},
	#endif
	#if ENABLED(CODE_M250)
		{250, gcode_M250},
	#endif
	#if ENABLED(CODE_M251)
		{251, gcode_M251},
	#endif
	#if ENABLED(CODE_M252)
		{252, gcode_M252},
	#endif
	#if ENABLED(CODE_M253)
		{253, gcode_M253},
	#endif
	#if ENABLED(CODE_M254)
		{254, gcode_M254},
	#endif
	#if ENABLED(CODE_M255)
		{255, gcode_M255},
	#endif
	#if ENABLED(CODE_M256)
		{256, gcode_M256},
	#endif
	#if ENABLED(CODE_M257)
		{257, gcode_M257},
	#endif
	#if ENABLED(CODE_M258)
		{258, gcode_M258},
	#endif
	#if ENABLED(CODE_M259)
		{259, gcode_M259},
	#endif
	#if ENABLED(CODE_M260)
		{260, gcode_M260},
	#endif
	#if ENABLED(CODE_M261)
		{261, gcode_M261},
	#endif
	#if ENABLED(CODE_M262)
		{262, gcode_M262},
	#endif
	#if ENABLED(CODE_M263)
		{263, gcode_M263},
	#endif
	#if ENABLED(CODE_M264)
		{264, gcode_M264},
	#endif
	#if ENABLED(CODE_M265)
		{265, gcode_M265},
	#endif
	#if ENABLED(CODE_M266)
		{266, gcode_M266},
	#endif
	#if ENABLED(CODE_M267)
		{267, gcode_M267},
	#endif
	#if ENABLED(CODE_M268)
		{268, gcode_M268},
	#endif
	#if ENABLED(CODE_M269)
		{269, gcode_M269},
	#endif
	#if ENABLED(CODE_M270)
		{270, gcode_M270},
	#endif
	#if ENABLED(CODE_M271)
		{271, gcode_M271},
	#endif
	#if ENABLED(CODE_M272)
		{272, gcode_M272},
	#endif
	#if ENABLED(CODE_M273)
		{273, gcode_M273},
	#endif
	#if ENABLED(CODE_M274)
		{274, gcode_M274},
	#endif
	#if ENABLED(CODE_M275)
		{275, gcode_M275},
	#endif
	#if ENABLED(CODE_M276)
		{276, gcode_M276},
	#endif
	#if ENABLED(CODE_M277)
		{277, gcode_M277},
	#endif
	#if ENABLED(CODE_M278)
		{278, gcode_M278},
	#endif
	#if ENABLED(CODE_M279)
		{279, gcode_M279},
	#endif
	#if ENABLED(CODE_M280)
		{280, gcode_M280},
	#endif
	#if ENABLED(CODE_M281)
		{281, gcode_M281},
	#endif
	#if ENABLED(CODE_M282)
		{282, gcode_M282},
	#endif
	#if ENABLED(CODE_M283)
		{283, gcode_M283},
	#endif
	#if ENABLED(CODE_M284)
		{284, gcode_M284},
	#endif
	#if ENABLED(CODE_M285)
		{285, gcode_M285},
	#endif
	#if ENABLED(CODE_M286)
		{286, gcode_M286},
	#endif
	#if ENABLED(CODE_M287)
		{287, gcode_M287},
	#endif
	#if ENABLED(CODE_M288)
		{288, gcode_M288},
	#endif
	#if ENABLED(CODE_M289)
		{289, gcode_M289},
	#endif
	#if ENABLED(CODE_M290)
		{290, gcode_M290},
	#endif
	#if ENABLED(CODE_M291)
		{291, gcode_M291},
	#endif
	#if ENABLED(CODE_M292)
		{292, gcode_M292},
	#endif
	#if ENABLED(CODE_M293)
		{293, gcode_M293},
	#endif
	#if ENABLED(CODE_M294)
		{294, gcode_M294},
	#endif
	#if ENABLED(CODE_M295)
		{295, gcode_M295},
	#endif
	#if ENABLED(CODE_M296)
		{296, gcode_M296},
	#endif
	#if ENABLED(CODE_M297)
		{297, gcode_M297},
	#endif
	#if ENABLED(CODE_M298)
		{298, gcode_M298},
	#endif
	#if ENABLED(CODE_M299)
		{299, gcode_M299},
	#endif
	#if ENABLED(CODE_M300)
		{300, gcode_M300},
	#endif
	#if ENABLED(CODE_M301)
		{301, gcode_M301},
	#endif
	#if ENABLED(CODE_M302)
		{302, gcode_M302},
	#endif
	#if ENABLED(CODE_M303)
		{303, gcode_M303},
	#endif
	#if ENABLED(CODE_M304)
		{304, gcode_M304},
	#endif
	#if ENABLED(CODE_M305)
		{305, gcode_M305},
	#endif
	#if ENABLED(CODE_M306)
		{306, gcode_M306},
	#endif
	#if ENABLED(CODE_M307)
		{307, gcode_M307},
	#endif
	#if ENABLED(CODE_M308)
		{308, gcode_M308},
	#endif
	#if ENABLED(CODE_M309)
		{309, gcode_M309},
	#endif
	#if ENABLED(CODE_M310)
		{310, gcode_M310},
	#endif
	#if ENABLED(CODE_M311)
		{311, gcode_M311},
	#endif
	#if ENABLED(CODE_M312)
		{312, gcode_M312},
	#endif
	#if ENABLED(CODE_M313)
		{313, gcode_M313},
	#endif
	#if ENABLED(CODE_M314)
		{314, gcode_M314},
	#endif
	#if ENABLED(CODE_M315)
		{315, gcode_M315},
	#endif
	#if ENABLED(CODE_M316)
		{316, gcode_M316},
	#endif
	#if ENABLED(CODE_M317)
		{317, gcode_M317},
	#endif
	#if ENABLED(CODE_M318)
		{318, gcode_M318},
	#endif
	#if ENABLED(CODE_M319)
		{319, gcode_M319},
	#endif
	#if ENABLED(CODE_M320)
		{320, gcode_M320},
	#endif
	#if ENABLED(CODE_M321)
		{321, gcode_M321},
	#endif
	#if ENABLED(CODE_M322)
		{322, gcode_M322},
	#endif
	#if ENABLED(CODE_M323)
		{323, gcode_M323},
	#endif
	#if ENABLED(CODE_M324)
		{324, gcode_M324},
	#endif
	#if ENABLED(CODE_M325)
		{325, gcode_M325},
	#endif
	#if ENABLED(CODE_M326)
		{326, gcode_M326},
	#endif
	#if ENABLED(CODE_M327)
		{327, gcode_M327},
	#endif
	#if ENABLED(CODE_M328)
		{328, gcode_M328},
	#endif
	#if ENABLED(CODE_M329)
		{329, gcode_M329},
	#endif
	#if ENABLED(CODE_M330)
		{330, gcode_M330},
	#endif
	#if ENABLED(CODE_M331)
		{331, gcode_M331},
	#endif
	#if ENABLED(CODE_M332)
		{332, gcode_M332},
	#endif
	#if ENABLED(CODE_M333)
		{333, gcode_M333},
	#endif
	#if ENABLED(CODE_M334)
		{334, gcode_M334},
	#endif
	#if ENABLED(CODE_M335)
		{335, gcode_M335},
	#endif
	#if ENABLED(CODE_M336)
		{336, gcode_M336},
	#endif
	#if ENABLED(CODE_M337)
		{337, gcode_M337},
	#endif
	#if ENABLED(CODE_M338)
		{338, gcode_M338},
	#endif
	#if ENABLED(CODE_M339)
		{339, gcode_M339},
	#endif
	#if ENABLED(CODE_M340)
		{340, gcode_M340},
	#endif
	#if ENABLED(CODE_M341)
		{341, gcode_M341},
	#endif
	#if ENABLED(CODE_M342)
		{342, gcode_M342},
	#endif
	#if ENABLED(CODE_M343)
		{343, gcode_M343},
	#endif
	#if ENABLED(CODE_M344)
		{344, gcode_M344},
	#endif
	#if ENABLED(CODE_M345)
		{345, gcode_M345},
	#endif
	#if ENABLED(CODE_M346)
		{346, gcode_M346},
	#endif
	#if ENABLED(CODE_M347)
		{347, gcode_M347},
	#endif
	#if ENABLED(CODE_M348)
		{348, gcode_M348},
	#endif
	#if ENABLED(CODE_M349)
		{349, gcode_M349},
	#endif
	#if ENABLED(CODE_M350)
		{350, gcode_M350},
	#endif
	#if ENABLED(CODE_M351)
		{351, gcode_M351},
	#endif
	#if ENABLED(CODE_M352)
		{352, gcode_M352},
	#endif
	#if ENABLED(CODE_M353)
		{353, gcode_M353},
	#endif
	#if ENABLED(CODE_M354)
		{354, gcode_M354},
	#endif
	#if ENABLED(CODE_M355)
		{355, gcode_M355},
	#endif
	#if ENABLED(CODE_M356)
		{356, gcode_M356},
	#endif
	#if ENABLED(CODE_M357)
		{357, gcode_M357},
	#endif
	#if ENABLED(CODE_M358)
		{358, gcode_M358},
	#endif
	#if ENABLED(CODE_M359)
		{359, gcode_M359},
	#endif
	#if ENABLED(CODE_M360)
		{360, gcode_M360},
	#endif
	#if ENABLED(CODE_M361)
		{361, gcode_M361},
	#endif
	#if ENABLED(CODE_M362)
		{362, gcode_M362},
	#endif
	#if ENABLED(CODE_M363)
		{363, gcode_M363},
	#endif
	#if ENABLED(CODE_M364)
		{364, gcode_M364},
	#endif
	#if ENABLED(CODE_M365)
		{365, gcode_M365},
	#endif
	#if ENABLED(CODE_M366)
		{366, gcode_M366},
	#endif
	#if ENABLED(CODE_M367)
		{367, gcode_M367},
	#endif
	#if ENABLED(CODE_M368)
		{368, gcode_M368},
	#endif
	#if ENABLED(CODE_M369)
		{369, gcode_M369},
	#endif
	#if ENABLED(CODE_M370)
		{370, gcode_M370},
	#endif
	#if ENABLED(CODE_M371)
		{371, gcode_M371},
	#endif
	#if ENABLED(CODE_M372)
		{372, gcode_M372},
	#endif
	#if ENABLED(CODE_M373)
		{373, gcode_M373},
	#endif
	#if ENABLED(CODE_M374)
		{374, gcode_M374},
	#endif
	#if ENABLED(CODE_M375)
		{375, gcode_M375},
	#endif
	#if ENABLED(CODE_M376)
		{376, gcode_M376},
	#endif
	#if ENABLED(CODE_M377)
		{377, gcode_M377},
	#endif
	#if ENABLED(CODE_M378)
		{378, gcode_M378},
	#endif
	#if ENABLED(CODE_M379)
		{379, gcode_M379},
	#endif
	#if ENABLED(CODE_M380)
		{380, gcode_M380},
	#endif
	#if ENABLED(CODE_M381)
		{381, gcode_M381},
	#endif
	#if ENABLED(CODE_M382)
		{382, gcode_M382},
	#endif
	#if ENABLED(CODE_M383)
		{383, gcode_M383},
	#endif
	#if ENABLED(CODE_M384)
		{384, gcode_M384},
	#endif
	#if ENABLED(CODE_M385)
		{385, gcode_M385},
	#endif
	#if ENABLED(CODE_M386)
		{386, gcode_M386},
	#endif
	#if ENABLED(CODE_M387)
		{387, gcode_M387},
	#endif
	#if ENABLED(CODE_M388)
		{388, gcode_M388},
	#endif
	#if ENABLED(CODE_M389)
		{389, gcode_M389},
	#endif
	#if ENABLED(CODE_M390)
		{390, gcode_M390},
	#endif
	#if ENABLED(CODE_M391)
		{391, gcode_M391},
	#endif
	#if ENABLED(CODE_M392)
		{392, gcode_M392},
	#endif
	#if ENABLED(CODE_M393)
		{393, gcode_M393},
	#endif
	#if ENABLED(CODE_M394)
		{394, gcode_M394},
	#endif
	#if ENABLED(CODE_M395)
		{395, gcode_M395},
	#endif
	#if ENABLED(CODE_M396)
		{396, gcode_M396},
	#endif
	#if ENABLED(CODE_M397)
		{397, gcode_M397},
	#endif
	#if ENABLED(CODE_M398)
		{398, gcode_M398},
	#endif
	#if ENABLED(CODE_M399)
		{399, gcode_M399},
	#endif
	#if ENABLED(CODE_M400)
		{400, gcode_M400},
	#endif
	#if ENABLED(CODE_M401)
		{401, gcode_M401},
	#endif
	#if ENABLED(CODE_M402)
		{402, gcode_M402},
	#endif
	#if ENABLED(CODE_M403)
		{403, gcode_M403},
	#endif
	#if ENABLED(CODE_M404)
		{404, gcode_M404},
	#endif
	#if ENABLED(CODE_M405)
		{405, gcode_M405},
	#endif
	#if ENABLED(CODE_M406)
		{406, gcode_M406},
	#endif
	#if ENABLED(CODE_M407)
		{407, gcode_M407},
	#endif
	#if ENABLED(CODE_M408)
		{408, gcode_M408},
	#endif
	#if ENABLED(CODE_M409)
		{409, gcode_M409},
	#endif
	#if ENABLED(CODE_M410)
		{410, gcode_M410},
	#endif
	#if ENABLED(CODE_M411)
		{411, gcode_M411},
	#endif
	#if ENABLED(CODE_M412)
		{412, gcode_M412},
	#endif
	#if ENABLED(CODE_M413)
		{413, gcode_M413},
	#endif
	#if ENABLED(CODE_M414)
		{414, gcode_M414},
	#endif
	#if ENABLED(CODE_M415)
		{415, gcode_M415},
	#endif
	#if ENABLED(CODE_M416)
		{416, gcode_M416},
	#endif
	#if ENABLED(CODE_M417)
		{417, gcode_M417},
	#endif
	#if ENABLED(CODE_M418)
		{418, gcode_M418},
	#endif
	#if ENABLED(CODE_M419)
		{419, gcode_M419},
	#endif
	#if ENABLED(CODE_M420)
		{420, gcode_M420},
	#endif
	#if ENABLED(CODE_M421)
		{421, gcode_M421},
	#endif
	#if ENABLED(CODE_M422)
		{422, gcode_M422},
	#endif
	#if ENABLED(CODE_M423)
		{423, gcode_M423},
	#endif
	#if ENABLED(CODE_M424)
		{424, gcode_M424},
	#endif
	#if ENABLED(CODE_M425)
		{425, gcode_M425},
	#endif
	#if ENABLED(CODE_M426)
		{426, gcode_M426},
	#endif
	#if ENABLED(CODE_M427)
		{427, gcode_M427},
	#endif
	#if ENABLED(CODE_M428)
		{428, gcode_M428},
	#endif
	#if ENABLED(CODE_M429)
		{429, gcode_M429},
	#endif
	#if ENABLED(CODE_M430)
		{430, gcode_M430},
	#endif
	#if ENABLED(CODE_M431)
		{431, gcode_M431},
	#endif
	#if ENABLED(CODE_M432)
		{432, gcode_M432},
	#endif
	#if ENABLED(CODE_M433)
		{433, gcode_M433},
	#endif
	#if ENABLED(CODE_M434)
		{434, gcode_M434},
	#endif
	#if ENABLED(CODE_M435)
		{435, gcode_M435},
	#endif
	#if ENABLED(CODE_M436)
		{436, gcode_M436},
	#endif
	#if ENABLED(CODE_M437)
		{437, gcode_M437},
	#endif
	#if ENABLED(CODE_M438)
		{438, gcode_M438},
	#endif
	#if ENABLED(CODE_M439)
		{439, gcode_M439},
	#endif
	#if ENABLED(CODE_M440)
		{440, gcode_M440},
	#endif
	#if ENABLED(CODE_M441)
		{441, gcode_M441},
	#endif
	#if ENABLED(CODE_M442)
		{442, gcode_M442},
	#endif
	#if ENABLED(CODE_M443)
		{443, gcode_M443},
	#endif
	#if ENABLED(CODE_M444)
		{444, gcode_M444},
	#endif
	#if ENABLED(CODE_M445)
		{445, gcode_M445},
	#endif
	#if ENABLED(CODE_M446)
		{446, gcode_M446},
	#endif
	#if ENABLED(CODE_M447)
		{447, gcode_M447},
	#endif
	#if ENABLED(CODE_M448)
		{448, gcode_M448},
	#endif
	#if ENABLED(CODE_M449)
		{449, gcode_M449},
	#endif
	#if ENABLED(CODE_M450)
		{450, gcode_M450},
	#endif
	#if ENABLED(CODE_M451)
		{451, gcode_M451},
	#endif
	#if ENABLED(CODE_M452)
		{452, gcode_M452},
	#endif
	#if ENABLED(CODE_M453)
		{453, gcode_M453},
	#endif
	#if ENABLED(CODE_M454)
		{454, gcode_M454},
	#endif
	#if ENABLED(CODE_M455)
		{455, gcode_M455},
	#endif
	#if ENABLED(CODE_M456)
		{456, gcode_M456},
	#endif
	#if ENABLED(CODE_M457)
		{457, gcode_M457},
	#endif
	#if ENABLED(CODE_M458)
		{458, gcode_M458},
	#endif
	#if ENABLED(CODE_M459)
		{459, gcode_M459},
	#endif
	#if ENABLED(CODE_M460)
		{460, gcode_M460},
	#endif
	#if ENABLED(CODE_M461)
		{461, gcode_M461},
	#endif
	#if ENABLED(CODE_M462)
		{462, gcode_M462},
	#endif
	#if ENABLED(CODE_M463)
		{463, gcode_M463},
	#endif
	#if ENABLED(CODE_M464)
		{464, gcode_M464},
	#endif
	#if ENABLED(CODE_M465)
		{465, gcode_M465},
	#endif
	#if ENABLED(CODE_M466)
		{466, gcode_M466},
	#endif
	#if ENABLED(CODE_M467)
		{467, gcode_M467},
	#endif
	#if ENABLED(CODE_M468)
		{468, gcode_M468},
	#endif
	#if ENABLED(CODE_M469)
		{469, gcode_M469},
	#endif
	#if ENABLED(CODE_M470)
		{470, gcode_M470},
	#endif
	#if ENABLED(CODE_M471)
		{471, gcode_M471},
	#endif
	#if ENABLED(CODE_M472)
		{472, gcode_M472},
	#endif
	#if ENABLED(CODE_M473)
		{473, gcode_M473},
	#endif
	#if ENABLED(CODE_M474)
		{474, gcode_M474},
	#endif
	#if ENABLED(CODE_M475)
		{475, gcode_M475},
	#endif
	#if ENABLED(CODE_M476)
		{476, gcode_M476},
	#endif
	#if ENABLED(CODE_M477)
		{477, gcode_M477},
	#endif
	#if ENABLED(CODE_M478)
		{478, gcode_M478},
	#endif
	#if ENABLED(CODE_M479)
		{479, gcode_M479},
	#endif
	#if ENABLED(CODE_M480)
		{480, gcode_M480},
	#endif
	#if ENABLED(CODE_M481)
		{481, gcode_M481},
	#endif
	#if ENABLED(CODE_M482)
		{482, gcode_M482},
	#endif
	#if ENABLED(CODE_M483)
		{483, gcode_M483},
	#endif
	#if ENABLED(CODE_M484)
		{484, gcode_M484},
	#endif
	#if ENABLED(CODE_M485)
		{485, gcode_M485},
	#endif
	#if ENABLED(CODE_M486)
		{486, gcode_M486},
	#endif
	#if ENABLED(CODE_M487)
		{487, gcode_M487},
	#endif
	#if ENABLED(CODE_M488)
		{488, gcode_M488},
	#endif
	#if ENABLED(CODE_M489)
		{489, gcode_M489},
	#endif
	#if ENABLED(CODE_M490)
		{490, gcode_M490},
	#endif
	#if ENABLED(CODE_M491)
		{491, gcode_M491},
	#endif
	#if ENABLED(CODE_M492)
		{492, gcode_M492},
	#endif
	#if ENABLED(CODE_M493)
		{493, gcode_M493},
	#endif
	#if ENABLED(CODE_M494)
		{494, gcode_M494},
	#endif
	#if ENABLED(CODE_M495)
		{495, gcode_M495},
	#endif
	#if ENABLED(CODE_M496)
		{496, gcode_M496},
	#endif
	#if ENABLED(CODE_M497)
		{497, gcode_M497},
	#endif
	#if ENABLED(CODE_M498)
		{498, gcode_M498},
	#endif
	#if ENABLED(CODE_M499)
		{499, gcode_M499},
	#endif
	#if ENABLED(CODE_M500)
		{500, gcode_M500},
	#endif
	#if ENABLED(CODE_M501)
		{501, gcode_M501},
	#endif
	#if ENABLED(CODE_M502)
		{502, gcode_M502},
	#endif
	#if ENABLED(CODE_M503)
		{503, gcode_M503},
	#endif
	#if ENABLED(CODE_M504)
		{504, gcode_M504},
	#endif
	#if ENABLED(CODE_M505)
		{505, gcode_M505},
	#endif
	#if ENABLED(CODE_M506)
		{506, gcode_M506},
	#endif
	#if ENABLED(CODE_M507)
		{507, gcode_M507},
	#endif
	#if ENABLED(CODE_M508)
		{508, gcode_M508},
	#endif
	#if ENABLED(CODE_M509)
		{509, gcode_M509},
	#endif
	#if ENABLED(CODE_M510)
		{510, gcode_M510},
	#endif
	#if ENABLED(CODE_M511)
		{511, gcode_M511},
	#endif
	#if ENABLED(CODE_M512)
		{512, gcode_M512},
	#endif
	#if ENABLED(CODE_M513)
		{513, gcode_M513},
	#endif
	#if ENABLED(CODE_M514)
		{514, gcode_M514},
	#endif
	#if ENABLED(CODE_M515)
		{515, gcode_M515},
	#endif
	#if ENABLED(CODE_M516)
		{516, gcode_M516},
	#endif
	#if ENABLED(CODE_M517)
		{517, gcode_M517},
	#endif
	#if ENABLED(CODE_M518)
		{518, gcode_M518},
	#endif
	#if ENABLED(CODE_M519)
		{519, gcode_M519},
	#endif
	#if ENABLED(CODE_M520)
		{520, gcode_M520},
	#endif
	#if ENABLED(CODE_M521)
		{521, gcode_M521},
	#endif
	#if ENABLED(CODE_M522)
		{522, gcode_M522},
	#endif
	#if ENABLED(CODE_M523)
		{523, gcode_M523},
	#endif
	#if ENABLED(CODE_M524)
		{524, gcode_M524},
	#endif
	#if ENABLED(CODE_M525)
		{525, gcode_M525},
	#endif
	#if ENABLED(CODE_M526)
		{526, gcode_M526},
	#endif
	#if ENABLED(CODE_M527)
		{527, gcode_M527},
	#endif
	#if ENABLED(CODE_M528)
		{528, gcode_M528},
	#endif
	#if ENABLED(CODE_M529)
		{529, gcode_M529},
	#endif
	#if ENABLED(CODE_M530)
		{530, gcode_M530},
	#endif
	#if ENABLED(CODE_M531)
		{531, gcode_M531},
	#endif
	#if ENABLED(CODE_M532)
		{532, gcode_M532},
	#endif
	#if ENABLED(CODE_M533)
		{533, gcode_M533},
	#endif
	#if ENABLED(CODE_M534)
		{534, gcode_M534},
	#endif
	#if ENABLED(CODE_M535)
		{535, gcode_M535},
	#endif
	#if ENABLED(CODE_M536)
		{536, gcode_M536},
	#endif
	#if ENABLED(CODE_M537)
		{537, gcode_M537},
	#endif
	#if ENABLED(CODE_M538)
		{538, gcode_M538},
	#endif
	#if ENABLED(CODE_M539)
		{539, gcode_M539},
	#endif
	#if ENABLED(CODE_M540)
		{540, gcode_M540},
	#endif
	#if ENABLED(CODE_M541)
		{541, gcode_M541},
	#endif
	#if ENABLED(CODE_M542)
		{542, gcode_M542},
	#endif
	#if ENABLED(CODE_M543)
		{543, gcode_M543},
	#endif
	#if ENABLED(CODE_M544)
		{544, gcode_M544},
	#endif
	#if ENABLED(CODE_M545)
		{545, gcode_M545},
	#endif
	#if ENABLED(CODE_M546)
		{546, gcode_M546},
	#endif
	#if ENABLED(CODE_M547)
		{547, gcode_M547},
	#endif
	#if ENABLED(CODE_M548)
		{548, gcode_M548},
	#endif
	#if ENABLED(CODE_M549)
		{549, gcode_M549},
	#endif
	#if ENABLED(CODE_M550)
		{550, gcode_M550},
	#endif
	#if ENABLED(CODE_M551)
		{551, gcode_M551},
	#endif
	#if ENABLED(CODE_M552)
		{552, gcode_M552},
	#endif
	#if ENABLED(CODE_M553)
		{553, gcode_M553},
	#endif
	#if ENABLED(CODE_M554)
		{554, gcode_M554},
	#endif
	#if ENABLED(CODE_M555)
		{555, gcode_M555},
	#endif
	#if ENABLED(CODE_M556)
		{556, gcode_M556},
	#endif
	#if ENABLED(CODE_M557)
		{557, gcode_M557},
	#endif
	#if ENABLED(CODE_M558)
		{558, gcode_M558},
	#endif
	#if ENABLED(CODE_M559)
		{559, gcode_M559},
	#endif
	#if ENABLED(CODE_M560)
		{560, gcode_M560},
	#endif
	#if ENABLED(CODE_M561)
		{561, gcode_M561},
	#endif
	#if ENABLED(CODE_M562)
		{562, gcode_M562},
	#endif
	#if ENABLED(CODE_M563)
		{563, gcode_M563},
	#endif
	#if ENABLED(CODE_M564)
		{564, gcode_M564},
	#endif
	#if ENABLED(CODE_M565)
		{565, gcode_M565},
	#endif
	#if ENABLED(CODE_M566)
		{566, gcode_M566},
	#endif
	#if ENABLED(CODE_M567)
		{567, gcode_M567},
	#endif
	#if ENABLED(CODE_M568)
		{568, gcode_M568},
	#endif
	#if ENABLED(CODE_M569)
		{569, gcode_M569},
	#endif
	#if ENABLED(CODE_M570)
		{570, gcode_M570},
	#endif
	#if ENABLED(CODE_M571)
		{571, gcode_M571},
	#endif
	#if ENABLED(CODE_M572)
		{572, gcode_M572},
	#endif
	#if ENABLED(CODE_M573)
		{573, gcode_M573},
	#endif
	#if ENABLED(CODE_M574)
		{574, gcode_M574},
	#endif
	#if ENABLED(CODE_M575)
		{575, gcode_M575},
	#endif
	#if ENABLED(CODE_M576)
		{576, gcode_M576},
	#endif
	#if ENABLED(CODE_M577)
		{577, gcode_M577},
	#endif
	#if ENABLED(CODE_M578)
		{578, gcode_M578},
	#endif
	#if ENABLED(CODE_M579)
		{579, gcode_M579},
	#endif
	#if ENABLED(CODE_M580)
		{580, gcode_M580},
	#endif
	#if ENABLED(CODE_M581)
		{581, gcode_M581},
	#endif
	#if ENABLED(CODE_M582)
		{582, gcode_M582},
	#endif
	#if ENABLED(CODE_M583)
		{583, gcode_M583},
	#endif
	#if ENABLED(CODE_M584)
		{584, gcode_M584},
	#endif
	#if ENABLED(CODE_M585)
		{585, gcode_M585},
	#endif
	#if ENABLED(CODE_M586)
		{586, gcode_M586},
	#endif
	#if ENABLED(CODE_M587)
		{587, gcode_M587},
	#endif
	#if ENABLED(CODE_M588)
		{588, gcode_M588},
	#endif
	#if ENABLED(CODE_M589)
		{589, gcode_M589},
	#endif
	#if ENABLED(CODE_M590)
		{590, gcode_M590},
	#endif
	#if ENABLED(CODE_M591)
		{591, gcode_M591},
	#endif
	#if ENABLED(CODE_M592)
		{592, gcode_M592},
	#endif
	#if ENABLED(CODE_M593)
		{593, gcode_M593},
	#endif
	#if ENABLED(CODE_M594)
		{594, gcode_M594},
	#endif
	#if ENABLED(CODE_M595)
		{595, gcode_M595},
	#endif
	#if ENABLED(CODE_M596)
		{596, gcode_M596},
	#endif
	#if ENABLED(CODE_M597)
		{597, gcode_M597},
	#endif
	#if ENABLED(CODE_M598)
		{598, gcode_M598},
	#endif
	#if ENABLED(CODE_M599)
		{599, gcode_M599},
	#endif
	#if ENABLED(CODE_M600)
		{600, gcode_M600},
	#endif
	#if ENABLED(CODE_M601)
		{601, gcode_M601},
	#endif
	#if ENABLED(CODE_M602)
		{602, gcode_M602},
	#endif
	#if ENABLED(CODE_M603)
		{603, gcode_M603},
	#endif
	#if ENABLED(CODE_M604)
		{604, gcode_M604},
	#endif
	#if ENABLED(CODE_M605)
		{605, gcode_M605},
	#endif
	#if ENABLED(CODE_M606)
		{606, gcode_M606},
	#endif
	#if ENABLED(CODE_M607)
		{607, gcode_M607},
	#endif
	#if ENABLED(CODE_M608)
		{608, gcode_M608},
	#endif
	#if ENABLED(CODE_M609)
		{609, gcode_M609},
	#endif
	#if ENABLED(CODE_M610)
		{610, gcode_M610},
	#endif
	#if ENABLED(CODE_M611)
		{611, gcode_M611},
	#endif
	#if ENABLED(CODE_M612)
		{612, gcode_M612},
	#endif
	#if ENABLED(CODE_M613)
		{613, gcode_M613},
	#endif
	#if ENABLED(CODE_M614)
		{614, gcode_M614},
	#endif
	#if ENABLED(CODE_M615)
		{615, gcode_M615},
	#endif
	#if ENABLED(CODE_M616)
		{616, gcode_M616},
	#endif
	#if ENABLED(CODE_M617)
		{617, gcode_M617},
	#endif
	#if ENABLED(CODE_M618)
		{618, gcode_M618},
	#endif
	#if ENABLED(CODE_M619)
		{619, gcode_M619},
	#endif
	#if ENABLED(CODE_M620)
		{620, gcode_M620},
	#endif
	#if ENABLED(CODE_M621)
		{621, gcode_M621},
	#endif
	#if ENABLED(CODE_M622)
		{622, gcode_M622},
	#endif
	#if ENABLED(CODE_M623)
		{623, gcode_M623},
	#endif
	#if ENABLED(CODE_M624)
		{624, gcode_M624},
	#endif
	#if ENABLED(CODE_M625)
		{625, gcode_M625},
	#endif
	#if ENABLED(CODE_M626)
		{626, gcode_M626},
	#endif
	#if ENABLED(CODE_M627)
		{627, gcode_M627},
	#endif
	#if ENABLED(CODE_M628)
		{628, gcode_M628},
	#endif
	#if ENABLED(CODE_M629)
		{629, gcode_M629},
	#endif
	#if ENABLED(CODE_M630)
		{630, gcode_M630},
	#endif
	#if ENABLED(CODE_M631)
		{631, gcode_M631},
	#endif
	#if ENABLED(CODE_M632)
		{632, gcode_M632},
	#endif
	#if ENABLED(CODE_M633)
		{633, gcode_M633},
	#endif
	#if ENABLED(CODE_M634)
		{634, gcode_M634},
	#endif
	#if ENABLED(CODE_M635)
		{635, gcode_M635},
	#endif
	#if ENABLED(CODE_M636)
		{636, gcode_M636},
	#endif
	#if ENABLED(CODE_M637)
		{637, gcode_M637},
	#endif
	#if ENABLED(CODE_M638)
		{638, gcode_M638},
	#endif
	#if ENABLED(CODE_M639)
		{639, gcode_M639},
	#endif
	#if ENABLED(CODE_M640)
		{640, gcode_M640},
	#endif
	#if ENABLED(CODE_M641)
		{641, gcode_M641},
	#endif
	#if ENABLED(CODE_M642)
		{642, gcode_M642},
	#endif
	#if ENABLED(CODE_M643)
		{643, gcode_M643},
	#endif
	#if ENABLED(CODE_M644)
		{644, gcode_M644},
	#endif
	#if ENABLED(CODE_M645)
		{645, gcode_M645},
	#endif
	#if ENABLED(CODE_M646)
		{646, gcode_M646},
	#endif
	#if ENABLED(CODE_M647)
		{647, gcode_M647},
	#endif
	#if ENABLED(CODE_M648)
		{648, gcode_M648},
	#endif
	#if ENABLED(CODE_M649)
		{649, gcode_M649},
	#endif
	#if ENABLED(CODE_M650)
		{650, gcode_M650},
	#endif
	#if ENABLED(CODE_M651)
		{651, gcode_M651},
	#endif
	#if ENABLED(CODE_M652)
		{652, gcode_M652},
	#endif
	#if ENABLED(CODE_M653)
		{653, gcode_M653},
	#endif
	#if ENABLED(CODE_M654)
		{654, gcode_M654},
	#endif
	#if ENABLED(CODE_M655)
		{655, gcode_M655},
	#endif
	#if ENABLED(CODE_M656)
		{656, gcode_M656},
	#endif
	#if ENABLED(CODE_M657)
		{657, gcode_M657},
	#endif
	#if ENABLED(CODE_M658)
		{658, gcode_M658},
	#endif
	#if ENABLED(CODE_M659)
		{659, gcode_M659},
	#endif
	#if ENABLED(CODE_M660)
		{660, gcode_M660},
	#endif
	#if ENABLED(CODE_M661)
		{661, gcode_M661},
	#endif
	#if ENABLED(CODE_M662)
		{662, gcode_M662},
	#endif
	#if ENABLED(CODE_M663)
		{663, gcode_M663},
	#endif
	#if ENABLED(CODE_M664)
		{664, gcode_M664},
	#endif
	#if ENABLED(CODE_M665)
		{665, gcode_M665},
	#endif
	#if ENABLED(CODE_M666)
		{666, gcode_M666},
	#endif
	#if ENABLED(CODE_M667)
		{667, gcode_M667},
	#endif
	#if ENABLED(CODE_M668)
		{668, gcode_M668},
	#endif
	#if ENABLED(CODE_M669)
		{669, gcode_M669},
	#endif
	#if ENABLED(CODE_M670)
		{670, gcode_M670},
	#endif
	#if ENABLED(CODE_M671)
		{671, gcode_M671},
	#endif
	#if ENABLED(CODE_M672)
		{672, gcode_M672},
	#endif
	#if ENABLED(CODE_M673)
		{673, gcode_M673},
	#endif
	#if ENABLED(CODE_M674)
		{674, gcode_M674},
	#endif
	#if ENABLED(CODE_M675)
		{675, gcode_M675},
	#endif
	#if ENABLED(CODE_M676)
		{676, gcode_M676},
	#endif
	#if ENABLED(CODE_M677)
		{677, gcode_M677},
	#endif
	#if ENABLED(CODE_M678)
		{678, gcode_M678},
	#endif
	#if ENABLED(CODE_M679)
		{679, gcode_M679},
	#endif
	#if ENABLED(CODE_M680)
		{680, gcode_M680},
	#endif
	#if ENABLED(CODE_M681)
		{681, gcode_M681},
	#endif
	#if ENABLED(CODE_M682)
		{682, gcode_M682},
	#endif
	#if ENABLED(CODE_M683)
		{683, gcode_M683},
	#endif
	#if ENABLED(CODE_M684)
		{684, gcode_M684},
	#endif
	#if ENABLED(CODE_M685)
		{685, gcode_M685},
	#endif
	#if ENABLED(CODE_M686)
		{686, gcode_M686},
	#endif
	#if ENABLED(CODE_M687)
		{687, gcode_M687},
	#endif
	#if ENABLED(CODE_M688)
		{688, gcode_M688},
	#endif
	#if ENABLED(CODE_M689)
		{689, gcode_M689},
	#endif
	#if ENABLED(CODE_M690)
		{690, gcode_M690},
	#endif
	#if ENABLED(CODE_M691)
		{691, gcode_M691},
	#endif
	#if ENABLED(CODE_M692)
		{692, gcode_M692},
	#endif
	#if ENABLED(CODE_M693)
		{693, gcode_M693},
	#endif
	#if ENABLED(CODE_M694)
		{694, gcode_M694},
	#endif
	#if ENABLED(CODE_M695)
		{695, gcode_M695},
	#endif
	#if ENABLED(CODE_M696)
		{696, gcode_M696},
	#endif
	#if ENABLED(CODE_M697)
		{697, gcode_M697},
	#endif
	#if ENABLED(CODE_M698)
		{698, gcode_M698},
	#endif
	#if ENABLED(CODE_M699)
		{699, gcode_M699},
	#endif
	#if ENABLED(CODE_M700)
		{700, gcode_M700},
	#endif
	#if ENABLED(CODE_M701)
		{701, gcode_M701},
	#endif
	#if ENABLED(CODE_M702)
		{702, gcode_M702},
	#endif
	#if ENABLED(CODE_M703)
		{703, gcode_M703},
	#endif
	#if ENABLED(CODE_M704)
		{704, gcode_M704},
	#endif
	#if ENABLED(CODE_M705)
		{705, gcode_M705},
	#endif
	#if ENABLED(CODE_M706)
		{706, gcode_M706},
	#endif
	#if ENABLED(CODE_M707)
		{707, gcode_M707},
	#endif
	#if ENABLED(CODE_M708)
		{708, gcode_M708},
	#endif
	#if ENABLED(CODE_M709)
		{709, gcode_M709},
	#endif
	#if ENABLED(CODE_M710)
		{710, gcode_M710},
	#endif
	#if ENABLED(CODE_M711)
		{711, gcode_M711},
	#endif
	#if ENABLED(CODE_M712)
		{712, gcode_M712},
	#endif
	#if ENABLED(CODE_M713)
		{713, gcode_M713},
	#endif
	#if ENABLED(CODE_M714)
		{714, gcode_M714},
	#endif
	#if ENABLED(CODE_M715)
		{715, gcode_M715},
	#endif
	#if ENABLED(CODE_M716)
		{716, gcode_M716},
	#endif
	#if ENABLED(CODE_M717)
		{717, gcode_M717},
	#endif
	#if ENABLED(CODE_M718)
		{718, gcode_M718},
	#endif
	#if ENABLED(CODE_M719)
		{719, gcode_M719},
	#endif
	#if ENABLED(CODE_M720)
		{720, gcode_M720},
	#endif
	#if ENABLED(CODE_M721)
		{721, gcode_M721},
	#endif
	#if ENABLED(CODE_M722)
		{722, gcode_M722},
	#endif
	#if ENABLED(CODE_M723)
		{723, gcode_M723},
	#endif
	#if ENABLED(CODE_M724)
		{724, gcode_M724},
	#endif
	#if ENABLED(CODE_M725)
		{725, gcode_M725},
	#endif
	#if ENABLED(CODE_M726)
		{726, gcode_M726},
	#endif
	#if ENABLED(CODE_M727)
		{727, gcode_M727},
	#endif
	#if ENABLED(CODE_M728)
		{728, gcode_M728},
	#endif
	#if ENABLED(CODE_M729)
		{729, gcode_M729},
	#endif
	#if ENABLED(CODE_M730)
		{730, gcode_M730},
	#endif
	#if ENABLED(CODE_M731)
		{731, gcode_M731},
	#endif
	#if ENABLED(CODE_M732)
		{732, gcode_M732},
	#endif
	#if ENABLED(CODE_M733)
		{733, gcode_M733},
	#endif
	#if ENABLED(CODE_M734)
		{734, gcode_M734},
	#endif
	#if ENABLED(CODE_M735)
		{735, gcode_M735},
	#endif
	#if ENABLED(CODE_M736)
		{736, gcode_M736},
	#endif
	#if ENABLED(CODE_M737)
		{737, gcode_M737},
	#endif
	#if ENABLED(CODE_M738)
		{738, gcode_M738},
	#endif
	#if ENABLED(CODE_M739)
		{739, gcode_M739},
	#endif
	#if ENABLED(CODE_M740)
		{740, gcode_M740},
	#endif
	#if ENABLED(CODE_M741)
		{741, gcode_M741},
	#endif
	#if ENABLED(CODE_M742)
		{742, gcode_M742},
	#endif
	#if ENABLED(CODE_M743)
		{743, gcode_M743},
	#endif
	#if ENABLED(CODE_M744)
		{744, gcode_M744},
	#endif
	#if ENABLED(CODE_M745)
		{745, gcode_M745},
	#endif
	#if ENABLED(CODE_M746)
		{746, gcode_M746},
	#endif
	#if ENABLED(CODE_M747)
		{747, gcode_M747},
	#endif
	#if ENABLED(CODE_M748)
		{748, gcode_M748},
	#endif
	#if ENABLED(CODE_M749)
		{749, gcode_M749},
	#endif
	#if ENABLED(CODE_M750)
		{750, gcode_M750},
	#endif
	#if ENABLED(CODE_M751)
		{751, gcode_M751},
	#endif
	#if ENABLED(CODE_M752)
		{752, gcode_M752},
	#endif
	#if ENABLED(CODE_M753)
		{753, gcode_M753},
	#endif
	#if ENABLED(CODE_M754)
		{754, gcode_M754},
	#endif
	#if ENABLED(CODE_M755)
		{755, gcode_M755},
	#endif
	#if ENABLED(CODE_M756)
		{756, gcode_M756},
	#endif
	#if ENABLED(CODE_M757)
		{757, gcode_M757},
	#endif
	#if ENABLED(CODE_M758)
		{758, gcode_M758},
	#endif
	#if ENABLED(CODE_M759)
		{759, gcode_M759},
	#endif
	#if ENABLED(CODE_M760)
		{760, gcode_M760},
	#endif
	#if ENABLED(CODE_M761)
		{761, gcode_M761},
	#endif
	#if ENABLED(CODE_M762)
		{762, gcode_M762},
	#endif
	#if ENABLED(CODE_M763)
		{763, gcode_M763},
	#endif
	#if ENABLED(CODE_M764)
		{764, gcode_M764},
	#endif
	#if ENABLED(CODE_M765)
		{765, gcode_M765},
	#endif
	#if ENABLED(CODE_M766)
		{766, gcode_M766},
	#endif
	#if ENABLED(CODE_M767)
		{767, gcode_M767},
	#endif
	#if ENABLED(CODE_M768)
		{768, gcode_M768},
	#endif
	#if ENABLED(CODE_M769)
		{769, gcode_M769},
	#endif
	#if ENABLED(CODE_M770)
		{770, gcode_M770},
	#endif
	#if ENABLED(CODE_M771)
		{771, gcode_M771},
	#endif
	#if ENABLED(CODE_M772)
		{772, gcode_M772},
	#endif
	#if ENABLED(CODE_M773)
		{773, gcode_M773},
	#endif
	#if ENABLED(CODE_M774)
		{774, gcode_M774},
	#endif
	#if ENABLED(CODE_M775)
		{775, gcode_M775},
	#endif
	#if ENABLED(CODE_M776)
		{776, gcode_M776},
	#endif
	#if ENABLED(CODE_M777)
		{777, gcode_M777},
	#endif
	#if ENABLED(CODE_M778)
		{778, gcode_M778},
	#endif
	#if ENABLED(CODE_M779)
		{779, gcode_M779},
	#endif
	#if ENABLED(CODE_M780)
		{780, gcode_M780},
	#endif
	#if ENABLED(CODE_M781)
		{781, gcode_M781},
	#endif
	#if ENABLED(CODE_M782)
		{782, gcode_M782},
	#endif
	#if ENABLED(CODE_M783)
		{783, gcode_M783},
	#endif
	#if ENABLED(CODE_M784)
		{784, gcode_M784},
	#endif
	#if ENABLED(CODE_M785)
		{785, gcode_M785},
	#endif
	#if ENABLED(CODE_M786)
		{786, gcode_M786},
	#endif
	#if ENABLED(CODE_M787)
		{787, gcode_M787},
	#endif
	#if ENABLED(CODE_M788)
		{788, gcode_M788},
	#endif
	#if ENABLED(CODE_M789)
		{789, gcode_M789},
	#endif
	#if ENABLED(CODE_M790)
		{790, gcode_M790},
	#endif
	#if ENABLED(CODE_M791)
		{791, gcode_M791},
	#endif
	#if ENABLED(CODE_M792)
		{792, gcode_M792},
	#endif
	#if ENABLED(CODE_M793)
		{793, gcode_M793},
	#endif
	#if ENABLED(CODE_M794)
		{794, gcode_M794},
	#endif
	#if ENABLED(CODE_M795)
		{795, gcode_M795},
	#endif
	#if ENABLED(CODE_M796)
		{796, gcode_M796},
	#endif
	#if ENABLED(CODE_M797)
		{797, gcode_M797},
	#endif
	#if ENABLED(CODE_M798)
		{798, gcode_M798},
	#endif
	#if ENABLED(CODE_M799)
		{799, gcode_M799},
	#endif
	#if ENABLED(CODE_M800)
		{800, gcode_M800},
	#endif
	#if ENABLED(CODE_M801)
		{801, gcode_M801},
	#endif
	#if ENABLED(CODE_M802)
		{802, gcode_M802},
	#endif
	#if ENABLED(CODE_M803)
		{803, gcode_M803},
	#endif
	#if ENABLED(CODE_M804)
		{804, gcode_M804},
	#endif
	#if ENABLED(CODE_M805)
		{805, gcode_M805},
	#endif
	#if ENABLED(CODE_M806)
		{806, gcode_M806},
	#endif
	#if ENABLED(CODE_M807)
		{807, gcode_M807},
	#endif
	#if ENABLED(CODE_M808)
		{808, gcode_M808},
	#endif
	#if ENABLED(CODE_M809)
		{809, gcode_M809},
	#endif
	#if ENABLED(CODE_M810)
		{810, gcode_M810},
	#endif
	#if ENABLED(CODE_M811)
		{811, gcode_M811},
	#endif
	#if ENABLED(CODE_M812)
		{812, gcode_M812},
	#endif
	#if ENABLED(CODE_M813)
		{813, gcode_M813},
	#endif
	#if ENABLED(CODE_M814)
		{814, gcode_M814},
	#endif
	#if ENABLED(CODE_M815)
		{815, gcode_M815},
	#endif
	#if ENABLED(CODE_M816)
		{816, gcode_M816},
	#endif
	#if ENABLED(CODE_M817)
		{817, gcode_M817},
	#endif
	#if ENABLED(CODE_M818)
		{818, gcode_M818},
	#endif
	#if ENABLED(CODE_M819)
		{819, gcode_M819},
	#endif
	#if ENABLED(CODE_M820)
		{820, gcode_M820},
	#endif
	#if ENABLED(CODE_M821)
		{821, gcode_M821},
	#endif
	#if ENABLED(CODE_M822)
		{822, gcode_M822},
	#endif
	#if ENABLED(CODE_M823)
		{823, gcode_M823},
	#endif
	#if ENABLED(CODE_M824)
		{824, gcode_M824},
	#endif
	#if ENABLED(CODE_M825)
		{825, gcode_M825},
	#endif
	#if ENABLED(CODE_M826)
		{826, gcode_M826},
	#endif
	#if ENABLED(CODE_M827)
		{827, gcode_M827},
	#endif
	#if ENABLED(CODE_M828)
		{828, gcode_M828},
	#endif
	#if ENABLED(CODE_M829)
		{829, gcode_M829},
	#endif
	#if ENABLED(CODE_M830)
		{830, gcode_M830},
	#endif
	#if ENABLED(CODE_M831)
		{831, gcode_M831},
	#endif
	#if ENABLED(CODE_M832)
		{832, gcode_M832},
	#endif
	#if ENABLED(CODE_M833)
		{833, gcode_M833},
	#endif
	#if ENABLED(CODE_M834)
		{834, gcode_M834},
	#endif
	#if ENABLED(CODE_M835)
		{835, gcode_M835},
	#endif
	#if ENABLED(CODE_M836)
		{836, gcode_M836},
	#endif
	#if ENABLED(CODE_M837)
		{837, gcode_M837},
	#endif
	#if ENABLED(CODE_M838)
		{838, gcode_M838},
	#endif
	#if ENABLED(CODE_M839)
		{839, gcode_M839},
	#endif
	#if ENABLED(CODE_M840)
		{840, gcode_M840},
	#endif
	#if ENABLED(CODE_M841)
		{841, gcode_M841},
	#endif
	#if ENABLED(CODE_M842)
		{842, gcode_M842},
	#endif
	#if ENABLED(CODE_M843)
		{843, gcode_M843},
	#endif
	#if ENABLED(CODE_M844)
		{844, gcode_M844},
	#endif
	#if ENABLED(CODE_M845)
		{845, gcode_M845},
	#endif
	#if ENABLED(CODE_M846)
		{846, gcode_M846},
	#endif
	#if ENABLED(CODE_M847)
		{847, gcode_M847},
	#endif
	#if ENABLED(CODE_M848)
		{848, gcode_M848},
	#endif
	#if ENABLED(CODE_M849)
		{849, gcode_M849},
	#endif
	#if ENABLED(CODE_M850)
		{850, gcode_M850},
	#endif
	#if ENABLED(CODE_M851)
		{851, gcode_M851},
	#endif
	#if ENABLED(CODE_M852)
		{852, gcode_M852},
	#endif
	#if ENABLED(CODE_M853)
		{853, gcode_M853},
	#endif
	#if ENABLED(CODE_M854)
		{854, gcode_M854},
	#endif
	#if ENABLED(CODE_M855)
		{855, gcode_M855},
	#endif
	#if ENABLED(CODE_M856)
		{856, gcode_M856},
	#endif
	#if ENABLED(CODE_M857)
		{857, gcode_M857},
	#endif
	#if ENABLED(CODE_M858)
		{858, gcode_M858},
	#endif
	#if ENABLED(CODE_M859)
		{859, gcode_M859},
	#endif
	#if ENABLED(CODE_M860)
		{860, gcode_M860},
	#endif
	#if ENABLED(CODE_M861)
		{861, gcode_M861},
	#endif
	#if ENABLED(CODE_M862)
		{862, gcode_M862},
	#endif
	#if ENABLED(CODE_M863)
		{863, gcode_M863},
	#endif
	#if ENABLED(CODE_M864)
		{864, gcode_M864},
	#endif
	#if ENABLED(CODE_M865)
		{865, gcode_M865},
	#endif
	#if ENABLED(CODE_M866)
		{866, gcode_M866},
	#endif
	#if ENABLED(CODE_M867)
		{867, gcode_M867},
	#endif
	#if ENABLED(CODE_M868)
		{868, gcode_M868},
	#endif
	#if ENABLED(CODE_M869)
		{869, gcode_M869},
	#endif
	#if ENABLED(CODE_M870)
		{870, gcode_M870},
	#endif
	#if ENABLED(CODE_M871)
		{871, gcode_M871},
	#endif
	#if ENABLED(CODE_M872)
		{872, gcode_M872},
	#endif
	#if ENABLED(CODE_M873)
		{873, gcode_M873},
	#endif
	#if ENABLED(CODE_M874)
		{874, gcode_M874},
	#endif
	#if ENABLED(CODE_M875)
		{875, gcode_M875},
	#endif
	#if ENABLED(CODE_M876)
		{876, gcode_M876},
	#endif
	#if ENABLED(CODE_M877)
		{877, gcode_M877},
	#endif
	#if ENABLED(CODE_M878)
		{878, gcode_M878},
	#endif
	#if ENABLED(CODE_M879)
		{879, gcode_M879},
	#endif
	#if ENABLED(CODE_M880)
		{880, gcode_M880},
	#endif
	#if ENABLED(CODE_M881)
		{881, gcode_M881},
	#endif
	#if ENABLED(CODE_M882)
		{882, gcode_M882},
	#endif
	#if ENABLED(CODE_M883)
		{883, gcode_M883},
	#endif
	#if ENABLED(CODE_M884)
		{884, gcode_M884},
	#endif
	#if ENABLED(CODE_M885)
		{885, gcode_M885},
	#endif
	#if ENABLED(CODE_M886)
		{886, gcode_M886},
	#endif
	#if ENABLED(CODE_M887)
		{887, gcode_M887},
	#endif
	#if ENABLED(CODE_M888)
		{888, gcode_M888},
	#endif
	#if ENABLED(CODE_M889)
		{889, gcode_M889},
	#endif
	#if ENABLED(CODE_M890)
		{890, gcode_M890},
	#endif
	#if ENABLED(CODE_M891)
		{891, gcode_M891},
	#endif
	#if ENABLED(CODE_M892)
		{892, gcode_M892},
	#endif
	#if ENABLED(CODE_M893)
		{893, gcode_M893},
	#endif
	#if ENABLED(CODE_M894)
		{894, gcode_M894},
	#endif
	#if ENABLED(CODE_M895)
		{895, gcode_M895},
	#endif
	#if ENABLED(CODE_M896)
		{896, gcode_M896},
	#endif
	#if ENABLED(CODE_M897)
		{897, gcode_M897},
	#endif
	#if ENABLED(CODE_M898)
		{898, gcode_M898},
	#endif
	#if ENABLED(CODE_M899)
		{899, gcode_M899},
	#endif
	#if ENABLED(CODE_M900)
		{900, gcode_M900},
	#endif
	#if ENABLED(CODE_M901)
		{901, gcode_M901},
	#endif
	#if ENABLED(CODE_M902)
		{902, gcode_M902},
	#endif
	#if ENABLED(CODE_M903)
		{903, gcode_M903},
	#endif
	#if ENABLED(CODE_M904)
		{904, gcode_M904},
	#endif
	#if ENABLED(CODE_M905)
		{905, gcode_M905},
	#endif
	#if ENABLED(CODE_M906)
		{906, gcode_M906},
	#endif
	#if ENABLED(CODE_M907)
		{907, gcode_M907},
	#endif
	#if ENABLED(CODE_M908)
		{908, gcode_M908},
	#endif
	#if ENABLED(CODE_M909)
		{909, gcode_M909},
	#endif
	#if ENABLED(CODE_M910)
		{910, gcode_M910},
	#endif
	#if ENABLED(CODE_M911)
		{911, gcode_M911},
	#endif
	#if ENABLED(CODE_M912)
		{912, gcode_M912},
	#endif
	#if ENABLED(CODE_M913)
		{913, gcode_M913},
	#endif
	#if ENABLED(CODE_M914)
		{914, gcode_M914},
	#endif
	#if ENABLED(CODE_M915)
		{915, gcode_M915},
	#endif
	#if ENABLED(CODE_M916)
		{916, gcode_M916},
	#endif
	#if ENABLED(CODE_M917)
		{917, gcode_M917},
	#endif
	#if ENABLED(CODE_M918)
		{918, gcode_M918},
	#endif
	#if ENABLED(CODE_M919)
		{919, gcode_M919},
	#endif
	#if ENABLED(CODE_M920)
		{920, gcode_M920},
	#endif
	#if ENABLED(CODE_M921)
		{921, gcode_M921},
	#endif
	#if ENABLED(CODE_M922)
		{922, gcode_M922},
	#endif
	#if ENABLED(CODE_M923)
		{923, gcode_M923},
	#endif
	#if ENABLED(CODE_M924)
		{924, gcode_M924},
	#endif
	#if ENABLED(CODE_M925)
		{925, gcode_M925},
	#endif
	#if ENABLED(CODE_M926)
		{926, gcode_M926},
	#endif
	#if ENABLED(CODE_M927)
		{927, gcode_M927},
	#endif
	#if ENABLED(CODE_M928)
		{928, gcode_M928},
	#endif
	#if ENABLED(CODE_M929)
		{929, gcode_M929},
	#endif
	#if ENABLED(CODE_M930)
		{930, gcode_M930},
	#endif
	#if ENABLED(CODE_M931)
		{931, gcode_M931},
	#endif
	#if ENABLED(CODE_M932)
		{932, gcode_M932},
	#endif
	#if ENABLED(CODE_M933)
		{933, gcode_M933},
	#endif
	#if ENABLED(CODE_M934)
		{934, gcode_M934},
	#endif
	#if ENABLED(CODE_M935)
		{935, gcode_M935},
	#endif
	#if ENABLED(CODE_M936)
		{936, gcode_M936},
	#endif
	#if ENABLED(CODE_M937)
		{937, gcode_M937},
	#endif
	#if ENABLED(CODE_M938)
		{938, gcode_M938},
	#endif
	#if ENABLED(CODE_M939)
		{939, gcode_M939},
	#endif
	#if ENABLED(CODE_M940)
		{940, gcode_M940},
	#endif
	#if ENABLED(CODE_M941)
		{941, gcode_M941},
	#endif
	#if ENABLED(CODE_M942)
		{942, gcode_M942},
	#endif
	#if ENABLED(CODE_M943)
		{943, gcode_M943},
	#endif
	#if ENABLED(CODE_M944)
		{944, gcode_M944},
	#endif
	#if ENABLED(CODE_M945)
		{945, gcode_M945},
	#endif
	#if ENABLED(CODE_M946)
		{946, gcode_M946},
	#endif
	#if ENABLED(CODE_M947)
		{947, gcode_M947},
	#endif
	#if ENABLED(CODE_M948)
		{948, gcode_M948},
	#endif
	#if ENABLED(CODE_M949)
		{949, gcode_M949},
	#endif
	#if ENABLED(CODE_M950)
		{950, gcode_M950},
	#endif
	#if ENABLED(CODE_M951)
		{951, gcode_M951},
	#endif
	#if ENABLED(CODE_M952)
		{952, gcode_M952},
	#endif
	#if ENABLED(CODE_M953)
		{953, gcode_M953},
	#endif
	#if ENABLED(CODE_M954)
		{954, gcode_M954},
	#endif
	#if ENABLED(CODE_M955)
		{955, gcode_M955},
	#endif
	#if ENABLED(CODE_M956)
		{956, gcode_M956},
	#endif
	#if ENABLED(CODE_M957)
		{957, gcode_M957},
	#endif
	#if ENABLED(CODE_M958)
		{958, gcode_M958},
	#endif
	#if ENABLED(CODE_M959)
		{959, gcode_M959},
	#endif
	#if ENABLED(CODE_M960)
		{960, gcode_M960},
	#endif
	#if ENABLED(CODE_M961)
		{961, gcode_M961},
	#endif
	#if ENABLED(CODE_M962)
		{962, gcode_M962},
	#endif
	#if ENABLED(CODE_M963)
		{963, gcode_M963},
	#endif
	#if ENABLED(CODE_M964)
		{964, gcode_M964},
	#endif
	#if ENABLED(CODE_M965)
		{965, gcode_M965},
	#endif
	#if ENABLED(CODE_M966)
		{966, gcode_M966},
	#endif
	#if ENABLED(CODE_M967)
		{967, gcode_M967},
	#endif
	#if ENABLED(CODE_M968)
		{968, gcode_M968},
	#endif
	#if ENABLED(CODE_M969)
		{969, gcode_M969},
	#endif
	#if ENABLED(CODE_M970)
		{970, gcode_M970},
	#endif
	#if ENABLED(CODE_M971)
		{971, gcode_M971},
	#endif
	#if ENABLED(CODE_M972)
		{972, gcode_M972},
	#endif
	#if ENABLED(CODE_M973)
		{973, gcode_M973},
	#endif
	#if ENABLED(CODE_M974)
		{974, gcode_M974},
	#endif
	#if ENABLED(CODE_M975)
		{975, gcode_M975},
	#endif
	#if ENABLED(CODE_M976)
		{976, gcode_M976},
	#endif
	#if ENABLED(CODE_M977)
		{977, gcode_M977},
	#endif
	#if ENABLED(CODE_M978)
		{978, gcode_M978},
	#endif
	#if ENABLED(CODE_M979)
		{979, gcode_M979},
	#endif
	#if ENABLED(CODE_M980)
		{980, gcode_M980},
	#endif
	#if ENABLED(CODE_M981)
		{981, gcode_M981},
	#endif
	#if ENABLED(CODE_M982)
		{982, gcode_M982},
	#endif
	#if ENABLED(CODE_M983)
		{983, gcode_M983},
	#endif
	#if ENABLED(CODE_M984)
		{984, gcode_M984},
	#endif
	#if ENABLED(CODE_M985)
		{985, gcode_M985},
	#endif
	#if ENABLED(CODE_M986)
		{986, gcode_M986},
	#endif
	#if ENABLED(CODE_M987)
		{987, gcode_M987},
	#endif
	#if ENABLED(CODE_M988)
		{988, gcode_M988},
	#endif
	#if ENABLED(CODE_M989)
		{989, gcode_M989},
	#endif
	#if ENABLED(CODE_M990)
		{990, gcode_M990},
	#endif
	#if ENABLED(CODE_M991)
		{991, gcode_M991},
	#endif
	#if ENABLED(CODE_M992)
		{992, gcode_M992},
	#endif
	#if ENABLED(CODE_M993)
		{993, gcode_M993},
	#endif
	#if ENABLED(CODE_M994)
		{994, gcode_M994},
	#endif
	#if ENABLED(CODE_M995)
		{995, gcode_M995},
	#endif
	#if ENABLED(CODE_M996)
		{996, gcode_M996},
	#endif
	#if ENABLED(CODE_M997)
		{997, gcode_M997},
	#endif
	#if ENABLED(CODE_M998)
		{998, gcode_M998},
	#endif
	#if ENABLED(CODE_M999)
		{999, gcode_M999}
	#endif
};
