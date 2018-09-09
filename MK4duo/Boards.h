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

#ifndef _BOARD_H_
#define _BOARD_H_

// Macros for board type
#define BOARD_UNKNOWN -1
#define MB(board) (MOTHERBOARD==BOARD_##board)

/**
 * RAMPS 1.3 / 1.4 - ATmega1280, ATmega2560
 */
#define BOARD_RAMPS_OLD          3    // MEGA/RAMPS up to 1.2
#define BOARD_RAMPS_13_HFB      33    // RAMPS 1.3 / 1.4 (Power outputs: Hotend, Fan, Bed)
#define BOARD_RAMPS_13_HHB      34    // RAMPS 1.3 / 1.4 (Power outputs: Hotend0, Hotend1, Bed)
#define BOARD_RAMPS_13_HFF      35    // RAMPS 1.3 / 1.4 (Power outputs: Hotend, Fan, Fan)
#define BOARD_RAMPS_13_HHF      36    // RAMPS 1.3 / 1.4 (Power outputs: Hotend0, Hotend1, Fan)
#define BOARD_RAMPS_13_HHH      37    // RAMPS 1.3 / 1.4 (Power outputs: Hotend0, Hotend1, Hotend2)

/**
 * RAMPS Derivates - ATmega1280, ATmega2560
 */
#define BOARD_ULTIMAKER          7    // Ultimaker
#define BOARD_SAINSMART_2IN1    39    // SainSmart 2 in 1
#define BOARD_MKS_BASE          40    // MKS BASE 1.0
#define BOARD_MKS_13            47    // MKS v1.3 or 1.4 (maybe higher)
#define BOARD_TRIGORILLA        49    // TRIGORILLA (ANYCUBIC)
#define BOARD_MKS_GEN_L         53    // MKS GEN L
#define BOARD_AZTEEG_X3         67    // Azteeg X3
#define BOARD_AZTEEG_X3_PRO     68    // Azteeg X3 Pro
#define BOARD_ULTIMAKER_OLD     71    // Ultimaker (Older electronics. Pre 1.5.4. This is rare)
#define BOARD_ULTIMAIN_2        72    // Ultimainboard 2.x (Uses TEMP_SENSOR 20)
#define BOARD_GT2560_REV_A      74    // Geeetech GT2560 Rev. A
#define BOARD_GT2560_REV_A_PLUS 75    // Geeetech GT2560 Rev. A+
#define BOARD_3DRAG             77    // 3Drag Controller
#define BOARD_K8200             78    // Vellemann K8200 Controller (variant of the 3Drag Controller)
#define BOARD_3DVERTEX          79    // 3DVertex Controller (Velleman K8400)
#define BOARD_RUMBA             80    // Rumba
#define BOARD_ANET             100    // Anet board for A2, A6 and A8
#define BOARD_RAMPS_ENDER_4    243    // Creality Ender-4, CR-8
#define BOARD_MAKEBOARD_MINI   431    // MakeBoard Mini v2.1.2 is a control board sold by MicroMake

/**
 * Other ATmega1280, ATmega2560
 */
#define BOARD_CHEAPTRONIC        2    // Cheaptronic v1.0
#define BOARD_ELEFU_3           21    // Elefu Ra Board (v3)
#define BOARD_MKS_MINI          41    // MKS MINI 1.0
#define BOARD_MEGATRONICS       70    // Megatronics
#define BOARD_RAMBO            301    // Rambo
#define BOARD_MINIRAMBO_10A    302    // Mini-Rambo 1.0a
#define BOARD_MINIRAMBO        303    // Mini-Rambo
#define BOARD_PIBOT            316    // PiBot Controller Rev2.0
#define BOARD_MEGATRONICS_2    701    // Megatronics v2.0
#define BOARD_MEGATRONICS_3    703    // Megatronics v3.0
#define BOARD_LEAPFROG         999    // Leapfrog

/**
 * ATmega1281, ATmega2561
 */
#define BOARD_MINITRONICS      702    // Minitronics v1.0

/**
 * Sanguinololu and Derivatives - ATmega644P, ATmega1284P
 */
#define BOARD_SANGUINOLOLU_11    6    // Sanguinololu < 1.2
#define BOARD_SANGUINOLOLU_12   62    // Sanguinololu 1.2 and above
#define BOARD_MELZI             63    // Melzi
#define BOARD_STB_11            64    // STB V1.1
#define BOARD_AZTEEG_X1         65    // Azteeg X1
#define BOARD_MELZI_CREALITY    89    // Melzi Creality3D board (for CR-10 etc)

/**
 * Other ATmega644P, ATmega644, ATmega1284P
 */
#define BOARD_GEN6               5    // Gen6
#define BOARD_GEN3_PLUS          9    // Gen3+
#define BOARD_GEN7_CUSTOM       10    // Gen7 custom (Alfons3 Version) "https://github.com/Alfons3/Generation_7_Electronics"
#define BOARD_GEN7_12           11    // Gen7 v1.1, v1.2
#define BOARD_GEN7_13           12    // Gen7 v1.3
#define BOARD_GEN7_14           13    // Gen7 v1.4
#define BOARD_SETHI             20    // Sethi 3D_1
#define BOARD_GEN3_MONOLITHIC   22    // Gen3 Monolithic Electronics
#define BOARD_OMCA_A            90    // Alpha OMCA board
#define BOARD_OMCA              91    // Final OMCA board

/**
 * Teensyduino - AT90USB1286, AT90USB1286P
 */
#define BOARD_TEENSYLU           8    // Teensylu

/**
 * Custom Boards
 */
#define BOARD_99                99    // Custom Board


/**
 * SAM3X8E ARM Cortex M3
 */
#define BOARD_PICCOLO_3D      1400    // PICCOLO_3D ARM 32 Arduino DUE
#define BOARD_RADDS           1401    // RADDS ARM 32 bit board
#define BOARD_RAMPS_FD_V1     1403    // RAMPS-FD V1 ARM 32 bit board
#define BOARD_RAMPS_FD_V2     1404    // RAMPS-FD V2 ARM 32 bit board
#define BOARD_CNCONTROLS_V14  1405    // CNControls V14
#define BOARD_ULTIMAKER4DUE   1407    // Ultimaker Shield + Arduino DUE Aleksandr Varaksa mods (https://iworld4us.com/diy-3d-printer.html)
#define BOARD_RAMPS_SMART_HFB 1411    // RAMPS-SMART (Power outputs: Hotend, Fan, Bed)
#define BOARD_RAMPS_SMART_HHB 1412    // RAMPS-SMART (Power outputs: Hotend0, Hotend1, Bed)
#define BOARD_RAMPS_SMART_HFF 1413    // RAMPS-SMART (Power outputs: Hotend, Fan0, Fan1)
#define BOARD_RAMPS_SMART_HHF 1414    // RAMPS-SMART (Power outputs: Hotend0, Hotend1, Fan)
#define BOARD_RAMPS4DUE       1430    // RAMPS4DUE with AndrewBCN's RAMPS mods (http://forums.reprap.org/read.php?219,479626,page=1) ARM 32 bit board
#define BOARD_RAMPS_17_HFB    1433    // RAMPS 1.7 (Power outputs: Hotend, Fan, Bed)
#define BOARD_ALLIGATOR_R2    1502    // ALLIGATOR R2 ARM 32 bit board
#define BOARD_ALLIGATOR_R3    1503    // ALLIGATOR R3 ARM 32 bit board
#define BOARD_RURAMPS4D_V11   1550    // RuRAMPS4Duo v1.1 (Power outputs: Hotend0, Hotend1, Hotend2, Fan0, Fan1, Bed)
#define BOARD_RURAMPS4D_V13   1551    // RuRAMPS4Duo v1.3 (Power outputs: Hotend0, Hotend1, Hotend2, Fan0, Fan1, Bed)
#define BOARD_ARCHIM2         1590    // UltiMachine Archim2 (with TMC2130 drivers)
#define BOARD_ULTRATRONICS    1705    // Ultratronics v1.0 ARM 32 bit board

/**
 * SAMD21J18
 */
#define BOARD_MINITRONICS_V2  2706    // Minitronics v2.0

#endif /* _BOARD_H_ */
